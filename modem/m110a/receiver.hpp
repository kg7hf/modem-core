// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/cir_equalizer.hpp"
#include "modem/common/dsp/timing_recovery.hpp"
#include "modem/common/status.hpp"
#include "modem/common/types.hpp"
#include "modem/m110a/body_waveform.hpp"
#include "modem/m110a/equalizer.hpp"
#include "modem/m110a/scrambler.hpp"
#include "modem/m110a/synchronization.hpp"

#include <array>
#include <limits>

// =============================================================================
// receiver.hpp - BodyAdaptiveReceiver: adaptive DFE + carrier equalizer engine
// =============================================================================
// The equalizer engine that trains on the preamble (with the frequency race),
// runs under the geometry its caller configures it with (the caller maps the
// measured delay spread to one with select_body_receiver_geometry below), and
// equalizes the body frame by frame (collapse retrain). The model-based engine
// (cir_mmse) is a runtime choice in a host build (M110_BODY_RECEIVER_CIR_ENGINE).
// The comments on the constants give the reason behind each value.
// Teaching walkthrough: modem/m110a/receiver-core-equalizer-explainer.md
// =============================================================================

namespace modem::m110a
{

using ::modem::common::BitSpan;
using ::modem::common::CarrierTracker;
using ::modem::common::cir_maximum_rows;
using ::modem::common::cir_maximum_taps;
using ::modem::common::cir_precursor_taps;
using ::modem::common::CirPolyphaseProblem;
using ::modem::common::CirSolveScratch;
using ::modem::common::DataRate;
using ::modem::common::IQSample;
using ::modem::common::IQSampleSpan;
using ::modem::common::MutableIQSampleSpan;
using ::modem::common::Status;

// Default body-receiver geometry, shared by every caller so they all describe
// one receiver. The 24-tap T/2 feedforward span exists because a 2 ms delayed
// path sits ~9.6 T/2 taps past a centered cursor: a 17-tap window cannot reach
// it, and a realization whose direct path fades is then unequalizable. 24 taps
// at T/2 span 12 symbols, holding the 2 ms echo inside the window with margin;
// the 5 ms robust-mode echo falls outside it and takes the long-delay geometry
// that select_body_receiver_geometry maps from the measured delay spread.
inline constexpr std::size_t body_receiver_feedforward_taps = 24U;
inline constexpr std::size_t body_receiver_feedback_taps = 8U;
// Rate-dependent NLMS equalizer step. Only the robust low rates (75/150/300),
// whose Table 10.1 conformance points are the hard 5 ms/5 Hz corner, use the
// gentle step: there the aggressive step misadjusts (gradient noise drives the
// equalizer off the true channel). The gentle step helps the robust low rates;
// the fast rates need the aggressive one: 2400/4800 are convergence-limited, and
// 600/1200 (tested at 2 ms/1 Hz) lose bits at the gentle step when fading biases
// the frequency estimate.
inline constexpr float body_receiver_equalizer_step_fast = 0.40F;
inline constexpr float body_receiver_equalizer_step_robust = 0.10F;
inline constexpr float body_receiver_equalizer_step = body_receiver_equalizer_step_fast;

[[nodiscard]] constexpr float body_receiver_equalizer_step_for(DataRate rate) noexcept
{
    return (rate == DataRate::bps75 || rate == DataRate::bps150 || rate == DataRate::bps300) ? body_receiver_equalizer_step_robust : body_receiver_equalizer_step_fast;
}

static_assert(body_receiver_equalizer_step_for(DataRate::bps75) == body_receiver_equalizer_step_robust);
static_assert(body_receiver_equalizer_step_for(DataRate::bps150) == body_receiver_equalizer_step_robust);
static_assert(body_receiver_equalizer_step_for(DataRate::bps300) == body_receiver_equalizer_step_robust);
static_assert(body_receiver_equalizer_step_for(DataRate::bps600) == body_receiver_equalizer_step_fast);
static_assert(body_receiver_equalizer_step_for(DataRate::bps1200) == body_receiver_equalizer_step_fast);
static_assert(body_receiver_equalizer_step_for(DataRate::bps2400) == body_receiver_equalizer_step_fast);
static_assert(body_receiver_equalizer_step_for(DataRate::bps4800) == body_receiver_equalizer_step_fast);

inline constexpr float body_receiver_carrier_proportional_gain = 0.02F;
inline constexpr float body_receiver_carrier_integral_gain = 0.0002F;
// Absolute outage term of the adaptive collapse retrain (see
// BodyAdaptiveReceiver::set_collapse_absolute_threshold). A probe residual at
// or above 90 % of the constellation power is a lost frame regardless of the
// running baseline, which at low SNR is itself noise-dominated (~0.3-0.5) and
// never lets the relative test fire. Applies to the batch and the streaming
// (embedded) frame walk alike.
inline constexpr float body_receiver_collapse_absolute_threshold = 0.9F;
// Adaptation family for the probe-trained DFE. The receiver uses NLMS. RLS,
// kept as an option, converges within roughly twice the tap count where NLMS
// needs many tens of symbols; its forgetting factor sets the tracking memory
// (~1/(1-lambda) symbols).
inline constexpr DfeAdaptation body_receiver_adaptation = DfeAdaptation::nlms;
inline constexpr float body_receiver_rls_forgetting = 0.99F;

// Data-symbol equalization architecture. The adaptive family filters data
// symbols with gradient-adapted taps that follow the channel from probe to
// probe and through reliable decisions. The cir_mmse family estimates the
// short channel response from each probe instead, computes the MMSE-DFE taps
// from the estimate, and interpolates the solved taps across each data frame,
// so both bracketing probes inform every data symbol. Neither wins everywhere:
// each engine's strengths depend on the channel and the rate, and Part 6 of
// the article series measures both. The whole-burst turbo equalizer is Part 7's.
enum class BodyDataEqualizer : std::uint8_t
{
    adaptive,
    cir_mmse
};

// Compile-time AVAILABILITY of the channel-estimate MMSE-DFE engine. The
// engine is selected at RUNTIME with BodyAdaptiveReceiver::select_data_equalizer;
// the default is always the adaptive gradient DFE. Host builds compile the
// engine in, so a host tool can compare the two engines; embedded profiles
// leave it out, so the embedded receiver does not hold ~14 KiB of
// never-executed CirEqualizerState on every instance.
// The gate itself. This header DEFINES M110_BODY_RECEIVER_CIR_ENGINE to 0 by
// default, so any translation unit that includes receiver.hpp without the build
// setting it sees the adaptive-only receiver. The native build sets it to 1
// (CMakeLists.txt: an option, ON by default; an embedded build turns it off),
// which compiles in every `#if M110_BODY_RECEIVER_CIR_ENGINE` block in
// receiver.hpp/.cpp. When 0 that entire engine is compiled out.
#ifndef M110_BODY_RECEIVER_CIR_ENGINE
#define M110_BODY_RECEIVER_CIR_ENGINE 0
#endif

// True when `engine` can be selected in this build.
[[nodiscard]] constexpr bool body_receiver_engine_available(BodyDataEqualizer engine) noexcept
{
    return engine == BodyDataEqualizer::adaptive || M110_BODY_RECEIVER_CIR_ENGINE != 0;
}

// Delay-adaptive equalizer geometry. The window must hold the delayed path: a
// 2 ms echo sits +9.6 T/2 taps past the cursor and the default geometry covers
// it centered, while long robust-mode echoes (5 ms = +24 T/2 taps) need the
// asymmetric long-delay geometry (short precursor, 12.5-symbol postcursor,
// feedback spanning the echo). The caller selects it from the measured
// preamble delay spread, never from the mode label, and configures the
// receiver with it.
struct BodyReceiverGeometry
{
    std::size_t feedforward_taps{};
    std::size_t cursor{};
    std::size_t feedback_taps{};
};

// The longest delay spread, in symbols, the default geometry holds: a 2 ms row
// (5 symbols) stays on it, a 5 ms row (12 symbols) takes the long-delay one.
inline constexpr std::size_t body_receiver_default_delay_symbols = 6U;

[[nodiscard]] constexpr BodyReceiverGeometry select_body_receiver_geometry(std::size_t delay_spread_symbols) noexcept
{
    if (delay_spread_symbols > body_receiver_default_delay_symbols)
    {
        return {32U, 6U, 14U};
    }

    return {body_receiver_feedforward_taps, body_receiver_feedforward_taps / 2U, body_receiver_feedback_taps};
}

// Cross-frame state of the adaptive body equalizer within one interleaver
// block, externalized so a streaming receiver can equalize frame by frame as
// samples arrive. The block-scoped scrambler and channel-state seeds live
// here; the equalizer taps, decision history, and carrier loop remain inside
// the receiver. begin_block_frames seeds it at every block boundary and
// end_block_frames commits the block's residual back to the receiver, which
// reproduces equalize_block exactly.
struct BodyFrameState
{
    BodyDataRandomizer randomizer{};
    float previous_probe_error{};
    float baseline_probe_error{};
    float block_probe_error_energy{};
    std::size_t block_probe_symbols{};
    std::size_t frame_index{};
};

class BodyAdaptiveReceiver
{
public:
    void reset() noexcept;
    [[nodiscard]] Status rearm() noexcept;
    [[nodiscard]] Status configure(std::size_t equalizer_taps, float equalizer_step_size, float carrier_proportional_gain, float carrier_integral_gain, std::size_t feedback_taps = 8U) noexcept;
    [[nodiscard]] Status configure_fractionally_spaced(std::size_t feedforward_taps, std::size_t feedback_taps, float equalizer_step_size, float carrier_proportional_gain, float carrier_integral_gain,
                                                       std::size_t cursor = 0U) noexcept;
    [[nodiscard]] Status train_preamble(IQSampleSpan received_preamble, BitSpan expected_tribits, const BodyAcquisition& acquisition) noexcept;
    [[nodiscard]] Status train_preamble_fractional(IQSampleSpan received_half_symbols, BitSpan expected_tribits, const BodyAcquisition& acquisition) noexcept;

    // Resumable preamble training. The streaming driver arms one training
    // with begin_preamble_training, then advances it a bounded quota of preamble
    // symbol-updates per audio callback with step_preamble_training until it is no
    // longer running; on complete the receiver holds the same trained state the
    // batch train_preamble above would produce. The batch path drives this to
    // completion internally, so every host caller exercises the sliced code.
    enum class PreambleStepOutcome : std::uint8_t
    {
        running,
        complete,
        failed
    };
    [[nodiscard]] Status begin_preamble_training(IQSampleSpan received_samples, std::size_t samples_per_symbol, BitSpan expected_tribits, const BodyAcquisition& acquisition) noexcept;
    [[nodiscard]] PreambleStepOutcome step_preamble_training(std::size_t quota) noexcept;
    [[nodiscard]] bool preamble_training_running() const noexcept;

    [[nodiscard]] Status equalize_block(IQSampleSpan received_symbols, const BodyBlockPlan& plan, MutableIQSampleSpan equalized_symbols) noexcept;
    [[nodiscard]] Status equalize_block_fractional(IQSampleSpan received_half_symbols, const BodyBlockPlan& plan, MutableIQSampleSpan equalized_symbols) noexcept;
    // Supervised re-equalization for iterative decode feedback: every symbol
    // trains against the caller's reference (transmitted-domain tribits, e.g.
    // a re-encoded first-pass decode), so a block whose first pass carried a
    // handful of decision errors is re-run with near-perfect training. The
    // caller owns pass selection: copy the receiver before the first pass,
    // restore, and keep whichever pass re-encodes more consistently.
    [[nodiscard]] Status equalize_block_supervised(IQSampleSpan received_symbols, const BodyBlockPlan& plan, BitSpan reference_tribits, MutableIQSampleSpan equalized_symbols) noexcept;
    [[nodiscard]] Status equalize_block_supervised_fractional(IQSampleSpan received_half_symbols, const BodyBlockPlan& plan, BitSpan reference_tribits, MutableIQSampleSpan equalized_symbols) noexcept;

    // Frame-resumable first-pass (adaptive-family) body equalization; the
    // whole-block entry points route through these. The sample span may be
    // any window that covers the frame plus the feedforward reach - cursor
    // taps before the frame's first cursor sample and feedforward_taps-1 -
    // cursor after its last - with frame_first_symbol indexing symbols
    // within that window; positions outside the span zero-fill. A caller
    // passing block-truncated spans reproduces equalize_block exactly; the
    // streaming receiver instead supplies real neighbor samples in the
    // margins, so at interleaver-block edges its windows see the preamble
    // tail and the adjacent block where the batch path sees zeros: a known
    // difference from the whole-block reference, not an equivalence. One equalize_frame call processes the frame's unknown
    // data symbols plus its trailing probe, including collapse retrain and
    // channel-state weighting of the data outputs.
    void begin_block_frames(BodyFrameState& state) noexcept;
    [[nodiscard]] Status equalize_frame(IQSampleSpan received_symbols, std::size_t frame_first_symbol, const BodyBlockPlan& plan, BodyFrameState& state,
                                        MutableIQSampleSpan equalized_symbols) noexcept;
    [[nodiscard]] Status equalize_frame_fractional(IQSampleSpan received_half_symbols, std::size_t frame_first_symbol, const BodyBlockPlan& plan, BodyFrameState& state,
                                                   MutableIQSampleSpan equalized_symbols) noexcept;
    // One 75-bps 32-chip spread group: open-loop candidate correlation, then
    // supervised re-training on the winning spread pattern. No probes, no
    // channel-state weighting, and no residual commit - callers skip
    // end_block_frames on this path, matching equalize_block.
    [[nodiscard]] Status equalize_75_group(IQSampleSpan received_symbols, std::size_t group_first_symbol, bool exceptional_set, BodyFrameState& state, MutableIQSampleSpan equalized_symbols) noexcept;
    [[nodiscard]] Status equalize_75_group_fractional(IQSampleSpan received_half_symbols, std::size_t group_first_symbol, bool exceptional_set, BodyFrameState& state,
                                                      MutableIQSampleSpan equalized_symbols) noexcept;
    void end_block_frames(const BodyFrameState& state) noexcept;

    [[nodiscard]] const ProbeDfe& equalizer() const noexcept { return mEqualizer; }

    [[nodiscard]] const CarrierTracker& carrier_tracker() const noexcept { return mCarrierTracker; }

    [[nodiscard]] bool is_configured() const noexcept { return mConfigured; }

    [[nodiscard]] bool is_trained() const noexcept { return mTrained; }

    // Mean probe-training residual power of the most recently equalized block
    // (unit-power constellation reference). In-lock blocks measure well below
    // one; a block equalized against noise or a lost carrier saturates toward
    // the constellation power. Streaming owners use this as the calibrated
    // loss-of-lock metric; it is not updated by the 75-bps spread path, which
    // carries no probes.
    [[nodiscard]] float last_block_probe_error() const noexcept { return mLastBlockProbeError; }

    // Initial carrier frequency (radians per symbol) of the hypothesis the
    // most recent preamble training selected by measured residual. Owners
    // that need a trustworthy derotation frequency after training (the
    // delay-spread estimator feeding geometry selection) read this instead
    // of re-deciding between the acquisition and refined estimates.
    [[nodiscard]] float selected_training_frequency_radians() const noexcept { return mSelectedTrainingFrequencyRadians; }

    // True when the most recent preamble training escalated past the cheap
    // acquisition hypothesis into the differential/grid frequency race.
    // Ideal-to-clean channels, whose acquisition residual reaches the clamped
    // noise floor, stay false and skip the expensive race; a fading channel
    // that biases the acquisition estimate escalates. Diagnostic only: it never
    // changes the selected receiver definition, and the escalated path
    // reproduces the ungated three-hypothesis result exactly.
    [[nodiscard]] bool last_preamble_race_escalated() const noexcept { return mLastPreambleRaceEscalated; }

    // Diagnostic view of the most recent frequency race: every
    // hypothesis trained (acquisition first, then the whole-preamble
    // differential and the channel-fit grid when distinct), its supervised
    // tail residual, and which one won. Pure copies of the race state.
    struct PreambleRaceDiagnostics
    {
        std::array<float, 3U> frequency_radians{};
        std::array<float, 3U> residual{};
        std::size_t count{};
        std::size_t winner{};
        bool escalated{};
    };

    [[nodiscard]] PreambleRaceDiagnostics last_preamble_race() const noexcept
    {
        PreambleRaceDiagnostics diagnostics{};
        diagnostics.frequency_radians = mTrainState.candidate_frequencies;
        diagnostics.residual = mTrainState.candidate_residuals;
        diagnostics.count = mLastPreambleRaceEscalated ? mTrainState.candidate_count : 1U;
        diagnostics.winner = mLastPreambleRaceEscalated ? mTrainState.best_index : 0U;
        diagnostics.escalated = mLastPreambleRaceEscalated;

        if (!mLastPreambleRaceEscalated)
        {
            diagnostics.frequency_radians[0] = mSelectedTrainingFrequencyRadians;
            diagnostics.residual[0] = mLastProbeError;
        }

        return diagnostics;
    }

    // Absolute probe-residual term for the adaptive collapse retrain: the
    // relative test alone cannot declare a full outage at low SNR because the
    // baseline is itself noise-dominated (the CIR family carries this term as
    // absolute_outage_error = 0.9). The default is
    // body_receiver_collapse_absolute_threshold; +infinity turns the absolute
    // term off. A receiver-definition setting: reset() and configure() leave
    // it alone.
    void set_collapse_absolute_threshold(float threshold) noexcept { mCollapseAbsoluteThreshold = threshold; }

    // Runtime data-symbol equalizer engine selection: adaptive (the default)
    // or cir_mmse. Select BEFORE configure/train: the CIR engine seeds its
    // channel state at the end of every preamble training hypothesis, and an
    // engine selected after training finds no seed and falls back to the
    // adaptive block walk for that burst. The selection is a
    // receiver-definition setting (not touched by reset()/configure()).
    // Returns unavailable when the engine is compiled out of this build
    // (body_receiver_engine_available).
    [[nodiscard]] Status select_data_equalizer(BodyDataEqualizer engine) noexcept;

    [[nodiscard]] BodyDataEqualizer data_equalizer() const noexcept { return mDataEqualizer; }

private:
    // Winner memoization is valid only for the adaptive engine
    // under NLMS (see receiver.cpp); the CIR engine re-runs the winner.
    [[nodiscard]] bool memoize_winner() const noexcept { return mDataEqualizer == BodyDataEqualizer::adaptive && body_receiver_adaptation == DfeAdaptation::nlms; }

    // Always-present preamble channel-fit frequency-search workspace.
    // The third training frequency hypothesis (the grid_build_rows/advance_search_grid
    // channel-fit grid) runs on every fractionally-spaced mode regardless of the
    // data-symbol equalizer family, so its polyphase solve buffers cannot live inside the
    // compile-time-gated CirEqualizerState. When the CIR family is compiled it
    // reuses these same buffers - every solve rebuilds each row it reads before
    // use, so the search and the CIR equalizer never alias in time.
    // support_precursor is the shared CIR precursor length: the search reads it
    // and, in cir_mmse builds, seed_cir_state writes it.
    struct PreambleFitWorkspace
    {
        CirPolyphaseProblem even_problem{};
        CirPolyphaseProblem odd_problem{};
        CirSolveScratch scratch{};
        std::size_t support_precursor{cir_precursor_taps};
        // Per-grid-point observation inputs, kept here so the sliced grid can
        // resume between points: each point rebuilds observations[] by rotating
        // these frozen raw samples through the trial derotation slope.
        std::array<IQSample, cir_maximum_rows> even_raw{};
        std::array<IQSample, cir_maximum_rows> odd_raw{};
        std::array<float, cir_maximum_rows> even_offset{};
        std::array<float, cir_maximum_rows> odd_offset{};
    };

    // Memoized end-of-training state of the winning frequency
    // hypothesis so the race need not retrain the winner on identical data. It
    // covers exactly the NLMS-mutated equalizer state plus the carrier loop, and
    // is valid only for the NLMS adaptation family (see ProbeDfe::NlmsState); RLS
    // and CIR builds keep the literal retrain.
    struct PreambleWinnerSnapshot
    {
        ProbeDfe::NlmsState equalizer{};
        float carrier_phase{};
        float carrier_frequency{};
    };

    // Resumable preamble training. The acquisition hypothesis's prefix channel
    // estimate and its multi-pass replay (too long for one audio callback on an
    // embedded target) are advanced a bounded quota of symbol-updates per step so
    // the streaming callback stays under its deadline; the trained state lives in
    // mEqualizer/mCarrierTracker (members) and the loop cursors/accumulators live
    // here, so a resumed continuation performs exactly the operations of an
    // uninterrupted one.
    struct PreambleTrainState
    {
        // The escalation (frequency race) is resumable too, so the outer
        // stage machine carries the refine/grid/candidate race across audio
        // callbacks. hypothesis drives the reusable inner trainer for whichever
        // Phase is active; refine/search_build/search_grid advance the frequency
        // race one quantum at a time; complete/failed are terminal.
        enum class Stage : std::uint8_t
        {
            idle,
            hypothesis,
            refine,
            search_build,
            search_grid,
            complete,
            failed
        };
        // What the inner trainer's finalize transition does next.
        enum class Phase : std::uint8_t
        {
            acquisition,
            candidate,
            winner
        };
        enum class InnerSub : std::uint8_t
        {
            prefix,
            replay
        };

        Stage stage{Stage::idle};
        Phase phase{Phase::acquisition};
        Status result{};

        // Frozen inputs (caller keeps the underlying buffers valid until done).
        IQSampleSpan samples{};
        BitSpan tribits{};
        std::size_t samples_per_symbol{1U};
        BodyAcquisition acquisition{};
        float acquisition_frequency_radians{};
        std::size_t pretraining_support_precursor{};
        float acquisition_residual{};

        // Inner-trainer cursor + accumulators (one hypothesis at current_frequency).
        bool inner_active{};
        InnerSub sub{InnerSub::prefix};
        float current_frequency{};
        IQSample channel_cross{};
        float reference_energy{};
        std::size_t prefix_index{};
        std::size_t estimate_symbols{};
        std::size_t pass{};
        std::size_t symbol{};
        std::size_t training_passes{};
        std::size_t residual_symbols{};
        std::size_t preamble_length{};
        float training_residual_energy{};

        // Resumable escalation cursors/accumulators. refine_accum sums the
        // whole-preamble lag-1 differential in index order (starting at 1).
        // grid_offset is the running float accumulator (advanced += 0.008F); it
        // is never recomputed from an integer index, so the trial frequencies
        // stay the same however the work is sliced. best_snapshot
        // memoizes the winning hypothesis end-state (NLMS build) so the winner
        // need not be retrained, and must survive yields, hence its home here.
        std::size_t refine_index{};
        IQSample refine_accum{};
        std::array<float, 3U> candidate_frequencies{};
        // Diagnostic only: each candidate's supervised tail
        // residual as measured, so the race decision can be audited.
        std::array<float, 3U> candidate_residuals{};
        std::size_t candidate_count{};
        std::size_t candidate_index{};
        std::size_t best_index{};
        float best_residual{};
        float grid_offset{};
        float grid_best_frequency{};
        float grid_best_residual{};
        bool search_valid{};
        PreambleWinnerSnapshot best_snapshot{};
    };

#if M110_BODY_RECEIVER_CIR_ENGINE
    // Channel-estimate (CIR-MMSE) equalizer state, compiled only with the
    // engine: the previous CIR (the ridge prior and
    // frequency-nudge reference), the left anchor's solved taps, the
    // committed-symbol feedback seed, and the outage re-acquisition
    // accumulators. The polyphase solve buffers and the precursor length live
    // in PreambleFitWorkspace above, shared with the always-present search.
    struct CirEqualizerState
    {
        std::array<IQSample, cir_maximum_taps> channel{};
        std::array<IQSample, ProbeDfe::maximum_feedforward_taps> anchor_feedforward{};
        std::array<IQSample, ProbeDfe::maximum_feedback_taps> anchor_feedback{};
        std::array<IQSample, ProbeDfe::maximum_feedback_taps> committed_history{};
        // Re-acquisition accumulators: decision-free probe rows stacked
        // across consecutive outage frames until the channel is
        // overdetermined without any prior, because a single probe's clean
        // rows alone cannot re-identify the channel after a deep fade.
        CirPolyphaseProblem recovery_even_problem{};
        CirPolyphaseProblem recovery_odd_problem{};
        float anchor_residual{0.01F};
        float anchor_symbol{};
        float noise_variance{0.05F};
        bool seeded{};
    };
#endif

    [[nodiscard]] Status train_preamble_impl(IQSampleSpan received_samples, std::size_t samples_per_symbol, BitSpan expected_tribits, const BodyAcquisition& acquisition) noexcept;
    // Resumable inner trainer. inner_trainer_begin arms one hypothesis at
    // frequency_radians; inner_trainer_step advances its prefix estimate + multi-
    // pass replay by up to `quota` symbol-updates, setting finished/status on
    // completion. Both the acquisition hypothesis and every escalation candidate
    // (and, in the RLS/CIR fallback, the winner re-run) drive this same trainer.
    void inner_trainer_begin(float frequency_radians) noexcept;
    [[nodiscard]] std::size_t inner_trainer_step(std::size_t quota, bool& finished, Status& status) noexcept;
    // Resumable escalation (the fading frequency race), driven by
    // step_preamble_training after the acquisition hypothesis trains with a
    // residual above the skip floor. finalize_hypothesis is the inner trainer's
    // finalize transition (branch on Phase); advance_refine walks the whole-
    // preamble differential one index/quantum; grid_build_rows builds the polyphase
    // rows once; advance_search_grid solves the 21-point channel-fit grid one
    // point/quantum; dispatch_race starts the candidate trainings (or completes on
    // a lone acquisition hypothesis); select_winner restores/re-runs the strict-
    // minimum-residual winner. Each drives mEqualizer/mCarrierTracker/mFitWorkspace
    // in place, in the same order as an uninterrupted run: no parallel decode
    // path.
    void finalize_hypothesis() noexcept;
    [[nodiscard]] std::size_t advance_refine(std::size_t credits) noexcept;
    void grid_build_rows() noexcept;
    [[nodiscard]] std::size_t advance_search_grid(std::size_t credits) noexcept;
    void dispatch_race() noexcept;
    void select_winner() noexcept;
    // The single successful terminal transition of the training machine
    // (every path that leaves the winner's trained state live lands here).
    void complete_preamble_training() noexcept;
    [[nodiscard]] Status equalize_block_impl(IQSampleSpan received_samples, std::size_t samples_per_symbol, const BodyBlockPlan& plan, MutableIQSampleSpan equalized_symbols) noexcept;
    [[nodiscard]] Status equalize_frame_impl(IQSampleSpan received_samples, std::size_t samples_per_symbol, std::size_t frame_first_symbol, const BodyBlockPlan& plan, BodyFrameState& state,
                                             MutableIQSampleSpan equalized_symbols) noexcept;
    [[nodiscard]] Status equalize_75_group_impl(IQSampleSpan received_samples, std::size_t samples_per_symbol, std::size_t group_first_symbol, bool exceptional_set, BodyFrameState& state,
                                                MutableIQSampleSpan equalized_symbols) noexcept;
    [[nodiscard]] Status equalize_block_supervised_impl(IQSampleSpan received_samples, std::size_t samples_per_symbol, const BodyBlockPlan& plan, BitSpan reference_tribits,
                                                        MutableIQSampleSpan equalized_symbols) noexcept;
#if M110_BODY_RECEIVER_CIR_ENGINE
    // The preamble CIR seed search: adopts the searched carrier ramp into
    // mCarrierTracker and the chosen precursor into mFitWorkspace (the CIR
    // walk continues from them) and seeds mCirState.
    void seed_cir_state(IQSampleSpan received_samples, std::size_t samples_per_symbol, BitSpan expected_tribits) noexcept;
    [[nodiscard]] Status equalize_block_cir(IQSampleSpan received_half_symbols, const BodyBlockPlan& plan, MutableIQSampleSpan equalized_symbols) noexcept;
    [[nodiscard]] Status equalize_block_supervised_cir(IQSampleSpan received_half_symbols, const BodyBlockPlan& plan, BitSpan reference_tribits, MutableIQSampleSpan equalized_symbols) noexcept;
#endif

    ProbeDfe mEqualizer{};
    CarrierTracker mCarrierTracker{};
    PreambleFitWorkspace mFitWorkspace{};
#if M110_BODY_RECEIVER_CIR_ENGINE
    CirEqualizerState mCirState{};
#endif
    std::size_t mSamplesPerSymbol{1U};
    float mLastProbeError{};
    float mLastBlockProbeError{};
    float mSelectedTrainingFrequencyRadians{};
    bool mConfigured{};
    bool mTrained{};
    bool mLastPreambleRaceEscalated{};
    float mCollapseAbsoluteThreshold{body_receiver_collapse_absolute_threshold};
    BodyDataEqualizer mDataEqualizer{BodyDataEqualizer::adaptive};
    PreambleTrainState mTrainState{};
};

} // namespace modem::m110a
