// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

/*
================================================================================
receiver.cpp - BodyAdaptiveReceiver: the adaptive DFE + carrier equalizer engine
================================================================================
Turns a located burst into equalized symbols. It trains on the preamble (running
a frequency RACE between three hypotheses to beat fade-biased acquisition), runs
under the equalizer geometry its caller configured from the measured delay
spread, equalizes the body frame by frame (re-training on each probe,
collapse-retraining a frame a fade knocked it off), and optionally runs the
model-based CIR-MMSE engine instead. Its resumable training lets a streaming
caller stay under a per-callback deadline and still produce exactly the result
of the one-call path.

  INPUT  : received (T/2) symbols + a BodyAcquisition; the expected preamble tribits.
  OUTPUT : equalized symbols (to the demapper/FEC); diagnostics.
  DEPENDS: equalizer (ProbeDfe), cir_equalizer, synchronization (CarrierTracker),
           demapper, scrambler.

KEY GROUPS
----------
  configure / train_preamble / begin+step_preamble_training   setup + resumable training
  the frequency race: finalize_hypothesis, advance_refine, grid_build_rows,
      advance_search_grid, dispatch_race, select_winner, complete_preamble_training
  equalize_block / equalize_frame (+ _impl) / equalize_75_group   the data walk
  select_data_equalizer / seed_cir_state / *_cir                  the CIR engine
The runtime engine (adaptive or cir_mmse) is chosen by select_data_equalizer;
the default is always the adaptive gradient DFE. The CIR engine is host-only
(compile-gated by M110_BODY_RECEIVER_CIR_ENGINE, 0 by default in receiver.hpp;
the native build sets 1).
Teaching walkthrough: modem/m110a/receiver-core-equalizer-explainer.md
================================================================================
*/

#include "modem/m110a/receiver.hpp"

#include "modem/common/constants.hpp"
#include "modem/common/convert.hpp"
#include "modem/m110a/demapper.hpp"
#include "modem/m110a/scrambler.hpp"

#include <algorithm>
#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <tuple>

#include "modem/common/status.hpp"
namespace modem::m110a
{

using ::modem::common::as_real;
using ::modem::common::BitSpan;
using ::modem::common::BodyInterleave;
using ::modem::common::cir_maximum_rows;
using ::modem::common::cir_maximum_taps;
using ::modem::common::cir_mmse_dfe_solve;
using ::modem::common::cir_noise_inflation;
using ::modem::common::cir_polyphase_solve;
using ::modem::common::cir_polyphase_taps;
using ::modem::common::cir_preamble_seed_symbols;
using ::modem::common::cir_precursor_taps;
using ::modem::common::cir_residual_dof;
using ::modem::common::cir_tap_index;
using ::modem::common::CirMmseGeometry;
using ::modem::common::CirPolyphaseProblem;
using ::modem::common::DataRate;
using ::modem::common::index_in;
using ::modem::common::IQSample;
using ::modem::common::IQSampleSpan;
using ::modem::common::make_cir_layout;
using ::modem::common::MutableIQSampleSpan;
using ::modem::common::Status;
using ::modem::common::StatusCode;
using ::modem::common::to_offset;
using ::modem::common::to_subscript;
using ::modem::common::two_pi;

// Lower bound on the per-frame probe residual power used for channel-state
// weighting, relative to the unit-power constellation. It caps the weight a
// single short probe run can claim and guards the reciprocal.
constexpr float probe_noise_floor = 0.01F;

// Frequency-hypothesis gating threshold. When the acquisition-trained
// preamble residual is at or below this value the frequency race is skipped.
// It sits at the residual floor deliberately: inner_trainer_step clamps
// mLastProbeError at probe_noise_floor, so at the floor no differential
// or grid hypothesis can score strictly lower, and the race keeps the strict
// minimum, which ties to the first (acquisition) hypothesis. Skipping there is
// therefore outcome-identical to running the full race: the WCET is removed on
// ideal-to-clean channels with no change to the selected receiver definition on
// any channel. Raising it above the floor trades provable equivalence for an
// empirical no-regression measurement.
constexpr float body_preamble_race_skip_residual = probe_noise_floor;

// A collapse retrain fires only when the mean probe residual is both a large
// fraction of the constellation power and far above the block's running
// residual baseline: the absolute floor alone would false-fire continuously
// at low SNR (where the noise-dominated residual is legitimately large and a
// retrain only overfits the short probe), while the relative test isolates a
// genuine equalizer collapse against clean operation at high SNR. The pass
// count bounds the extra work per collapsed frame.
constexpr float collapse_retrain_threshold = 0.50F;
constexpr float collapse_retrain_baseline_ratio = 2.0F;
constexpr std::size_t collapse_retrain_passes = 4U;

// Resumable escalation grid geometry. The grid is a running-float accumulator
// (grid_offset advanced by preamble_grid_search_step) with the exact termination
// bound below; together they fix the trial frequencies, so they must never be
// replaced by a recomputed integer index.
constexpr float preamble_grid_search_span = 0.08F;
constexpr float preamble_grid_search_step = 0.008F;
// Conservative credit weight of one grid quantum relative to one preamble
// symbol-update: a grid point runs two cir_polyphase_solve over <=48 rows / <=16
// taps plus ~96 std::polar. Over-charging only makes the real step finish under
// budget; it never changes the decode (quota is a pure yield-placement knob).
constexpr std::size_t preamble_grid_point_weight = 20U;

// Winner memoization is complete only for the NLMS adaptation family
// (ProbeDfe::NlmsState captures exactly what train_nlms mutates). RLS's
// inverse-correlation matrix and the CIR engine's seeded mCirState are not
// captured, so those configurations fall back to the literal winner re-run.
// The engine is a runtime selection, so the decision is the member predicate
// memoize_winner() (adaptive engine + NLMS).

static void make_equalizer_window(IQSampleSpan received_samples, std::size_t symbol_index, std::size_t samples_per_symbol, float carrier_phase, float frequency_radians_per_symbol,
                                  const ProbeDfe& equalizer, std::array<IQSample, ProbeDfe::maximum_feedforward_taps>& window) noexcept
{
    window.fill(IQSample{});
    const auto center_sample = symbol_index * samples_per_symbol + samples_per_symbol - 1U;

    // The per-tap derotation phasor advances by a constant angle
    // (-frequency_radians_per_symbol / samples_per_symbol) as the tap index
    // increases, so seed it once and step it with one complex multiply per tap
    // instead of a std::polar (~24 transcendentals/symbol). The ~1e-6 recurrence
    // drift over the small feedforward span is far below the soft-decision
    // margin.
    const auto inverse_samples_per_symbol = 1.0F / as_real(samples_per_symbol);
    const auto phasor_step = std::polar(1.0F, -frequency_radians_per_symbol * inverse_samples_per_symbol);
    const auto initial_symbol_offset = -as_real(equalizer.cursor()) * inverse_samples_per_symbol;
    auto phasor = std::polar(1.0F, -(carrier_phase + frequency_radians_per_symbol * initial_symbol_offset));
    // Signed: the precursor taps reach before the first received sample.
    auto sample_index = to_offset(center_sample) - to_offset(equalizer.cursor());

    for (std::size_t tap = 0U; tap < equalizer.feedforward_taps(); ++tap, ++sample_index)
    {
        // The try-index form, not element_or: an out-of-range tap keeps the +0 fill
        // (fallback * phasor could produce -0).
        if (const auto sample = index_in(sample_index, received_samples.size()))
        {
            window[tap] = received_samples[*sample] * phasor;
        }

        phasor *= phasor_step;
    }
}

void BodyAdaptiveReceiver::reset() noexcept
{
    mEqualizer.reset();
    mCarrierTracker.reset(0.0F, 0.0F);
    // The preamble frequency-search workspace is stale-safe - every solve
    // rebuilds each row it reads before use - so only its shared precursor
    // length needs re-seeding here.
    mFitWorkspace.support_precursor = cir_precursor_taps;
#if M110_BODY_RECEIVER_CIR_ENGINE
    // Reconstruct directly in the member's storage. Assignment from
    // CirEqualizerState{} materializes a multi-KiB temporary that overflows
    // the embedded application task's bounded stack; the in-place reconstruct
    // avoids it. The default (adaptive) build carries no mCirState at all.
    std::destroy_at(std::addressof(mCirState));
    std::construct_at(std::addressof(mCirState));
#endif
    mSamplesPerSymbol = 1U;
    mLastProbeError = probe_noise_floor;
    mLastBlockProbeError = 0.0F;
    mConfigured = false;
    mTrained = false;
}

Status BodyAdaptiveReceiver::select_data_equalizer(BodyDataEqualizer engine) noexcept
{
    if (!body_receiver_engine_available(engine))
    {
        return {StatusCode::unavailable, "requested body equalizer engine is not compiled into this build"};
    }

    mDataEqualizer = engine;
    return Status::success();
}

Status BodyAdaptiveReceiver::rearm() noexcept
{
    if (!mConfigured)
    {
        return {StatusCode::unavailable, "adaptive receiver is not configured"};
    }

    mEqualizer.reset();
    mCarrierTracker.reset(0.0F, 0.0F);
    mFitWorkspace.support_precursor = cir_precursor_taps;
#if M110_BODY_RECEIVER_CIR_ENGINE
    std::destroy_at(std::addressof(mCirState));
    std::construct_at(std::addressof(mCirState));
#endif
    mLastProbeError = probe_noise_floor;
    mLastBlockProbeError = 0.0F;
    mTrained = false;
    return Status::success();
}

Status BodyAdaptiveReceiver::configure(std::size_t equalizer_taps, float equalizer_step_size, float carrier_proportional_gain, float carrier_integral_gain, std::size_t feedback_taps) noexcept
{
    reset();
    const auto equalizer_status = mEqualizer.configure(equalizer_taps, feedback_taps, 1U, equalizer_taps / 2U, equalizer_step_size, 0.0F, body_receiver_adaptation, body_receiver_rls_forgetting);

    if (!equalizer_status.is_ok())
    {
        return equalizer_status;
    }

    const auto carrier_status = mCarrierTracker.configure(carrier_proportional_gain, carrier_integral_gain);

    if (!carrier_status.is_ok())
    {
        return carrier_status;
    }

    mConfigured = true;
    mTrained = false;
    mSamplesPerSymbol = 1U;
    return Status::success();
}

Status BodyAdaptiveReceiver::configure_fractionally_spaced(std::size_t feedforward_taps, std::size_t feedback_taps, float equalizer_step_size, float carrier_proportional_gain,
                                                           float carrier_integral_gain, std::size_t cursor) noexcept
{
    reset();
    const auto equalizer_status =
        mEqualizer.configure(feedforward_taps, feedback_taps, 2U, cursor == 0U ? feedforward_taps / 2U : cursor, equalizer_step_size, 0.0F, body_receiver_adaptation, body_receiver_rls_forgetting);

    if (!equalizer_status.is_ok())
    {
        return equalizer_status;
    }

    const auto carrier_status = mCarrierTracker.configure(carrier_proportional_gain, carrier_integral_gain);

    if (!carrier_status.is_ok())
    {
        return carrier_status;
    }

    mConfigured = true;
    mTrained = false;
    mSamplesPerSymbol = 2U;
    return Status::success();
}

Status BodyAdaptiveReceiver::train_preamble(IQSampleSpan received_preamble, BitSpan expected_tribits, const BodyAcquisition& acquisition) noexcept
{
    return train_preamble_impl(received_preamble, 1U, expected_tribits, acquisition);
}

Status BodyAdaptiveReceiver::train_preamble_fractional(IQSampleSpan received_half_symbols, BitSpan expected_tribits, const BodyAcquisition& acquisition) noexcept
{
    return train_preamble_impl(received_half_symbols, 2U, expected_tribits, acquisition);
}

namespace
{

#if M110_BODY_RECEIVER_CIR_ENGINE
// Ridge-prior weight for the per-probe CIR solve: the Bayesian ratio of the
// measurement noise to the per-tap prior mismatch, where the mismatch is the
// channel's own innovation across one anchor interval plus the prior
// estimate's error. The anchor-interval correlation is a design constant for
// the conformance fading rates; the clamp keeps the solve's memory near one
// coherence interval of the channel.
constexpr float cir_anchor_correlation = 0.87F;

[[nodiscard]] float cir_ridge_lambda(float noise_variance, std::size_t support_taps, float prior_power, float mean_row_weight) noexcept
{
    const auto support = as_real(support_taps);
    const auto innovation = 2.0F * (1.0F - cir_anchor_correlation) * std::max(prior_power, 0.1F);
    const auto prior_error = support * noise_variance / std::max(mean_row_weight, 1.0F);
    const auto lambda = noise_variance * support / (innovation + prior_error);
    return lambda < 1.0F ? 1.0F : lambda > 16.0F ? 16.0F : lambda;
}
#endif

} // namespace

Status BodyAdaptiveReceiver::train_preamble_impl(IQSampleSpan received_samples, std::size_t samples_per_symbol, BitSpan expected_tribits, const BodyAcquisition& acquisition) noexcept
{
    // Batch entry: arm the resumable machine and drive it to completion with an
    // unbounded quota. Every batch caller reaches the sliced code this way, so
    // every batch test exercises it too.
    const auto status = begin_preamble_training(received_samples, samples_per_symbol, expected_tribits, acquisition);

    if (!status.is_ok())
    {
        return status;
    }

    auto outcome = step_preamble_training(std::numeric_limits<std::size_t>::max());

    while (outcome == PreambleStepOutcome::running)
    {
        outcome = step_preamble_training(std::numeric_limits<std::size_t>::max());
    }

    return mTrainState.result;
}

Status BodyAdaptiveReceiver::begin_preamble_training(IQSampleSpan received_samples, std::size_t samples_per_symbol, BitSpan expected_tribits, const BodyAcquisition& acquisition) noexcept
{
    if (!mConfigured || mSamplesPerSymbol != samples_per_symbol || received_samples.empty() || received_samples.size() != expected_tribits.size() * samples_per_symbol ||
        acquisition.symbol_rate_hz <= 0.0F)
    {
        mTrainState.stage = PreambleTrainState::Stage::idle;
        return {StatusCode::invalid_argument, "adaptive receiver preamble buffers are invalid"};
    }

    auto& state = mTrainState;
    state.samples = received_samples;
    state.tribits = expected_tribits;
    state.samples_per_symbol = samples_per_symbol;
    state.acquisition = acquisition;
    state.acquisition_frequency_radians = acquisition.frequency_offset_hz * two_pi / acquisition.symbol_rate_hz;
    state.result = Status::success();

    // Frequency-hypothesis gating. Train the cheap acquisition
    // hypothesis first (resumable, sliced across callbacks) and measure its
    // supervised residual; on clean channels that residual reaches the clamped
    // floor and the race is skipped outcome-identically in step_preamble_training.
    mLastPreambleRaceEscalated = false;
    // The optional CIR-MMSE training path updates the shared support precursor.
    // Candidate construction must use the value the ungated race would have seen
    // before any hypothesis training.
    state.pretraining_support_precursor = mFitWorkspace.support_precursor;

    state.phase = PreambleTrainState::Phase::acquisition;
    inner_trainer_begin(state.acquisition_frequency_radians);
    state.stage = PreambleTrainState::Stage::hypothesis;
    return Status::success();
}

// -----------------------------------------------------------------------------
// step_preamble_training  (resumable training + the frequency RACE)
// -----------------------------------------------------------------------------
// 50K view: Advance preamble training a bounded quantum; train the equalizer and
//   pick the best carrier frequency.
// Detailed view: First the acquisition-frequency hypothesis; if its residual is above
//   the skip floor, ESCALATE to a whole-preamble differential estimate (advance_refine)
//   and a 21-point channel-fit grid (grid_build_rows/advance_search_grid), train each
//   candidate (dispatch_race/inner_trainer_step), and select_winner keeps the strict-
//   minimum residual (memoizing the NLMS winner). Fully resumable so a streaming
//   callback stays under its deadline; byte-identical to the monolithic path.
// 5th-grade view: Try a few tuning-dial positions quickly and keep the clearest, a
//   little at a time so you never hog the airtime.
// -----------------------------------------------------------------------------
BodyAdaptiveReceiver::PreambleStepOutcome BodyAdaptiveReceiver::step_preamble_training(std::size_t quota) noexcept
{
    // Resumable outer stage machine. Spend up to `quota` credits across
    // whatever stages are active (one preamble symbol-update = 1 credit, one grid
    // point = preamble_grid_point_weight), yielding `running` when the budget is
    // exhausted and more work remains. The batch path (quota == max) drives the
    // whole acquisition + refine + grid + candidate race + winner select in one
    // call, so the credit model is a pure yield-placement knob and the decode is
    // invariant to it. Each stage handler runs at least one whole quantum before
    // checking the budget, so any quota >= 1 makes forward progress.
    auto& state = mTrainState;
    std::size_t spent = 0U;

    for (;;)
    {
        if (state.stage == PreambleTrainState::Stage::complete)
        {
            return PreambleStepOutcome::complete;
        }

        if (state.stage == PreambleTrainState::Stage::failed || state.stage == PreambleTrainState::Stage::idle)
        {
            return PreambleStepOutcome::failed;
        }

        if (spent >= quota)
        {
            return PreambleStepOutcome::running;
        }

        const auto remaining = quota - spent;

        switch (state.stage)
        {
        case PreambleTrainState::Stage::hypothesis:
        {
            bool finished = false;
            Status status = Status::success();
            spent += inner_trainer_step(remaining, finished, status);

            if (!finished)
            {
                return PreambleStepOutcome::running;
            }

            if (!status.is_ok())
            {
                state.result = status;
                state.stage = PreambleTrainState::Stage::failed;
                return PreambleStepOutcome::failed;
            }

            finalize_hypothesis();
            break;
        }
        case PreambleTrainState::Stage::refine:
            spent += advance_refine(remaining);
            break;
        case PreambleTrainState::Stage::search_build:
            if (state.tribits.size() < cir_preamble_seed_symbols + 16U)
            {
                // The preamble tail is too short for the channel-fit grid; the
                // race proceeds with whatever candidates refine produced.
                state.search_valid = false;
                dispatch_race();
            }
            else
            {
                grid_build_rows();
                state.grid_offset = -preamble_grid_search_span;
                state.grid_best_frequency = state.acquisition_frequency_radians;
                state.grid_best_residual = std::numeric_limits<float>::infinity();
                state.stage = PreambleTrainState::Stage::search_grid;
            }
            break;
        case PreambleTrainState::Stage::search_grid:
            spent += advance_search_grid(remaining);
            break;
        default:
            return PreambleStepOutcome::failed;
        }
    }
}

bool BodyAdaptiveReceiver::preamble_training_running() const noexcept
{
    const auto stage = mTrainState.stage;
    return stage != PreambleTrainState::Stage::idle && stage != PreambleTrainState::Stage::complete && stage != PreambleTrainState::Stage::failed;
}

void BodyAdaptiveReceiver::finalize_hypothesis() noexcept
{
    // The inner trainer's finalize transition. What happens next depends on which
    // hypothesis just finished (Phase); the frozen inputs and the frequency race
    // cursors persist in mTrainState so the machine resumes across callbacks.
    auto& state = mTrainState;

    switch (state.phase)
    {
    case PreambleTrainState::Phase::acquisition:
    {
        state.acquisition_residual = mLastProbeError;

        // Skip gate: on clean channels the acquisition residual reaches the
        // clamped floor, so no differential/grid hypothesis can score strictly
        // lower and the race is outcome-identical to skipping it.
        if (state.acquisition_residual <= body_preamble_race_skip_residual)
        {
            complete_preamble_training();
            return;
        }

        // Fading channel: escalate. Every frequency estimate is only a candidate
        // hypothesis, never a blind replacement: on fading realizations whose
        // acquisition estimate was already correct, the whole-preamble differential
        // can itself pick up a multi-hertz fading-Doppler bias, and at 5 Hz
        // spreads both estimators can be biased by tens of hertz at
        // once, which is why the channel-fit grid supplies a third candidate. The
        // decision is always by measurement: train under every distinct hypothesis
        // and keep whichever leaves the smallest supervised residual over the
        // preamble tail. The already-trained acquisition hypothesis is the first
        // entrant, so the training order is immaterial (each attempt fully resets
        // the trained state) and the strict minimum ties to it, reproducing the
        // ungated three-hypothesis race.
        mLastPreambleRaceEscalated = true;
        state.candidate_frequencies[0] = state.acquisition_frequency_radians;
        state.candidate_residuals[0] = state.acquisition_residual;
        state.candidate_count = 1U;
        state.best_index = 0U;
        state.best_residual = state.acquisition_residual;

        // Winner memoization. Capture the acquisition end-state
        // now: refine/search_build/search_grid provably touch neither mEqualizer
        // nor mCarrierTracker, so this is byte-identical to capturing it just
        // before the candidate loop (as the monolith did), and it covers the common
        // best_index == 0 win without a retrain. NLMS only; RLS/CIR re-run instead.
        if (memoize_winner())
        {
            state.best_snapshot = PreambleWinnerSnapshot{mEqualizer.capture_nlms_state(), mCarrierTracker.phase_radians(), mCarrierTracker.frequency_radians_per_symbol()};
        }

        state.refine_index = 1U;
        state.refine_accum = IQSample{};
        state.stage = PreambleTrainState::Stage::refine;
        return;
    }
    case PreambleTrainState::Phase::candidate:
    {
        state.candidate_residuals[state.candidate_index] = mLastProbeError;

        // Keep the strict-minimum-residual winner; ties keep the earlier entrant
        // (acquisition first), reproducing the ungated race. Snapshot the new best
        // now, before the next candidate's initialize_cursor_gain wipes it.
        if (mLastProbeError < state.best_residual)
        {
            state.best_residual = mLastProbeError;
            state.best_index = state.candidate_index;

            if (memoize_winner())
            {
                state.best_snapshot = PreambleWinnerSnapshot{mEqualizer.capture_nlms_state(), mCarrierTracker.phase_radians(), mCarrierTracker.frequency_radians_per_symbol()};
            }
        }

        ++state.candidate_index;

        if (state.candidate_index < state.candidate_count)
        {
            state.phase = PreambleTrainState::Phase::candidate;
            inner_trainer_begin(state.candidate_frequencies[state.candidate_index]);
            state.stage = PreambleTrainState::Stage::hypothesis;
        }
        else
        {
            select_winner();
        }

        return;
    }
    case PreambleTrainState::Phase::winner:
        // RLS/CIR literal winner re-run finished; the live equalizer/carrier now
        // hold the winner (and its mLastProbeError/selected frequency).
        complete_preamble_training();
        return;
    }
}

std::size_t BodyAdaptiveReceiver::advance_refine(std::size_t credits) noexcept
{
    // Accumulate the whole-preamble lag-1 differential one index/quantum,
    // starting at index 1, in a fixed summation order, so std::arg(refine_accum)
    // does not depend on how the work is sliced. On completion, append the refined candidate (if in the trust
    // window) and dispatch to the grid (fractional modes) or the race.
    auto& state = mTrainState;
    const auto samples_per_symbol = state.samples_per_symbol;
    std::size_t spent = 0U;

    for (;;)
    {
        if (state.refine_index >= state.tribits.size() || state.refine_index * samples_per_symbol + samples_per_symbol > state.samples.size())
        {
            const auto refined_frequency_radians = std::arg(state.refine_accum);
            const auto refinement_difference = std::fabs(refined_frequency_radians - state.acquisition_frequency_radians);

            if (refinement_difference > 0.004F && refinement_difference < 0.08F)
            {
                state.candidate_frequencies[state.candidate_count++] = refined_frequency_radians;
            }

            if (samples_per_symbol == 2U)
            {
                state.stage = PreambleTrainState::Stage::search_build;
            }
            else
            {
                dispatch_race();
            }

            return spent;
        }

        const auto index = state.refine_index;
        const auto current = state.samples[index * samples_per_symbol + samples_per_symbol - 1U] * std::conj(psk8_symbol(state.tribits[index]));
        const auto previous = state.samples[(index - 1U) * samples_per_symbol + samples_per_symbol - 1U] * std::conj(psk8_symbol(state.tribits[index - 1U]));
        state.refine_accum += current * std::conj(previous);
        ++state.refine_index;
        ++spent;

        if (spent >= credits)
        {
            return spent;
        }
    }
}

void BodyAdaptiveReceiver::grid_build_rows() noexcept
{
    // One-shot build of the two T-spaced polyphase problems for the channel-fit
    // grid. The frozen raw samples / offsets are promoted into mFitWorkspace so the
    // per-point solve can resume between grid points.
    auto& state = mTrainState;
    const auto received_samples = state.samples;
    const auto expected_tribits = state.tribits;
    const auto feedforward_taps = mEqualizer.feedforward_taps();
    const auto support_precursor = state.pretraining_support_precursor;
    auto& even = mFitWorkspace.even_problem;
    auto& odd = mFitWorkspace.odd_problem;

    const auto layout = make_cir_layout(feedforward_taps, support_precursor);
    const auto symbol_count = expected_tribits.size();
    const auto first_symbol = symbol_count - cir_preamble_seed_symbols;
    even.rows = 0U;
    even.taps = layout.even_taps;
    odd.rows = 0U;
    odd.taps = layout.odd_taps;
    const auto reference_symbol = [&](std::ptrdiff_t symbol) noexcept -> IQSample
    {
        const auto index = index_in(symbol, symbol_count);
        return index ? psk8_symbol(expected_tribits[*index]) : IQSample{};
    };

    const auto symbol_end = to_offset(symbol_count);

    for (std::size_t symbol = first_symbol; symbol < symbol_count; ++symbol)
    {
        const auto cursor_sample = symbol * 2U + 1U;
        const auto symbols_before_end = as_real(symbol_count - symbol);
        const auto position = to_offset(symbol);

        if (position - layout.even_first_lag < symbol_end && even.rows < cir_maximum_rows)
        {
            auto& row = even.regressors[even.rows];
            auto source = position - layout.even_first_lag;

            for (std::size_t index = 0U; index < layout.even_taps; ++index, --source)
            {
                row[index] = reference_symbol(source);
            }

            mFitWorkspace.even_raw[even.rows] = received_samples[cursor_sample];
            mFitWorkspace.even_offset[even.rows] = symbols_before_end;
            even.weights[even.rows] = 1.0F;
            ++even.rows;
        }

        if (position - layout.odd_first_lag < symbol_end && odd.rows < cir_maximum_rows)
        {
            auto& row = odd.regressors[odd.rows];
            auto source = position - layout.odd_first_lag;

            for (std::size_t index = 0U; index < layout.odd_taps; ++index, --source)
            {
                row[index] = reference_symbol(source);
            }

            mFitWorkspace.odd_raw[odd.rows] = received_samples[cursor_sample - 1U];
            mFitWorkspace.odd_offset[odd.rows] = symbols_before_end + 0.5F;
            odd.weights[odd.rows] = 1.0F;
            ++odd.rows;
        }
    }
}

std::size_t BodyAdaptiveReceiver::advance_search_grid(std::size_t credits) noexcept
{
    // The channel-fit grid, one grid point/quantum. grid_offset is the running
    // float accumulator advanced by preamble_grid_search_step with a fixed
    // termination bound, never a recomputed integer index, so the trial
    // frequencies and the strict minimum do not depend on how the work is
    // sliced. On completion, adopt the
    // grid minimum (if any distinct point solved) and dispatch the race.
    auto& state = mTrainState;
    auto& even = mFitWorkspace.even_problem;
    auto& odd = mFitWorkspace.odd_problem;
    const auto center_frequency_radians = state.acquisition_frequency_radians;
    const auto epsilon = 1.0e-3F * as_real(even.rows);
    std::array<IQSample, cir_polyphase_taps> solution{};
    std::array<IQSample, cir_polyphase_taps> zero_prior{};
    std::size_t spent = 0U;

    for (;;)
    {
        if (!(state.grid_offset <= preamble_grid_search_span + 0.5F * preamble_grid_search_step))
        {
            state.search_valid = std::isfinite(state.grid_best_residual);

            if (state.search_valid)
            {
                const auto searched_frequency_radians = state.grid_best_frequency;
                auto distinct = true;

                for (std::size_t index = 0U; index < state.candidate_count; ++index)
                {
                    if (std::fabs(searched_frequency_radians - state.candidate_frequencies[index]) <= 0.004F)
                    {
                        distinct = false;
                    }
                }

                if (distinct)
                {
                    state.candidate_frequencies[state.candidate_count++] = searched_frequency_radians;
                }
            }

            dispatch_race();
            return spent;
        }

        const auto frequency = center_frequency_radians + state.grid_offset;

        for (std::size_t row = 0U; row < even.rows; ++row)
        {
            even.observations[row] = mFitWorkspace.even_raw[row] * std::polar(1.0F, frequency * mFitWorkspace.even_offset[row]);
        }

        for (std::size_t row = 0U; row < odd.rows; ++row)
        {
            odd.observations[row] = mFitWorkspace.odd_raw[row] * std::polar(1.0F, frequency * mFitWorkspace.odd_offset[row]);
        }

        float even_residual{};
        float odd_residual{};

        if (cir_polyphase_solve(even, 0.0F, epsilon, zero_prior, mFitWorkspace.scratch, solution, even_residual).is_ok())
        {
            auto total = even_residual;

            if (cir_polyphase_solve(odd, 0.0F, epsilon, zero_prior, mFitWorkspace.scratch, solution, odd_residual).is_ok())
            {
                total += odd_residual;

                // A grid minimum at the +-span boundary is not a validated
                // interior optimum: the residual was still falling toward the
                // edge, the signature of a frequency alias overfitting a faded
                // preamble segment rather than a true carrier offset. Only
                // interior points are eligible as the estimate. A genuine large
                // offset still lands interior because acquisition centres the
                // +-30 Hz search on it, and clean channels skip the whole race.
                // The grid_offset accumulation and termination bound are the
                // same for every point, so the trial frequencies do not
                // change.
                const auto interior_point = std::fabs(state.grid_offset) < preamble_grid_search_span - 0.5F * preamble_grid_search_step;

                if (interior_point && total < state.grid_best_residual)
                {
                    state.grid_best_residual = total;
                    state.grid_best_frequency = frequency;
                }
            }
        }

        state.grid_offset += preamble_grid_search_step;
        spent += preamble_grid_point_weight;

        if (spent >= credits)
        {
            return spent;
        }
    }
}

void BodyAdaptiveReceiver::dispatch_race() noexcept
{
    auto& state = mTrainState;

    if (state.candidate_count == 1U)
    {
        // No distinct alternative hypothesis survived; the acquisition training
        // still held in mEqualizer/mCarrierTracker is the ungated result.
        complete_preamble_training();
        return;
    }

    state.candidate_index = 1U;
    state.phase = PreambleTrainState::Phase::candidate;
    inner_trainer_begin(state.candidate_frequencies[state.candidate_index]);
    state.stage = PreambleTrainState::Stage::hypothesis;
}

// -----------------------------------------------------------------------------
// select_winner  (keep the strict-minimum-residual frequency hypothesis)
// -----------------------------------------------------------------------------
// 50K view: End the frequency race by adopting the lowest-residual hypothesis.
// Detailed view: Restore the memoized NLMS winner state (or literally re-run it for
//   RLS/CIR), leaving mEqualizer/mCarrierTracker holding the winner's trained state.
//   The strict-minimum tie-break favors the first (acquisition) hypothesis.
// 5th-grade view: Settle on the dial position that sounded clearest.
// -----------------------------------------------------------------------------
void BodyAdaptiveReceiver::select_winner() noexcept
{
    auto& state = mTrainState;

    if (state.best_index + 1U != state.candidate_count)
    {
        // The winner was not the last hypothesis trained, so the live equalizer
        // belongs to a losing candidate and must be replaced by the winner's.
        // Restoring the memoized snapshot is bit-identical to retraining the winner
        // (each attempt is a pure function of the samples and the frequency, so the
        // cursor re-seed reproduces the same end state) at no training cost;
        // RLS/CIR builds cannot snapshot fully, so they re-run the winner.
        if (memoize_winner())
        {
            mEqualizer.restore_nlms_state(state.best_snapshot.equalizer);
            mCarrierTracker.restore(state.best_snapshot.carrier_phase, state.best_snapshot.carrier_frequency);
            mLastProbeError = state.best_residual;
            mSelectedTrainingFrequencyRadians = state.candidate_frequencies[state.best_index];
            complete_preamble_training();
        }
        else
        {
            state.phase = PreambleTrainState::Phase::winner;
            inner_trainer_begin(state.candidate_frequencies[state.best_index]);
            state.stage = PreambleTrainState::Stage::hypothesis;
        }
    }
    else
    {
        // The winner was trained last; the live equalizer already holds it.
        complete_preamble_training();
    }
}

void BodyAdaptiveReceiver::complete_preamble_training() noexcept
{
    // Every path that ends a preamble training successfully lands here with
    // the winner's trained state live in mEqualizer/mCarrierTracker: trained
    // last, restored from the memoized snapshot, or literally re-run.
    auto& state = mTrainState;

    state.result = Status::success();
    state.stage = PreambleTrainState::Stage::complete;
}

void BodyAdaptiveReceiver::inner_trainer_begin(float frequency_radians) noexcept
{
    auto& state = mTrainState;
    state.current_frequency = frequency_radians;
    mCarrierTracker.reset(state.acquisition.carrier_phase_radians, frequency_radians);
    state.channel_cross = IQSample{};
    state.reference_energy = 0.0F;
    state.prefix_index = 0U;
    // A coarse acquisition-frequency estimate can retain a small residual error;
    // the fixed acquisition prefix keeps that phase rotation bounded for a stable
    // flat-channel estimate while the carrier loop tracks the remainder.
    state.estimate_symbols = std::min(state.tribits.size(), body_preamble_acquisition_prefix_symbols);
    state.preamble_length = state.tribits.size();
    // The 75-bps short preamble carries one eighth of the supervised symbols of
    // its long-preamble counterpart, so it replays eight times to give the
    // feed-forward equalizer the same adaptation budget; the other modes replay
    // three times to converge the equalizer past the flat cursor seed.
    state.training_passes = state.acquisition.recognition.mode.data_rate == DataRate::bps75 && state.acquisition.recognition.mode.interleave == BodyInterleave::short_block ? 8U : 3U;
    state.residual_symbols = std::min<std::size_t>(state.tribits.size(), 96U);
    state.training_residual_energy = 0.0F;
    state.sub = PreambleTrainState::InnerSub::prefix;
    state.inner_active = true;
}

std::size_t BodyAdaptiveReceiver::inner_trainer_step(std::size_t quota, bool& finished, Status& status) noexcept
{
    auto& state = mTrainState;
    finished = false;
    status = Status::success();
    std::size_t spent = 0U;
    std::array<IQSample, ProbeDfe::maximum_feedforward_taps> window{};

    while (spent < quota && state.inner_active)
    {
        if (state.sub == PreambleTrainState::InnerSub::prefix)
        {
            if (state.prefix_index < state.estimate_symbols)
            {
                const auto index = state.prefix_index;
                const auto desired = psk8_symbol(state.tribits[index]);
                const auto received = state.samples[index * state.samples_per_symbol + state.samples_per_symbol - 1U] *
                                      std::polar(1.0F, -(state.acquisition.carrier_phase_radians + state.current_frequency * as_real(index)));
                state.channel_cross += received * std::conj(desired);
                state.reference_energy += std::norm(desired);
                ++state.prefix_index;
                ++spent;
            }
            else
            {
                const auto channel_status = mEqualizer.initialize_cursor_gain(state.channel_cross / state.reference_energy);

                if (!channel_status.is_ok())
                {
                    status = channel_status;
                    finished = true;
                    state.inner_active = false;
                    return spent;
                }

                state.pass = 0U;
                state.symbol = 0U;
                state.training_residual_energy = 0.0F;
                state.sub = PreambleTrainState::InnerSub::replay;
            }
        }
        else if (state.pass < state.training_passes)
        {
            if (state.symbol == 0U)
            {
                // Each replay rewinds the feedforward window to the preamble start,
                // so the feedback register and carrier are reset per pass. This
                // reset is bundled with symbol 0 as one indivisible quantum.
                mEqualizer.clear_decision_history();
                mCarrierTracker.reset(state.acquisition.carrier_phase_radians, state.current_frequency);
            }

            if (state.symbol < state.preamble_length)
            {
                const auto index = state.symbol;
                const auto desired = psk8_symbol(state.tribits[index]);
                make_equalizer_window(state.samples, index, state.samples_per_symbol, mCarrierTracker.phase_radians(), mCarrierTracker.frequency_radians_per_symbol(), mEqualizer, window);
                const auto equalized = mEqualizer.train(std::span<const IQSample>{window}.first(mEqualizer.feedforward_taps()), desired);
                std::ignore = mCarrierTracker.update(equalized * std::polar(1.0F, mCarrierTracker.phase_radians()), desired);

                if (state.pass + 1U == state.training_passes && index + state.residual_symbols >= state.preamble_length)
                {
                    state.training_residual_energy += std::norm(desired - equalized);
                }

                ++state.symbol;
                ++spent;
            }
            else
            {
                ++state.pass;
                state.symbol = 0U;
            }
        }
        else
        {
            // The trained residual over the preamble tail seeds the first frame's
            // channel-state weight; record the selected frequency and finish.
            mLastProbeError = std::max(state.training_residual_energy / as_real(state.residual_symbols), probe_noise_floor);
            mSelectedTrainingFrequencyRadians = state.current_frequency;
            mTrained = true;

#if M110_BODY_RECEIVER_CIR_ENGINE
            // The CIR engine seeds its channel state from every hypothesis's
            // training; the winner's, trained or re-run last, is the one kept.
            if (mDataEqualizer == BodyDataEqualizer::cir_mmse && state.samples_per_symbol == 2U)
            {
                seed_cir_state(state.samples, state.samples_per_symbol, state.tribits);
            }
#endif

            finished = true;
            state.inner_active = false;
        }
    }

    return spent;
}

Status BodyAdaptiveReceiver::equalize_block(IQSampleSpan received_symbols, const BodyBlockPlan& plan, MutableIQSampleSpan equalized_symbols) noexcept
{
    return equalize_block_impl(received_symbols, 1U, plan, equalized_symbols);
}

Status BodyAdaptiveReceiver::equalize_block_fractional(IQSampleSpan received_half_symbols, const BodyBlockPlan& plan, MutableIQSampleSpan equalized_symbols) noexcept
{
    return equalize_block_impl(received_half_symbols, 2U, plan, equalized_symbols);
}

Status BodyAdaptiveReceiver::equalize_block_supervised(IQSampleSpan received_symbols, const BodyBlockPlan& plan, BitSpan reference_tribits, MutableIQSampleSpan equalized_symbols) noexcept
{
    return equalize_block_supervised_impl(received_symbols, 1U, plan, reference_tribits, equalized_symbols);
}

Status BodyAdaptiveReceiver::equalize_block_supervised_fractional(IQSampleSpan received_half_symbols, const BodyBlockPlan& plan, BitSpan reference_tribits,
                                                                  MutableIQSampleSpan equalized_symbols) noexcept
{
    return equalize_block_supervised_impl(received_half_symbols, 2U, plan, reference_tribits, equalized_symbols);
}

Status BodyAdaptiveReceiver::equalize_block_supervised_impl(IQSampleSpan received_samples, std::size_t samples_per_symbol, const BodyBlockPlan& plan, BitSpan reference_tribits,
                                                            MutableIQSampleSpan equalized_symbols) noexcept
{
    if (!mConfigured || !mTrained || mSamplesPerSymbol != samples_per_symbol || received_samples.size() != plan.transmitted_symbols * samples_per_symbol ||
        reference_tribits.size() != plan.transmitted_symbols || equalized_symbols.size() < plan.transmitted_symbols)
    {
        return {StatusCode::invalid_argument, "adaptive receiver supervised buffers are invalid"};
    }

#if M110_BODY_RECEIVER_CIR_ENGINE
    // The CIR engine re-equalizes on its own supervised walk.
    if (mDataEqualizer == BodyDataEqualizer::cir_mmse && samples_per_symbol == 2U && plan.mode.data_rate != DataRate::bps75 && mCirState.seeded)
    {
        return equalize_block_supervised_cir(received_samples, plan, reference_tribits, equalized_symbols);
    }
#endif

    std::array<IQSample, ProbeDfe::maximum_feedforward_taps> window{};
    std::size_t received_index{};
    const auto train_reference_symbol = [&]() noexcept -> float
    {
        const auto desired = psk8_symbol(reference_tribits[received_index]);
        make_equalizer_window(received_samples, received_index, samples_per_symbol, mCarrierTracker.phase_radians(), mCarrierTracker.frequency_radians_per_symbol(), mEqualizer, window);
        equalized_symbols[received_index] = mEqualizer.train(std::span<const IQSample>{window}.first(mEqualizer.feedforward_taps()), desired);
        std::ignore = mCarrierTracker.update(equalized_symbols[received_index] * std::polar(1.0F, mCarrierTracker.phase_radians()), desired);
        const auto residual = std::norm(equalized_symbols[received_index] - desired);
        ++received_index;
        return residual;
    };

    if (plan.mode.data_rate == DataRate::bps75)
    {
        if (mEqualizer.feedback_taps() != 0U)
        {
            return {StatusCode::invalid_argument, "75-bps spread-symbol equalization requires decision feedback to be disabled"};
        }

        // Matches the unsupervised 75-bps path: fully trained symbols, no
        // channel-state weighting (the spread correlator supplies diversity).
        while (received_index < plan.transmitted_symbols)
        {
            train_reference_symbol();
        }

        return Status::success();
    }

    const auto frames = plan.data_channel_symbols / plan.unknown_symbols_per_probe;
    auto previous_probe_error = mLastProbeError;
    float block_residual_energy{};

    for (std::size_t frame = 0U; frame < frames; ++frame)
    {
        const auto frame_first_symbol = received_index;
        float frame_residual_energy{};

        for (std::size_t symbol = 0U; symbol < plan.unknown_symbols_per_probe + plan.known_symbols_per_probe; ++symbol)
        {
            frame_residual_energy += train_reference_symbol();
        }

        // Every supervised symbol contributes to the local channel-state
        // estimate, so the weight resolution improves over the probe-only
        // estimate of the unsupervised pass; the demapper contract (scale by
        // the reciprocal local residual power) is unchanged. The weight stays
        // uniform across the frame, without the unsupervised walk's per-symbol
        // interpolation: a caller that compares passes by re-encode
        // consistency needs a stable per-frame confidence, and interpolating
        // here measurably hurt the iterative pass.
        const auto frame_symbols = plan.frame_symbols();
        const auto frame_error = std::max(frame_residual_energy / as_real(frame_symbols), probe_noise_floor);
        const auto weight = 2.0F / (previous_probe_error + frame_error);
        previous_probe_error = frame_error;
        block_residual_energy += frame_residual_energy;

        for (std::size_t symbol = 0U; symbol < plan.unknown_symbols_per_probe; ++symbol)
        {
            equalized_symbols[frame_first_symbol + symbol] *= weight;
        }
    }

    mLastProbeError = previous_probe_error;
    mLastBlockProbeError = block_residual_energy / as_real(plan.transmitted_symbols);
    return received_index == plan.transmitted_symbols ? Status::success() : Status{StatusCode::internal_error, "adaptive receiver supervised count mismatch"};
}

Status BodyAdaptiveReceiver::equalize_block_impl(IQSampleSpan received_samples, std::size_t samples_per_symbol, const BodyBlockPlan& plan, MutableIQSampleSpan equalized_symbols) noexcept
{
    if (!mConfigured || !mTrained || mSamplesPerSymbol != samples_per_symbol || received_samples.size() != plan.transmitted_symbols * samples_per_symbol ||
        equalized_symbols.size() < plan.transmitted_symbols)
    {
        return {StatusCode::invalid_argument, "adaptive receiver body buffers are invalid"};
    }

#if M110_BODY_RECEIVER_CIR_ENGINE
    if (mDataEqualizer == BodyDataEqualizer::cir_mmse && samples_per_symbol == 2U && plan.mode.data_rate != DataRate::bps75 && mCirState.seeded)
    {
        return equalize_block_cir(received_samples, plan, equalized_symbols);
    }
#endif

    BodyFrameState frame_state;
    begin_block_frames(frame_state);

    if (plan.mode.data_rate == DataRate::bps75)
    {
        for (std::size_t dibit = 0U; dibit < plan.data_channel_symbols; ++dibit)
        {
            const auto status = equalize_75_group_impl(received_samples, samples_per_symbol, dibit * 32U, dibit + 1U == plan.data_channel_symbols, frame_state, equalized_symbols);

            if (!status.is_ok())
            {
                return status;
            }
        }

        return plan.data_channel_symbols * 32U == plan.transmitted_symbols ? Status::success() : Status{StatusCode::internal_error, "75-bps spread-symbol equalizer count mismatch"};
    }

    const auto frames = plan.data_channel_symbols / plan.unknown_symbols_per_probe;
    const auto frame_symbols = plan.frame_symbols();

    for (std::size_t frame = 0U; frame < frames; ++frame)
    {
        const auto status = equalize_frame_impl(received_samples, samples_per_symbol, frame * frame_symbols, plan, frame_state, equalized_symbols);

        if (!status.is_ok())
        {
            return status;
        }
    }

    end_block_frames(frame_state);
    return frames * frame_symbols == plan.transmitted_symbols ? Status::success() : Status{StatusCode::internal_error, "adaptive receiver body count mismatch"};
}

void BodyAdaptiveReceiver::begin_block_frames(BodyFrameState& state) noexcept
{
    // Seed the first frame's channel-state weight from the last measured
    // residual (preamble-tail training for block 0, the final probe of the
    // previous block afterwards) instead of the optimistic floor. The
    // baseline deliberately restarts from the same seed at every block.
    state = BodyFrameState{};
    state.previous_probe_error = mLastProbeError;
    state.baseline_probe_error = mLastProbeError;
}

void BodyAdaptiveReceiver::end_block_frames(const BodyFrameState& state) noexcept
{
    mLastProbeError = state.previous_probe_error;

    if (state.block_probe_symbols != 0U)
    {
        mLastBlockProbeError = state.block_probe_error_energy / as_real(state.block_probe_symbols);
    }
}

Status BodyAdaptiveReceiver::equalize_frame(IQSampleSpan received_symbols, std::size_t frame_first_symbol, const BodyBlockPlan& plan, BodyFrameState& state,
                                            MutableIQSampleSpan equalized_symbols) noexcept
{
    return equalize_frame_impl(received_symbols, 1U, frame_first_symbol, plan, state, equalized_symbols);
}

Status BodyAdaptiveReceiver::equalize_frame_fractional(IQSampleSpan received_half_symbols, std::size_t frame_first_symbol, const BodyBlockPlan& plan, BodyFrameState& state,
                                                       MutableIQSampleSpan equalized_symbols) noexcept
{
    return equalize_frame_impl(received_half_symbols, 2U, frame_first_symbol, plan, state, equalized_symbols);
}

Status BodyAdaptiveReceiver::equalize_75_group(IQSampleSpan received_symbols, std::size_t group_first_symbol, bool exceptional_set, BodyFrameState& state,
                                               MutableIQSampleSpan equalized_symbols) noexcept
{
    return equalize_75_group_impl(received_symbols, 1U, group_first_symbol, exceptional_set, state, equalized_symbols);
}

Status BodyAdaptiveReceiver::equalize_75_group_fractional(IQSampleSpan received_half_symbols, std::size_t group_first_symbol, bool exceptional_set, BodyFrameState& state,
                                                          MutableIQSampleSpan equalized_symbols) noexcept
{
    return equalize_75_group_impl(received_half_symbols, 2U, group_first_symbol, exceptional_set, state, equalized_symbols);
}

// -----------------------------------------------------------------------------
// equalize_frame_impl  (equalize one data frame + collapse retrain)
// -----------------------------------------------------------------------------
// 50K view: Equalize one frame's unknown data symbols, train on its trailing probe,
//   and re-train if the equalizer lost the channel.
// Detailed view: Run the ProbeDfe over the frame's unknown symbols (channel-state
//   weighting the outputs), train on the known probe, and if the mean probe residual
//   is a large fraction of constellation power AND far above the block baseline (or
//   above the absolute 0.9 outage floor), do a bounded COLLAPSE RETRAIN - the fix for
//   a frame a deep fade knocked the equalizer off.
// 5th-grade view: Clean each symbol; if the checkpoint phrase shows you've drifted
//   badly, quickly re-tune on it before moving on.
// -----------------------------------------------------------------------------
Status BodyAdaptiveReceiver::equalize_frame_impl(IQSampleSpan received_samples, std::size_t samples_per_symbol, std::size_t frame_first_symbol, const BodyBlockPlan& plan, BodyFrameState& state,
                                                 MutableIQSampleSpan equalized_symbols) noexcept
{
    const auto frame_symbols = plan.frame_symbols();

    if (!mConfigured || !mTrained || mSamplesPerSymbol != samples_per_symbol || plan.unknown_symbols_per_probe == 0U || plan.mode.data_rate == DataRate::bps75 ||
        state.frame_index >= plan.data_channel_symbols / plan.unknown_symbols_per_probe || (frame_first_symbol + frame_symbols) * samples_per_symbol > received_samples.size() ||
        equalized_symbols.size() < frame_first_symbol + frame_symbols)
    {
        return {StatusCode::invalid_argument, "adaptive receiver frame buffers are invalid"};
    }

    const auto frames = plan.data_channel_symbols / plan.unknown_symbols_per_probe;
    auto& randomizer = state.randomizer;
    std::size_t received_index = frame_first_symbol;
    std::array<IQSample, ProbeDfe::maximum_feedforward_taps> window{};
    const auto process_data_symbol = [&]() noexcept
    {
        make_equalizer_window(received_samples, received_index, samples_per_symbol, mCarrierTracker.phase_radians(), mCarrierTracker.frequency_radians_per_symbol(), mEqualizer, window);
        const auto output = mEqualizer.filter(std::span<const IQSample>{window}.first(mEqualizer.feedforward_taps()));
        const auto decision = psk8_symbol(psk8_hard_demapper(output));
        // Decision-directed adaptation between probes, gated on decision
        // reliability: the output must carry real energy and its residual must
        // stay inside the distance to the nearest wrong 8-PSK decision, so a
        // noise-only or likely-wrong decision can neither train the taps nor
        // steer the carrier loop. The rejected path still commits the decision
        // to the feedback register and advances the carrier phase open-loop.
        constexpr float squared_adjacent_decision_distance = 0.58578644F;
        const bool decision_reliable = std::norm(output) >= 0.25F && std::norm(decision - output) <= squared_adjacent_decision_distance;

        if (decision_reliable)
        {
            mEqualizer.adapt(std::span<const IQSample>{window}.first(mEqualizer.feedforward_taps()), decision);
            std::ignore = mCarrierTracker.update(output * std::polar(1.0F, mCarrierTracker.phase_radians()), decision);
        }
        else
        {
            mEqualizer.commit_decision(decision);
            mCarrierTracker.advance();
        }

        equalized_symbols[received_index++] = output;
    };

    for (std::size_t symbol = 0U; symbol < plan.unknown_symbols_per_probe; ++symbol)
    {
        process_data_symbol();
        randomizer.advance();
    }

    const std::uint8_t known_value = state.frame_index + 2U == frames ? plan.designators.d1 : state.frame_index + 1U == frames ? plan.designators.d2 : std::uint8_t{0U};
    const auto probe_first_symbol = received_index;
    const auto randomizer_at_probe = randomizer;
    const auto tracker_at_probe = mCarrierTracker;
    float probe_error_energy{};

    for (std::size_t symbol = 0U; symbol < plan.known_symbols_per_probe; ++symbol)
    {
        const auto probe_tribit = plan.known_symbols_per_probe == 20U && symbol >= 16U ? 0U : body_channel_symbol_tribit(known_value, symbol);
        const auto tribit = tribit_add(probe_tribit, randomizer.next_tribit());
        const auto desired = psk8_symbol(tribit);
        make_equalizer_window(received_samples, received_index, samples_per_symbol, mCarrierTracker.phase_radians(), mCarrierTracker.frequency_radians_per_symbol(), mEqualizer, window);
        equalized_symbols[received_index] = mEqualizer.train(std::span<const IQSample>{window}.first(mEqualizer.feedforward_taps()), desired);
        std::ignore = mCarrierTracker.update(equalized_symbols[received_index] * std::polar(1.0F, mCarrierTracker.phase_radians()), desired);
        probe_error_energy += std::norm(equalized_symbols[received_index] - desired);
        ++received_index;
    }

    // A probe residual near the constellation power AND far above the
    // running baseline means the fade broke the equalizer: its feedback
    // window holds fade-era garbage and one training pass has not
    // restored lock. Clear the decision history and replay this probe's
    // training so the next data frame starts from a reconverged filter
    // instead of the collapsed equilibrium.
    const auto mean_probe_error = probe_error_energy / as_real(plan.known_symbols_per_probe);

    // The absolute term (default body_receiver_collapse_absolute_threshold,
    // 0.9) declares the full outage the relative test misses at low SNR; see
    // set_collapse_absolute_threshold.
    if ((mean_probe_error > collapse_retrain_threshold && mean_probe_error > collapse_retrain_baseline_ratio * state.baseline_probe_error) || mean_probe_error > mCollapseAbsoluteThreshold)
    {
        mEqualizer.clear_decision_history();

        for (std::size_t replay = 0U; replay < collapse_retrain_passes; ++replay)
        {
            auto replay_randomizer = randomizer_at_probe;
            auto replay_tracker = tracker_at_probe;

            for (std::size_t symbol = 0U; symbol < plan.known_symbols_per_probe; ++symbol)
            {
                const auto probe_tribit = plan.known_symbols_per_probe == 20U && symbol >= 16U ? 0U : body_channel_symbol_tribit(known_value, symbol);
                const auto tribit = tribit_add(probe_tribit, replay_randomizer.next_tribit());
                const auto desired = psk8_symbol(tribit);
                make_equalizer_window(received_samples, probe_first_symbol + symbol, samples_per_symbol, replay_tracker.phase_radians(), replay_tracker.frequency_radians_per_symbol(), mEqualizer,
                                      window);
                const auto replayed = mEqualizer.train(std::span<const IQSample>{window}.first(mEqualizer.feedforward_taps()), desired);
                std::ignore = replay_tracker.update(replayed * std::polar(1.0F, replay_tracker.phase_radians()), desired);
            }
        }
    }

    // Max-log pairwise distance differences against a unit-modulus 8-PSK
    // constellation scale exactly linearly with the symbol magnitude, so
    // scaling one frame's data symbols by the reciprocal of the local
    // noise-plus-residual power turns the downstream soft demapper into a
    // channel-state-weighted demapper without an interface change. The
    // probes bracketing the frame supply that local power estimate; the
    // floor bounds the weight a short probe run can claim.
    const auto probe_error = std::max(mean_probe_error, probe_noise_floor);
    // Per-symbol channel-state weighting. The frame's unknown
    // symbols sit between the previous probe (frame start) and this frame's
    // probe (frame end); under fast (5 Hz) fading the reliability varies across
    // the frame, but a single frame-average weight demaps every symbol with the
    // same confidence. Interpolate the two bracketing probe residuals per symbol
    // so a symbol drifting into a fade is demapped with a smaller LLR magnitude
    // than one in a strong stretch, giving the Viterbi combine intra-frame CSI.
    // At the frame midpoint this reduces exactly to the previous 2/(prev+this).
    const auto frame_start_error = state.previous_probe_error;
    state.previous_probe_error = probe_error;

    // Slow EMA of the typical residual; a collapsed frame is excluded so
    // one deep fade cannot drag the baseline up to its own level.
    if (mean_probe_error < collapse_retrain_baseline_ratio * state.baseline_probe_error)
    {
        state.baseline_probe_error = 0.9F * state.baseline_probe_error + 0.1F * probe_error;
    }

    state.block_probe_error_energy += probe_error_energy;
    state.block_probe_symbols += plan.known_symbols_per_probe;

    const auto interpolation_span = as_real(plan.unknown_symbols_per_probe) + 1.0F;

    for (std::size_t symbol = 0U; symbol < plan.unknown_symbols_per_probe; ++symbol)
    {
        const auto alpha = (as_real(symbol) + 1.0F) / interpolation_span;
        const auto interpolated_error = (1.0F - alpha) * frame_start_error + alpha * probe_error;
        equalized_symbols[frame_first_symbol + symbol] *= 1.0F / interpolated_error;
    }

    ++state.frame_index;
    return Status::success();
}

Status BodyAdaptiveReceiver::equalize_75_group_impl(IQSampleSpan received_samples, std::size_t samples_per_symbol, std::size_t group_first_symbol, bool exceptional_set, BodyFrameState& state,
                                                    MutableIQSampleSpan equalized_symbols) noexcept
{
    constexpr std::size_t spread_symbols = 32U;

    if (!mConfigured || !mTrained || mSamplesPerSymbol != samples_per_symbol || (group_first_symbol + spread_symbols) * samples_per_symbol > received_samples.size() ||
        equalized_symbols.size() < group_first_symbol + spread_symbols)
    {
        return {StatusCode::invalid_argument, "adaptive receiver 75-bps group buffers are invalid"};
    }

    if (mEqualizer.feedback_taps() != 0U)
    {
        return {StatusCode::invalid_argument, "75-bps spread-symbol equalization requires decision feedback to be disabled"};
    }

    auto& randomizer = state.randomizer;
    std::array<IQSample, ProbeDfe::maximum_feedforward_taps> window{};
    std::array<IQSample, spread_symbols> candidate_symbols{};
    const auto tracker_at_group_start = mCarrierTracker;

    for (std::size_t spread = 0U; spread < spread_symbols; ++spread)
    {
        make_equalizer_window(received_samples, group_first_symbol + spread, samples_per_symbol, mCarrierTracker.phase_radians(), mCarrierTracker.frequency_radians_per_symbol(), mEqualizer, window);
        candidate_symbols[spread] = mEqualizer.filter(std::span<const IQSample>{window}.first(mEqualizer.feedforward_taps()));
        mCarrierTracker.advance();
    }

    std::uint8_t best_candidate{};
    float best_correlation_energy = -std::numeric_limits<float>::infinity();

    for (std::uint8_t candidate = 0U; candidate < 4U; ++candidate)
    {
        auto candidate_randomizer = randomizer;
        IQSample correlation{};

        for (std::size_t spread = 0U; spread < spread_symbols; ++spread)
        {
            const auto expected_tribit = tribit_add(body_75_spread_tribit(candidate, exceptional_set, spread), candidate_randomizer.next_tribit());
            correlation += candidate_symbols[spread] * std::conj(psk8_symbol(expected_tribit));
        }

        const auto correlation_energy = std::norm(correlation);

        if (correlation_energy > best_correlation_energy)
        {
            best_correlation_energy = correlation_energy;
            best_candidate = candidate;
        }
    }

    mCarrierTracker = tracker_at_group_start;
    auto received_index = group_first_symbol;

    for (std::size_t spread = 0U; spread < spread_symbols; ++spread)
    {
        const auto expected_tribit = tribit_add(body_75_spread_tribit(best_candidate, exceptional_set, spread), randomizer.next_tribit());
        const auto desired = psk8_symbol(expected_tribit);
        make_equalizer_window(received_samples, received_index, samples_per_symbol, mCarrierTracker.phase_radians(), mCarrierTracker.frequency_radians_per_symbol(), mEqualizer, window);
        equalized_symbols[received_index] = mEqualizer.train(std::span<const IQSample>{window}.first(mEqualizer.feedforward_taps()), desired);
        std::ignore = mCarrierTracker.update(equalized_symbols[received_index] * std::polar(1.0F, mCarrierTracker.phase_radians()), desired);
        ++received_index;
    }

    return Status::success();
}

#if M110_BODY_RECEIVER_CIR_ENGINE
// --- Channel-estimate (CIR-MMSE) data equalization -------------------------
//
// A T/2 CIR of length 2M is two interleaved T-spaced sub-channels, each
// observed at symbol rate through its own sample parity: the cursor-parity
// stream sees the even taps, the other stream the odd taps. Each polyphase
// least-squares problem is therefore small (<= 16 unknowns) and genuinely
// white-noise (the raised-cosine noise autocorrelation has Nyquist zeros at
// every nonzero even half-symbol lag), where the joint T/2 problem would be
// neither.

void BodyAdaptiveReceiver::seed_cir_state(IQSampleSpan received_samples, std::size_t samples_per_symbol, BitSpan expected_tribits) noexcept
{
    mCirState.seeded = false;

    if (samples_per_symbol != 2U || expected_tribits.size() < cir_preamble_seed_symbols + 16U)
    {
        return;
    }

    const auto symbol_count = expected_tribits.size();
    const auto first_symbol = symbol_count - cir_preamble_seed_symbols;
    const auto theta_end = mCarrierTracker.phase_radians();
    const auto trained_frequency = mCarrierTracker.frequency_radians_per_symbol();
    const auto reference_symbol = [&](std::ptrdiff_t symbol) noexcept -> IQSample
    {
        const auto index = index_in(symbol, symbol_count);
        return index ? psk8_symbol(expected_tribits[*index]) : IQSample{};
    };

    // Joint seed search over the support alignment and the carrier
    // frequency, both decided by the measured least-squares residual over
    // the known preamble tail. The alignment axis exists because the
    // acquisition's timing lock can place the direct path outside a fixed
    // short precursor, silently unmodeling its energy as noise; the
    // frequency axis exists because the training-frequency selection can be
    // fading-biased by tens of hertz, and a static ramp error rotates
    // successive channel anchors by radians, which tap interpolation cannot
    // survive (per-symbol tracking only degrades). The regressors are
    // frequency-independent, so each alignment builds its rows once and the
    // frequency grid only re-derotates the observations. The seed anchor
    // phase is fixed at the body start, so the adopted slope keeps the body
    // entry phase-continuous.
    auto& even = mFitWorkspace.even_problem;
    auto& odd = mFitWorkspace.odd_problem;
    std::array<IQSample, cir_maximum_rows> even_raw{};
    std::array<IQSample, cir_maximum_rows> odd_raw{};
    std::array<float, cir_maximum_rows> even_offset{};
    std::array<float, cir_maximum_rows> odd_offset{};
    std::array<IQSample, cir_polyphase_taps> even_taps{};
    std::array<IQSample, cir_polyphase_taps> odd_taps{};
    std::array<IQSample, cir_polyphase_taps> trial_even{};
    std::array<IQSample, cir_polyphase_taps> trial_odd{};
    std::array<IQSample, cir_polyphase_taps> zero_prior{};
    constexpr std::array<std::size_t, 4U> precursor_candidates{2U, 4U, 6U, 8U};
    constexpr float search_span = 0.10F;
    constexpr float search_step = 0.01F;
    auto best_frequency = trained_frequency;
    auto best_precursor = cir_precursor_taps;
    auto best_normalized = std::numeric_limits<float>::infinity();
    float best_residual{};
    float best_rows_total{};

    for (const auto precursor : precursor_candidates)
    {
        const auto layout = make_cir_layout(mEqualizer.feedforward_taps(), precursor);
        even.rows = 0U;
        even.taps = layout.even_taps;
        odd.rows = 0U;
        odd.taps = layout.odd_taps;

        const auto symbol_end = to_offset(symbol_count);

        for (std::size_t symbol = first_symbol; symbol < symbol_count; ++symbol)
        {
            const auto cursor_sample = symbol * 2U + 1U;
            const auto symbols_before_body = as_real(symbol_count - symbol);
            const auto position = to_offset(symbol);

            if (position - layout.even_first_lag < symbol_end && even.rows < cir_maximum_rows)
            {
                auto& row = even.regressors[even.rows];
                auto source = position - layout.even_first_lag;

                for (std::size_t index = 0U; index < layout.even_taps; ++index, --source)
                {
                    row[index] = reference_symbol(source);
                }

                even_raw[even.rows] = received_samples[cursor_sample];
                even_offset[even.rows] = symbols_before_body;
                even.weights[even.rows] = 1.0F;
                ++even.rows;
            }

            if (position - layout.odd_first_lag < symbol_end && odd.rows < cir_maximum_rows)
            {
                auto& row = odd.regressors[odd.rows];
                auto source = position - layout.odd_first_lag;

                for (std::size_t index = 0U; index < layout.odd_taps; ++index, --source)
                {
                    row[index] = reference_symbol(source);
                }

                odd_raw[odd.rows] = received_samples[cursor_sample - 1U];
                odd_offset[odd.rows] = symbols_before_body + 0.5F;
                odd.weights[odd.rows] = 1.0F;
                ++odd.rows;
            }
        }

        const auto epsilon = 1.0e-3F * as_real(even.rows);
        const auto rows_total = as_real(even.rows + odd.rows);

        for (auto offset = -search_span; offset <= search_span + 0.5F * search_step; offset += search_step)
        {
            const auto frequency = trained_frequency + offset;

            for (std::size_t row = 0U; row < even.rows; ++row)
            {
                even.observations[row] = even_raw[row] * std::polar(1.0F, -(theta_end - frequency * even_offset[row]));
            }

            for (std::size_t row = 0U; row < odd.rows; ++row)
            {
                odd.observations[row] = odd_raw[row] * std::polar(1.0F, -(theta_end - frequency * odd_offset[row]));
            }

            float even_residual{};
            float odd_residual{};

            if (!cir_polyphase_solve(even, 0.0F, epsilon, zero_prior, mFitWorkspace.scratch, trial_even, even_residual).is_ok() ||
                !cir_polyphase_solve(odd, 0.0F, epsilon, zero_prior, mFitWorkspace.scratch, trial_odd, odd_residual).is_ok())
            {
                continue;
            }

            const auto normalized = (even_residual + odd_residual) / rows_total;

            if (normalized < best_normalized)
            {
                best_normalized = normalized;
                best_residual = even_residual + odd_residual;
                best_rows_total = rows_total;
                best_frequency = frequency;
                best_precursor = precursor;
                even_taps = trial_even;
                odd_taps = trial_odd;
            }
        }
    }

    if (!std::isfinite(best_normalized))
    {
        return;
    }

    const auto layout = make_cir_layout(mEqualizer.feedforward_taps(), best_precursor);
    // The CIR walk continues from the searched carrier ramp and precursor.
    mCarrierTracker.reset(theta_end, best_frequency);
    mFitWorkspace.support_precursor = best_precursor;
    mCirState.channel.fill(IQSample{});

    for (std::size_t index = 0U; index < layout.even_taps; ++index)
    {
        mCirState.channel[cir_tap_index(layout, true, index)] = even_taps[index];
    }

    for (std::size_t index = 0U; index < layout.odd_taps; ++index)
    {
        mCirState.channel[cir_tap_index(layout, false, index)] = odd_taps[index];
    }

    const auto degrees = cir_residual_dof(best_rows_total, layout.support);
    const auto residual = best_residual / degrees;
    mCirState.noise_variance = residual < 1.0e-3F ? 1.0e-3F : residual > 2.0F ? 2.0F : residual;

    const auto mean_rows = 0.5F * best_rows_total;
    const auto sigma_effective = mCirState.noise_variance * cir_noise_inflation(layout.support, 2.0F * mean_rows);
    const CirMmseGeometry geometry{mEqualizer.feedforward_taps(), mEqualizer.cursor(), mEqualizer.feedback_taps(), layout.precursor, layout.postcursor};

    if (!cir_mmse_dfe_solve(std::span<const IQSample>{mCirState.channel.data(), layout.support}, geometry, sigma_effective, mFitWorkspace.scratch, mCirState.anchor_feedforward,
                            mCirState.anchor_feedback)
             .is_ok())
    {
        return;
    }

    for (std::size_t index = 0U; index < mEqualizer.feedback_taps(); ++index)
    {
        mCirState.committed_history[index] = psk8_symbol(expected_tribits[symbol_count - 1U - index]);
    }

    mCirState.anchor_residual = mLastProbeError;
    mCirState.anchor_symbol = -0.5F * as_real(cir_preamble_seed_symbols + 1U);
    mCirState.seeded = true;
}

Status BodyAdaptiveReceiver::equalize_block_cir(IQSampleSpan received_half_symbols, const BodyBlockPlan& plan, MutableIQSampleSpan equalized_symbols) noexcept
{
    const auto layout = make_cir_layout(mEqualizer.feedforward_taps(), mFitWorkspace.support_precursor);
    const auto feedforward_taps = mEqualizer.feedforward_taps();
    const auto feedback_taps = mEqualizer.feedback_taps();
    const auto unknown = plan.unknown_symbols_per_probe;
    const auto known = plan.known_symbols_per_probe;
    const auto frames = plan.data_channel_symbols / unknown;
    const CirMmseGeometry mmse_geometry{feedforward_taps, mEqualizer.cursor(), feedback_taps, layout.precursor, layout.postcursor};

    BodyDataRandomizer randomizer;
    std::array<IQSample, ProbeDfe::maximum_feedforward_taps> window{};
    std::array<IQSample, ProbeDfe::maximum_feedback_taps> history{};
    std::array<IQSample, 64U> data_decisions{};
    std::array<bool, 64U> data_reliable{};
    std::array<IQSample, 32U> probe_reference{};
    std::array<IQSample, ProbeDfe::maximum_feedforward_taps> frame_feedforward{};
    std::array<IQSample, ProbeDfe::maximum_feedback_taps> frame_feedback{};
    std::array<IQSample, cir_maximum_taps> candidate_channel{};
    std::array<IQSample, cir_polyphase_taps> even_taps{};
    std::array<IQSample, cir_polyphase_taps> odd_taps{};
    std::array<IQSample, cir_polyphase_taps> even_prior{};
    std::array<IQSample, cir_polyphase_taps> odd_prior{};

    if (unknown > data_decisions.size() || known > probe_reference.size())
    {
        return {StatusCode::invalid_argument, "CIR equalizer frame layout is unsupported"};
    }

    auto theta = mCarrierTracker.phase_radians();
    auto frequency = mCarrierTracker.frequency_radians_per_symbol();
    auto baseline = mLastProbeError;
    auto previous_weight_error = std::max(mCirState.anchor_residual, probe_noise_floor);
    float block_probe_error_energy{};
    std::size_t block_probe_symbols{};

    const auto commit_history = [&](IQSample decision) noexcept
    {
        for (std::size_t tap = feedback_taps; tap > 1U; --tap)
        {
            history[tap - 1U] = history[tap - 2U];
        }

        if (feedback_taps != 0U)
        {
            history[0] = decision;
        }
    };

    for (std::size_t frame = 0U; frame < frames; ++frame)
    {
        const auto frame_first = frame * (unknown + known);
        const auto probe_first = frame_first + unknown;
        const auto frame_phase = [&](std::size_t symbol) noexcept { return theta + frequency * as_real(symbol - frame_first); };

        // Pass 1: provisional decisions from the left anchor's taps plus a
        // leaky decision-directed delta - the anchor rows' deepest data
        // entries are this frame's final decisions, made exactly where the
        // held taps are stalest under fast fading, so the delta's
        // intra-frame agility directly cleans the rows the next anchor
        // solves on.
        history = mCirState.committed_history;
        std::array<IQSample, ProbeDfe::maximum_feedforward_taps> pass1_delta_ff{};
        std::array<IQSample, ProbeDfe::maximum_feedback_taps> pass1_delta_fb{};
        constexpr float pass1_delta_step = 0.25F;
        constexpr float pass1_delta_leak = 0.95F;

        for (std::size_t symbol = 0U; symbol < unknown; ++symbol)
        {
            make_equalizer_window(received_half_symbols, frame_first + symbol, 2U, frame_phase(frame_first + symbol), frequency, mEqualizer, window);
            IQSample output{};

            for (std::size_t tap = 0U; tap < feedforward_taps; ++tap)
            {
                output += (mCirState.anchor_feedforward[tap] + pass1_delta_ff[tap]) * window[tap];
            }

            for (std::size_t tap = 0U; tap < feedback_taps; ++tap)
            {
                output -= (mCirState.anchor_feedback[tap] + pass1_delta_fb[tap]) * history[tap];
            }

            const auto decision = psk8_symbol(psk8_hard_demapper(output));
            constexpr float squared_adjacent_decision_distance = 0.58578644F;
            data_decisions[symbol] = decision;
            data_reliable[symbol] = std::norm(output) >= 0.25F && std::norm(decision - output) <= squared_adjacent_decision_distance;

            if (data_reliable[symbol])
            {
                float energy{};

                for (std::size_t tap = 0U; tap < feedforward_taps; ++tap)
                {
                    energy += std::norm(window[tap]);
                }

                for (std::size_t tap = 0U; tap < feedback_taps; ++tap)
                {
                    energy += std::norm(history[tap]);
                }

                const auto normalized_step = pass1_delta_step / std::max(energy, 1.0e-6F);
                const auto error = decision - output;

                for (std::size_t tap = 0U; tap < feedforward_taps; ++tap)
                {
                    pass1_delta_ff[tap] = pass1_delta_leak * pass1_delta_ff[tap] + normalized_step * error * std::conj(window[tap]);
                }

                for (std::size_t tap = 0U; tap < feedback_taps; ++tap)
                {
                    pass1_delta_fb[tap] = pass1_delta_leak * pass1_delta_fb[tap] - normalized_step * error * std::conj(history[tap]);
                }
            }
            else
            {
                for (std::size_t tap = 0U; tap < feedforward_taps; ++tap)
                {
                    pass1_delta_ff[tap] *= pass1_delta_leak;
                }

                for (std::size_t tap = 0U; tap < feedback_taps; ++tap)
                {
                    pass1_delta_fb[tap] *= pass1_delta_leak;
                }
            }

            commit_history(decision);
            randomizer.advance();
        }

        const std::uint8_t known_value = frame + 2U == frames ? plan.designators.d1 : frame + 1U == frames ? plan.designators.d2 : std::uint8_t{0U};

        for (std::size_t symbol = 0U; symbol < known; ++symbol)
        {
            const auto probe_tribit = known == 20U && symbol >= 16U ? 0U : body_channel_symbol_tribit(known_value, symbol);
            probe_reference[symbol] = psk8_symbol(tribit_add(probe_tribit, randomizer.next_tribit()));
        }

        // The scrambled-domain symbol stream around the probe: probe symbols
        // are known, earlier symbols are this frame's pass-1 decisions.
        const auto frame_position = to_offset(frame_first);
        const auto probe_position = to_offset(probe_first);
        const auto stream_symbol = [&](std::ptrdiff_t symbol) noexcept -> IQSample
        {
            if (symbol >= probe_position)
            {
                return probe_reference[to_subscript(symbol - probe_position)];
            }

            return data_decisions[to_subscript(symbol - frame_position)];
        };
        const auto stream_reliable = [&](std::ptrdiff_t symbol) noexcept { return symbol >= probe_position || data_reliable[to_subscript(symbol - frame_position)]; };
        const auto build_rows_into = [&](CirPolyphaseProblem& even, CirPolyphaseProblem& odd, bool clean_rows_only, bool append) noexcept
        {
            if (!append)
            {
                even.rows = 0U;
                odd.rows = 0U;
            }

            even.taps = layout.even_taps;
            odd.taps = layout.odd_taps;

            for (std::size_t phase = 0U; phase < 2U; ++phase)
            {
                const auto is_even = phase == 0U;
                auto& problem = is_even ? even : odd;
                const auto first_lag = is_even ? layout.even_first_lag : layout.odd_first_lag;
                const auto tap_count = is_even ? layout.even_taps : layout.odd_taps;
                const auto deepest_lag = first_lag + to_offset(tap_count) - 1;

                for (std::size_t symbol = probe_first; symbol < probe_first + known; ++symbol)
                {
                    const auto position = to_offset(symbol);

                    // Rows whose future regressors reach past the probe end
                    // touch the next frame's unknown data: dropped.
                    if (position - first_lag >= to_offset(probe_first + known))
                    {
                        continue;
                    }

                    if (clean_rows_only && position - deepest_lag < probe_position)
                    {
                        continue;
                    }

                    if (problem.rows >= cir_maximum_rows)
                    {
                        continue;
                    }

                    auto& row = problem.regressors[problem.rows];
                    std::size_t unreliable{};
                    auto source = position - first_lag;

                    for (std::size_t index = 0U; index < tap_count; ++index, --source)
                    {
                        row[index] = stream_symbol(source);

                        if (!stream_reliable(source))
                        {
                            ++unreliable;
                        }
                    }

                    const auto cursor_sample = symbol * 2U + 1U;
                    problem.observations[problem.rows] = is_even ? received_half_symbols[cursor_sample] * std::polar(1.0F, -frame_phase(symbol))
                                                                 : received_half_symbols[cursor_sample - 1U] * std::polar(1.0F, -(frame_phase(symbol) - 0.5F * frequency));
                    auto weight = 1.0F;

                    for (std::size_t count = 0U; count < unreliable; ++count)
                    {
                        weight *= 0.5F;
                    }

                    problem.weights[problem.rows] = weight < 0.25F ? 0.25F : weight;
                    ++problem.rows;
                }
            }
        };

        for (std::size_t index = 0U; index < layout.even_taps; ++index)
        {
            even_prior[index] = mCirState.channel[cir_tap_index(layout, true, index)];
        }

        for (std::size_t index = 0U; index < layout.odd_taps; ++index)
        {
            odd_prior[index] = mCirState.channel[cir_tap_index(layout, false, index)];
        }

        float prior_power{};

        for (std::size_t index = 0U; index < layout.support; ++index)
        {
            prior_power += std::norm(mCirState.channel[index]);
        }

        // Solve the trailing anchor: LS -> candidate CIR -> MMSE taps. The
        // lambdas are shared with the collapse recovery, which re-solves on
        // clean rows with a strong prior.
        auto sigma_candidate = mCirState.noise_variance;
        bool degraded = false;
        bool reacquired = false;
        const auto solve_anchor = [&](bool clean_rows_only, float lambda_scale) noexcept
        {
            build_rows_into(mFitWorkspace.even_problem, mFitWorkspace.odd_problem, clean_rows_only, false);
            auto& even = mFitWorkspace.even_problem;
            auto& odd = mFitWorkspace.odd_problem;
            float weight_sum{};

            for (std::size_t row = 0U; row < even.rows; ++row)
            {
                weight_sum += even.weights[row];
            }

            for (std::size_t row = 0U; row < odd.rows; ++row)
            {
                weight_sum += odd.weights[row];
            }

            const auto mean_row_weight = 0.5F * weight_sum;
            const auto lambda = lambda_scale * cir_ridge_lambda(mCirState.noise_variance, layout.support, prior_power, mean_row_weight);
            const auto epsilon = 1.0e-3F * std::max(mean_row_weight, 1.0F);
            float even_residual{};
            float odd_residual{};

            if (!cir_polyphase_solve(even, lambda, epsilon, even_prior, mFitWorkspace.scratch, even_taps, even_residual).is_ok() ||
                !cir_polyphase_solve(odd, lambda, epsilon, odd_prior, mFitWorkspace.scratch, odd_taps, odd_residual).is_ok())
            {
                return false;
            }

            candidate_channel.fill(IQSample{});
            const auto retention = mean_row_weight / (mean_row_weight + lambda);
            const auto prune_threshold = 2.0F * mCirState.noise_variance / (mean_row_weight + lambda);
            float channel_power{};

            for (std::size_t index = 0U; index < layout.even_taps; ++index)
            {
                const auto value = std::norm(even_taps[index]) < prune_threshold ? IQSample{} : even_taps[index];
                candidate_channel[cir_tap_index(layout, true, index)] = value;
                channel_power += std::norm(value);
            }

            for (std::size_t index = 0U; index < layout.odd_taps; ++index)
            {
                const auto value = std::norm(odd_taps[index]) < prune_threshold ? IQSample{} : odd_taps[index];
                candidate_channel[cir_tap_index(layout, false, index)] = value;
                channel_power += std::norm(value);
            }

            if (channel_power < 0.05F)
            {
                return false;
            }

            const auto degrees = cir_residual_dof(weight_sum, layout.support, retention);
            const auto residual = (even_residual + odd_residual) / degrees;
            const auto clamped = residual < 1.0e-3F ? 1.0e-3F : residual > 2.0F ? 2.0F : residual;
            sigma_candidate = 0.5F * mCirState.noise_variance + 0.5F * clamped;
            const auto sigma_effective = sigma_candidate * cir_noise_inflation(layout.support, 2.0F * (mean_row_weight + lambda));

            if (!cir_mmse_dfe_solve(std::span<const IQSample>{candidate_channel.data(), layout.support}, mmse_geometry, sigma_effective, mFitWorkspace.scratch, frame_feedforward, frame_feedback)
                     .is_ok())
            {
                frame_feedforward = mCirState.anchor_feedforward;
                frame_feedback = mCirState.anchor_feedback;
            }

            return true;
        };

        // Pass 2 plus the probe emission, shared with the anchor selection.
        const auto anchor_symbol_b = as_real(probe_first) + 0.5F * as_real(known - 1U);
        const auto interpolation_span = anchor_symbol_b - mCirState.anchor_symbol;
        float probe_error_energy{};
        const auto run_outputs = [&]() noexcept
        {
            history = mCirState.committed_history;
            std::array<IQSample, ProbeDfe::maximum_feedforward_taps> taps{};
            std::array<IQSample, ProbeDfe::maximum_feedback_taps> taps_feedback{};
            // Leaky decision-directed delta on top of the interpolated
            // anchors: linear tap interpolation cannot follow a fade-notch
            // phase jump inside one frame (measured: fast-fading
            // realizations fail even noiseless while per-symbol adaptation
            // rides through), and per-symbol adaptation alone spirals. The
            // delta gives the intra-frame agility, its leak drains any bad
            // update within ~20 symbols, and the solved anchors reset the
            // base every probe, so neither failure mode survives.
            constexpr float delta_step = 0.25F;
            constexpr float delta_leak = 0.95F;
            std::array<IQSample, ProbeDfe::maximum_feedforward_taps> delta_feedforward{};
            std::array<IQSample, ProbeDfe::maximum_feedback_taps> delta_feedback{};
            const auto adapt_delta = [&](const std::array<IQSample, ProbeDfe::maximum_feedforward_taps>& window_view, IQSample error) noexcept
            {
                float energy{};

                for (std::size_t tap = 0U; tap < feedforward_taps; ++tap)
                {
                    energy += std::norm(window_view[tap]);
                }

                for (std::size_t tap = 0U; tap < feedback_taps; ++tap)
                {
                    energy += std::norm(history[tap]);
                }

                const auto normalized_step = delta_step / std::max(energy, 1.0e-6F);

                for (std::size_t tap = 0U; tap < feedforward_taps; ++tap)
                {
                    delta_feedforward[tap] = delta_leak * delta_feedforward[tap] + normalized_step * error * std::conj(window_view[tap]);
                }

                for (std::size_t tap = 0U; tap < feedback_taps; ++tap)
                {
                    delta_feedback[tap] = delta_leak * delta_feedback[tap] - normalized_step * error * std::conj(history[tap]);
                }
            };
            const auto filter_with_delta = [&](std::size_t symbol_index, std::span<const IQSample> base_ff, std::span<const IQSample> base_fb, IQSample& output) noexcept
            {
                make_equalizer_window(received_half_symbols, symbol_index, 2U, frame_phase(symbol_index), frequency, mEqualizer, window);
                output = IQSample{};

                for (std::size_t tap = 0U; tap < feedforward_taps; ++tap)
                {
                    output += (base_ff[tap] + delta_feedforward[tap]) * window[tap];
                }

                for (std::size_t tap = 0U; tap < feedback_taps; ++tap)
                {
                    output -= (base_fb[tap] + delta_feedback[tap]) * history[tap];
                }
            };

            for (std::size_t symbol = 0U; symbol < unknown; ++symbol)
            {
                const auto symbol_index = frame_first + symbol;
                const auto alpha_raw = (as_real(symbol_index) - mCirState.anchor_symbol) / interpolation_span;
                const auto alpha = alpha_raw < 0.0F ? 0.0F : alpha_raw > 1.0F ? 1.0F : alpha_raw;

                for (std::size_t tap = 0U; tap < feedforward_taps; ++tap)
                {
                    taps[tap] = (1.0F - alpha) * mCirState.anchor_feedforward[tap] + alpha * frame_feedforward[tap];
                }

                for (std::size_t tap = 0U; tap < feedback_taps; ++tap)
                {
                    taps_feedback[tap] = (1.0F - alpha) * mCirState.anchor_feedback[tap] + alpha * frame_feedback[tap];
                }

                IQSample output{};
                filter_with_delta(symbol_index, taps, taps_feedback, output);
                equalized_symbols[symbol_index] = output;
                const auto decision = psk8_symbol(psk8_hard_demapper(output));
                constexpr float squared_adjacent_decision_distance = 0.58578644F;

                if (std::norm(output) >= 0.25F && std::norm(decision - output) <= squared_adjacent_decision_distance)
                {
                    adapt_delta(window, decision - output);
                }
                else
                {
                    for (std::size_t tap = 0U; tap < feedforward_taps; ++tap)
                    {
                        delta_feedforward[tap] *= delta_leak;
                    }

                    for (std::size_t tap = 0U; tap < feedback_taps; ++tap)
                    {
                        delta_feedback[tap] *= delta_leak;
                    }
                }

                commit_history(decision);
            }

            probe_error_energy = 0.0F;

            for (std::size_t symbol = 0U; symbol < known; ++symbol)
            {
                const auto symbol_index = probe_first + symbol;
                IQSample output{};
                filter_with_delta(symbol_index, frame_feedforward, frame_feedback, output);
                equalized_symbols[symbol_index] = output;
                probe_error_energy += std::norm(output - probe_reference[symbol]);
                adapt_delta(window, probe_reference[symbol] - output);
                commit_history(probe_reference[symbol]);
            }
        };

        // Dual-candidate anchor, decided by measurement: the decision-row
        // anchor tracks best while pass-1 decisions hold, but once they rot
        // the least squares fits the wrong decisions self-consistently and
        // the estimate spirals into whole-block outages on the 5 ms/5 Hz
        // channel. The decision-free anchor (clean probe rows plus the
        // previous-channel prior) cannot spiral, only lag. The probe
        // residual over known symbols is an unbiased arbiter between them,
        // the same measured selection the training-frequency race uses.
        struct AnchorCandidate
        {
            std::array<IQSample, cir_maximum_taps> channel{};
            std::array<IQSample, ProbeDfe::maximum_feedforward_taps> feedforward{};
            std::array<IQSample, ProbeDfe::maximum_feedback_taps> feedback{};
            float sigma{};
            float error{};
            bool valid{};
        };
        AnchorCandidate decision_anchor{};
        AnchorCandidate clean_anchor{};

        for (auto* candidate : {&decision_anchor, &clean_anchor})
        {
            candidate->valid = solve_anchor(candidate == &clean_anchor, 1.0F);

            if (candidate->valid)
            {
                run_outputs();
                candidate->channel = candidate_channel;
                candidate->feedforward = frame_feedforward;
                candidate->feedback = frame_feedback;
                candidate->sigma = sigma_candidate;
                candidate->error = probe_error_energy / as_real(known);
            }
        }

        const auto* winner = &clean_anchor;

        if (decision_anchor.valid && (!clean_anchor.valid || decision_anchor.error <= clean_anchor.error))
        {
            winner = &decision_anchor;
        }

        if (winner->valid)
        {
            candidate_channel = winner->channel;
            frame_feedforward = winner->feedforward;
            frame_feedback = winner->feedback;
            sigma_candidate = winner->sigma;

            if (winner == &decision_anchor && clean_anchor.valid)
            {
                // The clean candidate ran last; restore the winner's outputs.
                run_outputs();
            }
        }
        else
        {
            degraded = true;
            frame_feedforward = mCirState.anchor_feedforward;
            frame_feedback = mCirState.anchor_feedback;
            run_outputs();
        }

        auto mean_probe_error = probe_error_energy / as_real(known);
        // Outage detection cannot rely on the relative baseline alone: at
        // low SNR the baseline is itself noise-dominated (~0.3-0.5), and a
        // full outage (~1.0, the constellation power) never clears twice the
        // baseline, so the death spiral runs undeclared. A probe residual
        // near the constellation power is lost regardless of the noise
        // level.
        constexpr float absolute_outage_error = 0.9F;
        const auto probe_collapse = [&](float error) noexcept { return (error > collapse_retrain_threshold && error > collapse_retrain_baseline_ratio * baseline) || error > absolute_outage_error; };

        if (!degraded && probe_collapse(mean_probe_error))
        {
            // Both anchor candidates failed the probes: the fade is real.
            degraded = true;
        }

        if (degraded)
        {
            // Outage re-acquisition: one probe's decision-free rows cannot
            // re-identify the channel (five clean rows per polyphase against
            // sixteen unknowns), so stack clean rows across consecutive
            // outage frames - the frequency ramp is frozen during outage, so
            // the observations share one derotation frame - and re-solve
            // without any prior once the stack is overdetermined. Adopting
            // the re-acquired anchor ends the outage exactly like a healthy
            // frame; a still-faded channel simply fails the power guard and
            // keeps stacking.
            build_rows_into(mCirState.recovery_even_problem, mCirState.recovery_odd_problem, true, true);

            if (mCirState.recovery_even_problem.rows >= layout.even_taps + 4U && mCirState.recovery_odd_problem.rows >= layout.odd_taps + 4U)
            {
                std::array<IQSample, cir_polyphase_taps> zero_prior{};
                float even_residual{};
                float odd_residual{};
                const auto epsilon = 1.0e-3F * as_real(mCirState.recovery_even_problem.rows);

                if (cir_polyphase_solve(mCirState.recovery_even_problem, 0.0F, epsilon, zero_prior, mFitWorkspace.scratch, even_taps, even_residual).is_ok() &&
                    cir_polyphase_solve(mCirState.recovery_odd_problem, 0.0F, epsilon, zero_prior, mFitWorkspace.scratch, odd_taps, odd_residual).is_ok())
                {
                    candidate_channel.fill(IQSample{});
                    float channel_power{};

                    for (std::size_t index = 0U; index < layout.even_taps; ++index)
                    {
                        candidate_channel[cir_tap_index(layout, true, index)] = even_taps[index];
                        channel_power += std::norm(even_taps[index]);
                    }

                    for (std::size_t index = 0U; index < layout.odd_taps; ++index)
                    {
                        candidate_channel[cir_tap_index(layout, false, index)] = odd_taps[index];
                        channel_power += std::norm(odd_taps[index]);
                    }

                    if (channel_power >= 0.05F)
                    {
                        const auto rows_total = as_real(mCirState.recovery_even_problem.rows + mCirState.recovery_odd_problem.rows);
                        const auto degrees = cir_residual_dof(rows_total, layout.support);
                        const auto residual = (even_residual + odd_residual) / degrees;
                        sigma_candidate = residual < 1.0e-3F ? 1.0e-3F : residual > 2.0F ? 2.0F : residual;
                        const auto sigma_effective = sigma_candidate * cir_noise_inflation(layout.support, rows_total);

                        if (cir_mmse_dfe_solve(std::span<const IQSample>{candidate_channel.data(), layout.support}, mmse_geometry, sigma_effective, mFitWorkspace.scratch, frame_feedforward,
                                               frame_feedback)
                                .is_ok())
                        {
                            degraded = false;
                            reacquired = true;
                            mCirState.recovery_even_problem.rows = 0U;
                            mCirState.recovery_odd_problem.rows = 0U;
                            // The outage left the stale left anchor useless,
                            // so this frame filters entirely on the
                            // re-acquired taps; the rerun also restores a
                            // valid probe residual for the CSI weight.
                            mCirState.anchor_feedforward = frame_feedforward;
                            mCirState.anchor_feedback = frame_feedback;
                            run_outputs();
                            mean_probe_error = probe_error_energy / as_real(known);
                        }
                    }
                }
            }
        }
        else
        {
            mCirState.recovery_even_problem.rows = 0U;
            mCirState.recovery_odd_problem.rows = 0U;
        }

        const auto probe_error = std::max(mean_probe_error, probe_noise_floor);
        const auto weight = 2.0F / (previous_weight_error + probe_error);
        previous_weight_error = probe_error;

        for (std::size_t symbol = 0U; symbol < unknown; ++symbol)
        {
            equalized_symbols[frame_first + symbol] *= weight;
        }

        if (mean_probe_error < collapse_retrain_baseline_ratio * baseline)
        {
            baseline = 0.9F * baseline + 0.1F * probe_error;
        }

        block_probe_error_energy += probe_error_energy;
        block_probe_symbols += known;
        const auto theta_end = theta + frequency * as_real(unknown + known);

        if (!degraded)
        {
            // Integral-only frequency correction from the anchor-to-anchor
            // rotation of the channel estimate, applied phase-continuously so
            // the next frame's windows stay on one linear derotation ramp.
            IQSample rotation{};

            for (std::size_t index = 0U; index < layout.support; ++index)
            {
                rotation += candidate_channel[index] * std::conj(mCirState.channel[index]);
            }

            // A re-acquired anchor has no valid rotation reference: the
            // frozen pre-outage channel is many coherence intervals old, so
            // its cross-rotation is meaningless and the frequency holds.
            if (!reacquired)
            {
                const auto rotation_angle = std::arg(rotation);
                auto frequency_step = 0.25F * rotation_angle / interpolation_span;
                frequency_step = frequency_step > 5.0e-3F ? 5.0e-3F : frequency_step < -5.0e-3F ? -5.0e-3F : frequency_step;
                const auto maximum = mCarrierTracker.maximum_frequency_radians_per_symbol();
                auto next_frequency = frequency + frequency_step;
                next_frequency = next_frequency > maximum ? maximum : next_frequency < -maximum ? -maximum : next_frequency;
                frequency = next_frequency;
            }

            mCirState.channel = candidate_channel;
            mCirState.anchor_feedforward = frame_feedforward;
            mCirState.anchor_feedback = frame_feedback;
            mCirState.noise_variance = sigma_candidate;
        }

        mCirState.anchor_residual = probe_error;
        mCirState.anchor_symbol = anchor_symbol_b;
        mCirState.committed_history = history;
        mCarrierTracker.reset(theta_end, frequency);
        theta = theta_end;
    }

    mCirState.anchor_symbol -= as_real(plan.transmitted_symbols);
    mLastProbeError = previous_weight_error;

    if (block_probe_symbols != 0U)
    {
        mLastBlockProbeError = block_probe_error_energy / as_real(block_probe_symbols);
    }

    return Status::success();
}

Status BodyAdaptiveReceiver::equalize_block_supervised_cir(IQSampleSpan received_half_symbols, const BodyBlockPlan& plan, BitSpan reference_tribits, MutableIQSampleSpan equalized_symbols) noexcept
{
    const auto layout = make_cir_layout(mEqualizer.feedforward_taps(), mFitWorkspace.support_precursor);
    const auto feedforward_taps = mEqualizer.feedforward_taps();
    const auto feedback_taps = mEqualizer.feedback_taps();
    const auto unknown = plan.unknown_symbols_per_probe;
    const auto known = plan.known_symbols_per_probe;
    const auto frames = plan.data_channel_symbols / unknown;
    const CirMmseGeometry mmse_geometry{feedforward_taps, mEqualizer.cursor(), feedback_taps, layout.precursor, layout.postcursor};

    std::array<IQSample, ProbeDfe::maximum_feedforward_taps> window{};
    std::array<IQSample, ProbeDfe::maximum_feedback_taps> history{};
    std::array<IQSample, ProbeDfe::maximum_feedforward_taps> frame_feedforward{};
    std::array<IQSample, ProbeDfe::maximum_feedback_taps> frame_feedback{};
    std::array<IQSample, cir_maximum_taps> candidate_channel{};
    std::array<IQSample, cir_polyphase_taps> even_taps{};
    std::array<IQSample, cir_polyphase_taps> odd_taps{};
    std::array<IQSample, cir_polyphase_taps> even_prior{};
    std::array<IQSample, cir_polyphase_taps> odd_prior{};

    auto theta = mCarrierTracker.phase_radians();
    auto frequency = mCarrierTracker.frequency_radians_per_symbol();
    auto previous_weight_error = std::max(mCirState.anchor_residual, probe_noise_floor);
    float block_residual_energy{};

    const auto reference_symbol = [&](std::ptrdiff_t symbol) noexcept -> IQSample
    {
        const auto index = index_in(symbol, plan.transmitted_symbols);
        return index ? psk8_symbol(reference_tribits[*index]) : IQSample{};
    };
    const auto filter_symbol = [&](std::size_t symbol_index, float phase, std::span<const IQSample> feedforward, std::span<const IQSample> feedback) noexcept
    {
        make_equalizer_window(received_half_symbols, symbol_index, 2U, phase, frequency, mEqualizer, window);
        IQSample value{};

        for (std::size_t tap = 0U; tap < feedforward_taps; ++tap)
        {
            value += feedforward[tap] * window[tap];
        }

        for (std::size_t tap = 0U; tap < feedback_taps; ++tap)
        {
            value -= feedback[tap] * history[tap];
        }

        return value;
    };
    const auto commit_history = [&](IQSample decision) noexcept
    {
        for (std::size_t tap = feedback_taps; tap > 1U; --tap)
        {
            history[tap - 1U] = history[tap - 2U];
        }

        if (feedback_taps != 0U)
        {
            history[0] = decision;
        }
    };

    for (std::size_t frame = 0U; frame < frames; ++frame)
    {
        const auto frame_first = frame * (unknown + known);
        const auto probe_first = frame_first + unknown;
        const auto frame_phase = [&](std::size_t symbol) noexcept { return theta + frequency * as_real(symbol - frame_first); };

        // Every design-matrix entry is exact under supervision, so no rows
        // are dropped for decision contamination and the block interior
        // keeps the rows whose regressors reach the following data frame.
        auto& even = mFitWorkspace.even_problem;
        auto& odd = mFitWorkspace.odd_problem;
        even.rows = 0U;
        even.taps = layout.even_taps;
        odd.rows = 0U;
        odd.taps = layout.odd_taps;

        for (std::size_t phase = 0U; phase < 2U; ++phase)
        {
            const auto is_even = phase == 0U;
            auto& problem = is_even ? even : odd;
            const auto first_lag = is_even ? layout.even_first_lag : layout.odd_first_lag;
            const auto tap_count = is_even ? layout.even_taps : layout.odd_taps;

            for (std::size_t symbol = probe_first; symbol < probe_first + known; ++symbol)
            {
                const auto position = to_offset(symbol);

                if (position - first_lag >= to_offset(plan.transmitted_symbols))
                {
                    continue;
                }

                auto& row = problem.regressors[problem.rows];
                auto source = position - first_lag;

                for (std::size_t index = 0U; index < tap_count; ++index, --source)
                {
                    row[index] = reference_symbol(source);
                }

                const auto cursor_sample = symbol * 2U + 1U;
                problem.observations[problem.rows] = is_even ? received_half_symbols[cursor_sample] * std::polar(1.0F, -frame_phase(symbol))
                                                             : received_half_symbols[cursor_sample - 1U] * std::polar(1.0F, -(frame_phase(symbol) - 0.5F * frequency));
                problem.weights[problem.rows] = 1.0F;
                ++problem.rows;
            }
        }

        for (std::size_t index = 0U; index < layout.even_taps; ++index)
        {
            even_prior[index] = mCirState.channel[cir_tap_index(layout, true, index)];
        }

        for (std::size_t index = 0U; index < layout.odd_taps; ++index)
        {
            odd_prior[index] = mCirState.channel[cir_tap_index(layout, false, index)];
        }

        float prior_power{};

        for (std::size_t index = 0U; index < layout.support; ++index)
        {
            prior_power += std::norm(mCirState.channel[index]);
        }

        float weight_sum{};

        for (std::size_t row = 0U; row < even.rows; ++row)
        {
            weight_sum += even.weights[row];
        }

        for (std::size_t row = 0U; row < odd.rows; ++row)
        {
            weight_sum += odd.weights[row];
        }

        const auto mean_row_weight = 0.5F * weight_sum;
        const auto lambda = cir_ridge_lambda(mCirState.noise_variance, layout.support, prior_power, mean_row_weight);
        const auto epsilon = 1.0e-3F * std::max(mean_row_weight, 1.0F);
        float even_residual{};
        float odd_residual{};
        bool degraded = false;

        if (cir_polyphase_solve(even, lambda, epsilon, even_prior, mFitWorkspace.scratch, even_taps, even_residual).is_ok() &&
            cir_polyphase_solve(odd, lambda, epsilon, odd_prior, mFitWorkspace.scratch, odd_taps, odd_residual).is_ok())
        {
            candidate_channel.fill(IQSample{});
            const auto retention = mean_row_weight / (mean_row_weight + lambda);
            const auto prune_threshold = 2.0F * mCirState.noise_variance / (mean_row_weight + lambda);
            float channel_power{};

            for (std::size_t index = 0U; index < layout.even_taps; ++index)
            {
                const auto value = std::norm(even_taps[index]) < prune_threshold ? IQSample{} : even_taps[index];
                candidate_channel[cir_tap_index(layout, true, index)] = value;
                channel_power += std::norm(value);
            }

            for (std::size_t index = 0U; index < layout.odd_taps; ++index)
            {
                const auto value = std::norm(odd_taps[index]) < prune_threshold ? IQSample{} : odd_taps[index];
                candidate_channel[cir_tap_index(layout, false, index)] = value;
                channel_power += std::norm(value);
            }

            if (channel_power < 0.05F)
            {
                degraded = true;
            }
            else
            {
                const auto degrees = cir_residual_dof(weight_sum, layout.support, retention);
                const auto residual = (even_residual + odd_residual) / degrees;
                const auto clamped = residual < 1.0e-3F ? 1.0e-3F : residual > 2.0F ? 2.0F : residual;
                mCirState.noise_variance = 0.5F * mCirState.noise_variance + 0.5F * clamped;
                const auto sigma_effective = mCirState.noise_variance * cir_noise_inflation(layout.support, 2.0F * (mean_row_weight + lambda));

                if (!cir_mmse_dfe_solve(std::span<const IQSample>{candidate_channel.data(), layout.support}, mmse_geometry, sigma_effective, mFitWorkspace.scratch, frame_feedforward, frame_feedback)
                         .is_ok())
                {
                    frame_feedforward = mCirState.anchor_feedforward;
                    frame_feedback = mCirState.anchor_feedback;
                }
            }
        }
        else
        {
            degraded = true;
        }

        if (degraded)
        {
            frame_feedforward = mCirState.anchor_feedforward;
            frame_feedback = mCirState.anchor_feedback;
        }

        const auto anchor_symbol_b = as_real(probe_first) + 0.5F * as_real(known - 1U);
        const auto interpolation_span = anchor_symbol_b - mCirState.anchor_symbol;
        history = mCirState.committed_history;
        float frame_residual_energy{};
        std::array<IQSample, ProbeDfe::maximum_feedforward_taps> taps{};
        std::array<IQSample, ProbeDfe::maximum_feedback_taps> taps_feedback{};

        for (std::size_t symbol = 0U; symbol < unknown + known; ++symbol)
        {
            const auto symbol_index = frame_first + symbol;
            const auto alpha_raw = (as_real(symbol_index) - mCirState.anchor_symbol) / interpolation_span;
            const auto alpha = alpha_raw < 0.0F ? 0.0F : alpha_raw > 1.0F ? 1.0F : alpha_raw;

            for (std::size_t tap = 0U; tap < feedforward_taps; ++tap)
            {
                taps[tap] = (1.0F - alpha) * mCirState.anchor_feedforward[tap] + alpha * frame_feedforward[tap];
            }

            for (std::size_t tap = 0U; tap < feedback_taps; ++tap)
            {
                taps_feedback[tap] = (1.0F - alpha) * mCirState.anchor_feedback[tap] + alpha * frame_feedback[tap];
            }

            const auto output = filter_symbol(symbol_index, frame_phase(symbol_index), taps, taps_feedback);
            equalized_symbols[symbol_index] = output;
            const auto reference = reference_symbol(to_offset(symbol_index));
            frame_residual_energy += std::norm(output - reference);
            commit_history(reference);
        }

        const auto frame_symbols = as_real(unknown + known);
        const auto frame_error = std::max(frame_residual_energy / frame_symbols, probe_noise_floor);
        const auto weight = 2.0F / (previous_weight_error + frame_error);
        previous_weight_error = frame_error;
        block_residual_energy += frame_residual_energy;

        for (std::size_t symbol = 0U; symbol < unknown; ++symbol)
        {
            equalized_symbols[frame_first + symbol] *= weight;
        }

        const auto theta_end = theta + frequency * as_real(unknown + known);

        if (!degraded)
        {
            IQSample rotation{};

            for (std::size_t index = 0U; index < layout.support; ++index)
            {
                rotation += candidate_channel[index] * std::conj(mCirState.channel[index]);
            }

            const auto rotation_angle = std::arg(rotation);
            auto frequency_step = 0.25F * rotation_angle / interpolation_span;
            frequency_step = frequency_step > 5.0e-3F ? 5.0e-3F : frequency_step < -5.0e-3F ? -5.0e-3F : frequency_step;
            const auto maximum = mCarrierTracker.maximum_frequency_radians_per_symbol();
            auto next_frequency = frequency + frequency_step;
            next_frequency = next_frequency > maximum ? maximum : next_frequency < -maximum ? -maximum : next_frequency;
            mCirState.channel = candidate_channel;
            mCirState.anchor_feedforward = frame_feedforward;
            mCirState.anchor_feedback = frame_feedback;
            frequency = next_frequency;
        }

        mCirState.anchor_residual = frame_error;
        mCirState.anchor_symbol = anchor_symbol_b;
        mCirState.committed_history = history;
        mCarrierTracker.reset(theta_end, frequency);
        theta = theta_end;
    }

    mCirState.anchor_symbol -= as_real(plan.transmitted_symbols);
    mLastProbeError = previous_weight_error;
    mLastBlockProbeError = block_residual_energy / as_real(plan.transmitted_symbols);
    return Status::success();
}

#endif // M110_BODY_RECEIVER_CIR_ENGINE

} // namespace modem::m110a
