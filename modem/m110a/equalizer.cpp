// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

/*
================================================================================
equalizer.cpp - the receiver's ADAPTIVE channel equalizers
================================================================================

WHAT THIS FILE IS
-----------------
The stage that undoes channel smearing (inter-symbol interference + fading) AFTER
acquisition has found the burst and BEFORE the demapper turns symbols into soft
bits. This file holds the ADAPTIVE family: filters that LEARN their coefficients
from the data. Its sibling cir_equalizer.cpp holds the MODEL-BASED family (measure
the channel, then SOLVE the optimal filter).

  INPUT  : the received T/2 sample window around each symbol; known probe symbols.
  OUTPUT : equalized symbol estimates z (fed to the demapper); trained taps.
  DEPENDS: types.hpp (IQSample), status.hpp. No heap - fixed std::array buffers.

WHAT LIVES HERE
---------------
  estimate_flat_channel()  one-tap ML channel gain from probes (seeds the DFE cursor).
  LmsEqualizer             textbook T-spaced NLMS transversal equalizer (<=32 taps).
  ProbeDfe                 the fielded probe-trained DECISION-FEEDBACK equalizer:
                           <=32 T/2 feed-forward taps at a cursor + <=16 feedback
                           taps on past DECISIONS, adapted by NLMS or RLS.

THE DFE EQUATION (shared with cir_mmse_dfe_solve)
-------------------------------------------------
  z = sum_t w[t]*window[t]  -  sum_j b[j]*decision[k-1-j]
  - feed-forward w acts on received T/2 samples: cleans precursor ISI + noise.
  - feedback b acts on PAST DECISIONS: cancels the postcursor echo they leave, with
    no noise enhancement (those symbols are already decided).
  Plain complex products; w is stored conjugated so sum(w*window) == u^H window.

ADAPTIVE vs MODEL-BASED (why both exist)
----------------------------------------
The adaptive path follows the channel: every probe symbol, and every data decision
that passes a reliability test, moves the taps a small step. The model-based path
(cir_equalizer.cpp) measures the channel from each probe and solves the filter.
Neither wins everywhere: on the standard's two-path channels the model-based path
does better at 2400 bps and the adaptive one at 600 bps and below (Part 6 of the
article series measures both). NLMS is the receiver's adaptation; RLS (converges
in ~2x the tap count) is kept as an option the receiver does not select.
Build note: the model-based path is host-only (gated by
M110_BODY_RECEIVER_CIR_ENGINE, default ON native / OFF embedded); this adaptive
file compiles in every target, so on embedded it is the only body equalizer.

WORKED EXAMPLE / DEEP DIVE
--------------------------
    modem/m110a/receiver-core-equalizer-explainer.md
================================================================================
*/

#include "modem/m110a/equalizer.hpp"

#include "modem/common/convert.hpp"

#include <algorithm>
#include <complex>
#include <cstddef>
#include <expected>
#include <functional>
#include <numeric>

#include "modem/common/status.hpp"
namespace modem::m110a
{

using ::modem::common::as_real;
using ::modem::common::IQSample;
using ::modem::common::IQSampleSpan;
using ::modem::common::Result;
using ::modem::common::Status;
using ::modem::common::StatusCode;

// -----------------------------------------------------------------------------
// estimate_flat_channel
// -----------------------------------------------------------------------------
// 50K view: Least-squares fit of ONE complex gain g mapping the known probes onto
//   the received samples, plus the residual MSE. Seeds the DFE cursor.
// Detailed view: g = (sum r*conj(k)) / (sum |k|^2); MSE = mean |r - g*k|^2. The ML
//   flat-channel estimate; high MSE means the channel is dispersive (use the DFE).
// 5th-grade view: One number for how much the channel shrank and twisted the
//   signal, by comparing what you heard to what you know was sent.
// -----------------------------------------------------------------------------
std::expected<ChannelEstimate, Status> estimate_flat_channel(IQSampleSpan received_probes, IQSampleSpan known_probes) noexcept
{
    if (received_probes.empty() || received_probes.size() != known_probes.size())
    {
        return std::unexpected(Status{StatusCode::invalid_argument, "channel-estimate probe buffers are invalid"});
    }

    // The conjugated cross-correlation and the reference energy are inner products:
    // strict left-to-right folds (std::inner_product), deterministic, and index-free.
    // Not std::reduce, which may regroup the sum; float addition isn't associative, so
    // that would make the result unspecified and not reproducible build-to-build.
    const IQSample cross = std::inner_product(received_probes.begin(), received_probes.end(), known_probes.begin(), IQSample{}, std::plus<>{}, [](IQSample r, IQSample k) { return r * std::conj(k); });

    const float reference_energy = std::inner_product(known_probes.begin(), known_probes.end(), known_probes.begin(), 0.0F, std::plus<>{}, [](IQSample k, IQSample) { return std::norm(k); });

    if (reference_energy <= 0.0F)
    {
        return std::unexpected(Status{StatusCode::invalid_argument, "channel-estimate reference has no energy"});
    }

    const auto gain = cross / reference_energy;

    const float error_energy =
        std::inner_product(received_probes.begin(), received_probes.end(), known_probes.begin(), 0.0F, std::plus<>{}, [gain](IQSample r, IQSample k) { return std::norm(r - gain * k); });

    return ChannelEstimate{gain, error_energy / as_real(known_probes.size())};
}

Status LmsEqualizer::configure(std::size_t taps, float step_size, float leakage) noexcept
{
    if (taps == 0U || taps > maximum_taps || step_size <= 0.0F || leakage < 0.0F || leakage >= 1.0F)
    {
        return {StatusCode::invalid_argument, "LMS equalizer configuration is invalid"};
    }

    mTaps = taps;
    mStepSize = step_size;
    mLeakage = leakage;
    reset();
    return Status::success();
}

void LmsEqualizer::reset() noexcept
{
    mCoefficients.fill(IQSample{});
    mHistory.fill(IQSample{});
    mCoefficients[0] = IQSample{1.0F, 0.0F};
}

IQSample LmsEqualizer::filter(IQSample input) noexcept
{
    return update(input, {}, false);
}

IQSample LmsEqualizer::train(IQSample input, IQSample desired) noexcept
{
    return update(input, desired, true);
}

IQSample LmsEqualizer::decision_directed(IQSample input, IQSample decision) noexcept
{
    return update(input, decision, true);
}

IQSample LmsEqualizer::update(IQSample input, IQSample desired, bool adapt) noexcept
{
    for (std::size_t index = mTaps; index > 1U; --index)
    {
        mHistory[index - 1U] = mHistory[index - 2U];
    }

    mHistory[0] = input;
    IQSample output{};

    for (std::size_t index = 0U; index < mTaps; ++index)
    {
        output += mCoefficients[index] * mHistory[index];
    }

    if (adapt)
    {
        const auto error = desired - output;
        const auto retention = 1.0F - mLeakage;
        float history_energy{};

        for (std::size_t index = 0U; index < mTaps; ++index)
        {
            history_energy += std::norm(mHistory[index]);
        }

        constexpr float minimum_history_energy = 1.0e-6F;
        const auto normalized_step = mStepSize / std::max(history_energy, minimum_history_energy);

        for (std::size_t index = 0U; index < mTaps; ++index)
        {
            mCoefficients[index] = retention * mCoefficients[index] + normalized_step * error * std::conj(mHistory[index]);
        }
    }

    return output;
}

Status ProbeDfe::configure(std::size_t feedforward_taps, std::size_t feedback_taps, std::size_t samples_per_symbol, std::size_t cursor, float step_size, float leakage, DfeAdaptation adaptation,
                           float rls_forgetting) noexcept
{
    if (feedforward_taps == 0U || feedforward_taps > maximum_feedforward_taps || feedback_taps > maximum_feedback_taps || samples_per_symbol == 0U || samples_per_symbol > 2U ||
        cursor >= feedforward_taps || step_size <= 0.0F || leakage < 0.0F || leakage >= 1.0F || rls_forgetting <= 0.5F || rls_forgetting > 1.0F)
    {
        return {StatusCode::invalid_argument, "probe DFE configuration is invalid"};
    }

    mFeedforwardTaps = feedforward_taps;
    mFeedbackTaps = feedback_taps;
    mSamplesPerSymbol = samples_per_symbol;
    mCursor = cursor;
    mStepSize = step_size;
    mLeakage = leakage;
    mAdaptation = adaptation;
    mRlsForgetting = rls_forgetting;
    reset();
    return Status::success();
}

void ProbeDfe::reset_rls() noexcept
{
    // P starts as a large multiple of the identity: an uninformative prior
    // whose first few updates converge in a couple of tap counts.
    constexpr float initial_inverse_correlation = 100.0F;
    const auto joint = mFeedforwardTaps + mFeedbackTaps;
    std::fill_n(mRlsInverseCorrelation.begin(), joint * joint, IQSample{});

    for (std::size_t index = 0U; index < joint; ++index)
    {
        mRlsInverseCorrelation[index * joint + index] = IQSample{initial_inverse_correlation, 0.0F};
    }
}

void ProbeDfe::reset() noexcept
{
    mFeedforwardCoefficients.fill(IQSample{});
    mFeedbackCoefficients.fill(IQSample{});
    mDecisionHistory.fill(IQSample{});
    mFeedforwardCoefficients[mCursor] = IQSample{1.0F, 0.0F};
    reset_rls();
}

// -----------------------------------------------------------------------------
// ProbeDfe::initialize_cursor_gain
// -----------------------------------------------------------------------------
// 50K view: Seed the cursor feed-forward tap from a flat channel estimate, FULLY
//   re-seeding all state.
// Detailed view: w[cursor] = conj(g)/|g|^2 (matched-filter inverse of the flat
//   gain); FF, FB, and decision history all cleared. The full re-seed matters
//   because preamble training may run more than once per burst (frequency-
//   hypothesis selection) and each attempt must be a pure function of the samples
//   + frequency, not of a prior attempt's taps.
// 5th-grade view: Point your ears roughly right before fine-tuning, wiping the
//   slate completely each retry.
// -----------------------------------------------------------------------------
Status ProbeDfe::initialize_cursor_gain(IQSample channel_gain) noexcept
{
    const auto energy = std::norm(channel_gain);

    if (energy <= 1.0e-8F)
    {
        return {StatusCode::invalid_argument, "probe DFE channel gain is zero"};
    }

    // Full re-seed, feedback state included: preamble training may run more
    // than once per burst (the frequency-hypothesis selection), and each
    // attempt must be a pure function of the samples and the frequency, not
    // of a previous attempt's trained feedback taps.
    mFeedforwardCoefficients.fill(IQSample{});
    mFeedbackCoefficients.fill(IQSample{});
    mDecisionHistory.fill(IQSample{});
    mFeedforwardCoefficients[mCursor] = std::conj(channel_gain) / energy;
    reset_rls();
    return Status::success();
}

// -----------------------------------------------------------------------------
// ProbeDfe::output  (the DFE equation)
// -----------------------------------------------------------------------------
// 50K view: Compute one equalized symbol estimate z.
// Detailed view: z = sum_t w[t]*window[t] - sum_j b[j]*decision_history[j]. Plain
//   complex products (w is stored conjugated). Feed-forward cleans precursor ISI +
//   noise on the T/2 window; feedback cancels the postcursor echo of past
//   DECISIONS with no noise enhancement. Identical convention to cir_mmse_dfe_solve.
// 5th-grade view: Add the ear-filter's view of the current word, then subtract the
//   remembered echoes of the words you already know.
// -----------------------------------------------------------------------------
IQSample ProbeDfe::output(IQSampleSpan feedforward_window) const noexcept
{
    IQSample value{};

    for (std::size_t tap = 0U; tap < mFeedforwardTaps; ++tap)
    {
        value += mFeedforwardCoefficients[tap] * feedforward_window[tap];
    }

    for (std::size_t tap = 0U; tap < mFeedbackTaps; ++tap)
    {
        value -= mFeedbackCoefficients[tap] * mDecisionHistory[tap];
    }

    return value;
}

IQSample ProbeDfe::filter(IQSampleSpan feedforward_window) const noexcept
{
    if (feedforward_window.size() < mFeedforwardTaps)
    {
        return {};
    }

    return output(feedforward_window);
}

IQSample ProbeDfe::train(IQSampleSpan feedforward_window, IQSample desired) noexcept
{
    if (feedforward_window.size() < mFeedforwardTaps)
    {
        return {};
    }

    return mAdaptation == DfeAdaptation::rls ? train_rls(feedforward_window, desired) : train_nlms(feedforward_window, desired);
}

void ProbeDfe::adapt(IQSampleSpan feedforward_window, IQSample desired) noexcept
{
    if (feedforward_window.size() < mFeedforwardTaps)
    {
        return;
    }

    if (mAdaptation == DfeAdaptation::rls)
    {
        train_rls(feedforward_window, desired);
    }
    else
    {
        train_nlms(feedforward_window, desired);
    }
}

// -----------------------------------------------------------------------------
// ProbeDfe::train_nlms  (normalized LMS adaptation)
// -----------------------------------------------------------------------------
// 50K view: One gradient step of both filters from a training symbol.
// Detailed view: error = desired - output; normalize the step by the JOINT energy
//   of the FF window + FB history; w <- (1-leak)w + step*e*conj(window);
//   b <- (1-leak)b - step*e*conj(history) (negative because feedback is
//   subtracted). Energy-normalization makes convergence level-independent; leakage
//   bounds runaway on noise-only input.
// 5th-grade view: One gentle correction to both the ear-filter and the echo-memory,
//   sized to how loud things are right now.
// -----------------------------------------------------------------------------
IQSample ProbeDfe::train_nlms(IQSampleSpan feedforward_window, IQSample desired) noexcept
{
    const auto value = output(feedforward_window);
    const auto error = desired - value;
    float energy{};

    for (std::size_t tap = 0U; tap < mFeedforwardTaps; ++tap)
    {
        energy += std::norm(feedforward_window[tap]);
    }

    for (std::size_t tap = 0U; tap < mFeedbackTaps; ++tap)
    {
        energy += std::norm(mDecisionHistory[tap]);
    }

    const auto normalized_step = mStepSize / std::max(energy, 1.0e-6F);
    const auto retention = 1.0F - mLeakage;

    for (std::size_t tap = 0U; tap < mFeedforwardTaps; ++tap)
    {
        mFeedforwardCoefficients[tap] = retention * mFeedforwardCoefficients[tap] + normalized_step * error * std::conj(feedforward_window[tap]);
    }

    for (std::size_t tap = 0U; tap < mFeedbackTaps; ++tap)
    {
        mFeedbackCoefficients[tap] = retention * mFeedbackCoefficients[tap] - normalized_step * error * std::conj(mDecisionHistory[tap]);
    }

    commit_decision(desired);
    return value;
}

// -----------------------------------------------------------------------------
// ProbeDfe::train_rls  (exponentially weighted recursive least squares)
// -----------------------------------------------------------------------------
// 50K view: Converge in ~2x the tap count (not many tens of symbols), which fast
//   fading needs - at O(taps^2)/symbol and an (ff+fb)^2 complex P matrix.
// Detailed view: joint input u = [window; -history]; maintain P = inverse weighted
//   correlation. g = P*conj(u); denom = lambda + u^T g; k = g/denom; w += k*e;
//   P = (P - k*g^H)/lambda. A trace cap stabilizes P (the 1/lambda growth winds it
//   up along un-excited directions and overflows float within minutes); a
//   non-finite trace restarts from the uninformative prior P = 100*I.
// 5th-grade view: A smarter learner that keeps a running summary of every past word
//   so it locks in after a few - with a safety valve so bookkeeping never blows up.
// -----------------------------------------------------------------------------
IQSample ProbeDfe::train_rls(IQSampleSpan feedforward_window, IQSample desired) noexcept
{
    // Exponentially weighted RLS over the joint input u = [window; -history]
    // with the estimation model y = sum_j w_j u_j and cost
    // sum_i lambda^(n-i) |d_i - y_i|^2. With P the inverse of the weighted
    // correlation R = sum lambda^(n-i) u* u^T:
    //   g = P u*, denom = lambda + u^T g, k = g / denom,
    //   w += k (d - y), P = (P - k g^H) / lambda.
    const auto joint = mFeedforwardTaps + mFeedbackTaps;

    for (std::size_t tap = 0U; tap < mFeedforwardTaps; ++tap)
    {
        mRlsInput[tap] = feedforward_window[tap];
    }

    for (std::size_t tap = 0U; tap < mFeedbackTaps; ++tap)
    {
        mRlsInput[mFeedforwardTaps + tap] = -mDecisionHistory[tap];
    }

    const auto value = output(feedforward_window);
    const auto error = desired - value;

    for (std::size_t row = 0U; row < joint; ++row)
    {
        IQSample accumulator{};
        const auto* matrix_row = &mRlsInverseCorrelation[row * joint];

        for (std::size_t column = 0U; column < joint; ++column)
        {
            accumulator += matrix_row[column] * std::conj(mRlsInput[column]);
        }

        mRlsGain[row] = accumulator;
    }

    IQSample projection{};

    for (std::size_t index = 0U; index < joint; ++index)
    {
        projection += mRlsInput[index] * mRlsGain[index];
    }

    // u^T P u* is real and non-negative for Hermitian positive-definite P;
    // the real part guards against numerical asymmetry.
    const auto denominator = mRlsForgetting + std::max(projection.real(), 0.0F);
    const auto inverse_denominator = 1.0F / denominator;
    const auto inverse_forgetting = 1.0F / mRlsForgetting;
    float trace{};

    for (std::size_t row = 0U; row < joint; ++row)
    {
        const auto gain = mRlsGain[row] * inverse_denominator;
        auto* matrix_row = &mRlsInverseCorrelation[row * joint];

        for (std::size_t column = 0U; column < joint; ++column)
        {
            matrix_row[column] = (matrix_row[column] - gain * std::conj(mRlsGain[column])) * inverse_forgetting;
        }

        trace += matrix_row[row].real();
        const auto update = gain * error;

        if (row < mFeedforwardTaps)
        {
            mFeedforwardCoefficients[row] += update;
        }
        else
        {
            // The joint input already carries -history, so the joint weight
            // IS the subtracted feedback coefficient: no sign flip.
            mFeedbackCoefficients[row - mFeedforwardTaps] += update;
        }
    }

    // The 1/lambda growth winds P up without bound along directions the
    // input never excites, overflowing float within minutes; capping the
    // trace bounds the adaptation gain without disturbing the excited
    // directions' relative geometry. A non-finite trace means the recursion
    // already broke - restart it from the uninformative prior.
    const auto trace_cap = 200.0F * as_real(joint);

    if (!(trace > 0.0F) || trace > 1.0e12F)
    {
        reset_rls();
    }
    else if (trace > trace_cap)
    {
        const auto scale = trace_cap / trace;

        for (std::size_t index = 0U; index < joint * joint; ++index)
        {
            mRlsInverseCorrelation[index] *= scale;
        }
    }

    commit_decision(desired);
    return value;
}

void ProbeDfe::clear_decision_history() noexcept
{
    mDecisionHistory.fill(IQSample{});
}

ProbeDfe::NlmsState ProbeDfe::capture_nlms_state() const noexcept
{
    return NlmsState{mFeedforwardCoefficients, mFeedbackCoefficients, mDecisionHistory};
}

void ProbeDfe::restore_nlms_state(const NlmsState& state) noexcept
{
    mFeedforwardCoefficients = state.feedforward;
    mFeedbackCoefficients = state.feedback;
    mDecisionHistory = state.decision_history;
}

void ProbeDfe::commit_decision(IQSample decision) noexcept
{
    if (mFeedbackTaps == 0U)
    {
        return;
    }

    for (std::size_t tap = mFeedbackTaps; tap > 1U; --tap)
    {
        mDecisionHistory[tap - 1U] = mDecisionHistory[tap - 2U];
    }

    mDecisionHistory[0] = decision;
}

} // namespace modem::m110a
