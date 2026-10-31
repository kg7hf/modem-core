// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "modem/common/types.hpp"

#include <array>
#include <cstddef>
#include <expected>

// =============================================================================
// equalizer.hpp - adaptive channel equalizers (LMS + probe-trained DFE)
// =============================================================================
// The ADAPTIVE equalizer family. estimate_flat_channel() gives a one-tap channel
// gain (seeds the DFE cursor). LmsEqualizer is a textbook T-spaced NLMS transversal
// filter. ProbeDfe is the fielded probe-trained DECISION-FEEDBACK equalizer:
// feed-forward taps on the received T/2 window + feedback taps on past DECISIONS,
// adapted by NLMS or RLS. Its model-based counterpart (measure the channel, then
// SOLVE the MMSE-DFE) is cir_equalizer.hpp; both drive the same filter equation
//   z = sum_t w[t]*window[t] - sum_j b[j]*d[k-j].
// This adaptive family compiles in EVERY target: it is the receiver's default
// body equalizer, and the only one in an embedded build. Its model-based
// counterpart is host-only, gated by the M110_BODY_RECEIVER_CIR_ENGINE build
// option (default ON native, OFF embedded) - see cir_equalizer.hpp.
// Teaching walkthrough (worked example, equations, fading modes, debugging):
//   modem/m110a/receiver-core-equalizer-explainer.md
// =============================================================================

namespace modem::m110a
{

using ::modem::common::IQSample;
using ::modem::common::IQSampleSpan;
using ::modem::common::Result;
using ::modem::common::Status;

struct ChannelEstimate
{
    IQSample gain{};
    float mean_squared_error{};
};

[[nodiscard]] std::expected<ChannelEstimate, Status> estimate_flat_channel(IQSampleSpan received_probes, IQSampleSpan known_probes) noexcept;

class LmsEqualizer
{
public:
    static constexpr std::size_t maximum_taps = 32U;

    [[nodiscard]] Status configure(std::size_t taps, float step_size, float leakage = 0.0F) noexcept;
    void reset() noexcept;
    [[nodiscard]] IQSample filter(IQSample input) noexcept;
    [[nodiscard]] IQSample train(IQSample input, IQSample desired) noexcept;
    [[nodiscard]] IQSample decision_directed(IQSample input, IQSample decision) noexcept;

    [[nodiscard]] std::size_t taps() const noexcept { return mTaps; }

    [[nodiscard]] std::span<const IQSample> coefficients() const noexcept { return std::span<const IQSample>{mCoefficients.data(), mTaps}; }

private:
    [[nodiscard]] IQSample update(IQSample input, IQSample desired, bool adapt) noexcept;

    std::array<IQSample, maximum_taps> mCoefficients{};
    std::array<IQSample, maximum_taps> mHistory{};
    std::size_t mTaps{1U};
    float mStepSize{};
    float mLeakage{};
};

// Adaptation algorithm for the probe-trained DFE. NLMS converges over many
// tens of symbol periods, which loses fast-fading channels between probes;
// RLS (exponentially weighted recursive least squares) converges in roughly
// twice the tap count and is the fielded-receiver family (Kalman/RLS DFEs in
// the reference survey). RLS costs O(taps^2) per trained symbol and an
// (ff+fb)^2 complex P matrix.
enum class DfeAdaptation : std::uint8_t
{
    nlms,
    rls
};

class ProbeDfe
{
public:
    static constexpr std::size_t maximum_feedforward_taps = 32U;
    static constexpr std::size_t maximum_feedback_taps = 16U;
    static constexpr std::size_t maximum_joint_taps = maximum_feedforward_taps + maximum_feedback_taps;

    [[nodiscard]] Status configure(std::size_t feedforward_taps, std::size_t feedback_taps, std::size_t samples_per_symbol, std::size_t cursor, float step_size, float leakage = 0.0F,
                                   DfeAdaptation adaptation = DfeAdaptation::nlms, float rls_forgetting = 0.98F) noexcept;
    void reset() noexcept;
    [[nodiscard]] Status initialize_cursor_gain(IQSample channel_gain) noexcept;
    [[nodiscard]] IQSample filter(IQSampleSpan feedforward_window) const noexcept;
    [[nodiscard]] IQSample train(IQSampleSpan feedforward_window, IQSample desired) noexcept;
    // train() for a caller that needs the adaptation only, not the output.
    void adapt(IQSampleSpan feedforward_window, IQSample desired) noexcept;
    void commit_decision(IQSample decision) noexcept;
    void clear_decision_history() noexcept;

    [[nodiscard]] std::size_t feedforward_taps() const noexcept { return mFeedforwardTaps; }

    [[nodiscard]] std::size_t feedback_taps() const noexcept { return mFeedbackTaps; }

    [[nodiscard]] std::size_t samples_per_symbol() const noexcept { return mSamplesPerSymbol; }

    [[nodiscard]] std::size_t cursor() const noexcept { return mCursor; }

    [[nodiscard]] std::span<const IQSample> feedforward_coefficients() const noexcept { return std::span<const IQSample>{mFeedforwardCoefficients.data(), mFeedforwardTaps}; }

    [[nodiscard]] std::span<const IQSample> feedback_coefficients() const noexcept { return std::span<const IQSample>{mFeedbackCoefficients.data(), mFeedbackTaps}; }

    // Lightweight snapshot of exactly the state NLMS training mutates: the
    // feed-forward/feedback coefficients and the decision history. It excludes
    // the RLS inverse-correlation matrix (which train_nlms never touches), so it
    // is a complete end-of-training snapshot only under the NLMS adaptation
    // family; RLS callers must retrain instead of restoring.
    struct NlmsState
    {
        std::array<IQSample, maximum_feedforward_taps> feedforward{};
        std::array<IQSample, maximum_feedback_taps> feedback{};
        std::array<IQSample, maximum_feedback_taps> decision_history{};
    };

    [[nodiscard]] NlmsState capture_nlms_state() const noexcept;
    void restore_nlms_state(const NlmsState& state) noexcept;

private:
    [[nodiscard]] IQSample output(IQSampleSpan feedforward_window) const noexcept;
    // Both return the output the update was computed from; adapt() calls them
    // for the update alone.
    IQSample train_nlms(IQSampleSpan feedforward_window, IQSample desired) noexcept;
    IQSample train_rls(IQSampleSpan feedforward_window, IQSample desired) noexcept;
    void reset_rls() noexcept;

    std::array<IQSample, maximum_feedforward_taps> mFeedforwardCoefficients{};
    std::array<IQSample, maximum_feedback_taps> mFeedbackCoefficients{};
    std::array<IQSample, maximum_feedback_taps> mDecisionHistory{};
    // RLS inverse-correlation matrix over the joint [feedforward; feedback]
    // input vector; row-major, joint_taps x joint_taps active.
    std::array<IQSample, maximum_joint_taps * maximum_joint_taps> mRlsInverseCorrelation{};
    std::array<IQSample, maximum_joint_taps> mRlsGain{};
    std::array<IQSample, maximum_joint_taps> mRlsInput{};
    std::size_t mFeedforwardTaps{1U};
    std::size_t mFeedbackTaps{1U};
    std::size_t mSamplesPerSymbol{1U};
    std::size_t mCursor{};
    float mStepSize{};
    float mLeakage{};
    float mRlsForgetting{0.98F};
    DfeAdaptation mAdaptation{DfeAdaptation::nlms};
};

} // namespace modem::m110a
