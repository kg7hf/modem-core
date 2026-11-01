// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// equalizer_tests.cpp - the equalizers' unit tests, on synthesized signals.
// Explained in the article series, Part 6 (the equalizer).
// -----------------------------------------------------------------------------
// The building blocks: the one-tap flat-channel estimate, the LMS equalizer, the
// probe DFE (symbol-spaced and fractionally spaced), and the channel-estimate solvers
// (the raised-cosine noise autocorrelation, the polyphase least-squares fit and the
// MMSE-DFE tap solve). Then the body receiver that drives them: training on the
// preamble and equalizing a block over a static two-path channel, the supervised
// re-equalization and its save-and-rerun contract, the receiver's lifecycle (configure,
// train, rearm, reset), loss of lock across a signal gap and recovery after rearm, a
// residual coarse-frequency error, the frequency race gate, the resumable training that
// matches the batch training bit for bit however it is fragmented, the receiver's
// footprint, and the runtime engine selection. The cases that use the cir_mmse engine
// build only with M110_BODY_RECEIVER_CIR_ENGINE on; with it off, selecting the engine
// reports it unavailable.
// =============================================================================

#include "modem/common/bits.hpp"
#include "modem/common/cir_equalizer.hpp"
#include "modem/common/convert.hpp"
#include "modem/common/types.hpp"
#include "modem/m110a/body_waveform.hpp"
#include "modem/m110a/demapper.hpp"
#include "modem/m110a/equalizer.hpp"
#include "modem/m110a/receiver.hpp"
#include "modem/m110a/synchronization.hpp"
#include "tests/support/impairments.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <span>
#include <tuple>
#include <vector>

using ::modem::common::as_real;
using ::modem::common::BodyInterleave;
using ::modem::common::DataRate;
using ::modem::common::Index;
using ::modem::common::IQSample;
using ::modem::common::IQSampleSpan;
using ::modem::common::low_bits;
using ::modem::common::MutableIQSampleSpan;
using ::modem::common::narrow;
using ::modem::common::to_offset;
using ::modem::common::to_subscript;
using ::modem::m110a::BodyAdaptiveReceiver;
using ::modem::m110a::BodyBlockPlan;
using ::modem::m110a::BodyDataEqualizer;
using ::modem::m110a::ProbeDfe;

namespace
{

constexpr float pi = 3.14159265358979323846F;
constexpr std::size_t preamble_symbols = 1440U;
constexpr std::size_t block_symbols = 1440U;
constexpr std::size_t burst_symbols = preamble_symbols + block_symbols;

[[nodiscard]] bool close(float lhs, float rhs, float tolerance = 1.0e-5F) noexcept
{
    return std::fabs(lhs - rhs) <= tolerance;
}

// One short-interleave burst: the block plan, one encoded body block of a fixed bit
// pattern ((index * multiplier) + offset, low bit), the preamble, and the clean symbols
// (preamble, then the block).
struct Burst
{
    BodyBlockPlan plan{};
    std::vector<std::uint8_t> information;
    std::vector<std::uint8_t> body_tribits;
    std::vector<std::uint8_t> preamble_tribits;
    std::vector<IQSample> clean;
};

Burst make_burst(DataRate rate, std::size_t multiplier, std::size_t offset)
{
    const auto plan = ::modem::m110a::body_block_plan({rate, BodyInterleave::short_block});
    REQUIRE(plan);
    Burst burst;
    burst.plan = plan.value();
    burst.information.resize(burst.plan.information_bits);
    for (std::size_t index = 0U; index < burst.information.size(); ++index)
    {
        burst.information[index] = low_bits<1>((index * multiplier) + offset);
    }
    std::vector<std::uint8_t> coded(burst.plan.coded_bits);
    std::vector<std::uint8_t> matrix(burst.plan.coded_bits);
    std::vector<std::uint8_t> interleaved(burst.plan.coded_bits);
    burst.body_tribits.resize(block_symbols);
    REQUIRE(::modem::m110a::encode_body_block(burst.information, burst.plan, {coded, matrix, interleaved}, burst.body_tribits).is_ok());
    burst.preamble_tribits.resize(preamble_symbols);
    REQUIRE(::modem::m110a::generate_body_preamble(burst.plan.mode, burst.preamble_tribits).is_ok());
    burst.clean.resize(burst_symbols);
    REQUIRE(::modem::m110a::body_tribits_to_iq(burst.preamble_tribits, MutableIQSampleSpan{burst.clean}.first(preamble_symbols)).is_ok());
    REQUIRE(::modem::m110a::body_tribits_to_iq(burst.body_tribits, MutableIQSampleSpan{burst.clean}.subspan(preamble_symbols)).is_ok());
    return burst;
}

// A static two-path channel: direct * x[n] + echo * x[n - delay].
std::vector<IQSample> two_path(std::span<const IQSample> clean, std::size_t delay, float direct, IQSample echo)
{
    std::vector<IQSample> received(clean.size());
    for (std::size_t index = 0U; index < clean.size(); ++index)
    {
        const auto delayed = index < delay ? IQSample{} : clean[index - delay];
        received[index] = direct * clean[index] + echo * delayed;
    }
    return received;
}

// Equalized symbols whose hard decision is not the transmitted tribit.
std::size_t symbol_errors(std::span<const IQSample> equalized, std::span<const std::uint8_t> tribits)
{
    std::size_t errors{};
    for (std::size_t index = 0U; index < equalized.size(); ++index)
    {
        errors += ::modem::m110a::psk8_hard_demapper(equalized[index]) == tribits[index] ? 0U : 1U;
    }
    return errors;
}

// Bit-identical spans: the same length and every sample exactly equal.
bool identical(std::span<const IQSample> lhs, std::span<const IQSample> rhs)
{
    return std::ranges::equal(lhs, rhs);
}

// A deterministic pseudo-random unit-modulus symbol stream. A short-period pattern would
// leave the least-squares design matrix rank-deficient.
IQSample scrambled_symbol(std::size_t index)
{
    auto state = narrow<std::uint32_t>(index) * 2654435761U + 0x9E3779B9U;
    state ^= state >> 15U;
    state *= 2246822519U;
    state ^= state >> 13U;
    return ::modem::m110a::psk8_symbol(low_bits<3>(state));
}

// The trained receiver definition the resumable training must reproduce: the escalation
// flag, the selected training frequency, the carrier loop, and the equalizer taps.
struct TrainedDefinition
{
    bool escalated{};
    float selected_frequency{};
    float carrier_phase{};
    float carrier_frequency{};
    std::vector<IQSample> feedforward;
    std::vector<IQSample> feedback;
};

TrainedDefinition trained_definition(const BodyAdaptiveReceiver& receiver)
{
    const auto feedforward = receiver.equalizer().feedforward_coefficients();
    const auto feedback = receiver.equalizer().feedback_coefficients();
    TrainedDefinition definition;
    definition.escalated = receiver.last_preamble_race_escalated();
    definition.selected_frequency = receiver.selected_training_frequency_radians();
    definition.carrier_phase = receiver.carrier_tracker().phase_radians();
    definition.carrier_frequency = receiver.carrier_tracker().frequency_radians_per_symbol();
    definition.feedforward.assign(feedforward.begin(), feedforward.end());
    definition.feedback.assign(feedback.begin(), feedback.end());
    return definition;
}

// Drives the resumable training to its end with a fixed quota per step.
BodyAdaptiveReceiver::PreambleStepOutcome step_to_end(BodyAdaptiveReceiver& receiver, std::size_t quota, std::size_t guard_limit)
{
    auto outcome = receiver.step_preamble_training(quota);
    std::size_t guard = 0U;
    while (outcome == BodyAdaptiveReceiver::PreambleStepOutcome::running && guard < guard_limit)
    {
        outcome = receiver.step_preamble_training(quota);
        ++guard;
    }
    return outcome;
}

} // namespace

TEST_CASE("The flat-channel estimate recovers a one-tap gain from the probes, and the LMS equalizer converges to its inverse", "[equalizer_unit]")
{
    constexpr std::array<IQSample, 4> known_probes{IQSample{1.0F, 0.0F}, IQSample{0.0F, 1.0F}, IQSample{-1.0F, 0.0F}, IQSample{0.0F, -1.0F}};
    constexpr IQSample channel_gain{0.75F, 0.25F};
    std::array<IQSample, known_probes.size()> received_probes{};
    for (std::size_t index = 0U; index < known_probes.size(); ++index)
    {
        received_probes[index] = channel_gain * known_probes[index];
    }

    const auto channel = ::modem::m110a::estimate_flat_channel(received_probes, known_probes);
    REQUIRE(channel);
    CHECK(close(channel.value().gain.real(), channel_gain.real()));
    CHECK(close(channel.value().gain.imag(), channel_gain.imag()));
    CHECK(close(channel.value().mean_squared_error, 0.0F));

    ::modem::m110a::LmsEqualizer equalizer;
    CHECK(equalizer.configure(1U, 0.05F).is_ok());
    for (std::size_t iteration = 0U; iteration < 200U; ++iteration)
    {
        const auto desired = known_probes[iteration % known_probes.size()];
        std::ignore = equalizer.train(channel_gain * desired, desired);
    }
    const auto equalized = equalizer.filter(channel_gain * known_probes[0]);
    CHECK(close(equalized.real(), 1.0F, 1.0e-3F));
    CHECK(close(equalized.imag(), 0.0F, 1.0e-3F));
}

TEST_CASE("The probe DFE cancels a one-symbol echo through its feedback, and its fractionally spaced geometry starts with a unit cursor", "[equalizer_unit]")
{
    ProbeDfe dfe;
    CHECK(dfe.configure(1U, 3U, 1U, 0U, 0.12F).is_ok());
    constexpr IQSample echo{0.83F, 0.46F};
    IQSample previous{};
    std::uint32_t state = 0x110B75U;

    for (std::size_t index = 0U; index < 800U; ++index)
    {
        state = state * 1664525U + 1013904223U;
        const auto desired = ::modem::m110a::psk8_symbol(::modem::common::bits<3, std::uint8_t>(state, 21U));
        const std::array<IQSample, 1> window{desired + echo * previous};
        dfe.adapt(window, desired);
        previous = desired;
    }

    float error_energy{};
    for (std::size_t index = 800U; index < 864U; ++index)
    {
        state = state * 1664525U + 1013904223U;
        const auto desired = ::modem::m110a::psk8_symbol(::modem::common::bits<3, std::uint8_t>(state, 21U));
        const std::array<IQSample, 1> window{desired + echo * previous};
        const auto output = dfe.filter(window);
        error_energy += std::norm(output - desired);
        dfe.commit_decision(desired);
        previous = desired;
    }

    CHECK(error_energy / 64.0F < 1.0e-3F);
    CHECK(std::abs(dfe.feedback_coefficients()[0]) > 0.25F);

    // With no feedback taps a committed decision has nowhere to go and is harmless.
    ProbeDfe feedforward_only;
    CHECK(feedforward_only.configure(3U, 0U, 1U, 1U, 0.05F).is_ok());
    CHECK(feedforward_only.feedback_taps() == 0U);
    feedforward_only.commit_decision(IQSample{1.0F, 0.0F});

    // T/2 spacing: the cursor tap starts at one; three samples per symbol is rejected.
    ProbeDfe fractionally_spaced;
    CHECK(fractionally_spaced.configure(17U, 8U, 2U, 8U, 0.05F).is_ok());
    CHECK(fractionally_spaced.samples_per_symbol() == 2U);
    CHECK(fractionally_spaced.cursor() == 8U);
    CHECK(close(fractionally_spaced.feedforward_coefficients()[8U].real(), 1.0F));
    CHECK(!fractionally_spaced.configure(17U, 8U, 3U, 8U, 0.05F).is_ok());
}

TEST_CASE("The channel-estimate solvers recover a sparse sub-channel, and the MMSE-DFE taps they give reproduce the transmitted symbols", "[equalizer_unit]")
{
    // Raised-cosine noise autocorrelation: unity at zero lag, the alpha = 0.25 half-symbol
    // value at lag one, Nyquist zeros at nonzero even lags.
    CHECK(close(::modem::common::raised_cosine_autocorrelation_half_lag(0U), 1.0F));
    CHECK(close(::modem::common::raised_cosine_autocorrelation_half_lag(1U), 0.6273F, 1.0e-3F));
    CHECK(close(::modem::common::raised_cosine_autocorrelation_half_lag(2U), 0.0F));
    CHECK(close(::modem::common::raised_cosine_autocorrelation_half_lag(3U), -0.1856F, 1.0e-3F));
    CHECK(close(::modem::common::raised_cosine_autocorrelation_half_lag(4U), 0.0F));

    // Polyphase least squares recovers a known sparse sub-channel from noiseless rows
    // built the same way the receiver builds them.
    ::modem::common::CirPolyphaseProblem problem{};
    problem.taps = 16U;
    problem.rows = 30U;
    std::array<IQSample, ::modem::common::cir_polyphase_taps> truth{};
    truth[2] = IQSample{0.9F, 0.0F};
    truth[5] = IQSample{0.0F, 0.4F};
    truth[9] = IQSample{-0.2F, 0.1F};

    for (std::size_t row = 0U; row < problem.rows; ++row)
    {
        IQSample observation{};
        for (std::size_t index = 0U; index < problem.taps; ++index)
        {
            problem.regressors[row][index] = scrambled_symbol(row * 3U + index);
            observation += problem.regressors[row][index] * truth[index];
        }
        problem.observations[row] = observation;
        problem.weights[row] = 1.0F;
    }

    ::modem::common::CirSolveScratch scratch{};
    std::array<IQSample, ::modem::common::cir_polyphase_taps> solved{};
    std::array<IQSample, ::modem::common::cir_polyphase_taps> zero_prior{};
    float residual_energy{};
    CHECK(::modem::common::cir_polyphase_solve(problem, 0.0F, 1.0e-4F * as_real(problem.rows), zero_prior, scratch, solved, residual_energy).is_ok());
    CHECK(residual_energy < 1.0e-4F);
    float recovery_error{};
    for (std::size_t index = 0U; index < problem.taps; ++index)
    {
        recovery_error += std::norm(solved[index] - truth[index]);
    }
    CHECK(recovery_error < 1.0e-4F);

    // MMSE-DFE from a two-path CIR: filtering a synthesized cursor-centered window with
    // the solved taps and the correct feedback reproduces the transmitted symbol closely.
    // This exercises the plain-product and conjugation conventions of the receiver's
    // output equation.
    const ::modem::common::CirMmseGeometry geometry{24U, 12U, 8U, 4U, 15U};
    const auto support = geometry.precursor_taps + geometry.postcursor_taps + 1U;
    std::array<IQSample, ::modem::common::cir_maximum_taps> channel{};
    channel[geometry.precursor_taps] = IQSample{1.0F, 0.0F};
    channel[geometry.precursor_taps + 8U] = IQSample{0.35F, -0.35F};
    std::array<IQSample, ProbeDfe::maximum_feedforward_taps> feedforward{};
    std::array<IQSample, ProbeDfe::maximum_feedback_taps> feedback{};
    CHECK(::modem::common::cir_mmse_dfe_solve(IQSampleSpan{channel}.first(support), geometry, 0.01F, scratch, feedforward, feedback).is_ok());
    float worst_error{};

    for (std::size_t symbol = 40U; symbol < 48U; ++symbol)
    {
        std::array<IQSample, ProbeDfe::maximum_feedforward_taps> window{};
        for (std::size_t tap = 0U; tap < geometry.feedforward_taps; ++tap)
        {
            for (Index lag = -8; lag <= 12; ++lag)
            {
                const Index q = to_offset(tap) - to_offset(geometry.cursor) + 2 * lag + to_offset(geometry.precursor_taps);
                if (q >= 0 && q < to_offset(support))
                {
                    window[tap] += channel[to_subscript(q)] * scrambled_symbol(to_subscript(to_offset(symbol) - lag));
                }
            }
        }

        IQSample output{};
        for (std::size_t tap = 0U; tap < geometry.feedforward_taps; ++tap)
        {
            output += feedforward[tap] * window[tap];
        }
        for (std::size_t index = 0U; index < geometry.feedback_taps; ++index)
        {
            output -= feedback[index] * scrambled_symbol(symbol - 1U - index);
        }
        worst_error = std::max(worst_error, std::norm(output - scrambled_symbol(symbol)));
    }

    CHECK(worst_error < 0.02F);
}

TEST_CASE("Preamble training over a static two-path channel leaves the first body block nearly error-free", "[equalizer_unit]")
{
    // One full-preamble pass converges on a static channel; a benefit from more passes
    // shows only on fading channels, so this case does not try to tell pass counts apart.
    const auto burst = make_burst(DataRate::bps600, 7U, 5U);
    auto received = two_path(burst.clean, 3U, 0.75F, IQSample{0.35F, 0.30F});
    modem_test::DeterministicRng noise{0x3A4110BULL};
    modem_test::add_awgn(received, 0.02F, noise);
    const auto acquisition = ::modem::m110a::acquire_body_preamble(received, {2400.0F, 0.55F, 40.0F, 1U});
    REQUIRE(acquisition);

    BodyAdaptiveReceiver receiver;
    CHECK(receiver.configure(17U, 0.08F, 0.02F, 0.0002F).is_ok());
    CHECK(receiver.train_preamble(IQSampleSpan{received}.first(preamble_symbols), burst.preamble_tribits, acquisition.value()).is_ok());
    std::array<IQSample, block_symbols> equalized{};
    CHECK(receiver.equalize_block(IQSampleSpan{received}.subspan(preamble_symbols), burst.plan, equalized).is_ok());
    CHECK(symbol_errors(equalized, burst.body_tribits) <= 40U);
}

TEST_CASE("Supervised re-equalization is at least as clean as the decision-directed pass, and a copied receiver reruns it exactly", "[equalizer_unit]")
{
    // The supervised pass trains on the transmitted reference (iterative decode feedback);
    // the save-and-restore contract is a copy of the receiver, and rerunning the block from
    // the copy reproduces the pass bit for bit.
    const auto burst = make_burst(DataRate::bps600, 11U, 3U);
    auto received = two_path(burst.clean, 3U, 0.75F, IQSample{0.35F, 0.30F});
    modem_test::DeterministicRng noise{0x3A4110BULL};
    modem_test::add_awgn(received, 0.02F, noise);
    const auto acquisition = ::modem::m110a::acquire_body_preamble(received, {2400.0F, 0.55F, 40.0F, 1U});
    REQUIRE(acquisition);

    BodyAdaptiveReceiver receiver;
    CHECK(receiver.configure(17U, 0.08F, 0.02F, 0.0002F).is_ok());
    CHECK(receiver.train_preamble(IQSampleSpan{received}.first(preamble_symbols), burst.preamble_tribits, acquisition.value()).is_ok());
    const auto receiver_at_block = receiver;
    const auto block = IQSampleSpan{received}.subspan(preamble_symbols);

    std::array<IQSample, block_symbols> equalized_unsupervised{};
    CHECK(receiver.equalize_block(block, burst.plan, equalized_unsupervised).is_ok());
    const auto unsupervised_errors = symbol_errors(equalized_unsupervised, burst.body_tribits);

    auto supervised_receiver = receiver_at_block;
    std::array<IQSample, block_symbols> equalized_supervised{};
    CHECK(supervised_receiver.equalize_block_supervised(block, burst.plan, burst.body_tribits, equalized_supervised).is_ok());
    CHECK(symbol_errors(equalized_supervised, burst.body_tribits) <= unsupervised_errors);

    auto replay_receiver = receiver_at_block;
    std::array<IQSample, block_symbols> equalized_replay{};
    CHECK(replay_receiver.equalize_block_supervised(block, burst.plan, burst.body_tribits, equalized_replay).is_ok());
    CHECK(identical(equalized_replay, equalized_supervised));

    // A reference of the wrong size is rejected.
    CHECK(!replay_receiver.equalize_block_supervised(block, burst.plan, std::span<const std::uint8_t>{burst.body_tribits}.first(100U), equalized_replay).is_ok());
}

TEST_CASE("The body receiver trains only when configured, rearm keeps the configuration and drops the training, and reset drops both", "[equalizer_unit]")
{
    BodyAdaptiveReceiver receiver;
    CHECK(!receiver.is_configured());
    CHECK(!receiver.is_trained());
    CHECK(!receiver.rearm().is_ok());
    CHECK(receiver.configure(8U, 0.08F, 0.02F, 0.0002F).is_ok());
    CHECK(receiver.is_configured());
    CHECK(!receiver.is_trained());

    const auto plan = ::modem::m110a::body_block_plan({DataRate::bps600, BodyInterleave::short_block});
    REQUIRE(plan);
    std::array<std::uint8_t, preamble_symbols> preamble{};
    CHECK(::modem::m110a::generate_body_preamble(plan.value().mode, preamble).is_ok());
    std::array<IQSample, preamble_symbols> preamble_symbols_iq{};
    CHECK(::modem::m110a::body_tribits_to_iq(preamble, preamble_symbols_iq).is_ok());
    const auto acquisition = ::modem::m110a::acquire_body_preamble(preamble_symbols_iq, {2400.0F, 0.85F, 1.0F, 1U});
    REQUIRE(acquisition);

    CHECK(receiver.train_preamble(preamble_symbols_iq, preamble, acquisition.value()).is_ok());
    CHECK(receiver.is_trained());
    CHECK(receiver.rearm().is_ok());
    CHECK(receiver.is_configured());
    CHECK(!receiver.is_trained());
    // An untrained receiver does not equalize.
    std::array<IQSample, block_symbols> equalized{};
    CHECK(!receiver.equalize_block(preamble_symbols_iq, plan.value(), equalized).is_ok());

    receiver.reset();
    CHECK(!receiver.is_configured());
    CHECK(!receiver.is_trained());
    CHECK(!receiver.train_preamble(preamble_symbols_iq, preamble, acquisition.value()).is_ok());
    CHECK(!receiver.configure(0U, 0.08F, 0.02F, 0.0002F).is_ok());
    CHECK(!receiver.is_configured());
    CHECK(!receiver.is_trained());
}

TEST_CASE("The block probe residual is low in lock, saturates across a signal gap, and is low again after rearm on the next burst", "[equalizer_unit]")
{
    // One continuous stream: a burst, a signal-loss gap, and a second burst with a
    // different payload.
    const auto first = make_burst(DataRate::bps600, 5U, 1U);
    const auto second = make_burst(DataRate::bps600, 11U, 2U);
    constexpr std::size_t gap_symbols = 480U;
    std::vector<IQSample> stream(2U * burst_symbols + gap_symbols);
    std::ranges::copy(first.clean, stream.begin());
    std::ranges::copy(second.clean, stream.begin() + to_offset(burst_symbols + gap_symbols));
    constexpr float radians_per_symbol = 2.0F * pi * 4.0F / 2400.0F;
    for (std::size_t index = 0U; index < stream.size(); ++index)
    {
        stream[index] *= IQSample{0.85F, 0.21F} * std::polar(1.0F, 0.4F + radians_per_symbol * as_real(index));
    }
    modem_test::DeterministicRng noise{0x3A2110BULL};
    modem_test::add_awgn(stream, 0.01F, noise);

    const auto acquisition_first = ::modem::m110a::acquire_body_preamble(stream, {2400.0F, 0.85F, 40.0F, 1U});
    REQUIRE(acquisition_first);
    BodyAdaptiveReceiver receiver;
    CHECK(receiver.configure(8U, 0.08F, 0.02F, 0.0002F).is_ok());
    CHECK(receiver.train_preamble(IQSampleSpan{stream}.first(preamble_symbols), first.preamble_tribits, acquisition_first.value()).is_ok());
    std::array<IQSample, block_symbols> equalized{};
    CHECK(receiver.equalize_block(IQSampleSpan{stream}.subspan(preamble_symbols, block_symbols), first.plan, equalized).is_ok());
    // In lock the block probe residual is near 0.02; a block equalized across the gap
    // saturates toward the unit constellation power. The 0.5 boundary sits an order of
    // magnitude above lock and well below saturation.
    constexpr float loss_of_lock_threshold = 0.5F;
    CHECK(receiver.last_block_probe_error() < 0.25F * loss_of_lock_threshold);

    // Continuing past the end of the transmission: the next block spans the gap and the
    // start of the unrelated second burst.
    CHECK(receiver.equalize_block(IQSampleSpan{stream}.subspan(burst_symbols, block_symbols), first.plan, equalized).is_ok());
    CHECK(receiver.last_block_probe_error() > loss_of_lock_threshold);
    CHECK(receiver.rearm().is_ok());

    const auto tail = IQSampleSpan{stream}.subspan(burst_symbols + gap_symbols);
    const auto acquisition_second = ::modem::m110a::acquire_body_preamble(tail, {2400.0F, 0.85F, 40.0F, 1U});
    REQUIRE(acquisition_second);
    CHECK(receiver.train_preamble(tail.first(preamble_symbols), second.preamble_tribits, acquisition_second.value()).is_ok());
    CHECK(receiver.equalize_block(tail.subspan(preamble_symbols, block_symbols), second.plan, equalized).is_ok());
    CHECK(receiver.last_block_probe_error() < 0.25F * loss_of_lock_threshold);
}

TEST_CASE("With a residual coarse-frequency error and no feedback taps, the carrier loop stays finite and inside its frequency clamp", "[equalizer_unit]")
{
    const auto burst = make_burst(DataRate::bps75, 11U, 3U);
    auto received = burst.clean;
    constexpr float actual_frequency_offset_hz = 4.0F;
    constexpr float radians_per_symbol = 2.0F * pi * actual_frequency_offset_hz / 2400.0F;
    for (std::size_t index = 0U; index < received.size(); ++index)
    {
        received[index] *= IQSample{0.82F, 0.23F} * std::polar(1.0F, 0.31F + radians_per_symbol * as_real(index));
    }

    auto acquisition = ::modem::m110a::acquire_body_preamble(IQSampleSpan{received}.first(preamble_symbols), {2400.0F, 0.85F, 40.0F, 1U});
    REQUIRE(acquisition);
    // A small coarse-frequency error left in on purpose: over a full preamble it turns the
    // flat-channel estimate through two complete turns, while the acquisition prefix stays
    // coherent enough to seed the supervised carrier and equalizer loops.
    acquisition.value().frequency_offset_hz += 3.3333333F;

    BodyAdaptiveReceiver receiver;
    CHECK(receiver.configure(17U, 0.08F, 0.02F, 0.0002F, 0U).is_ok());
    CHECK(receiver.equalizer().feedback_taps() == 0U);
    CHECK(receiver.train_preamble(IQSampleSpan{received}.first(preamble_symbols), burst.preamble_tribits, acquisition.value()).is_ok());
    std::array<IQSample, block_symbols> equalized{};
    CHECK(receiver.equalize_block(IQSampleSpan{received}.subspan(preamble_symbols), burst.plan, equalized).is_ok());
    CHECK(std::isfinite(receiver.carrier_tracker().frequency_radians_per_symbol()));
    CHECK(std::abs(receiver.carrier_tracker().frequency_radians_per_symbol()) <= receiver.carrier_tracker().maximum_frequency_radians_per_symbol());
}

TEST_CASE("The frequency race is skipped on an ideal channel and escalates on an elevated residual, and both bodies equalize", "[equalizer_unit]")
{
    // The acquisition-first gate skips the race when the acquisition-trained residual is
    // already at the clamped noise floor. The skipped path matches the full race's
    // outcome: no hypothesis can score below the floor, and the strict-minimum tie-break
    // favors the acquisition.
    const auto burst = make_burst(DataRate::bps600, 7U, 5U);

    const auto equalize_and_count = [&](IQSampleSpan source, bool expect_escalated, std::size_t error_limit)
    {
        const auto acquisition = ::modem::m110a::acquire_body_preamble(source, {2400.0F, 0.55F, 40.0F, 1U});
        REQUIRE(acquisition);
        BodyAdaptiveReceiver receiver;
        CHECK(receiver.configure(17U, 0.08F, 0.02F, 0.0002F).is_ok());
        CHECK(receiver.train_preamble(source.first(preamble_symbols), burst.preamble_tribits, acquisition.value()).is_ok());
        CHECK(receiver.last_preamble_race_escalated() == expect_escalated);
        std::array<IQSample, block_symbols> equalized{};
        CHECK(receiver.equalize_block(source.subspan(preamble_symbols), burst.plan, equalized).is_ok());
        CHECK(symbol_errors(equalized, burst.body_tribits) <= error_limit);
    };

    // Ideal channel, no dispersion and no noise: the race is skipped and every symbol is right.
    equalize_and_count(burst.clean, false, 0U);

    // A two-path echo plus noise lifts the acquisition residual well above the floor, so
    // the gate escalates to the full race, which still recovers the body; the bound leaves
    // room for float differences between optimization levels.
    auto received = two_path(burst.clean, 5U, 0.75F, IQSample{0.40F, 0.20F});
    modem_test::DeterministicRng noise{0x57110B8ULL};
    modem_test::add_awgn(received, 0.09F, noise);
    equalize_and_count(received, true, 40U);
}

TEST_CASE("Resumable preamble training gives the batch training's receiver bit for bit, whatever the quota per step", "[equalizer_unit]")
{
    // The quota only places the yields. The trained definition (equalizer taps, feedback
    // register, carrier loop, selected frequency, escalation flag) and the equalized block
    // match the batch path, an unbounded quota in one step, on an input that escalates
    // (refine, candidate race, winner selection).
    const auto burst = make_burst(DataRate::bps600, 7U, 5U);
    auto received = two_path(burst.clean, 5U, 0.75F, IQSample{0.40F, 0.20F});
    modem_test::DeterministicRng noise{0x57110B8ULL};
    modem_test::add_awgn(received, 0.09F, noise);
    const auto acquisition = ::modem::m110a::acquire_body_preamble(received, {2400.0F, 0.55F, 40.0F, 1U});
    REQUIRE(acquisition);
    const auto preamble = IQSampleSpan{received}.first(preamble_symbols);
    const auto body = IQSampleSpan{received}.subspan(preamble_symbols);

    // The batch reference, its definition taken before equalize_block changes the
    // equalizer and carrier on the data symbols.
    BodyAdaptiveReceiver reference;
    CHECK(reference.configure(17U, 0.08F, 0.02F, 0.0002F).is_ok());
    CHECK(reference.train_preamble(preamble, burst.preamble_tribits, acquisition.value()).is_ok());
    CHECK(reference.last_preamble_race_escalated());
    const auto expected = trained_definition(reference);
    std::array<IQSample, block_symbols> reference_equalized{};
    CHECK(reference.equalize_block(body, burst.plan, reference_equalized).is_ok());

    for (const auto quota : {std::size_t{1U}, std::size_t{7U}, std::size_t{64U}, std::size_t{257U}})
    {
        INFO("quota " << quota);
        BodyAdaptiveReceiver fragmented;
        CHECK(fragmented.configure(17U, 0.08F, 0.02F, 0.0002F).is_ok());
        CHECK(fragmented.begin_preamble_training(preamble, 1U, burst.preamble_tribits, acquisition.value()).is_ok());
        CHECK(step_to_end(fragmented, quota, 1000000U) == BodyAdaptiveReceiver::PreambleStepOutcome::complete);

        const auto actual = trained_definition(fragmented);
        CHECK(actual.escalated == expected.escalated);
        CHECK(actual.selected_frequency == expected.selected_frequency);
        CHECK(actual.carrier_phase == expected.carrier_phase);
        CHECK(actual.carrier_frequency == expected.carrier_frequency);
        CHECK(actual.feedforward.size() == expected.feedforward.size());
        CHECK(actual.feedback.size() == expected.feedback.size());
        CHECK(identical(actual.feedforward, expected.feedforward));
        CHECK(identical(actual.feedback, expected.feedback));

        std::array<IQSample, block_symbols> fragmented_equalized{};
        CHECK(fragmented.equalize_block(body, burst.plan, fragmented_equalized).is_ok());
        CHECK(identical(fragmented_equalized, reference_equalized));
    }
}

TEST_CASE("The body receiver's size stays inside its bound, with or without the cir_mmse engine compiled in", "[equalizer_unit]")
{
    // The adaptive-only build does not carry the channel-estimate engine's state; the
    // upper bound catches a change that brings it back. The preamble-fit workspace is
    // always present and keeps the receiver above the lower bound.
#if !M110_BODY_RECEIVER_CIR_ENGINE
    CHECK(sizeof(BodyAdaptiveReceiver) < 46U * 1024U);
#else
    // Host builds compile the runtime-selectable engine in, with its state.
    CHECK(sizeof(BodyAdaptiveReceiver) < 64U * 1024U);
#endif
    CHECK(sizeof(BodyAdaptiveReceiver) > 20U * 1024U);
}

TEST_CASE("The adaptive engine is the default and is unchanged by selecting it, and the cir_mmse engine is selectable exactly when compiled in", "[equalizer_unit]")
{
    // A receiver that never selects an engine and one that selects adaptive give
    // bit-identical output. A receiver that trained with the cir_mmse engine and then
    // reselects adaptive and reconfigures gives the adaptive output again: no state leaks
    // through the shared preamble-fit workspace.
    CHECK(::modem::m110a::body_receiver_engine_available(BodyDataEqualizer::adaptive));
    CHECK(::modem::m110a::body_receiver_engine_available(BodyDataEqualizer::cir_mmse) == (M110_BODY_RECEIVER_CIR_ENGINE != 0));

    const auto burst = make_burst(DataRate::bps600, 13U, 1U);
    // A T/2 stream through a two-path channel: the cursor (odd) samples carry the symbol
    // plus a 4-symbol echo, the even samples the half-symbol mean, so both engines have a
    // dispersive but exactly representable channel to work on.
    std::vector<IQSample> half_symbols(burst_symbols * 2U);
    const IQSample echo_gain{0.30F, -0.25F};
    for (std::size_t symbol = 0U; symbol < burst_symbols; ++symbol)
    {
        const auto echo = symbol < 4U ? IQSample{} : burst.clean[symbol - 4U];
        const auto previous = symbol == 0U ? IQSample{} : burst.clean[symbol - 1U];
        const auto previous_echo = symbol < 5U ? IQSample{} : burst.clean[symbol - 5U];
        const auto cursor = 0.8F * burst.clean[symbol] + echo_gain * echo;
        const auto before = 0.8F * previous + echo_gain * previous_echo;
        half_symbols[symbol * 2U + 1U] = cursor;
        half_symbols[symbol * 2U] = 0.5F * (cursor + before);
    }
    modem_test::DeterministicRng noise{0x616110BULL};
    modem_test::add_awgn(half_symbols, 0.01F, noise);
    std::vector<IQSample> symbol_spaced(burst_symbols);
    for (std::size_t symbol = 0U; symbol < symbol_spaced.size(); ++symbol)
    {
        symbol_spaced[symbol] = half_symbols[symbol * 2U + 1U];
    }
    const auto acquisition = ::modem::m110a::acquire_body_preamble(symbol_spaced, {2400.0F, 0.55F, 40.0F, 1U});
    REQUIRE(acquisition);

    const auto train_and_equalize = [&](BodyAdaptiveReceiver& receiver, MutableIQSampleSpan equalized) -> std::size_t
    {
        CHECK(receiver.configure_fractionally_spaced(24U, 8U, 0.10F, 0.02F, 0.0002F, 12U).is_ok());
        CHECK(receiver.train_preamble_fractional(IQSampleSpan{half_symbols}.first(burst_symbols), burst.preamble_tribits, acquisition.value()).is_ok());
        CHECK(receiver.equalize_block_fractional(IQSampleSpan{half_symbols}.subspan(burst_symbols), burst.plan, equalized).is_ok());
        return symbol_errors(equalized, burst.body_tribits);
    };

    // Never selected versus adaptive selected explicitly: bit-identical.
    std::vector<IQSample> equalized_default(block_symbols);
    std::vector<IQSample> equalized_adaptive(block_symbols);
    BodyAdaptiveReceiver default_receiver;
    CHECK(default_receiver.data_equalizer() == BodyDataEqualizer::adaptive);
    const auto default_errors = train_and_equalize(default_receiver, equalized_default);
    BodyAdaptiveReceiver adaptive_receiver;
    CHECK(adaptive_receiver.select_data_equalizer(BodyDataEqualizer::adaptive).is_ok());
    CHECK(train_and_equalize(adaptive_receiver, equalized_adaptive) == default_errors);
    CHECK(identical(equalized_default, equalized_adaptive));
    CHECK(default_errors <= 8U);

#if M110_BODY_RECEIVER_CIR_ENGINE
    // The cir_mmse engine is selectable, seeds from the preamble, and equalizes the
    // dispersive block cleanly through its own block walk.
    std::vector<IQSample> equalized_cir(block_symbols);
    BodyAdaptiveReceiver cir_receiver;
    CHECK(cir_receiver.select_data_equalizer(BodyDataEqualizer::cir_mmse).is_ok());
    CHECK(cir_receiver.data_equalizer() == BodyDataEqualizer::cir_mmse);
    CHECK(train_and_equalize(cir_receiver, equalized_cir) <= 8U);
    CHECK(!identical(equalized_cir, equalized_default));

    // Training with cir_mmse seeds the engine's state and rewrites the shared support
    // precursor; reselecting adaptive and reconfiguring reproduces the adaptive output.
    std::vector<IQSample> equalized_returned(block_symbols);
    CHECK(cir_receiver.select_data_equalizer(BodyDataEqualizer::adaptive).is_ok());
    CHECK(train_and_equalize(cir_receiver, equalized_returned) == default_errors);
    CHECK(identical(equalized_returned, equalized_default));
#else
    BodyAdaptiveReceiver unavailable_receiver;
    CHECK(unavailable_receiver.select_data_equalizer(BodyDataEqualizer::cir_mmse).code == ::modem::common::StatusCode::unavailable);
    CHECK(unavailable_receiver.data_equalizer() == BodyDataEqualizer::adaptive);
#endif
}
