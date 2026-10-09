// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// front_end_tests.cpp - the receive front end's tests
// =============================================================================
// Each test feeds known data through one piece of the front end and checks what
// comes out. The signals come from the real transmitter (support/test_waveform),
// the impairments from the channel stage (support/test_channel), and the scoring
// from the front-end harness (support/front_end_harness). Run them from VS Code's
// Testing panel or with ctest; set a breakpoint and debug any one of them. The
// [.long] test runs only when asked for (ctest -L long).
// Explained in the article series, Part 3 (the audio front end).
// =============================================================================

#include "modem/common/convert.hpp"
#include "modem/common/dsp/timing_recovery.hpp"
#include "modem/m110a/body_audio_stream_frontend.hpp"
#include "modem/m110a/body_waveform.hpp"
#include "modem/m110a/scrambler.hpp"
#include "tests/support/front_end_harness.hpp"
#include "tests/support/test_channel.hpp"
#include "tests/support/test_waveform.hpp"
#include "tests/walkthrough/front_end_steps.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <complex>
#include <cstdint>
#include <print>
#include <vector>

namespace
{

using namespace ::modem_test;
using ::modem::common::as_real;
using ::modem::common::narrow;
using ::modem::m110a::BodyAudioStreamFrontend;

constexpr std::size_t samples_per_symbol = ::modem::m110a::body_audio_samples_per_symbol; // 20 at 48 kHz

// The two-character message from Part 2, with the truth of what was sent.
TestWaveform hi_waveform()
{
    WaveformSpec spec;
    spec.payload = {'H', 'i'};
    const auto waveform = make_test_waveform(spec);
    REQUIRE(waveform.error().is_ok());
    return waveform.value();
}

FrontEndRunner run(const std::vector<float>& audio, const FrontEndOptions& options = {})
{
    FrontEndRunner runner;
    REQUIRE(runner.initialize(options).is_ok());
    REQUIRE(runner.feed(audio).is_ok());
    return runner;
}

} // namespace

// The article's walkthrough: each case runs one step from tests/walkthrough/front_end_steps
// (the same functions front_end_demo prints, step by step) and checks what it returned.
//   front_end_tests "[walkthrough]" --order decl
// prints every number the article quotes, in the article's order.

// --- Mixing down to baseband ---

TEST_CASE("The 1800 Hz carrier repeats every 80 samples: three whole cycles", "[front_end][carrier][walkthrough]")
{
    STATIC_REQUIRE(1800U * ::modem::m110a::body_carrier_period_samples == 3U * 48000U);
    const auto step = front_end_steps::carrier_table();
    front_end_steps::print(step);
    CHECK(step.worst_error < 6.0e-8); // every entry is the exact phasor, rounded once to float
    CHECK(step.quadrants_exact);
}

TEST_CASE("A tone on the carrier mixes down to a constant and one 10 Hz off turns", "[front_end][mixer][walkthrough]")
{
    const auto tone_hz = GENERATE(1800U, 1810U);
    const auto step = front_end_steps::tone_mix(tone_hz);
    REQUIRE(step.error().is_ok());
    front_end_steps::print(step.value());
    const auto& s = step.value();
    CHECK(std::fabs(s.step - s.expected_step) < 1.0e-4F);

    // On the carrier the output is exactly constant. Off it, the mix's mirror image near
    // 3600 Hz is far down the filter's stopband but not gone: a faint ripple.
    if (tone_hz == 1800U)
    {
        CHECK(s.largest - s.smallest < 1.0e-4F * s.largest);
    }
}

TEST_CASE("Mixing Part 2's tribit 6 down from 1800 Hz", "[front_end][carrier][walkthrough]")
{
    const auto step = front_end_steps::tribit6_mix();
    REQUIRE(step.error().is_ok());
    front_end_steps::print(step.value());
    CHECK(step.value().tribit == 6U);
}

// --- The matched filter ---

TEST_CASE("The receive filter has unit energy and is symmetric about its peak", "[front_end][pulse][walkthrough]")
{
    const auto step = front_end_steps::receive_filter();
    REQUIRE(step.error().is_ok());
    front_end_steps::print(step.value());
    CHECK(std::fabs(step.value().energy - 1.0F) < 1.0e-5F);
    CHECK(step.value().peak_in_middle);
    CHECK(step.value().worst_asymmetry < 1.0e-6F);
}

TEST_CASE("Transmit pulse then receive filter leaves the neighbors' instants nearly empty", "[front_end][pulse][walkthrough]")
{
    const auto receive_rolloff = GENERATE(0.25F, BodyAudioStreamFrontend::matched_filter_rolloff);
    const auto step = front_end_steps::pulse_neighbors(receive_rolloff);
    REQUIRE(step.error().is_ok());
    front_end_steps::print(step.value());
    CHECK(step.value().largest_neighbor < 0.02F * step.value().peak);
}

// --- Finding the sampling instant ---

TEST_CASE("The early-late error is zero on the peak and signed off it for any carrier phase", "[front_end][timing][walkthrough]")
{
    const auto step = front_end_steps::timing_error();
    REQUIRE(step.error().is_ok());
    front_end_steps::print(step.value());
    const auto& s = step.value();
    CHECK(std::fabs(s.on_peak) < 1.0e-6F);
    CHECK(s.early > 0.0F);
    CHECK(s.late < 0.0F);
    CHECK(std::fabs(s.early + s.late) < 1.0e-6F);
    CHECK(std::fabs(s.early_turned - s.early) < 1.0e-6F);
}

// --- Keeping time for hours ---

TEST_CASE("A float timing position falls off the grid; whole samples plus a fraction do not", "[front_end][timing][walkthrough]")
{
    const auto step = front_end_steps::float_position();
    front_end_steps::print(step);
    CHECK(std::bit_cast<std::uint32_t>(step.position) == std::bit_cast<std::uint32_t>(67108880.0F));
    CHECK(std::bit_cast<std::uint32_t>(step.earlier) == std::bit_cast<std::uint32_t>(16777236.0F));
    CHECK(step.whole == 67108885U);
    CHECK(std::fabs(step.fraction) < 1.0e-6F);
}

// --- Any block size, the same symbols ---

TEST_CASE("Any block size gives the same symbols bit for bit", "[front_end][stream][walkthrough]")
{
    const auto step = front_end_steps::block_sizes();
    REQUIRE(step.error().is_ok());
    front_end_steps::print(step.value());
    CHECK(step.value().symbols > 0U);
    CHECK(step.value().identical == step.value().symbols);
}

// --- Carrier tracking, part one ---

TEST_CASE("The carrier tracker learns a fixed phase and then a frequency offset", "[front_end][tracker][walkthrough]")
{
    const auto step = front_end_steps::carrier_tracking();
    REQUIRE(step.error().is_ok());
    front_end_steps::print(step.value());
    const auto& s = step.value();
    CHECK(std::fabs(s.learned_phase - 0.5F) < 0.01F);
    CHECK(s.phase_residual < 0.01F);
    CHECK(std::fabs(s.learned_hz - 5.0F) < 0.01F);
    CHECK(s.frequency_residual < 0.01F);
}

// --- Walk the symbols back out ---

TEST_CASE("Walk: Part 2's randomizer values come back out of the front end", "[front_end][walk][walkthrough]")
{
    const auto step = front_end_steps::walk();
    REQUIRE(step.error().is_ok());
    front_end_steps::print(step.value());
    const auto& s = step.value();
    CHECK(s.every_tribit_matches);
    CHECK(s.data_is_bpsk);
    REQUIRE(s.shown.size() == 7U); // the first six body symbols, then the first coded 1
    CHECK(s.shown.back().data == 4U);
}

// --- The test bench: whole bursts, many bursts, and a channel ---

TEST_CASE("Hi: after the loop locks every decision is the transmitted tribit", "[front_end][end_to_end]")
{
    const auto waveform = hi_waveform();
    const auto runner = run(waveform.audio);
    const auto score = score_burst(runner, waveform.bursts.front(), 0U);
    std::println("locked by symbol {}; {} of {} decisions match", score.locked_from, score.correct, score.compared);
    CHECK(score.locked);
    CHECK(score.compared > 10000U);
    CHECK(score.correct == score.compared);
}

namespace
{

struct ShortLock
{
    BurstScore score{};
    std::size_t preamble_symbols{};
};

// The 600S "Hi" burst behind `lead` samples of silence, through the front end. The lead moves
// where the loop starts against the symbols, which is not the receiver's choice.
ShortLock lock_600s(std::size_t lead)
{
    WaveformSpec spec;
    spec.mode = {::modem::common::DataRate::bps600, ::modem::common::BodyInterleave::short_block};
    spec.payload = {'H', 'i'};
    const auto made = make_test_waveform(spec);
    REQUIRE(made.error().is_ok());
    const auto& burst = made.value().bursts.front();

    std::vector<float> audio(lead, 0.0F);
    audio.insert(audio.end(), made.value().audio.begin(), made.value().audio.end());
    const auto runner = run(audio);
    const auto score = score_burst(runner, burst, lead);
    std::println("burst {:2} samples later: locked by symbol {:4} of the {} preamble symbols; {} of {} decisions match", lead, score.locked_from, burst.preamble_symbols, score.correct,
                 score.compared);
    return {score, burst.preamble_symbols};
}

} // namespace

TEST_CASE("600S: the loop locks inside the short preamble from every start but half a symbol off", "[front_end][end_to_end]")
{
    // With no lead the loop starts exactly half a symbol off the peaks; the next test has that case.
    for (std::size_t lead = 1U; lead < samples_per_symbol; ++lead)
    {
        const auto result = lock_600s(lead);
        CHECK(result.score.locked);
        CHECK(result.score.locked_from < result.preamble_symbols);
        CHECK(result.score.correct == result.score.compared);
    }
}

TEST_CASE("600S: a loop started half a symbol off still locks inside the short preamble", "[front_end][end_to_end][!shouldfail]")
{
    // A known limit. With no lead, the first on-time point (sample 170) sits exactly half a
    // symbol from the filtered peaks (20k + 80): the early-late detector's unstable zero, where
    // the early and late samples are equally strong, the error is near zero and the loop barely
    // moves. It locks about 2200 symbols in, after the 1440-symbol preamble, with or without
    // noise. [!shouldfail] keeps the suite green while the limit stands; once it is fixed, this
    // case reports its pass as a failure and the tag comes off.
    const auto result = lock_600s(0U);
    CHECK(result.score.locked);
    CHECK(result.score.locked_from < result.preamble_symbols);
    CHECK(result.score.correct == result.score.compared);
}

TEST_CASE("A train of ten random messages with gaps: every decision after lock matches", "[front_end][bench]")
{
    WaveformSpec spec;
    spec.random_payload_bytes = 32U;
    spec.seed = 7U;
    spec.bursts = 10U;
    spec.gap_samples = 12345U; // not a whole number of symbols, so each burst starts off the old grid
    const auto waveform = make_test_waveform(spec);
    REQUIRE(waveform.error().is_ok());

    const auto runner = run(waveform.value().audio);
    for (const auto& burst : waveform.value().bursts)
    {
        const auto score = score_burst(runner, burst, burst.first_sample);
        std::println("burst at sample {:8}: locked by symbol {:5}, {} of {} decisions match, carrier turned {:+6.1f} degrees", burst.first_sample, score.locked_from, score.correct, score.compared,
                     score.rotation_degrees);
        CHECK(score.locked);
        CHECK(score.correct == score.compared);
    }
}

TEST_CASE("Noise at 10 dB: the loop locks and most decisions are right", "[front_end][channel]")
{
    const auto seed = GENERATE(1U, 2U, 3U);
    auto waveform = hi_waveform();
    REQUIRE(apply_channel(waveform.audio, {.snr_db = 10.0F, .seed = seed}).is_ok());

    const auto runner = run(waveform.audio);
    const auto score = score_burst(runner, waveform.bursts.front(), 0U);
    const auto error_rate = 1.0F - as_real(score.correct) / as_real(std::max<std::size_t>(score.compared, 1U));
    std::println("AWGN, SNR 10 dB in 3 kHz, seed {}: locked by symbol {}, symbol errors {:.2f}%, rms phase error {:.1f} degrees", seed, score.locked_from, 100.0F * error_rate,
                 score.rms_phase_error_degrees);
    CHECK(score.locked);
    CHECK(error_rate < 0.10F);
}

TEST_CASE("A CW tone 10 dB below the signal: the loop locks and most decisions are right", "[front_end][channel]")
{
    auto waveform = hi_waveform();
    REQUIRE(apply_channel(waveform.audio, {.snr_db = 20.0F, .cw_db = -10.0F}).is_ok());

    const auto runner = run(waveform.audio);
    const auto score = score_burst(runner, waveform.bursts.front(), 0U);
    const auto error_rate = 1.0F - as_real(score.correct) / as_real(std::max<std::size_t>(score.compared, 1U));
    std::println("CW on the carrier at -10 dB, SNR 20 dB in 3 kHz: locked by symbol {}, symbol errors {:.2f}%", score.locked_from, 100.0F * error_rate);
    CHECK(score.locked);
    CHECK(error_rate < 0.10F);
}

TEST_CASE("Watterson fading at 2 ms and 1 Hz with 20 dB SNR: what the front end alone delivers", "[front_end][channel][.report]")
{
    auto waveform = hi_waveform();
    REQUIRE(apply_channel(waveform.audio, {.snr_db = 20.0F, .fading = FadingSpec{.delay_ms = 2.0F, .doppler_spread_hz = 1.0F}}).is_ok());

    const auto runner = run(waveform.audio);
    const auto score = score_burst(runner, waveform.bursts.front(), 0U);
    std::println("Watterson 2 ms / 1 Hz, SNR 20 dB in 3 kHz (engineering model, not conformance): locked {}, {} of {} decisions match", score.locked, score.correct, score.compared);
    SUCCEED("reported; the equalizer, in a later article, is what makes fading decodable");
}

TEST_CASE("Thirty minutes of continuous traffic: still exact past 2^26 samples", "[front_end][.long]")
{
    constexpr std::uint64_t thirty_minutes = 30U * 60U * 48000U;
    FrontEndRunner runner;
    REQUIRE(runner.initialize().is_ok());

    std::size_t bursts{};
    std::size_t clean_bursts{};
    WaveformSpec spec;

    while (runner.samples_fed() < thirty_minutes)
    {
        spec.seed = bursts + 1U;
        const auto waveform = make_test_waveform(spec);
        REQUIRE(waveform.error().is_ok());

        const auto first_sample = runner.samples_fed();
        REQUIRE(runner.feed(waveform.value().audio).is_ok());
        const auto score = score_burst(runner, waveform.value().bursts.front(), first_sample);
        clean_bursts += score.locked && score.correct == score.compared ? 1U : 0U;
        CHECK(score.locked);
        CHECK(score.correct == score.compared);
        runner.discard_symbols();
        ++bursts;
    }

    std::println("{} bursts back to back, {} samples ({} s); {} with every decision right", bursts, runner.samples_fed(), runner.samples_fed() / 48000U, clean_bursts);
    CHECK(runner.samples_fed() > (std::uint64_t{1} << 26U));
    CHECK(clean_bursts == bursts);
}
