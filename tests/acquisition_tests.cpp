// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// acquisition_tests.cpp - acquisition's tests
// =============================================================================
// Each test feeds known data to acquisition and checks what it found. The signals
// come from the real transmitter (support/test_waveform), the impairments from the
// channel stage (support/test_channel), the symbols from Part 3's front end
// (support/front_end_harness), and the truth of where each burst sits from what was
// sent. Run them from VS Code's Testing panel or with ctest; set a breakpoint and
// debug any one of them.
// Explained in the article series, Part 4 (acquisition: finding the burst).
// =============================================================================

#include "modem/common/convert.hpp"
#include "modem/m110a/body_waveform.hpp"
#include "modem/m110a/synchronization.hpp"
#include "tests/support/front_end_harness.hpp"
#include "tests/support/test_channel.hpp"
#include "tests/support/test_waveform.hpp"
#include "tests/walkthrough/acquisition_steps.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

namespace
{

using namespace ::modem_test;
using ::modem::common::as_real;
using ::modem::common::BodyInterleave;
using ::modem::common::DataRate;
using ::modem::common::IQSample;
using ::modem::common::narrow;
using ::modem::m110a::BodyAcquisitionConfig;
using ::modem::m110a::BodyDesignatorState;
using ::modem::m110a::BodyMode;

constexpr std::size_t segment_symbols = ::modem::m110a::body_preamble_segment_symbols;
constexpr std::size_t lead_samples = 24010U; // half a second of line before the burst

// The operating point of the streaming receiver later in the series, as the walkthrough uses it.
BodyAcquisitionConfig receiver_config() noexcept
{
    BodyAcquisitionConfig config;
    config.minimum_correlation = 0.35F;
    return config;
}

// A burst after half a second of line, through the channel and the front end: the symbols
// acquisition searches, and where the burst really is in them.
struct Received
{
    std::vector<IQSample> symbols{};
    std::size_t first_preamble{};
    std::size_t first_body{};
};

Received receive(const WaveformSpec& spec, const ChannelSpec& channel)
{
    auto waveform = make_test_waveform(spec);
    REQUIRE(waveform.error().is_ok());
    auto& audio = waveform.value().audio;
    audio.insert(audio.begin(), lead_samples, 0.0F);
    REQUIRE(apply_channel(audio, channel).is_ok());

    FrontEndRunner runner;
    REQUIRE(runner.initialize().is_ok());
    REQUIRE(runner.feed(audio).is_ok());
    const auto& burst = waveform.value().bursts.front();
    Received received;
    received.symbols = on_time_symbols(runner);
    received.first_preamble = narrow<std::size_t>(locate_burst(runner, burst, lead_samples));
    received.first_body = received.first_preamble + burst.preamble_symbols;
    return received;
}

ChannelSpec noisy(float snr_db, std::int32_t carrier_offset_hz, std::uint64_t seed) noexcept
{
    ChannelSpec channel;
    channel.snr_db = snr_db;
    channel.carrier_offset_hz = carrier_offset_hz;
    channel.seed = seed;
    return channel;
}

} // namespace

// The article's walkthrough: each case runs one step from tests/walkthrough/acquisition_steps
// (the same functions acquisition_demo prints, step by step) and checks what it returned.
//   acquisition_tests "[walkthrough]" --order decl
// prints every number the article quotes, in the article's order.

// --- The preamble ---

TEST_CASE("The 600L preamble: 24 segments of nine fixed symbols, the mode and a countdown", "[acquisition][preamble][walkthrough]")
{
    const auto step = acquisition_steps::preamble_segments();
    REQUIRE(step.error().is_ok());
    acquisition_steps::print(step.value());
    const auto& s = step.value();
    CHECK(s.segments == 24U);
    CHECK(s.every_chip_is_a_code);
    CHECK(s.prefix_on_every_segment);
    CHECK(s.countdown_runs_down);
    CHECK(s.d1 == 4U);
    CHECK(s.d2 == 6U);
    CHECK(s.recognition.state == BodyDesignatorState::supported);
    CHECK(s.recognition.mode.data_rate == DataRate::bps600);
    CHECK(s.recognition.mode.interleave == BodyInterleave::long_block);
}

// --- The reference ---

TEST_CASE("The reference is the first 288 symbols of every segment, bit for bit", "[acquisition][reference][walkthrough]")
{
    const auto step = acquisition_steps::prefix_reference();
    REQUIRE(step.error().is_ok());
    acquisition_steps::print(step.value());
    CHECK(step.value().chips == 288U);
    CHECK(step.value().matches_every_segment);
    // Channel symbol 0's code is all zeros, so the first chips are the sync randomizer's own.
    for (const auto& row : step.value().shown)
    {
        CHECK(row.code == 0U);
        CHECK(row.tribit == row.randomizer);
    }
}

// --- Correlating so a fade does not hide it ---

TEST_CASE("A fade that turns the phase cancels one long sum but not nine block sums", "[acquisition][correlation][walkthrough]")
{
    const auto step = acquisition_steps::fading_prefix();
    acquisition_steps::print(step);
    CHECK(std::fabs(step.clean_coherent - 1.0F) < 1.0e-5F);
    CHECK(std::fabs(step.clean_blocks - 1.0F) < 1.0e-5F);
    CHECK(std::fabs(step.flipped_coherent - 32.0F / 288.0F) < 1.0e-5F); // 160 chips one way, 128 the other
    CHECK(std::fabs(step.flipped_blocks - 1.0F) < 1.0e-5F);
}

TEST_CASE("A noisy burst: a match at the start of each segment, each with the carrier offset", "[acquisition][correlation][walkthrough]")
{
    const auto step = acquisition_steps::correlation_comb();
    REQUIRE(step.error().is_ok());
    acquisition_steps::print(step.value());
    const auto& s = step.value();
    std::size_t segment_starts{};
    for (const auto& peak : s.peaks)
    {
        if (peak.start >= 0)
        {
            // Inside the burst, only the segment starts reach the floor.
            CHECK(peak.start % narrow<std::ptrdiff_t>(segment_symbols) == 0);
            CHECK(peak.correlation > 0.9F);
            CHECK(std::fabs(peak.frequency_hz - as_real(s.carrier_offset_hz)) < 5.0F);
            ++segment_starts;
        }
        else
        {
            // Before it, the burst's own prefix seen early: weak, and never a whole match.
            CHECK(peak.correlation < 0.5F);
        }
    }
    CHECK(segment_starts == 3U);
    CHECK(s.largest_elsewhere < 0.35F);
}

// --- The carrier offset ---

TEST_CASE("The carrier estimate is exact to half the symbol rate and gated at 120 Hz", "[acquisition][carrier][walkthrough]")
{
    const auto step = acquisition_steps::carrier_range();
    REQUIRE(step.error().is_ok());
    acquisition_steps::print(step.value());
    for (const auto& row : step.value().rows)
    {
        // An angle between neighbours wraps at half a turn: 1200 Hz at 2400 symbols a second.
        const auto expected = row.true_hz > 1200.0F ? row.true_hz - 2400.0F : row.true_hz;
        CHECK(std::fabs(row.estimated_hz - expected) < 0.1F);
        CHECK(row.accepted == (std::fabs(row.true_hz) <= 120.0F));
    }
}

// --- Reading the mode and the countdown ---

TEST_CASE("The first valid segment names 600L and counts down to the body", "[acquisition][countdown][walkthrough]")
{
    const auto step = acquisition_steps::read_segment();
    REQUIRE(step.error().is_ok());
    acquisition_steps::print(step.value());
    const auto& s = step.value();
    CHECK(s.start == s.truth_preamble);
    CHECK(s.d1 == 4U);
    CHECK(s.d2 == 6U);
    CHECK(s.countdown == 23U);
    CHECK(s.first_body_symbol == s.truth_body);
    CHECK(s.first_preamble_symbol == s.truth_preamble);
    // Every start the third gate turned away lies just before the burst: its own prefix, early.
    for (const auto& rejected : s.rejected)
    {
        CHECK(rejected.start < s.truth_preamble);
        CHECK(s.truth_preamble - rejected.start <= 300U);
    }
}

TEST_CASE("Waking up mid-preamble: the countdown still finds the body", "[acquisition][countdown][walkthrough]")
{
    const auto step = acquisition_steps::wake_mid_preamble();
    REQUIRE(step.error().is_ok());
    acquisition_steps::print(step.value());
    const auto& s = step.value();
    CHECK(s.first_body_symbol == s.truth_body);
    CHECK(s.countdown == 23U - s.segment_read);
    CHECK_FALSE(s.preamble_start_in_buffer);
}

TEST_CASE("A mode this modem does not implement is not acquired", "[acquisition][countdown][walkthrough]")
{
    const auto step = acquisition_steps::unsupported_mode();
    REQUIRE(step.error().is_ok());
    acquisition_steps::print(step.value());
    const auto& s = step.value();
    CHECK(s.as_sent == BodyDesignatorState::supported);
    CHECK(s.as_sent_acquired);
    CHECK(s.patched == BodyDesignatorState::recognized_unsupported);
    CHECK_FALSE(s.patched_acquired);
}

// --- One segment, or all of them ---

TEST_CASE("At -4 dB the segments outvote a misread", "[acquisition][vote][walkthrough]")
{
    const auto step = acquisition_steps::segment_vote(-4.0F);
    REQUIRE(step.error().is_ok());
    acquisition_steps::print(step.value());
    const auto& s = step.value();
    CHECK(s.right + s.wrong + s.invalid == 24U);
    CHECK(s.right > 2U * s.wrong);
    REQUIRE_FALSE(s.groups.empty());
    // The largest group is the burst as sent.
    const auto largest = std::ranges::max_element(s.groups, {}, &acquisition_steps::VoteGroup::supporters);
    CHECK(largest->supporters == s.right);
    CHECK(s.acquired);
    CHECK(s.acquisition_right);
}

// --- No burst at all ---

TEST_CASE("Ten seconds of noise: nothing reaches the floor and nothing is acquired", "[acquisition][noise][walkthrough]")
{
    const auto step = acquisition_steps::noise_only();
    REQUIRE(step.error().is_ok());
    acquisition_steps::print(step.value());
    CHECK(step.value().largest_correlation < 0.35F);
    CHECK(step.value().over_floor == 0U);
    CHECK_FALSE(step.value().acquired);
}

// --- Part 3's hang-up, fixed while acquiring ---

TEST_CASE("With the phase search the loop pulls in within the short preamble from every start phase", "[acquisition][phase_search][walkthrough]")
{
    const auto step = acquisition_steps::phase_search();
    REQUIRE(step.error().is_ok());
    acquisition_steps::print(step.value());
    const auto& s = step.value();
    CHECK(s.searched_in_time == s.start_phases);
    CHECK(s.plain_in_time < s.start_phases); // the hang-up is there to fix
    for (const auto& start : s.late_starts)
    {
        CAPTURE(start.lead);
        CHECK(start.searched_pulled_in_at.has_value());
        CHECK(start.searched_moves >= 1U);
    }
}

// --- The clock-offset loop ---

TEST_CASE("Type 1 sits off the peaks by the drift over its gain and slips at 1000 ppm; type 2 sits on them", "[acquisition][clock][walkthrough]")
{
    const auto step = acquisition_steps::clock_offset();
    REQUIRE(step.error().is_ok());
    acquisition_steps::print(step.value());
    for (const auto& row : step.value().rows)
    {
        CAPTURE(row.clock_ppm, row.type_2, std::to_underlying(row.mode.interleave));
        if (!row.type_2 && std::fabs(row.clock_ppm) < 500.0F)
        {
            // The drift over the gain: 0.002 / (0.08 x 0.034), about 0.74 samples, on the side
            // the clock pushes it.
            CHECK(std::fabs(row.body_offset) > 0.5);
            CHECK(std::fabs(row.body_offset) < 0.9);
            CHECK((row.body_offset > 0.0) == (row.clock_ppm > 0.0F));
            CHECK(row.slips == 0U);
        }
        else if (!row.type_2)
        {
            CHECK(row.slips > 10U); // it cannot hold the offset 1000 ppm needs
        }
        else
        {
            REQUIRE(row.learned_ppm_off.has_value());
            CHECK(row.slips == 0U);
            CHECK(std::fabs(row.body_offset) < 0.5);
            CHECK(std::fabs(*row.learned_ppm_off) < 100.0F);
        }
    }
}

TEST_CASE("In noise the integrator takes far longer than 2880 symbols to learn the clock", "[acquisition][clock][walkthrough]")
{
    // Why a fixed window failed: at 20 dB the rate is within 100 ppm well inside 2880 symbols of
    // the first candidate; near a short mode's working SNR it takes several times as long.
    const auto step = acquisition_steps::learning_time();
    REQUIRE(step.error().is_ok());
    acquisition_steps::print(step.value());
    const auto& rows = step.value().rows;
    REQUIRE(rows.size() == 4U);
    const auto& clean = rows.front();
    const auto& noisy = rows.back();
    // At 20 dB every seed settles within about the window; at 4 dB none does, and at the window
    // the rate is still hundreds of ppm off in at least one seed.
    for (const auto& settled : clean.settled_at)
    {
        REQUIRE(settled.has_value());
        CHECK(*settled < 3600U);
    }
    float worst_noisy_error{};
    for (std::size_t seed = 0U; seed < noisy.settled_at.size(); ++seed)
    {
        CAPTURE(seed);
        CHECK((!noisy.settled_at[seed].has_value() || *noisy.settled_at[seed] > 2880U));
        worst_noisy_error = std::max(worst_noisy_error, std::fabs(noisy.error_at_window_ppm[seed]));
    }
    CHECK(worst_noisy_error > 300.0F);
}

TEST_CASE("On a long burst the mean rate tells no clock from a real one, where the momentary rate cannot", "[acquisition][clock][walkthrough]")
{
    const auto step = acquisition_steps::long_burst_rule();
    REQUIRE(step.error().is_ok());
    acquisition_steps::print(step.value());
    for (const auto& row : step.value().rows)
    {
        CAPTURE(row.clock_ppm, row.seed);
        // The momentary rate swings with the fade whether or not there is a clock; the mean
        // over 15 s is what tells them apart.
        CHECK(row.largest_rate_ppm > row.judged_mean_ppm);
        if (row.clock_ppm == 0.0F)
        {
            CHECK(row.retired);
            CHECK(row.largest_rate_ppm > step.value().threshold_ppm);
        }
        else
        {
            CHECK_FALSE(row.retired);
            CHECK(row.judged_mean_ppm > 175.0);
        }
    }
}

// --- Beyond the walkthrough ---

TEST_CASE("Every mode is found and read at 10 dB with the carrier 75 Hz off", "[acquisition][modes]")
{
    const auto mode = GENERATE(BodyMode{DataRate::bps75, BodyInterleave::short_block}, BodyMode{DataRate::bps75, BodyInterleave::long_block}, BodyMode{DataRate::bps150, BodyInterleave::short_block},
                               BodyMode{DataRate::bps150, BodyInterleave::long_block}, BodyMode{DataRate::bps300, BodyInterleave::short_block}, BodyMode{DataRate::bps300, BodyInterleave::long_block},
                               BodyMode{DataRate::bps600, BodyInterleave::short_block}, BodyMode{DataRate::bps600, BodyInterleave::long_block},
                               BodyMode{DataRate::bps1200, BodyInterleave::short_block}, BodyMode{DataRate::bps1200, BodyInterleave::long_block},
                               BodyMode{DataRate::bps2400, BodyInterleave::short_block}, BodyMode{DataRate::bps2400, BodyInterleave::long_block}, BodyMode{DataRate::bps4800, BodyInterleave::zero});
    const auto offset_hz = GENERATE(-75, 75);
    CAPTURE(std::to_underlying(mode.data_rate), std::to_underlying(mode.interleave), offset_hz);

    WaveformSpec spec;
    spec.mode = mode;
    spec.seed = 7U;
    const auto received = receive(spec, noisy(10.0F, offset_hz, 3U));
    const auto acquired = ::modem::m110a::acquire_body_preamble(received.symbols, receiver_config());
    REQUIRE(acquired.has_value());
    const auto& a = acquired.value();
    CHECK(a.recognition.state == BodyDesignatorState::supported);
    CHECK(a.recognition.mode.data_rate == mode.data_rate);
    CHECK(a.recognition.mode.interleave == mode.interleave);
    CHECK(a.first_body_symbol == received.first_body);
    CHECK(a.first_preamble_symbol == received.first_preamble);
    // One segment's estimate: a few hertz at 10 dB, which the carrier tracker then takes up.
    CHECK(std::fabs(a.frequency_offset_hz - as_real(offset_hz)) < 5.0F);
}

TEST_CASE("The carrier gate: 110 Hz off is found, 150 Hz off is not", "[acquisition][carrier]")
{
    const auto offset_hz = GENERATE(-150, -110, 110, 150);
    CAPTURE(offset_hz);
    WaveformSpec spec;
    spec.mode = BodyMode{DataRate::bps600, BodyInterleave::short_block};
    const auto received = receive(spec, noisy(10.0F, offset_hz, 5U));
    const auto acquired = ::modem::m110a::acquire_body_preamble(received.symbols, receiver_config());
    if (std::abs(offset_hz) <= 120)
    {
        REQUIRE(acquired.has_value());
        CHECK(acquired.value().first_body_symbol == received.first_body);
    }
    else
    {
        CHECK_FALSE(acquired.has_value());
    }
}

TEST_CASE("Waking anywhere in a long preamble with a whole segment left finds the body", "[acquisition][countdown]")
{
    // Where the receiver starts listening, in segments of the 24.
    const auto wake = GENERATE(0.5F, 3.0F, 11.7F, 19.2F, 22.9F);
    CAPTURE(wake);
    WaveformSpec spec;
    spec.payload = {'H', 'i'};
    const auto received = receive(spec, noisy(10.0F, 37, 4U));
    const auto cut = received.first_preamble + ::modem::common::round_to<std::size_t>(wake * as_real(segment_symbols));
    const auto acquired = ::modem::m110a::acquire_body_preamble(std::span<const IQSample>{received.symbols}.subspan(cut), receiver_config());
    REQUIRE(acquired.has_value());
    CHECK(acquired.value().first_body_symbol + cut == received.first_body);
    CHECK(acquired.value().recognition.mode.data_rate == DataRate::bps600);
}
