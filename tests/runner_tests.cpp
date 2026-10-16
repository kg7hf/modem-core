// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// runner_tests - the receive chain end to end, as the runner runs it
// =============================================================================
// The runner is how a reader runs the code on a message or a recording of their own, so
// its chain is tested like any stage: the WAV file in and out, the walkthrough's burst
// found where the bench put it, the clock learned, and a saved recording read back.
// =============================================================================

#include "modem/common/convert.hpp"
#include "modem/common/types.hpp"
#include "tests/support/receive_chain.hpp"
#include "tests/support/wav_file.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <print>
#include <string>
#include <system_error>
#include <vector>

using namespace modem_test;
using ::modem::common::as_real;
using ::modem::common::BodyInterleave;
using ::modem::common::DataRate;

namespace
{

// A file in the system's temporary folder (the current folder if there is none) for a test
// that writes one, removed when the test is done with it.
class TemporaryFile
{
public:
    explicit TemporaryFile(const char* name)
    {
        std::error_code error;
        auto folder = std::filesystem::temp_directory_path(error);
        if (error)
        {
            folder = ".";
        }
        mPath = (folder / name).string();
    }

    TemporaryFile(const TemporaryFile&) = delete;
    TemporaryFile& operator=(const TemporaryFile&) = delete;

    ~TemporaryFile()
    {
        std::error_code error;
        std::filesystem::remove(mPath, error);
    }

    [[nodiscard]] const std::string& path() const noexcept { return mPath; }

private:
    std::string mPath;
};

// The walkthrough's line: "Hi" in 600L, noise 10 dB down, the carrier 37 Hz off, seed 4.
ChainSpec walkthrough_spec()
{
    ChainSpec spec;
    spec.channel.snr_db = 10.0F;
    spec.channel.carrier_offset_hz = 37;
    spec.channel.seed = 4U;
    return spec;
}

} // namespace

TEST_CASE("A WAV file written and read back holds the same audio to 16 bits", "[runner][wav]")
{
    std::vector<float> audio(4800U);
    for (std::size_t n = 0U; n < audio.size(); ++n)
    {
        audio[n] = 0.9F * std::sin(as_real(n) * 0.0471F);
    }
    const TemporaryFile file("runner_tests_round_trip.wav");
    REQUIRE(write_wav(file.path(), audio, 48000U).is_ok());
    const auto read = read_wav(file.path());
    REQUIRE(read.has_value());
    CHECK(read.value().sample_rate_hz == 48000U);
    CHECK(read.value().channels == 1U);
    REQUIRE(read.value().samples.size() == audio.size());
    float largest_error{};
    for (std::size_t n = 0U; n < audio.size(); ++n)
    {
        largest_error = std::max(largest_error, std::fabs(read.value().samples[n] - audio[n]));
    }
    // Half a step of rounding, and up to one more: the writer's full scale is 32767 (so 1 does
    // not wrap) and the reader's is 32768 (so -32768 reads as -1).
    constexpr float step = 1.0F / 32768.0F;
    std::println("4800 samples through a 16-bit WAV file: largest error {:.6f}, {:.2f} of a 16-bit step", largest_error, largest_error / step);
    CHECK(largest_error <= 1.5F * step);
}

TEST_CASE("A file that is not a WAV file is refused", "[runner][wav]")
{
    const TemporaryFile text("runner_tests_not_a_wav.wav");
    {
        std::ofstream file(text.path(), std::ios::binary);
        file << "This is a text file, not a RIFF WAVE file.";
    }
    CHECK_FALSE(read_wav(text.path()).has_value());
    const TemporaryFile missing("runner_tests_no_such_file.wav");
    CHECK_FALSE(read_wav(missing.path()).has_value());
}

TEST_CASE("The modes parse as the runner's --mode names them", "[runner][mode]")
{
    CHECK(parse_mode("600L").value().data_rate == DataRate::bps600);
    CHECK(parse_mode("600L").value().interleave == BodyInterleave::long_block);
    CHECK(parse_mode("75S").value().interleave == BodyInterleave::short_block);
    CHECK(parse_mode("4800").value().interleave == BodyInterleave::zero);
    CHECK_FALSE(parse_mode("4800L").has_value());
    CHECK_FALSE(parse_mode("3200S").has_value());
    CHECK_FALSE(parse_mode("600X").has_value());
    CHECK(mode_text(parse_mode("2400S").value()) == "2400S");
}

TEST_CASE("The runner finds the walkthrough's burst where the bench put it", "[runner][chain]")
{
    const auto report = run_chain(walkthrough_spec());
    REQUIRE(report.has_value());
    print(report.value());
    const auto& acquisition = report.value().acquisition;
    REQUIRE(acquisition.acquired.has_value());
    CHECK(acquisition.acquired->recognition.mode.data_rate == DataRate::bps600);
    CHECK(acquisition.acquired->recognition.mode.interleave == BodyInterleave::long_block);
    CHECK(acquisition.acquired->first_preamble_symbol == acquisition.true_first_preamble.value());
    CHECK(acquisition.acquired->first_body_symbol == acquisition.true_first_body.value());
    CHECK(std::fabs(acquisition.acquired->frequency_offset_hz - 37.0F) < 5.0F);
}

TEST_CASE("At 1000 ppm Part 3's loop slips and the committed receiver learns the clock", "[runner][chain][clock]")
{
    auto spec = walkthrough_spec();
    spec.channel.snr_db = 20.0F;
    spec.channel.carrier_offset_hz.reset();
    spec.channel.clock_ppm = 1000.0F;
    const auto report = run_chain(spec);
    REQUIRE(report.has_value());
    print(report.value());
    const auto& front_end = report.value().front_end;
    const auto& acquisition = report.value().acquisition;
    REQUIRE(acquisition.acquired.has_value());
    REQUIRE(front_end.grid.has_value());
    REQUIRE(acquisition.grid.has_value());
    // Part 3's proportional loop cannot make the standing error 1000 ppm needs, so it slips.
    CHECK(front_end.grid->slipped_symbols != 0);
    // Committed at the first candidate, the search off and the integrator tracking: the clock
    // learned, the loop on the peaks, no slip.
    CHECK(std::fabs(acquisition.learned_clock_ppm - 1000.0) < 50.0);
    CHECK(std::fabs(acquisition.grid->mean_offset_samples) < 0.5);
    CHECK(acquisition.grid->slipped_symbols == 0);
}

TEST_CASE("A recording the runner saved gives the same burst when read back", "[runner][chain][wav]")
{
    const TemporaryFile heard("runner_tests_heard.wav");
    auto spec = walkthrough_spec();
    spec.wav_out = heard.path();
    const auto generated = run_chain(spec);
    REQUIRE(generated.has_value());
    REQUIRE(generated.value().acquisition.acquired.has_value());

    ChainSpec recorded;
    recorded.wav_in = spec.wav_out;
    const auto read_back = run_chain(recorded);
    REQUIRE(read_back.has_value());
    print(read_back.value());
    REQUIRE(read_back.value().acquisition.acquired.has_value());
    const auto& first = *generated.value().acquisition.acquired;
    const auto& second = *read_back.value().acquisition.acquired;
    CHECK(second.recognition.mode.data_rate == first.recognition.mode.data_rate);
    CHECK(second.recognition.mode.interleave == first.recognition.mode.interleave);
    CHECK(second.first_preamble_symbol == first.first_preamble_symbol);
    CHECK(second.first_body_symbol == first.first_body_symbol);
}
