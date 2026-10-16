// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// runner - the receive chain end to end, on your own message or your own recording
// =============================================================================
// The articles each explain one stage, with a demo that walks it on known data. The
// runner runs every stage published so far, in order, on what you give it, and prints
// what each one found:
//
//   Part 2  the transmitter turns your message into a burst of 48 kHz audio,
//           the channel adds noise, a carrier offset, a clock offset, fading,
//   Part 3  the front end turns the audio into one complex symbol per symbol,
//   Part 4  acquisition finds the burst, reads its mode and countdown, and the
//           front end's rate integrator learns the clock from there.
//
// Each article adds its stage, so the runner grows with the series. Give it a WAV file
// instead (48 kHz, 16-bit or float) and it runs the receive stages on that: a recording
// of a real transmission, or one it saved with --save.
//
//   Build:  cmake -B build && cmake --build build
//   Run:    ./build/runner "Hi" --mode 600L --snr 10 --carrier 37
//           ./build/runner "Hi" --clock 1000 --save heard.wav
//           ./build/runner --wav heard.wav
// =============================================================================

#include "modem/common/convert.hpp"
#include "modem/common/status.hpp"
#include "tests/support/receive_chain.hpp"

#include <charconv>
#include <cstdio>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>

using namespace modem_test;

namespace
{

constexpr std::string_view usage = R"(usage: runner [message] [options]
  message            the text to send (default "Hi")
  --mode M           75S 75L 150S 150L 300S 300L 600S 600L 1200S 1200L 2400S 2400L 4800 (default 600L)
  --snr DB           white noise, signal over noise in a 3 kHz band
  --carrier HZ       the carrier offset, whole hertz
  --clock PPM        the transmitter's sample-clock offset
  --fade MS,HZ       two-path Watterson fading: the second path's delay and the Doppler spread
  --seed N           the channel's seed (default 4, the walkthrough's)
  --save FILE.wav    save the audio the receiver is given
  --wav FILE.wav     run the receive stages on a recording instead (48 kHz)
)";

// A number from the command line, all of it, or nothing.
template <typename Number> std::optional<Number> number(std::string_view text)
{
    if (text.starts_with('+') && text.size() > 1U && text[1] != '-') // from_chars takes a minus sign, not a plus
    {
        text.remove_prefix(1U);
    }
    Number value{};
    const auto* end = text.data() + text.size();
    const auto [stop, error] = std::from_chars(text.data(), end, value);
    if (error != std::errc{} || stop != end)
    {
        return std::nullopt;
    }
    return value;
}

std::optional<FadingSpec> fading(std::string_view text)
{
    const auto comma = text.find(',');
    if (comma == std::string_view::npos)
    {
        return std::nullopt;
    }
    const auto delay = number<float>(text.substr(0U, comma));
    const auto doppler = number<float>(text.substr(comma + 1U));
    if (!delay || !doppler)
    {
        return std::nullopt;
    }
    return FadingSpec{.delay_ms = *delay, .doppler_spread_hz = *doppler};
}

// One option and its value into the spec; false for an option the runner does not know, or
// a value it cannot read.
bool apply_option(ChainSpec& spec, std::string_view option, std::string_view value)
{
    if (option == "--mode")
    {
        const auto mode = parse_mode(std::string{value});
        spec.mode = mode.value_or(spec.mode);
        return mode.has_value();
    }
    if (option == "--snr")
    {
        spec.channel.snr_db = number<float>(value);
        return spec.channel.snr_db.has_value();
    }
    if (option == "--carrier")
    {
        spec.channel.carrier_offset_hz = number<std::int32_t>(value);
        return spec.channel.carrier_offset_hz.has_value();
    }
    if (option == "--clock")
    {
        spec.channel.clock_ppm = number<float>(value);
        return spec.channel.clock_ppm.has_value();
    }
    if (option == "--fade")
    {
        spec.channel.fading = fading(value);
        return spec.channel.fading.has_value();
    }
    if (option == "--seed")
    {
        const auto seed = number<std::uint64_t>(value);
        spec.channel.seed = seed.value_or(spec.channel.seed);
        return seed.has_value();
    }
    if (option == "--save")
    {
        spec.wav_out = std::string{value};
        return true;
    }
    if (option == "--wav")
    {
        spec.wav_in = std::string{value};
        return true;
    }
    return false;
}

// The command line into a ChainSpec, or nothing (and a message) if it does not parse.
std::optional<ChainSpec> parse(std::span<char*> args)
{
    ChainSpec spec;
    spec.channel.seed = 4U; // the walkthrough's: "--snr 10 --carrier 37" hears the article's burst
    for (std::size_t index = 1U; index < args.size(); ++index)
    {
        const std::string_view arg = args[index];
        if (!arg.starts_with("--"))
        {
            spec.message = arg;
            continue;
        }
        const bool has_value = index + 1U < args.size();
        const std::string_view value = has_value ? std::string_view{args[index + 1U]} : std::string_view{};
        if (!has_value || !apply_option(spec, arg, value))
        {
            std::println(stderr, "runner: cannot read {} {}\n{}", arg, value, usage);
            return std::nullopt;
        }
        ++index;
    }
    return spec;
}

} // namespace

int main(int argc, char** argv)
{
    const std::span<char*> args(argv, ::modem::common::narrow<std::size_t>(argc));
    if (args.size() > 1U && (std::string_view{args[1]} == "--help" || std::string_view{args[1]} == "-h"))
    {
        std::print("{}", usage);
        return 0;
    }
    const auto spec = parse(args);
    if (!spec)
    {
        return 2;
    }
    const auto report = run_chain(*spec);
    if (!report)
    {
        std::println(stderr, "runner: {}", report.error().message);
        return 1;
    }
    print(report.value());
    return report.value().acquisition.acquired ? 0 : 1;
}
