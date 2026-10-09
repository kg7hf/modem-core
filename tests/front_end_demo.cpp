// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// front_end_demo - walk the receive front end, step by step
// =============================================================================
// The companion program to the Signal Path article "The audio front end: finding
// the symbols", as tx_demo is to "The transmit chain". Each numbered step runs one
// piece of the front end on known data (the 600L "Hi" burst from the real
// transmitter, a pure tone, the receive pulse) and prints the numbers the article
// quotes. The steps live in tests/walkthrough/front_end_steps.cpp, and
// front_end_tests checks the same functions with Catch2, so set a breakpoint in a
// step and step through it from either one.
//
//   Build:  cmake -B build && cmake --build build
//   Run:    ./build/front_end_demo        (every step)
//           ./build/front_end_demo 3      (one step; "all" runs every step)
// =============================================================================

#include "modem/common/convert.hpp"
#include "modem/common/status.hpp"
#include "tests/walkthrough/front_end_steps.hpp"

#include <array>
#include <charconv>
#include <cstdio>
#include <print>
#include <span>
#include <string_view>

namespace
{

using namespace ::modem_test::front_end_steps;
using ::modem::common::narrow;
using ::modem::common::Result;

// Prints a step's numbers, or why it could not run.
template <typename Step> bool show(const Result<Step>& step)
{
    if (!step)
    {
        std::println(stderr, "   failed: {}", step.error().message);
        return false;
    }
    print(step.value());
    return true;
}

struct DemoStep
{
    std::string_view title;
    bool (*run)();
};

constexpr std::array<DemoStep, 9> steps{{
    {"The 1800 Hz carrier: 80 samples, three whole cycles",
     []
     {
         print(carrier_table());
         return true;
     }},
    {"A tone on the carrier mixes down to a constant; one 10 Hz off turns", [] { return show(tone_mix(1800U)) && show(tone_mix(1810U)); }},
    {"Part 2's tribit 6, mixed down from 1800 Hz", [] { return show(tribit6_mix()); }},
    {"The matched filter, and what it leaves at the other symbols' instants", [] { return show(receive_filter()) && show(pulse_neighbors(0.25F)) && show(pulse_neighbors(0.35F)); }},
    {"The early-late timing error, on and off the peak", [] { return show(timing_error()); }},
    {"Keeping time for hours: a float position against whole samples plus a fraction",
     []
     {
         print(float_position());
         return true;
     }},
    {"Any block size, the same symbols", [] { return show(block_sizes()); }},
    {"Carrier tracking, part one", [] { return show(carrier_tracking()); }},
    {"Walk the symbols back out", [] { return show(walk()); }},
}};

} // namespace

int main(int argc, char** argv)
{
    const std::span<char*> args(argv, narrow<std::size_t>(argc));
    std::size_t only{};
    if (args.size() > 1U && std::string_view{args[1]} != "all")
    {
        const std::string_view text = args[1];
        const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), only);
        if (error != std::errc{} || end != text.data() + text.size() || only < 1U || only > steps.size())
        {
            std::println(stderr, "usage: front_end_demo [step 1-{} | all]", steps.size());
            return 1;
        }
    }

    std::println("== Receive front end walkthrough (600L, the message \"Hi\") ==\n");
    bool all_ran = true;
    for (std::size_t k = 0U; k < steps.size(); ++k)
    {
        if (only != 0U && only != k + 1U)
        {
            continue;
        }
        std::println("{}. {}", k + 1U, steps[k].title);
        all_ran = steps[k].run() && all_ran;
        std::println("");
    }
    return all_ran ? 0 : 1;
}
