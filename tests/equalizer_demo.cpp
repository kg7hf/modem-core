// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// equalizer_demo - how far the channel reaches, and two ways to undo it
// =============================================================================
// This is the companion program to the Signal Path article "The equalizer", as
// first_path_demo is to "The first path". Its steps run on Part 5's recordings (the
// "Hi" message through Table XX's two-path channels) and on one clean burst from the
// transmitter. In article order: how far behind the anchor the channel reaches, the
// window sized from that, training on the preamble with its frequency guesses, the
// adaptive equalizer through a 5 ms body frame by frame, the CIR-MMSE equalizer through
// the same body, and the frames where the two part company.
//
// Each step is one function in tests/walkthrough/equalizer_steps.cpp; print() shows what
// it found. equalizer_walkthrough_tests checks the same steps, so set a breakpoint in a
// step and step through it from either one.
//
//   Build:  cmake -S . -B build -DMODEM_ARTICLE_DATA=ON && cmake --build build
//   Run:    ./build/equalizer_demo
//           ./build/equalizer_demo --data-dir <the unpacked first-path-v1 folder>
// =============================================================================

#include "modem/common/convert.hpp"
#include "modem/common/status.hpp"
#include "tests/walkthrough/equalizer_steps.hpp"

#include <cstdio>
#include <print>
#include <span>
#include <string>
#include <string_view>

using namespace modem_test::equalizer_steps;
using ::modem::common::Result;

namespace
{
// Prints a step's numbers, or says why it could not run.
template <typename Step> bool show(const Result<Step>& step, const std::string& diagnostic)
{
    if (!step)
    {
        std::println(stderr, "   failed: {}{}{}", step.error().message, diagnostic.empty() ? "" : ": ", diagnostic);
        return false;
    }
    print(step.value());
    return true;
}
} // namespace

int main(int argc, char** argv)
{
    std::string root = MODEM_FIRST_PATH_DATA;
    const std::span<char*> args(argv, ::modem::common::narrow<std::size_t>(argc));
    if (args.size() == 3U && std::string_view{args[1]} == "--data-dir")
    {
        root = args[2];
    }
    else if (args.size() != 1U)
    {
        std::println(stderr, "usage: equalizer_demo [--data-dir <the first-path-v1 folder>]");
        return 2;
    }

    std::string diagnostic;
    const auto pack = open_pack(root, diagnostic);
    if (!pack)
    {
        std::println(stderr, "equalizer_demo: the recordings are not usable: {}\n  configure with -DMODEM_ARTICLE_DATA=ON, or give --data-dir", diagnostic);
        return 2;
    }

    std::println("== Equalizer walkthrough, on the article's recordings ==\n");
    bool ok = true;

    // 1. How far behind the anchor. The known prefix correlated at lags 0 to 15 behind the
    //    anchor, summed over the preamble's segments, sidelobes removed, the largest
    //    significant lag kept.
    std::println("1. How far behind the anchor");
    ok = show(delay_spread(pack.value(), diagnostic), diagnostic) && ok;

    // 2. The window, sized from that measurement and never from the mode.
    std::println("\n2. A window sized from the measurement");
    ok = show(window(pack.value(), diagnostic), diagnostic) && ok;

    // 3. Training on the preamble: a one-tap guess, three passes over the known symbols, and
    //    a check on the frequency, each guess scored on the preamble's tail.
    std::println("\n3. Training on the preamble");
    ok = show(training(pack.value(), diagnostic), diagnostic) && ok;

    // 4. Following the channel: the adaptive equalizer through the 5 ms body.
    std::println("\n4. Following the channel");
    ok = show(adaptive(pack.value(), diagnostic), diagnostic) && ok;

    // 5. Measuring the channel: the CIR-MMSE equalizer through the same body.
    std::println("\n5. Measuring the channel");
    ok = show(cir_mmse(pack.value(), diagnostic), diagnostic) && ok;

    // 6. When the channel moves: the frames where the two engines differ most.
    std::println("\n6. When the channel moves");
    ok = show(fade(pack.value(), diagnostic), diagnostic) && ok;

    return ok ? 0 : 1;
}
