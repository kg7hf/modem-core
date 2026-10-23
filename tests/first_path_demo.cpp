// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// first_path_demo - two paths, and which one acquisition anchors on
// =============================================================================
// This is the companion program to the Signal Path article "The first path", as
// acquisition_demo is to "Acquisition: finding the burst". Its steps run on the
// article's recordings: the "Hi" message through Table XX's two-path channels,
// rendered with pathsim and selected because each shows one situation the article
// explains. In article order: two paths make two groups, the window that decides how
// close an earlier path may be, a direct path with no group of its own, the
// reference's own sidelobes, a later path's sidelobe and the walk, and where in a
// symbol the burst arrives.
//
// Each step is one function in tests/walkthrough/first_path_steps.cpp; print() shows
// what it found. first_path_tests checks every recording against its manifest's
// expectations, so set a breakpoint in a step and step through it from either one.
//
//   Build:  cmake -S . -B build -DMODEM_ARTICLE_DATA=ON && cmake --build build
//   Run:    ./build/first_path_demo
//           ./build/first_path_demo --data-dir <the unpacked first-path-v1 folder>
// =============================================================================

#include "modem/common/convert.hpp"
#include "modem/common/status.hpp"
#include "tests/walkthrough/first_path_steps.hpp"

#include <cstdio>
#include <print>
#include <span>
#include <string>
#include <string_view>

using namespace modem_test::first_path_steps;
using ::modem::common::Result;

namespace
{
// Prints a step's numbers, or says why it could not run.
template <typename Step, typename Show> bool show(const Result<Step>& step, const std::string& diagnostic, Show print_step)
{
    if (!step)
    {
        std::println(stderr, "   failed: {}{}{}", step.error().message, diagnostic.empty() ? "" : ": ", diagnostic);
        return false;
    }
    print_step(step.value());
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
        std::println(stderr, "usage: first_path_demo [--data-dir <the first-path-v1 folder>]");
        return 2;
    }

    std::string diagnostic;
    const auto pack = open_pack(root, diagnostic);
    if (!pack)
    {
        std::println(stderr, "first_path_demo: the recordings are not usable: {}\n  configure with -DMODEM_ARTICLE_DATA=ON, or give --data-dir", diagnostic);
        return 2;
    }

    std::println("== First-path walkthrough, on the article's recordings ==\n");

    // 1. Two paths, two groups. Each path delivers its own copy of the preamble, so the scan
    //    forms a group for each, a path delay apart, and the strongest is the echo's here.
    std::println("1. Two paths, two groups");
    if (!show(two_groups(pack.value(), diagnostic), diagnostic, print_two_groups))
    {
        return 1;
    }

    // 2. How close an earlier path may be before the anchor moves to it: from 7 symbols for
    //    the adaptive equalizer, which reaches a precursor 6 symbols ahead, from 3 for the
    //    host's turbo decoder.
    std::println("\n2. The window");
    if (!show(anchor_window(pack.value(), diagnostic), diagnostic, [](const AnchorWindow& step) { print(step); }))
    {
        return 1;
    }

    // 3. A direct path faded through the segments the scan covers forms no group. The
    //    precursor search looks before the anchor over every segment, finds it, and walks
    //    the segments until one reads on it.
    std::println("\n3. A path with no group of its own");
    if (!show(faded_direct(pack.value(), diagnostic), diagnostic, print_faded_direct))
    {
        return 1;
    }

    // 4. Before lag 0 the reference correlates with itself, and with the end of the segment
    //    before it. The search subtracts that, modelled with the tail. (No recording: one
    //    clean path, straight from the transmitter.)
    std::println("\n4. The reference's own sidelobes");
    if (!show(sidelobe_model(), diagnostic, [](const SidelobeModel& step) { print(step); }))
    {
        return 1;
    }

    // 5. The anchor already on the direct path, the echo 12 symbols behind it: the echo's
    //    own sidelobe shows as a peak 11 symbols ahead, where nothing was sent, and the
    //    walk's tests reject it.
    std::println("\n5. A later path's sidelobe, and the walk");
    if (!show(sidelobe_path(pack.value(), diagnostic), diagnostic, print_sidelobe_path))
    {
        return 1;
    }

    // 6. A weak direct path, and the burst arriving at each phase of a symbol, with the
    //    short preamble's three segments and the long one's twenty-four.
    std::println("\n6. Where in a symbol the burst arrives");
    if (!show(arrival_phase(pack.value(), diagnostic), diagnostic, [](const ArrivalPhase& step) { print(step); }))
    {
        return 1;
    }
    return 0;
}
