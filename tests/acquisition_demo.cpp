// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// acquisition_demo - find the "Hi" burst in noise and read its mode and countdown
// =============================================================================
// This is the companion program to the Signal Path article "Acquisition: finding
// the burst", as front_end_demo is to "The audio front end". It works in the
// 600 bps, long-interleave mode ("600L") and runs the article's steps in order:
// the preamble the transmitter sends, the reference the receiver looks for,
// correlating in blocks so a fade cannot hide it, the comb of matches in a noisy
// burst, the carrier offset, reading the mode and the countdown, waking up
// mid-preamble, refusing a mode it does not implement, letting every segment
// vote, and noise with no burst at all.
//
// Each step is one function in tests/walkthrough/acquisition_steps.cpp that runs
// a piece of acquisition on known data and returns its numbers; print() shows
// them. acquisition_tests calls the same functions and checks what they return,
// so set a breakpoint in a step and step through it from either one.
//
//   Build:  cmake -B build && cmake --build build
//   Run:    ./build/acquisition_demo
// =============================================================================

#include "modem/common/status.hpp"
#include "tests/walkthrough/acquisition_steps.hpp"

#include <cstdio>
#include <print>

using namespace modem_test::acquisition_steps;
using ::modem::common::Result;

namespace
{
// Prints a step's numbers, or says why it could not run.
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
} // namespace

int main()
{
    std::println("== Acquisition walkthrough (600 bps, long interleave) ==\n");

    // 1. The preamble, read back from the transmitter's own symbols: 24 segments, each 15
    //    channel symbols of 32 chips. Nine fixed symbols, the mode (D1, D2), and a count of
    //    the segments still to come (C1, C2, C3), from 23 down to 0.
    std::println("1. The preamble the transmitter sends");
    if (!show(preamble_segments()))
    {
        return 1;
    }

    // 2. The reference: the nine fixed channel symbols, spread and scrambled exactly as the
    //    transmitter does it. 288 points, the same at the start of every segment.
    std::println("\n2. The reference the receiver looks for");
    if (!show(prefix_reference()))
    {
        return 1;
    }

    // 3. Correlating in blocks. One sum over all 288 chips is cancelled by a fade that turns
    //    the path's phase partway through; nine 32-chip sums added by magnitude are not.
    std::println("\n3. One sum against nine blocks, when a fade turns the phase");
    print(fading_prefix());

    // 4. The "Hi" burst, half a second into a noisy line with the carrier 37 Hz off, through
    //    Part 3's front end. Correlating at every start finds a match every 480 symbols, one
    //    per segment, and each match carries its own estimate of the carrier offset.
    std::println("\n4. A comb of matches, one per segment");
    if (!show(correlation_comb()))
    {
        return 1;
    }

    // 5. How far off the carrier can be. The estimate is the angle between neighbouring
    //    symbols, so it is unambiguous to half the symbol rate; acquisition accepts 120 Hz.
    std::println("\n5. The carrier offset estimate, and its range");
    if (!show(carrier_range()))
    {
        return 1;
    }

    // 6. The first start through both gates, read: D1 and D2 name the mode, and the
    //    countdown says how many segments remain, so where the body starts.
    std::println("\n6. Reading the mode and the countdown");
    if (!show(read_segment()))
    {
        return 1;
    }

    // 7. A receiver that wakes up in the middle of the preamble still finds the body: the
    //    countdown of the first whole segment it hears says how far away it is.
    std::println("\n7. Waking up mid-preamble");
    if (!show(wake_mid_preamble()))
    {
        return 1;
    }

    // 8. A burst whose D1 and D2 name a mode this modem does not implement: the standard says
    //    not to synchronize, and to keep searching.
    std::println("\n8. A mode this modem does not implement");
    if (!show(unsupported_mode()))
    {
        return 1;
    }

    // 9. At -4 dB, some segments read wrong on their own, and more cannot be read at all.
    //    Every segment carries the same mode and a consistent body boundary, so acquisition
    //    groups the readings by the burst they describe and goes with the largest group.
    std::println("\n9. One segment, or all of them");
    if (!show(segment_vote(-4.0F)))
    {
        return 1;
    }

    // 10. Ten seconds of noise and no burst: nothing reaches the correlation floor.
    std::println("\n10. No burst at all");
    if (!show(noise_only()))
    {
        return 1;
    }

    // 11. Part 3's hang-up: a loop started half a symbol off the peaks can take longer than
    //     a short preamble to pull in. The receiver runs a phase search while it acquires,
    //     which moves such a loop half a symbol, onto the peaks.
    std::println("\n11. Part 3's hang-up, fixed while acquiring");
    if (!show(phase_search()))
    {
        return 1;
    }

    // 12. The clock offset Part 3 left open. The type-1 loop sits off the symbol peaks by the
    //     drift over its gain, and at 1000 ppm it cannot hold. With a rate integrator running
    //     from the first candidate, it sits on them.
    std::println("\n12. The clock offset: type 1 against type 2");
    if (!show(clock_offset()))
    {
        return 1;
    }

    // 13. How long the integrator takes to learn the clock: in noise the loop pulls in far
    //     more slowly, which is why the receiver judges it late, on a mean.
    std::println("\n13. Learning the clock, against the signal-to-noise ratio");
    if (!show(learning_time()))
    {
        return 1;
    }

    // 14. A long burst through a slow fade: the integrator's momentary rate swings with the
    //     fade, clock or no clock. Its mean from 5 s to 20 s tells them apart, so the receiver
    //     judges the mean and retires an integrator that has no clock to carry.
    std::println("\n14. The rule for a long burst");
    if (!show(long_burst_rule()))
    {
        return 1;
    }
    return 0;
}
