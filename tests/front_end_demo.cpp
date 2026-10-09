// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// front_end_demo - walk the message "Hi" back out through the receive front end
// =============================================================================
// This is the companion program to the Signal Path article "The audio front end:
// finding the symbols", as tx_demo is to "The transmit chain". It works in the
// 600 bps, long-interleave mode ("600L") and runs the article's steps in order:
// the carrier, a pure tone, one symbol mixed down, the matched filter, the timing
// error, the timing position, the front end over the whole "Hi" burst, carrier
// tracking, and the symbols walked back out to Part 2's randomizer.
//
// Each step is one function in tests/walkthrough/front_end_steps.cpp that runs a
// piece of the front end on known data and returns its numbers; print() shows
// them. front_end_tests calls the same functions and checks what they return, so
// set a breakpoint in a step and step through it from either one.
//
//   Build:  cmake -B build && cmake --build build
//   Run:    ./build/front_end_demo
// =============================================================================

#include "modem/common/status.hpp"
#include "tests/walkthrough/front_end_steps.hpp"

#include <cstdio>
#include <print>

using namespace modem_test::front_end_steps;
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
    std::println("== Receive front end walkthrough (600 bps, long interleave) ==\n");

    // 1. The carrier. 1800 Hz at 48 kHz turns 13.5 degrees a sample and is back where it
    //    started after 80 samples (three cycles), so 80 constants hold all of it. The
    //    transmitter modulates with this table, and the receiver mixes with it.
    std::println("1. The 1800 Hz carrier: 80 samples, three whole cycles");
    print(carrier_table());

    // 2. A pure tone through the front end. On the carrier it mixes down to 0 Hz and comes
    //    out a constant; ten hertz off, it comes out turning by 10 Hz, 2 pi x 10 / 2400
    //    radians a symbol.
    std::println("\n2. A tone on the carrier mixes down to a constant; one 10 Hz off turns");
    if (!show(tone_mix(1800U)) || !show(tone_mix(1810U)))
    {
        return 1;
    }

    // 3. One symbol, mixed down by hand. Body symbol 5 of the "Hi" burst is tribit 6, Part 2's
    //    worked symbol at 270 degrees. Each mixed sample is the symbol plus a copy turning the
    //    other way at 3600 Hz; the matched filter averages the copy away and leaves one point.
    std::println("\n3. Part 2's tribit 6, mixed down from 1800 Hz");
    if (!show(tribit6_mix()))
    {
        return 1;
    }

    // 4. The matched filter: 161 root-raised-cosine taps. Then the transmit pulse through it,
    //    and how much of one symbol lands on its neighbors' instants, for a receive rolloff
    //    of 0.25 (matched to the transmitter) and of 0.35 (what the front end uses).
    std::println("\n4. The matched filter, and what it leaves at the other symbols' instants");
    if (!show(receive_filter()) || !show(pulse_neighbors(0.25F)) || !show(pulse_neighbors(0.35F)))
    {
        return 1;
    }

    // 5. The early-late timing error on the receive pulse: zero on the peak, positive when the
    //    loop reads early, negative when it reads late, and the same whatever the carrier phase.
    std::println("\n5. The early-late timing error, on and off the peak");
    if (!show(timing_error()))
    {
        return 1;
    }

    // 6. Why the timing position is whole samples plus a fraction. A float holds 24 bits; 23
    //    minutes in, it cannot even add the 20 samples of one symbol.
    std::println("\n6. Keeping time for hours: a float position against whole samples plus a fraction");
    print(float_position());

    // 7. The real front end over the whole "Hi" burst, 480 samples at a time, and again in
    //    ragged blocks of 1, 7, 33, 160 and 480 samples: the same symbols, bit for bit.
    std::println("\n7. Any block size, the same symbols");
    if (!show(block_sizes()))
    {
        return 1;
    }

    // 8. Carrier tracking with the receiver's gains: given symbols turned by a fixed 0.5 rad it
    //    learns 0.5 rad, and given symbols turning at 5 Hz it learns 5 Hz.
    std::println("\n8. Carrier tracking, part one");
    if (!show(carrier_tracking()))
    {
        return 1;
    }

    // 9. Walk the symbols back out: the nearest of the eight phases, minus Part 2's randomizer,
    //    is the BPSK data, tribit 0 for a coded 0 and tribit 4 for a coded 1.
    std::println("\n9. Walk the symbols back out");
    if (!show(walk()))
    {
        return 1;
    }
    return 0;
}
