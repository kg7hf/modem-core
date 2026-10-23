# The first path

*Part 5 of "Building a MIL-STD-188-110 HF Modem in Modern C++." Start with the [overview](01-introduction.md), Part 1, [the rules I build by](02-embedded-cpp-philosophy.md), Part 2, [the transmit chain](03-transmit-chain.md), Part 3, [the audio front end](04-audio-front-end.md), or Part 4, [acquisition](05-acquisition.md). The code for this article is on GitHub: [kg7hf/modem-core](https://github.com/kg7hf/modem-core/tree/part-5).*

Part 4 found the burst on a line with one path. HF seldom has one. The signal reaches the receiver by more than one route through the ionosphere, and each route delivers its own copy of the burst, a few milliseconds after the one before. MIL-STD-188-110B specifies its serial-tone performance on exactly that. In 5.3.2.5, Table XX, the fading channels are two independent Rayleigh paths of equal average power: 2 ms apart for 600 to 2400 bps, and 5 ms apart for 75 to 300 bps and one of the 2400 bps rows. At 2400 symbols a second, that is 4.8 symbols and 12.

The transmitter sends the burst once; the second copy is the channel's doing. Each copy carries the whole preamble, so acquisition can find either one, and the standard does not say which one a receiver should synchronize to. That is an implementation choice, and this article is mine: the receiver anchors on the first path, not the strongest. It covers why, how the receiver finds the first path even when the scan formed no group for it, how close a path has to be before the anchor moves to it, and what the choice measured.

Part 4 had `acquisition_demo`; this article has `first_path_demo`. Its steps run on recordings that are not in the repository: they are published beside it as a release, and the build fetches them when asked. Configure with the article data on, build, run it, and keep its output open beside this article:

```bash
cmake -S . -B build -DMODEM_ARTICLE_DATA=ON
cmake --build build
./build/first_path_demo
```

With `MODEM_ARTICLE_DATA` on, CMake downloads `testdata-first-path-v1.zip` from the repository's releases, checks its size and SHA-256 against the ones recorded in `tests/data/first-path/pack.json`, and unpacks it into `test-data/first-path-v1/` beside the source, where every build of the checkout finds it. An ordinary build downloads nothing. To fetch it by hand, download the same archive from the release page, check its hash, and unpack it in the same place; `first_path_demo --data-dir` reads it from anywhere else.

The steps run on recordings, not on a channel the program simulates: the "Hi" message from Parts 2 to 4, sent through Table XX's two-path channels with pathsim, in the mode each channel is specified for. I chose each recording because it shows one situation this article explains, so they are examples, not statistics; the statistics come from the campaign later in this article.

| Step | Function | Section |
|---|---|---|
| 1 | `two_groups()` | Two paths, two groups |
| 2 | `anchor_window()` | An earlier group, and the window |
| 3 | `faded_direct()` | A path with no group of its own |
| 4 | `sidelobe_model()` | The reference's own sidelobes |
| 5 | `sidelobe_path()` | Peaks, and the walk |
| 6 | `arrival_phase()` | A weak first path, and where the samples fall |

As in Parts 3 and 4, `tests/first_path_tests.cpp` calls the same functions and checks what they returned.

## Since Part 4

Each part also brings the code already published up to date. Three changes since Part 4:

- **The transmitter's pulse is a table.** Part 3 ended its section on the compiler with two loose ends: the root-raised-cosine taps were still computed with the library's cosine when the modem started, and Windows had not been checked. The pulse is now a table of exact values, `modem/m110a/body_shaping_pulse.hpp`, written once by `tools/generate_shaping_pulse.py`, and a test checks it against the run-time design to within 1e-6. With nothing left for a math library to round, the "Hi" burst's audio is the same bits on Windows and Linux, built with GCC and with Clang, and the golden test now checks every sample of it, as a hash of the float bits, instead of five decimal places. The receiver's matched filter is still designed when the modem starts; that is next.
- **The units have their tests.** The strong quantity types published with Part 2 (decibels, frequencies, sample rates and counts) come with their tests now. Those tests ran in an older harness of my own, and they move to Catch2 one suite at a time; this is the second.
- **Test names.** Catch2 reads a comma in a test spec as "or", so a test whose name has a comma is hard to select by name. Five of Part 4's had one; they are renamed, and the rule is in the repository's contributing guide.

## Two paths, two groups

Part 4's scan takes every start through three gates, the carrier offset, the correlation and a valid read, and groups the candidates that name the same mode and body boundary. With one path, every valid read names the same boundary. With two, the second path's copy of the preamble passes the same gates a path delay later, and its reads name a boundary a path delay later too. Each path forms a group of its own, and `strongest_group()` takes the one with the most segments behind it.

A receiver does not wait for the whole preamble before it commits. In this modem, the receiver that decodes a whole recorded burst tests starts only in the first 1200 symbols it is given, half a second; given a recording that starts with the burst, that is the first two and a half segments. The walkthrough scans the same 1200 symbols of each burst. The rest of the preamble is there for the vote and for the search later in this article.

Step 1, `two_groups()`, runs the scan on the first recording: 300 bps with the long interleave, on the Table's 300 bps channel, two paths 5 ms apart fading at 5 Hz, 7 dB.

```
fp1-echo-group, 300L on Table XX's 300 bps channel: two paths 5 ms (12 symbols) apart, fading at 5 Hz, 7 dB
the scan's groups (starts in the burst's first 1200 symbols):
  body boundary, from the direct path's   supporters  correlation sum  best correlation
  +12.0 symbols, the echo                          3            2.013             0.910  strongest
  +0.0 symbols, the direct path                    1            0.734             0.734
without the anchoring: the echo (body +12.0 symbols from the direct path's)
with it (the window from 7): the direct path (body +0.0 symbols from the direct path's), moved by an earlier group
```

Through the segments the scan covers, the direct path was the weaker of the two, and the echo's group has more segments behind it. Without the anchoring, acquisition returns the echo's group, as the code published with Part 4 did.

## Why the first path

Anchoring on one path does not mean listening to only one. The anchor is the equalizer's time reference, where its window of taps begins. Before training, the receiver measures how far behind the anchor the later paths arrive (the delay spread, Part 6), sizes the equalizer's window to reach them, and the equalizer gathers the energy of every path inside it: when one path fades, the other still carries the symbols. That is why the anchor goes on the first path. The window starts at the first arrival and extends to cover the later one.

The equalizer works out each symbol from the samples around the anchor, and it treats the two sides differently. Energy that arrives after a symbol's time carries symbols it has already decided, and it can take that energy out using those decisions. Energy that arrives before carries symbols it has not decided yet, and it reaches only a few symbols that way: 6, in the adaptive equalizer's short geometry. Anchored on the direct path, the echo arrives 12 symbols late, on the side the equalizer handles with decisions it already has. Anchored on the echo, the direct path arrives 12 symbols early, beyond what the equalizer reaches, and half the signal is interference on every symbol.

The strongest path is also a judgement about a moment. At Table XX's 5 Hz fading bandwidth the two paths trade places several times a second, so the strongest group says which path was louder while the scan looked, through the first half second of a burst that lasts several seconds. The first path stays first for the whole burst: its place is set by the length of the route, not by the fading.

Table XX's channel is a model: two paths a few milliseconds apart, each fading on its own, as one-hop and two-hop modes or the two magneto-ionic modes do on a real link. The waveform is built for that. The known symbols that follow every frame of data, 16 or 20 of them (MIL-STD-188-110B 5.3.2.2.2), let a receiver re-measure a channel a few milliseconds long. A long path arriving tens of milliseconds behind the short path is a different case: at 2400 symbols a second that is a couple of hundred symbols, beyond what those probes can follow, and this receiver does not combine the two. The anchoring window ends at 15 symbols for that reason. Beyond it, the receiver keeps the strongest group, and the other copy is interference. How often real links put a usable second path within reach, and how that changes through a session, is a question for recordings off the air, not for a simulator.

The full sequence of `acquire_body_preamble()`, of which Part 4 showed the first two steps and one vote:

```cpp
    const auto strongest = strongest_group(scan);

    if (!config.anchor_earliest_path)
    {
        return representative_of(symbol_rate_input, reference, scan.groups[strongest], config);
    }

    const auto earliest = representative_of(symbol_rate_input, reference, scan.groups[earliest_path_group(scan, strongest, config)], config);
    auto winner = vote_mode_and_boundary(symbol_rate_input, reference, earliest, config);
    anchor_to_precursor_path(symbol_rate_input, reference, config, winner);
    return vote_mode_and_boundary(symbol_rate_input, reference, winner, config);
```

After the scan and `strongest_group()`, `earliest_path_group()` looks for an earlier group, `vote_mode_and_boundary()` is Part 4's vote, `anchor_to_precursor_path()` looks for an earlier path that formed no group, and the vote runs again on the new anchor. `anchor_earliest_path` is on by default.

One thing changed in the part Part 4 showed: a group no longer keeps its best member whole. The anchoring's diagnostics travel with every acquisition result, which made each result about four times larger, and the scan's table of 32 groups would have needed about 7 KB of stack in one function; the repository's checks hold every function in the modem to 4 KB, the bounded stack of Part 1. So a group keeps what the grouping compares and the start where its best member was found, and `representative_of()` runs `evaluate_start()` at that start again. The function depends on nothing but its arguments, so it gives back the same member, bit for bit, and the frame drops from 8,080 bytes to 2,896 (GCC 14).

## An earlier group, and the window

`earliest_path_group()` looks for a second group of the same mode whose body boundary comes before the winner's:

```cpp
    for (std::size_t group = 0U; group < scan.count; ++group)
    {
        const auto& candidate = scan.groups[group].representative;
        const auto& earliest = scan.groups[earliest_group].representative;

        if (same_mode(candidate, chosen) && candidate.first_body_symbol < earliest.first_body_symbol &&
            chosen.first_body_symbol - candidate.first_body_symbol <= config.anchor_maximum_path_delay_symbols &&
            chosen.first_body_symbol - candidate.first_body_symbol >= config.anchor_minimum_path_delay_symbols)
        {
            earliest_group = group;
        }
    }
```

A group that close is the same burst through an earlier path. To form a group at all, a start has to clear the 288-chip correlation floor and read a valid mode and countdown (Part 4), so a group a few symbols early is a second path, not noise. The window is set in `BodyAcquisitionConfig`:

```cpp
    bool anchor_earliest_path{true};
    std::size_t anchor_maximum_path_delay_symbols{15U};
    // Re-anchor only when the earlier path lies BEYOND the adaptive equalizer's
    // precursor reach (its short geometry: 24 T/2 taps, cursor 12 = 6 symbols).
    // Inside that reach the adaptive equalizer already handles a precursor path
    // (the 2 ms rows of Table XX, 4.8 symbols), and moving the anchor there only
    // re-rolls a marginal equalizer. The 5 ms rows sit at 12 symbols, far beyond
    // it. The host's whole-burst turbo decoder needs the first path even at 2 ms,
    // and runs with 3.
    std::size_t anchor_minimum_path_delay_symbols{7U};
```

The maximum, 15 symbols, is a little beyond Table XX's longest delay of 12. The minimum depends on the equalizer that follows. The adaptive equalizer's short geometry has 24 taps half a symbol apart with its reference tap at 12, so it reaches 6 symbols ahead: a path 7 symbols or more ahead is beyond its taps, and a closer one is within them already. The host's whole-burst turbo decoder needs the anchor on the first path even when the paths are 2 ms apart, so the host runs acquisition with the minimum at 3.

Step 2, `anchor_window()`, runs both windows on the first recording, 5 ms, and on a second: 600 bps with the long interleave on the Table's 600 bps channel, two paths 2 ms apart fading at 1 Hz, 7 dB.

```
the anchor, with the window from 7 symbols and from 3:
fp1-echo-group, 300L on Table XX's 300 bps channel: two paths 5 ms (12 symbols) apart, fading at 5 Hz, 7 dB
  without the anchoring:  the echo (body +12.0 symbols from the direct path's)
  the window from 7:      the direct path (body +0.0 symbols from the direct path's), moved by an earlier group
  the window from 3:      the direct path (body +0.0 symbols from the direct path's), moved by an earlier group
fp2-window, 600L on Table XX's 600 bps channel: two paths 2 ms (4.8 symbols) apart, fading at 1 Hz, 7 dB
  without the anchoring:  the echo (body +4.9 symbols from the direct path's)
  the window from 7:      the echo (body +4.9 symbols from the direct path's), not moved
  the window from 3:      the direct path (body -0.1 symbols from the direct path's), moved by the precursor search, 5 symbols
```

At 5 ms, both windows move the anchor 12 symbols to the direct path. At 2 ms, the paths are 4.8 symbols apart, inside the adaptive equalizer's reach: the window from 7 leaves the anchor on the echo, and the window from 3 moves it. In this recording the direct path formed no group of its own, so the move at 2 ms is made by the precursor search later in this article; the same minimum bounds that search.

I measured that difference with a control, a condition where I expected the adaptive receiver to gain nothing: the Table's 1200 bps row, two paths 2 ms apart fading at 1 Hz, at 11 dB, 100 bursts through pathsim, each decoded with the anchoring and without it. With the window from 3, the anchoring moved the anchor 5 symbols earlier in 30 of the 100 bursts. The whole-burst turbo decoder went from 71 exact bursts to 100; in the 30 moved bursts, from 1 to 30. The adaptive receiver gained nothing: 95 exact bursts with the anchoring against 96 without, and in the 30 moved bursts 189 bit errors against 17. With the window from 7, the anchoring never moved the anchor, and both receivers' output was bit-identical to having no anchoring at all. So the window follows the equalizer: from 3 for the turbo decoder, from 7 for the adaptive one.

## A path with no group of its own

The scan covers two and a half segments. A direct path in a fade through exactly those forms no group, though on average it carries half the burst's energy. `anchor_to_precursor_path()` looks for it in every segment. At each lag from 0 to 15 symbols before the anchor, it correlates the prefix in Part 4's nine blocks of 32 chips, adds the blocks' powers, and sums them over every segment from the winner's detected one to the body boundary:

```cpp
        for (std::size_t lag = 0U; lag <= maximum_lag; ++lag)
        {
            float power = 0.0F;

            for (std::size_t block = 0U; block < prefix_blocks; ++block)
            {
                IQSample correlation{};

                for (std::size_t chip = 0U; chip < channel_symbol_chips; ++chip)
                {
                    const auto index = block * channel_symbol_chips + chip;
                    correlation += input[segment_start - lag + index] * rotated_reference[index];
                }

                power += std::norm(correlation);
            }

            precursor_powers[lag] += power;
        }
```

Lag 0 is the anchored path itself; a path 12 symbols earlier shows at lag 12. The reference is turned by the winner's carrier estimate first, `rotated_reference`, so the block sums line up. Summing powers over the segments, not the complex correlations, is what lets a fading path register: its phase can turn from one segment to the next, but its power adds.

Step 3, `faded_direct()`, runs on a third recording on the Table's 300 bps channel, chosen because its direct path fades through the segments the scan covers and comes back after them.

```
fp3-faded-direct, 300L on Table XX's 300 bps channel: two paths 5 ms (12 symbols) apart, fading at 5 Hz, 7 dB
the scan's groups (starts in the burst's first 1200 symbols):
  body boundary, from the direct path's   supporters  correlation sum  best correlation
  +12.0 symbols, the echo                          2            1.585             0.826  strongest
without the anchoring: the echo (body +12.0 symbols from the direct path's)
the precursor profile over 23 segments, as a fraction of the anchored path (lag 0):
  lag     1     2     3     4     5     6     7     8     9    10    11    12    13    14    15
      0.024 0.033 0.098 0.016 0.027 0.051 0.030 0.010 0.015 0.011 0.035 0.909 0.068 0.010 0.024
peaks at or above 1/8: 1, the earliest at lag 12 (0.909)
the walk: 3 segments tried; rejected by the carrier gate 0, the correlation floor 2, an invalid read 0, another mode 0
with the anchoring (the window from 7): the direct path (body +0.0 symbols from the direct path's), moved by the precursor search, 12 symbols
the prefix correlation at the segment it read on: 0.697
```

The scan formed one group, the echo's. The profile, summed over the whole preamble, shows the direct path at lag 12, at 0.91 of the anchored echo. The walk rejects the first two segments it tries at the correlation floor, where the direct path was still in its fade, and reads on the third; the anchor moves 12 symbols, to the direct path.

## The reference's own sidelobes

Before lag 0 the profile is not zero, even on one path. Shifted by a few symbols, the prefix correlates partly with itself, and with the end of the segment before it. That end is known: every segment ends with channel symbol 0, whose chips are the bare sync-randomizer values. `body_precursor_sidelobe_powers()` computes the profile the reference alone produces, with the previous segment's last 15 chips in front of the prefix, and the search subtracts it, scaled to the anchored path's power:

```cpp
    for (std::size_t lag = 1U; lag <= maximum_lag; ++lag)
    {
        precursor_powers[lag] = std::max(0.0F, precursor_powers[lag] - (sidelobe_powers[lag] / sidelobe_powers[0]) * precursor_powers[0]);
    }
```

Step 4, `sidelobe_model()`, needs no recording: one clean path, straight from the transmitter. It prints the profile as measured, what a model of the prefix alone leaves after the subtraction, and what the model with the tail leaves.

```
one clean path, 24 segments; precursor power as a fraction of lag 0 (* at or above 1/8):
  lag  measured  less the prefix's sidelobes  less the model with the tail
    1    0.062                        0.000                          0.000
    2    0.003                        0.000                          0.000
    3    0.012                        0.000                          0.000
    4    0.004                        0.000                          0.000
    5    0.034                        0.000                          0.000
    6    0.064                        0.000                          0.000
    7    0.015                        0.000                          0.000
    8    0.010                        0.000                          0.000
    9    0.055                        0.006                          0.000
   10    0.013                        0.002                          0.000
   11    0.024                        0.002                          0.000
   12    0.007                        0.000                          0.000
   13    0.011                        0.003                          0.000
   14    0.029                        0.008                          0.000
   15    0.008                        0.001                          0.000
```

On one clean path the reference's own sidelobes reach 0.064 of lag 0, at lag 6. The model with the tail leaves nothing of them; the prefix alone leaves a remainder, largest at lag 14, which is what the tail is in the model for.

## Peaks, and the walk

`precursor_peaks()` takes the profile's peaks, not every lag over the floor: a lag whose power is at least 1/8 of lag 0 and no less than either neighbour's, earliest first, up to four. The pulse puts a path's power partly into the lags beside it, so the earliest lag over the floor can be a symbol early; and a peak far out can clear the floor ahead of the real path, so every peak is tried in turn. A peak is still only evidence. `walk_to_earlier_path()` tests it on the signal: from the winner's detected segment, it shifts back by the peak's lag and tests one segment at a time up to the body boundary:

```cpp
        const auto segment_frequency_radians = estimate_body_prefix_frequency_radians(input, segment_start, reference);
        const auto segment_frequency_hz = segment_frequency_radians * config.symbol_rate_hz / two_pi;
        ++winner.anchor.walk_segments;

        if (std::fabs(segment_frequency_hz) > config.maximum_frequency_offset_hz)
        {
            ++winner.anchor.walk_rejected_frequency;
            continue;
        }

        BodyAcquisition shifted{};
        shifted.first_preamble_symbol = segment_start;
        shifted.frequency_offset_hz = segment_frequency_hz;
        float phase{};
        shifted.normalized_correlation = body_prefix_correlation(input, segment_start, reference, segment_frequency_radians, phase);
        shifted.carrier_phase_radians = phase;

        // The designator/countdown decode combines echoes over lags 0..15, so it
        // validates any anchor up to a path delay EARLY as readily as the true
        // one; the prefix correlation at the shifted start is the discriminating
        // test (a genuine path clears the acquisition floor there, a phantom
        // does not). A faded segment simply fails it and the walk moves on.
        if (shifted.normalized_correlation < config.minimum_correlation)
        {
            ++winner.anchor.walk_rejected_correlation;
            continue;
        }

        shifted = characterize_body_candidate(input, shifted, config);
```

Each shifted start gets its own carrier estimate (the winner's came from the other path, and a fade can bias it by tens of hertz), the 0.35 correlation floor, and a valid read of the same mode. The first segment that passes moves the anchor. The correlation is the test that matters. The designators and the countdown, read from a start a few symbols early, still read correctly, because the decode combines the signal over lags 0 to 15; a valid read does not prove the anchor right. A real path clears the correlation floor at its own start, and a sidelobe does not.

Step 5, `sidelobe_path()`, runs on a recording where the anchor is already on the direct path, with the echo 12 symbols behind it. Shifted 23 symbols back, into the previous segment's closing channel symbol, the prefix correlates with that known tail again; 23 is beyond the 15 lags the search looks at, so the anchored path's model has no entry there. A path d symbols behind the anchor puts that sidelobe 23 - d symbols ahead of it: 11, for the 5 ms rows. Nothing was sent there.

```
fp5-sidelobe-path, 300L on Table XX's 300 bps channel: two paths 5 ms (12 symbols) apart, fading at 5 Hz, 7 dB
the scan's groups (starts in the burst's first 1200 symbols):
  body boundary, from the direct path's   supporters  correlation sum  best correlation
  +0.0 symbols, the direct path                    2            1.274             0.806  strongest
  +12.0 symbols, the echo                          1            0.498             0.498
without the anchoring: the direct path (body +0.0 symbols from the direct path's)
the precursor profile over 23 segments, as a fraction of the anchored path (lag 0):
  lag     1     2     3     4     5     6     7     8     9    10    11    12    13    14    15
      0.034 0.059 0.027 0.020 0.045 0.064 0.045 0.026 0.023 0.067 0.127 0.042 0.032 0.061 0.039
peaks at or above 1/8: 1, the earliest at lag 11 (0.127)
the walk: 23 segments tried; rejected by the carrier gate 21, the correlation floor 2, an invalid read 0, another mode 0
the largest prefix correlation where the walk tested that lag: 0.220
with the anchoring (the window from 7): the direct path (body +0.0 symbols from the direct path's), not moved
```

The profile shows a peak at 11, at 0.127 of the anchor, just above 1/8. The walk tries 23 segments there and rejects every one: at 21 the carrier estimate falls outside the gate, and at the other 2 the correlation is under the floor; the largest correlation at any of those starts is 0.22. The anchor stays on the direct path. Then `vote_mode_and_boundary()` runs once more, on the anchor's own segments, as Part 4's vote did.

## A weak first path, and where the samples fall

The walk's tests are what reject a sidelobe, and its correlation floor can also reject a real path. A direct path that is weak through the whole preamble still shows in the profile, summed over every segment, but no single segment of it may clear the floor at a shifted start. Whether one does can depend on where the burst's samples fall within a symbol: the front end reads each symbol at one point, 20 samples apart, and a weak path arriving between those points reads lower.

The streaming receiver (Part 8) met exactly that. A 600 bps, short-interleave recording on the Table's 600 bps channel, two paths 2 ms apart, decoded exactly at only 4 of the 20 sample phases at which the burst can arrive; at the other 16, the direct path's segments did not clear the correlation floor, and the receiver anchored 5 symbols late. The streaming receiver now finds such a path from the profile alone, without the floor: it takes the profile's peak when it reaches 1/8 of the anchor and four times the profile's median, and that recording decodes at all 20 phases.

Without the walk, the profile alone has to separate a path from a sidelobe, and the first version of that change did not always. On the Table's 2400 bps row with the paths 5 ms apart, one test burst anchored 11 symbols before any signal: the later path's sidelobe at 23 - 12 had passed as a path. The streaming receiver now measures the profile on the later side of the anchor as well, and takes each significant later path's own sidelobes off every earlier lag before it chooses a peak. A higher floor for distant peaks was tried first and dropped: it also rejected real weak paths 12 symbols ahead, at 0.16 to 0.35 of the anchor, in bursts that decode only when anchored on them. The sidelobe sat at lag 11 and the real paths at 12, and their power could not tell them apart; the model of the sidelobes can.

Step 6, `arrival_phase()`, shows the phase on this article's receiver. It runs one recording, a weak direct path on the Table's 600 bps channel, at each of the 20 phases: for phase p it puts p samples of silence in front of the whole recording, and moves the expected answer with it. It runs the short-interleave burst, with three segments to walk, and the same channel with the long interleave, with twenty-four.

```
Table XX's 600 bps channel: two paths 2 ms (4.8 symbols) apart, fading at 1 Hz, 7 dB: a weak direct path; the window from 3; the burst arriving at each phase of a symbol
  arrival  600S, 3 segments                              600L, 24 segments
    phase  anchored on  how                   walked     anchored on  how                   walked
        0  direct       an earlier group           6     direct       an earlier group           0
        1  direct       an earlier group           6     direct       an earlier group           0
        2  direct       an earlier group           6     direct       an earlier group           0
        3  direct       an earlier group           6     direct       an earlier group           0
        4  direct       an earlier group           9     direct       an earlier group           0
        5  direct       an earlier group           6     direct       an earlier group           0
        6  direct       an earlier group           6     direct       an earlier group           0
        7  echo         not moved                  2     direct       the precursor search       5
        8  echo         not moved                  2     direct       the precursor search       5
        9  echo         not moved                  2     direct       the precursor search       5
       10  echo         not moved                  2     direct       the precursor search       5
       11  direct       an earlier group           9     direct       an earlier group           0
       12  direct       an earlier group           9     direct       an earlier group           0
       13  direct       an earlier group           9     direct       an earlier group           0
       14  direct       an earlier group           9     direct       an earlier group           0
       15  direct       an earlier group           9     direct       an earlier group           0
       16  direct       an earlier group           9     direct       an earlier group           0
       17  direct       an earlier group           9     direct       an earlier group           0
       18  direct       an earlier group           6     direct       an earlier group           0
       19  direct       an earlier group           6     direct       an earlier group           0
on the direct path: 600S at 16 of 20 phases, 600L at 20
```

At most phases the weak direct path clears the floor in the scan and forms a group of its own, and the earlier group moves the anchor to it. At four phases, 7 to 10, it does not. There the short preamble's walk has two segments to try and both fail, so the anchor stays on the echo; the long preamble's walk reads on the direct path at the fifth segment it tries. A decision that depends on a correlation floor depends on where the samples fall, and a test at one arrival phase is one sample of twenty. The campaign below varied each burst's arrival across all 20, and the stored regression sets replay the recording behind the streaming fix at all 20.

## What it measured

The campaign is one paired test: every burst rendered once through pathsim, the calibrated simulator, then decoded twice, with the earliest-path anchoring and without it (the next two sections call it the rule), by two of this modem's receivers: the adaptive equalizer with its Viterbi decoder, the streaming and embedded path, and the host's whole-burst turbo decoder. At 75 bps only the adaptive receiver decodes the payload. Each burst carried 256 bytes and arrived at one of the 20 sample phases. The Table rows ran 500 bursts each, with the window from 3; at 12 symbols, either window moves the anchor.

On the 5 ms rows, at each row's Table signal-to-noise ratio, acquisition on its own landed on the later path in 43 of 500 bursts at 300 bps, 35 at 150 bps and 50 at 75 bps. The anchoring moved every one of them 12 symbols earlier and changed nothing anywhere else: outside those bursts, the two decodes were bit-identical.

| Row (two paths 5 ms apart, 5 Hz) | Receiver | Exact, anchoring on | Exact, off | In the moved bursts, on / off |
|---|---|---|---|---|
| 300 bps, 7 dB | adaptive | 397 of 500 | 364 | 36 / 3 of 43 |
| 300 bps, 7 dB | turbo | 500 | 494 | 43 / 37 |
| 150 bps, 5 dB | adaptive | 498 | 490 | 33 / 25 of 35 |
| 150 bps, 5 dB | turbo | 499 | 498 | 34 / 33 |
| 75 bps, 2 dB | adaptive | 499 | 462 | 49 / 12 of 50 |

At 300 bps the anchoring cut the adaptive receiver's bit-error rate from 6.7e-3 to 1.0e-3 over all 500 bursts, and in the 43 moved bursts its bit errors from 5852 to 54. The turbo decoder's errors went from 1222 bit errors, all in the moved bursts, to none. The introduction's "roughly tenfold at 300 bps" came from an earlier version of this receiver; these are today's numbers.

At 150 bps, one burst went the other way: the anchoring cost the adaptive receiver a burst it decoded exactly without it, 1137 bit errors with the anchoring, while the turbo decoder decoded that burst exactly either way. That one burst makes the row's pooled adaptive bit-error rate higher with the anchoring, though the anchoring won 9 bursts and lost 1, so I quote the row by its counts. One more burst at 150 bps failed with and without the anchoring.

The same test ran below the Table's signal-to-noise ratios, 100 bursts a row. These results go beyond what the standard asks, and they carry a qualifier: the test system is still being validated. I measured pathsim the way MIL-STD-188-110D Appendix E measures a simulator, for shorter than the appendix asks: with its setting corrected by an offset I measured, 0.4 dB, it delivered within 0.3 dB of the target; the IONOS hardware simulator, measured the same way, delivered about 5 dB more than its setting on fading paths. These numbers are pathsim's, as measured. A hundred short bursts is not the hour Table 10.1 asks for: the runs compare the anchoring on and off, and show where each receiver stops.

At 300 bps the turbo decoder decoded 98 of 100 bursts exactly at 0 dB with the anchoring, 7 dB below the Table's figure, against 81 without it; at 1 dB, 98 against 87; at 3 dB, 100 against 91. The adaptive receiver decoded 35 of 100 at 5 dB with the anchoring against 31 without, 5 at 3 dB either way, and none below. At 150 bps the turbo decoder decoded 97 of 100 at -2 dB, 7 dB below that row's Table figure, against 84 without the anchoring. At 75 bps the adaptive receiver decoded all 100 at 0 dB with the anchoring, against 84 without it, and 85 against 70 at -2 dB. Below the Table figures, acquisition landed on the later path in 7 to 20 bursts of 100, against 7 to 10 percent at them.

## Why not decode both?

Once the receiver has two candidate anchors, the obvious question is why it picks one at all. A PC has the cycles to decode the burst from both and keep whichever result is better. Before building that, I asked what it could gain, and the test data already held the answer. Every burst in the anchor test was decoded twice, once with the rule and once without, so for each burst I could take whichever came out better. That is the most a decode-both receiver could do, because it can only choose between those same two results.

| Runs | Receiver | Exact, the rule alone | Exact, the better of the two |
|---|---|---|---|
| 300 bps, 7 dB, 500 bursts | adaptive | 397 | 397 |
| | turbo | 500 | 500 |
| 150 bps, 5 dB, 500 bursts | adaptive | 498 | 499 |
| | turbo | 499 | 499 |
| 75 bps, 2 dB, 500 bursts | adaptive | 499 | 499 |
| 1200 bps, 2 ms, 11 dB, 100 bursts | adaptive | 95 | 96 |
| | turbo | 100 | 100 |
| Below the Table SNR, 9 rows of 100 | adaptive | 353 | 353 |
| | turbo (the 7 rows above 75 bps) | 692 | 693 |

Across all 2,500 bursts, decoded by both decoders where both run, choosing the better anchor would have rescued three that the rule alone lost. The two results differ only in the bursts where the rule moved the anchor: 295 of the 2,500. In those, the rule rescued 92 bursts on the turbo and 132 on the adaptive receiver, and lost one and two.

Two more facts settle it. A receiver that decodes both has to choose without knowing the message, and this waveform carries no checksum: the candidates are the decoder's own quality figure, the share of re-encoded code bits that disagree with the received ones, and whether the end-of-message marker came through cleanly, and either can be wrong. And the cheap form of the idea, decoding twice only when the rule moved the anchor, still adds about a tenth more decoding, for three bursts in 2,500, and the selector would have to pick wrong fewer than three times in those 295 moved bursts to come out ahead. So the receiver keeps the rule and decodes once. If a channel turns up where the rule is wrong more often, the same paired test will show it, and the decision can be made again.

## Testing, part two: measuring one change, and running it again

The anchor rule is one change to the receiver, and the test for it was built to show that change and nothing else. Every burst was recorded once and decoded twice, with the rule and without it, so the noise and the fades were identical and any difference came from the rule. I counted the bursts only one side decoded (at 300 bps, 33 with the rule and none without), not just the two averages. I ran a control where I expected no effect, the 2 ms row in the window section, inside the adaptive equalizer's own reach, and it found one: with the window from 3, the turbo decoder went from 71 exact bursts of 100 to 100. Every burst's arrival point within the symbol was varied and recorded, because the anchor decision for a weak first path can depend on it. And I read the exceptions before the averages: at 150 bps one burst where the rule hurt the adaptive receiver was enough to make its average error rate look worse with the rule, while it won the exact-burst count 9 to 1, so that row is quoted by its counts.

I've found that deep, long-running tests are what separates the academic modem from the operational modem. Decoding a set of test bursts shows that a design works. An operational modem also has to decode for hours at a time, on PCs and sound cards it was not developed on, through the faults the PC and the channel add, and the only way to know it does is to test exactly that. The amount and the quality of testing that takes cannot be thrown together at the end; it has to be planned up front, alongside the modem. I did not do that. I built my tests in a random, ad hoc way, and soon it spiraled out of control and needed to become a more managed and controlled system.

Testing a modem arguably takes more infrastructure than the modem itself. Behind the numbers in this article are the channel simulators, each measured before its numbers were trusted (Part 4); the signal lab, a board with its own firmware that plays and records audio as a USB sound card and prepares test signals of its own; the waveform generator that renders the test signals on the PC; two other modems as a second opinion on the set-up; the tools that score every decode against the message that was sent; the stored recordings, versioned and checked by hash; and the system that builds the receiver once and runs all of it on several PCs. Each piece needed testing of its own: the hardware simulator delivered more signal-to-noise ratio than its setting (Part 4), and a fault in a bench PC, described below, cost messages on every modem it fed.

A paired test is worth most when it runs again: on the next build, on another PC, through another channel, without being set up again by hand. Until this month, each campaign was set up by hand; now the test system does that. One request names the code to test, a branch or a commit of the receiver and of the terminal program around it. One PC builds it once, and the same program and the same stored recordings, a versioned set checked by hash, go to the other PCs on my network. Each set of recordings runs whole on one PC and is never split, and the results come back as one report. Every run records the exact source commits and a hash of every input, so a result traces back to what produced it. A set that is missing, skipped or failed fails the whole run, and a test that could not run never counts as a pass.

Most of those tests feed a recording to the receiver directly, in step with the decoder, not in real time. The result then does not depend on how fast the PC is or what else it is running, so the PCs can share the work: the stored sets give the same results, card for card, on the Linux and Windows builds, and the same as the earlier real-time runs of the same recordings. One limit: a test in step with the decoder cannot show a race between the decoder and audio that keeps arriving, such as a repair still running when the next burst begins. That case stays a real-time test.

The signal enters at more than one point, and each adds something the one before it lacks:

- **pathsim on the PC**, seeded, so a run repeats exactly and two builds meet the same noise and the same fades. The stored recordings and the paired campaigns come from it.
- **The signal lab's board as a USB sound card**, playing a recording in real time, so the receiver hears it through a separate device's sample clock and the PC's audio path.
- **The board's own channel**: the waveform generator on the board prepares the same test case itself. The short board test plays one case both ways, the PC's recording and the board's own, and both must decode exactly.
- **The IONOS hardware simulator**, a real audio path in between, as the independent check on the software simulators from Part 4.

The stored sets, the hour-long recordings, the short board test and the live hour run from the test system on request. Fresh pathsim campaigns like the anchor test, the board's longer playlists and the IONOS runs are still started by hand.

In real time, the PC is part of the test. A real-time test does not start unless the PC is quiet, with no other receiver and no other heavy job running.

The stored recordings include an hour of each long-interleave rate from 2400 bps down to 75 bps, each through pathsim on its Table 10.1 row's channel and signal-to-noise ratio (at 75 bps, two equal paths 5 ms apart with a 5 Hz fading bandwidth, at 2 dB). Decoded offline, the decoder runs as fast as it can, so that tests the receiver over a long message, not an hour of real-time reception. So the 75 bps hour is also played live: the whole recording through the board's USB audio, in real time, into the adaptive receiver on Windows, and every one of its 33,750 bytes has to decode exactly. The first live run decoded all 33,750 bytes exactly, with no interruption in the USB audio over the hour. One limit: each hour is one fading realization, and Table 10.1 asks for at least five hours on each fading channel (Part 4); those runs are not part of the test system yet.

Two results from this project show why the runs have to be long. In Part 4, the timing integrator that a short burst keeps running lost every one of a set of five-minute 75 bps messages on a slow fade (pathsim, two paths 1 ms apart, 0.5 Hz fading bandwidth, at 2 dB). And long plays through the USB sound card lost a message a few times an hour to a gap of 10 to 60 ms in the audio. At 2400 symbols per second, 10 ms is 24 symbols, and the waveform carries nothing that restores symbol alignment mid-message, so each gap cost the rest of the message, on every modem on the bench at the time. The cause was a firewall's network driver holding the PC's processors for tens of milliseconds, long enough to starve the USB audio. Neither shows up in a test as long as one burst.

The test system is still being validated, as the simulators are, and like the code it is a work in progress. One example: pathsim built for Linux does not render a recording bit for bit as the Windows build does. The recordings differ, in my experience by about one bit, and on the reference set every decode agreed, burst for burst: the same exact bursts and the same bit errors. A difference that size is far below the noise the channel adds, so I treat the two as the same test.

## The tests

`tests/first_path_tests.cpp` runs every walkthrough step and checks what it returned against the expectations stored with the recordings: two groups and the echo's the strongest, then the direct path with the anchoring on; the window from 7 leaving a 2 ms echo alone and the window from 3 moving it; a precursor path found at lag 12 with no group of its own, and the walk rejecting a faded segment before it reads; the sidelobe at lag 11 rejected by the walk; and the arrival phases, some with the short preamble and all with the long, phase by phase. The sidelobe step's test needs no recording: the model with the tail leaves nothing, and the prefix alone leaves the most at lag 14. A recording that is missing, or whose hash does not match, is a setup error, never a skipped test. Run them on their own with:

```bash
ctest --test-dir build -L first-path --output-on-failure
```

## Run it on your own

The runner from Part 4 gains a Part 5 line: where the strongest group put the body, and what the anchoring did with it. It still runs the receive stages through acquisition only; the equalizer comes next week. Run it on a recording from this article:

```bash
./build/runner --wav test-data/first-path-v1/fp1-echo-group.wav
```

```
== Part 2  transmit: none; the audio is test-data/first-path-v1/fp1-echo-group.wav
== Channel: the recording's own
== Part 3  front end, Part 3's loop: 10.60 s of audio -> 25431 symbols
== Part 4  acquisition, with the phase search on (it moved the loop 1 times): 300 bps, long interleave
           read at symbol 8884: countdown 7, correlation 0.917, carrier +2.6 Hz
           preamble from symbol 1204, body from symbol 12724
           committed at symbol 1684: the phase search off, the rate integrator tracking
           the clock: the integrator learned +1 ppm through the body
== Part 5  the first path: the strongest group has the body from symbol 12736
           earliest-path anchoring moved it 12 symbols earlier, to an earlier group
== next: the equalizer (Part 6)
```

## Go read the code, and the standard

1. In 5.3.2.5, Table XX, which rows use two paths 5 ms apart? Work out both delays in symbols, and say what a delay that is not a whole number of symbols does to the precursor profile.
2. `earliest_path_group()` trusts any second group of the same mode a few symbols early. Why can a start a few symbols off the real one not form a group of its own? (Part 4's three gates.)
3. Work out the adaptive equalizer's reach from 24 taps half a symbol apart with the reference tap at 12. What would a window from 6 change, on the 2 ms rows?
4. The profile is summed over every segment to the body boundary, not over the segments the scan covered. What would summing only those miss?
5. `body_precursor_sidelobe_powers()` puts the previous segment's last 15 chips in front of the prefix. Why are they known, and why 15?
6. Why does `precursor_peaks()` take peaks rather than every lag over 1/8? What would a path 4.8 symbols early put at lags 4 and 5?
7. The walk tests the correlation at the shifted start before it reads the designators. Why does a valid read of D1, D2 and the countdown not prove an anchor right?
8. A path d symbols behind the anchor puts a sidelobe 23 - d symbols ahead of it. For which delays does that land inside the 15 lags the search covers, and what does that say about the 2 ms rows?
9. Step 6: with the short preamble, at how many of the 20 phases does the walk reach the weak path? What does that tell you about a test run at one arrival phase?
10. The rule moved the anchor in 295 of 2,500 bursts. What would a receiver that decodes both need in order to choose correctly, and what in this waveform could it use?
11. Every burst in the campaign was decoded twice. Why does a paired test need fewer bursts than two separate runs to show the same difference?
12. Find the known symbols that follow each frame of data in MIL-STD-188-110B 5.3.2.2.2 (and `known_symbols_per_probe` in `body_waveform.hpp`). How long a channel, in milliseconds, can a receiver re-measure from them, and how far apart would two routes have to arrive to be beyond it?

## Next

Next week: the equalizer. Acquisition hands it an anchor on the first path, a carrier estimate and the body boundary. The first thing it does is measure the channel's delay spread from the same preamble, at the lags behind the anchor this time, and pick its geometry from it. Then it trains on the known symbols and follows the channel through the data, one of three ways: an adaptive decision-feedback equalizer for the streaming and embedded path, a model of the channel solved in one shot, and the whole-burst turbo decoder on the host.
