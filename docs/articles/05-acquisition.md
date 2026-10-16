# Acquisition: finding the burst

*Part 4 of "Building a MIL-STD-188-110 HF Modem in Modern C++." Start with the [overview](01-introduction.md), Part 1, [the rules I build by](02-embedded-cpp-philosophy.md), Part 2, [the transmit chain](03-transmit-chain.md), or Part 3, [the audio front end](04-audio-front-end.md). The code for this article is on GitHub: [kg7hf/modem-core](https://github.com/kg7hf/modem-core/tree/part-4).*

Part 3 ended with symbols: the front end turns 48,000 audio samples a second into 2400 complex values a second, each read at a symbol's peak. It does not say what they are. A receiver listens to a channel that is mostly noise; somewhere in it a burst starts, in one of the standard's modes, with its carrier some hertz away from where the receiver expects it. Acquisition answers four questions from the symbols alone: is a burst there, where does it start, which mode is it, and where does its data begin. It also measures how far off the carrier is, so the stages after it start close.

The standard makes that possible: every burst begins with a preamble whose symbols it fixes in advance. This article reads the preamble the way the standard defines it, then builds the search for it.

Part 3 had `front_end_demo`; this article has `acquisition_demo`. Build it, run it, and keep its output open beside this article:

```bash
./build/acquisition_demo
```

It runs fourteen steps in the order this article takes them, and every output block below comes out of one of them. Each step is a function in `tests/walkthrough/acquisition_steps.cpp` that `main()` calls by name, and each section names its step. In VS Code, press F5 and choose *acquisition_demo (gdb)*; it stops at the top of `main()`, so step into each step in turn, or set a breakpoint in the function for the section you are reading and continue to it.

| Step | Function | Section |
|---|---|---|
| 1 | `preamble_segments()` | The preamble, as the standard defines it |
| 2 | `prefix_reference()` | The reference |
| 3 | `fading_prefix()` | Correlating so a fade does not hide it |
| 4 | `correlation_comb()` | A match at every segment |
| 5 | `carrier_range()` | The carrier offset |
| 6 | `read_segment()` | Reading the mode and the countdown |
| 7 | `wake_mid_preamble()` | Waking up mid-preamble |
| 8 | `unsupported_mode()` | A mode this modem does not implement |
| 9 | `segment_vote()` | One segment, or all of them |
| 10 | `noise_only()` | No burst at all |
| 11 | `phase_search()` | Part 3's hang-up, fixed while acquiring |
| 12 | `clock_offset()` | The clock offset, or a second loop |
| 13 | `learning_time()` | When to learn, and when to stop |
| 14 | `long_burst_rule()` | When to learn, and when to stop |

As in Part 3, the unit tests call the same functions: `tests/acquisition_tests.cpp` runs each step and checks what it returned, so the walkthrough is under automated regression too.

Steps 4, 6, 7 and 9 work on one recording: the "Hi" burst from Parts 2 and 3 (600 bps, long interleave), half a second into a noisy line, with white noise 10 dB below the signal in a 3 kHz band and the whole signal moved 37 Hz off frequency. Part 3's front end turns it into symbols, and acquisition gets nothing else.

## The preamble, as the standard defines it

MIL-STD-188-110B describes the preamble in 5.3.2.3.7.2. It is three 200 ms segments for the zero and short interleave settings, and twenty-four for the long. Each segment is 15 three-bit channel symbols:

0, 1, 3, 0, 1, 3, 1, 2, 0, D1, D2, C1, C2, C3, 0

The first nine are fixed. D1 and D2 name the bit rate and the interleave setting (Table XV); 600 bps with the long interleave is D1 4, D2 6. C1, C2 and C3 are a count of the segments still to come, from 2 (zero and short) or 23 (long) down to 0 in the last segment. The count is a six-bit word, C1 holding the most significant two bits, and each two-bit piece is sent as a three-bit symbol with a 1 in front of it, so 23, binary 010111, goes out as 5, 5, 7 (Table XVI).

Each channel symbol goes out as 32 transmitted symbols (Table XVII): one of eight patterns of tribits 0 and 4, phases 0 and 180 degrees, eight long and repeated four times. The sync randomizer (5.3.2.3.8.2) then adds its own 32-tribit sequence, mod 8, the way Part 2's data randomizer does in the body. Fifteen channel symbols of 32 make 480 symbols, exactly 200 ms at 2400 symbols a second.

Step 1, `preamble_segments()`, reads the transmitter's own preamble back: for each 32-symbol block it finds the channel symbol whose pattern, plus the randomizer, made those tribits:

```
segment   0 1 3 0 1 3 1 2 0 | D1 D2 | C1 C2 C3 | 0   countdown
      0   0 1 3 0 1 3 1 2 0 |  4  6 |  5  5  7 | 0          23
      1   0 1 3 0 1 3 1 2 0 |  4  6 |  5  5  6 | 0          22
  ...
     22   0 1 3 0 1 3 1 2 0 |  4  6 |  4  4  5 | 0           1
     23   0 1 3 0 1 3 1 2 0 |  4  6 |  4  4  4 | 0           0
24 segments; prefix on every one: yes; countdown runs down to 0: yes
D1 4, D2 6 on a long preamble: 600 bps, long interleave
```

Acquisition follows that table. The nine fixed symbols are the same in every segment of every burst, so the receiver can search for them without knowing anything else. Once it has found them, D1 and D2 in the same segment name the mode, and the countdown says how many segments remain before the data, so where the body starts.

## The reference

The receiver searches for the first nine channel symbols, 288 transmitted symbols (I call them chips here, the usual name for the symbols of a spreading code), built exactly as the transmitter builds them. Step 2, `prefix_reference()`, prints the first eight:

```
chip  code  randomizer  tribit       I       Q
   0     0           7       7  +0.707  -0.707
   1     0           4       4  -1.000  -0.000
   2     0           3       3  -0.707  +0.707
   3     0           0       0  +1.000  +0.000
   4     0           5       5  -0.707  -0.707
   5     0           1       1  +0.707  +0.707
   6     0           5       5  -0.707  -0.707
   7     0           0       0  +1.000  +0.000
 ...  288 chips; the same as the first 288 symbols of every transmitted segment: yes
```

Channel symbol 0's pattern is all zeros, so its 32 chips are the sync randomizer's own sequence. The randomizer's sequence is 32 long and a segment is 15 of its periods, so every segment starts it over: the 288 chips are the same at the start of every segment, and the step checks them bit for bit against all 24 segments the transmitter sent.

The code is `make_body_prefix_reference()`, in `modem/m110a/synchronization.cpp`:

```cpp
void make_body_prefix_reference(std::span<IQSample, prefix_symbols> reference) noexcept
{
    SyncRandomizer randomizer;
    std::size_t index{};

    for (const auto channel_symbol : acquisition_prefix)
    {
        for (std::size_t spread = 0U; spread < channel_symbol_chips; ++spread)
        {
            const auto tribit = tribit_add(body_channel_symbol_tribit(channel_symbol, spread), randomizer.next_tribit());
            reference[index++] = psk8_symbol(tribit);
        }
    }
}
```

`body_channel_symbol_tribit`, `SyncRandomizer`, `tribit_add` and `psk8_symbol` are Part 2's transmitter functions, so the reference cannot disagree with what the transmitter sends.

The reference never changes, so the scanner builds it once, on first use, into function-local static storage, the way `psk8_symbol` keeps its constellation table. That was not the first version. The scanner used to build it on its own stack: 288 complex floats, over 2 KB, in a frame of nearly 5 KB. When I moved the code into this repository, its stack-usage check, which holds every function in the modem library to 4 KB, failed. In static storage the reference costs no stack, and no time after the first call, and the frame is back inside the limit.

## Correlating so a fade does not hide it

A correlation multiplies the received symbols by the conjugate of the reference and adds the products. At the start of a segment every product lines up, and the sum is large; anywhere else the products point every which way, and the sum stays small. Divided by the energies of both, the result is 1 for a perfect match and near 0 for none.

One sum over all 288 chips assumes the channel holds still for the 120 ms they take. An HF channel does not. A fading path changes its gain and turns its phase as it goes, and a turn of half a circle partway through the prefix makes the second part of the sum cancel the first. `body_prefix_correlation()` sums each 32-chip block on its own and adds the nine magnitudes instead. A block lasts 13 ms, short enough for the channel to hold still, and a magnitude is the same whichever way its block points. Step 3, `fading_prefix()`, shows the difference on the reference itself, clean and then with the last 128 chips turned 180 degrees, as a fade can:

```
                            one sum   nine blocks
clean prefix                 1.000       1.000
last 128 chips turned 180    0.111       1.000
```

One sum keeps only the difference between the two parts: 160 chips against 128, 32 out of 288. The nine-block sum is unchanged. On a channel that holds still, the two are the same.

The block sum has a cost. Where nothing matches, the products are random, and a sum of random products grows only like the square root of its length. Nine short sums, their magnitudes added, grow faster than one long one, so away from a match the block correlation sits on a higher floor than the single sum would.

```cpp
    for (std::size_t index = 0U; index < reference.size(); ++index)
    {
        const auto rotation = std::polar(1.0F, frequency_radians * as_real(index));
        const auto product = input[start + index] * std::conj(reference[index] * rotation);
        coherent += product;
        block_correlation += product;
        input_energy += std::norm(input[start + index]);

        if ((index + 1U) % channel_symbol_chips == 0U)
        {
            noncoherent += std::abs(block_correlation);
            block_correlation = IQSample{};
        }
    }
```

The `rotation` takes the carrier offset out of the reference before it is compared; the next section is where the offset comes from. The one long sum, `coherent`, is still kept: its angle is the carrier phase at the start, which the equalizer starts from.

## A match at every segment

Step 4, `correlation_comb()`, runs the whole test: the noisy recording, through the front end, correlated at every start from 300 symbols before the burst to the end of its third segment. It lists every start inside the 120 Hz carrier gate that reaches a correlation of 0.35, with the carrier offset estimated there; starts are counted from the burst's first symbol:

```
10 dB SNR, carrier +37 Hz off; every start that reaches the 0.35 floor:
  start  correlation  carrier (Hz)
   -256        0.354         +46.6
   -160        0.362         +29.1
    -96        0.396         +58.1
      0        0.959         +35.2
    480        0.959         +36.2
    960        0.965         +37.6
largest correlation anywhere else, inside the carrier gate: 0.308, at start 938
```

A match every 480 symbols, one at the start of each segment, at 0.96, each with its own estimate of the 37 Hz offset. Between them, nothing reaches 0.35; the largest is 0.308.

The three weak ones before the burst are the burst too. A start 96 symbols early, three channel symbols, lays the last six blocks of the reference over the burst's first six. The prefix repeats itself (0, 1, 3, then 0, 1, 3 again), so three of those blocks carry the same channel symbol in both, and three out of nine blocks match. At 160 and 256 symbols early, fewer blocks overlap and fewer match. These are real partial matches, about a third of a full one, so a floor high enough to take them out would also lose weak bursts. Reading the mode and the countdown takes them out (Step 6).

## The carrier offset

Two single-sideband radios never tune to exactly the same frequency, so the signal arrives some hertz above or below where it left, and every symbol comes out of the front end turned a little further than the one before. The serial-tone section of MIL-STD-188-110B holds the transmitter's own carrier to 1 Hz (5.3.2.3.9b), but says nothing about how large an offset a receiver must accept. Its Appendix C sets a Doppler test for the higher-rate waveforms at 75 Hz (C.6.3), and this modem accepts up to 120 Hz.

`estimate_body_prefix_frequency_radians()` measures the offset over the prefix:

```cpp
    for (std::size_t index = 1U; index < reference.size(); ++index)
    {
        differential += input[start + index] * std::conj(input[start + index - 1U]) * std::conj(reference[index]) * reference[index - 1U];
    }

    return std::arg(differential);
```

Each term is a received symbol times the conjugate of the one before it, with the known modulation of both taken out. What is left in each term is the turn from one symbol to the next: the offset, in radians per symbol. The terms are added before taking the angle, so the noise averages out over 287 of them. The estimate needs neither the carrier phase nor the correlation, so the scan can run it first.

An angle is only known modulo a full turn, so the estimate wraps at half a turn per symbol: 1200 Hz either way at 2400 symbols a second. Step 5, `carrier_range()`, turns a clean segment by each offset:

```
  true (Hz)  estimated (Hz)  inside the 120 Hz gate
       +0.0            +0.0  yes
      +37.0           +37.0  yes
      -75.0           -75.0  yes
     +110.0          +110.0  yes
     +150.0          +150.0  no
     +600.0          +600.0  no
    +1300.0         -1100.0  no
```

The gate, `maximum_frequency_offset_hz`, is the scan's first test. It is the cheapest of three, and noise turns by random amounts, so most starts in noise fail it before the correlation is computed at all.

In Step 4, one segment's estimate came out between 35.2 and 37.6 Hz at 10 dB. That is close enough to start from; Part 3's carrier tracker takes up the rest.

## Reading the mode and the countdown

A start that passes the carrier gate and the correlation floor is a candidate. `characterize_body_candidate()` reads its D1, D2, C1, C2 and C3: for each, it rebuilds all eight channel symbols' chips, randomizer included, correlates each with the received block, and takes the largest magnitude. The eight patterns are orthogonal, so only the right one correlates fully, and the magnitude makes the decision independent of the carrier phase. (It also adds the strongest copy of the same block up to 15 symbols late, for a two-path channel whose first path has faded; that is next week's subject.)

Then the checks. Every count symbol is sent as 4 plus two bits, so a count symbol below 4 means the candidate is not the start of a segment. D1 and D2 must name a mode this modem implements, and the count must fit the preamble that mode sends. A candidate that passes is valid, and the arithmetic is short:

```cpp
            const auto elapsed_segments = segments - countdown - 1U;
            const auto elapsed_symbols = elapsed_segments * first_segment_symbols;
            candidate.first_body_symbol = detected_segment + (countdown + 1U) * first_segment_symbols;
```

The body starts after this segment and the `countdown` segments still to come; the preamble started `elapsed_segments` before this one.

Step 6, `read_segment()`, walks the scan's tests start by start through the recording until a candidate is valid:

```
start 940: correlation 0.354, carrier +46.6 Hz, but not a valid segment
start 1036: correlation 0.362, carrier +29.1 Hz, but not a valid segment
start 1100: correlation 0.396, carrier +58.1 Hz, but not a valid segment
first start through all three gates: symbol 1196 (the burst starts at 1196)
  correlation 0.959, carrier +35.2 Hz
  D1 4, D2 6: 600 bps, long interleave
  countdown 23: the body starts 24 segments on, at symbol 12716 (really 12716)
  so the preamble began at symbol 1196
```

The three starts that fail are Step 4's early partial matches: they passed the floor, and their blocks read as nothing the standard sends. The first valid one is the burst's first symbol, and the countdown puts the body exactly where it is.

### Waking up mid-preamble

The long preamble lasts 4.8 seconds, so a receiver can easily start listening in the middle of one. Step 7, `wake_mid_preamble()`, cuts the recording 10 segments and 137 symbols into the burst and runs the whole acquisition on what is left:

```
the receiver starts 4937 symbols into the burst (segment 10, symbol 137)
  it read the countdown from burst segment 13: 10, so 11 segments to the body
  body starts at symbol 6583 of what it heard (really 6583)
  preamble start in what it heard: no
```

The countdown does not depend on where the receiver started. Acquisition reports that the preamble began before the recording did, so nothing downstream counts on having all of it.

### A mode this modem does not implement

The standard is explicit: "If a demodulator receives any D1, D2 combination that it does not implement, it shall not synchronize but shall continue to search for synchronization" (5.3.2.3.7.2.1). Table XV lists D1 7, D2 7 as 2400 bps secure voice, which this modem does not implement, and a note reserves 5, 6 and 5, 7 for application-specific use. Step 8, `unsupported_mode()`, builds a 600S preamble, then the same preamble with D1 and D2 changed to 7, 7 in every segment:

```
D1 6, D2 6 (600S):        supported                     acquired: yes
D1 7, D2 7 (patched in):  recognized, not implemented   acquired: no
```

`recognize_body_designators()` distinguishes the two cases: a combination the standard defines but this modem does not implement, and one the standard does not define at all. Either way the candidate is not valid, nothing is acquired, and the scan goes on.

## One segment, or all of them

That is everything acquisition needs from one segment. A segment is 200 ms of a weak signal, and one decode at low signal-to-noise can be wrong. The burst sends the same information in every segment, so `acquire_body_preamble()` reads all of them:

```cpp
    const auto scan = scan_for_consensus(symbol_rate_input, reference, search_starts, config);

    if (scan.count == 0U)
    {
        return Status{StatusCode::unavailable, "body preamble countdown not acquired"};
    }

    const auto& strongest = scan.groups[strongest_group(scan)].representative;
    return vote_mode_and_boundary(symbol_rate_input, reference, strongest, config);
```

`scan_for_consensus()` takes every start through the three gates: the carrier offset, the correlation, and a valid read. Each valid candidate names a mode and a body boundary, and candidates that name the same ones are the same burst, read through different segments, so they join one group. `strongest_group()` takes the group with the most segments behind it; a tie goes to the larger sum of correlations. The group's representative, which supplies the carrier offset and phase the later stages start from, is its best-correlated member, not its earliest: an early segment that clears the floor during a fade can carry an estimate several hertz off, and starting the equalizer from it can cost a block that a cleaner segment of the same group would have decoded. Then `vote_mode_and_boundary()` reads every segment from the winner's on and lets the majority correct the mode and boundary, if at least two segments agree on something else. When the scan has covered the whole preamble, the strongest group already is the majority; the vote matters when the scan has seen only the first segment or two, as it has in a streaming receiver that commits early.

Step 9, `segment_vote()`, turns the noise up to 4 dB above the signal, reads each segment of the recording on its own, and groups the reads:

```
-4 dB SNR, each segment read on its own; the ones that did not read right:
segment  correlation  countdown  reads as
      0        0.257          -  not a valid segment
      3        0.505          -  not a valid segment
      8        0.485          -  not a valid segment
     10        0.434          -  not a valid segment
     15        0.218          -  not a valid segment
     16        0.243          -  not a valid segment
     18        0.493          -  not a valid segment
     20        0.378         19  600 bps, body at 20396  (wrong)
     21        0.410          -  not a valid segment
right 15, wrong 1, not valid 8
grouped by the burst they describe:
  600 bps, body at 12716: 15 segments
  600 bps, body at 20396: 1 segment
acquisition: right
```

Segment 20 read its countdown as 19 instead of 3, which would have put the body 3.2 seconds late. Alone it is a group of one; fifteen segments agree on the right answer.

## No burst at all

A receiver spends most of its time listening to noise, and every false acquisition costs something: a mode and a boundary the stages after it try to decode, until they give up. Step 10, `noise_only()`, runs ten seconds of white noise through the front end and the whole acquisition:

```
10 s of noise: largest correlation 0.264 against the 0.35 floor
  starts through both gates 0, of those reading as a valid segment 0; acquired: no
```

`BodyAcquisitionConfig` sets the correlation floor at a cautious 0.75 by default. This walkthrough uses 0.35, the floor the streaming receiver later in the series runs at, because the floor is only the second of three gates. A start in noise has to turn by under 120 Hz, then correlate above 0.35, then read as a segment of a mode this modem implements, with a count that fits; and to win, it has to outnumber the segments of any real burst. In ten seconds of noise nothing passed the first two.

How often acquisition finds a burst at each signal-to-noise ratio, and how often noise alone gets through, are measurements of their own, over many recordings; they come in a later article.

## Part 3's hang-up, fixed while acquiring

Part 3 found one start its timing loop handles badly. Started half a symbol off the symbol peaks, the early and late samples are equally strong, the error is close to zero, and the loop barely moves; on the short preamble it can still be off the peaks when the body begins. Part 3 left that as a documented limit, with a `[!shouldfail]` test. The fix belongs to acquisition, because only while the receiver looks for a burst can the loop be anywhere in the symbol; once it has locked to one, the loop is on the peaks.

The fix is a phase search: the energy comparison a batch receiver makes over a whole recording, made with the three points the loop already reads. Over every 120 symbols the front end adds up the energy of the on-time samples and the energy half a symbol off them. A loop on the peaks reads more energy on time: half a symbol off, where two neighbouring pulses overlap, a stream of raised-cosine pulses at the receive filter's 0.35 rolloff reads about 0.83 of its on-time energy. A loop hung half a symbol off reads more energy off time. When the off-time sum beats the on-time sum by 6 percent in two windows running, the loop is moved half a symbol, onto the peaks, and pulls in from there. It waits for two windows so that noise, whose two sums differ by a few percent, never moves a locked loop. The same comparison speeds the loop up while it is well off the peaks: after a window whose off-time energy reached 92 percent of the on-time energy, the next window runs at three times the tracking gain. In `BodyAudioStreamFrontend::process()`:

```cpp
        float phase_move = 0.0F;
        if (mPhaseSearch)
        {
            mPhaseOnEnergy += std::norm(on_time);
            mPhaseOffEnergy += (std::norm(early) + std::norm(late)) / off_time_points;

            if (++mPhaseWindowSymbols >= phase_search_window_symbols)
            {
                mPhaseWindowsOff = mPhaseOffEnergy > mPhaseOnEnergy * (1.0F + phase_search_margin) ? mPhaseWindowsOff + 1U : 0U;
                mPhaseFastGain = mPhaseOffEnergy > mPhaseOnEnergy * phase_search_fast_gain_ratio;

                if (mPhaseWindowsOff >= 2U)
                {
                    phase_move = as_real(half_symbol_samples);
                    ++mPhaseSearchMoves;
                    mPhaseWindowsOff = 0U;
                }

                mPhaseOnEnergy = 0.0F;
                mPhaseOffEnergy = 0.0F;
                mPhaseWindowSymbols = 0U;
            }
        }

        const auto loop_gain = mPhaseSearch && mPhaseFastGain ? phase_search_loop_gain : mTimingLoopGain;
```

The receiver turns the search on while it looks for and acquires a preamble, and off once it commits to a burst, so a locked body is never moved. Off, the loop is Part 3's loop exactly. Step 11, `phase_search()`, starts a 2400S burst at each of the twenty sample phases of a symbol, at 10 dB, with and without the search:

```
2400S, 10 dB SNR, the burst started at each of the 20 sample phases of a symbol
pulled in before symbol 1200 of the 1440-symbol preamble: 17 of 20 plain, 20 of 20 with the phase search
the starts the plain loop missed:
lead  plain loop pulled in at   with the search   moves
   0     not in the preamble               245       1
   1                    1328               379       1
  19     not in the preamble               431       1
```

Without the search, three starts around half a symbol off missed symbol 1200 of the 1440-symbol preamble, and two of them were not on the peaks anywhere in the preamble. With it, all twenty were; the three late ones were moved once each and were on the peaks by symbol 245 to 431. Part 3's `[!shouldfail]` test runs the front end alone, without the search, so it still fails as expected: the fix lives where the receiver acquires, and this article's tests check it there.

## The clock offset, or a second loop

Part 3 ended on what its timing loop cannot do. Two sample clocks never agree exactly, so the received symbols arrive slightly faster or slower than 20 samples apart, and part of the timing error is steady. The loop is proportional: each symbol it moves by its gain times the error. To keep up with a steady drift it needs a steady error, so it settles off the symbol peaks by the drift divided by its gain. At 100 ppm the drift is 0.002 samples a symbol; with the loop's gain of 0.08 and the detector's slope near the peak, that is about 0.74 samples. At 1000 ppm it would need ten times as much, which is past the peak of the detector's output, and the loop cannot hold it.

Three more steps measure that, and what fixes it. The bench now records where the loop reads every symbol and compares it with the grid the transmitter laid down, stretched by the clock offset (`grid_offsets()` in the test harness), so these numbers are the loop's own position, not a count of symbol decisions. Step 12, `clock_offset()`, runs 2400 bps bursts carrying 10 s of data each, at 20 dB. The interleaver pads a body to whole blocks, so the long-interleave body lasts 14.4 s and the short one 10.2 s:

```
20 dB SNR, 10 s of data in each burst; the loop against the symbol grid through the body
clock (ppm)  mode   loop    offset (samples)  wander  symbols slipped  integrator at the end
       +100  2400L  type 1             +0.68    0.22                0                      -
       -100  2400L  type 1             -0.64    0.21                0                      -
      +1000  2400L  type 1             +1.47   11.46               30                      -
       +100  2400L  type 2             +0.02    0.22                0             -3 ppm off
      +1000  2400L  type 2             +0.03    0.24                0             -6 ppm off
      -1000  2400L  type 2             +0.03    0.20                0             +1 ppm off
      +1000  2400S  type 2             +0.07    5.98                0            -11 ppm off
```

The first three rows are Part 3's loop, type 1. At 100 ppm either way it sits 0.64 to 0.68 samples off the peaks, on the side the clock pushes it. At 1000 ppm it slips 30 symbols through the body. The count starts a segment into the body, and from there the clock drifts by 34 symbols, so the loop barely follows the clock at all.

The fix is a second loop with an integrator, the same structure as Part 3's carrier tracker. The integrator adds up the error, so it settles at the clock's rate, and the proportional term is left with only the jitter. That makes it a type-2 loop. In `BodyAudioStreamFrontend::process()`:

```cpp
        if (mTimingIntegralGain > 0.0F && mTimingRateMode != TimingRateMode::held)
        {
            // Type 2: the integrator carries the clock's rate, and the proportional term only
            // the jitter. It integrates while tracking; frozen, the rate stands. Both are
            // limited to the step limit, which is far beyond any real clock offset.
            const auto normalized_error = timing_error / normalization;

            if (mTimingRateMode == TimingRateMode::tracking)
            {
                mTimingRate = std::clamp(mTimingRate + mTimingIntegralGain * normalized_error, -mMaximumTimingStep, mMaximumTimingStep);
            }

            mTimingRateSum += round_to<std::int64_t>(mTimingRate * timing_rate_mean_scale);
            ++mTimingRateSamples;

            correction = std::clamp(loop_gain * normalized_error + mTimingRate, -mMaximumTimingStep, mMaximumTimingStep);
        }
        else
        {
            // Type 1, the default, and the tracker while held: the proportional loop.
            correction = std::clamp(loop_gain * timing_error / normalization, -mMaximumTimingStep, mMaximumTimingStep);
        }
```

The integrator is off unless `configure_timing_tracker()` turns it on, and with it off, or held, the loop is Part 3's expression exactly, so every Part 3 number stands. In Step 12's type-2 rows it runs from the first preamble candidate to the end of the burst. On the long preamble it ends within 6 ppm of the clock, at 100 and at 1000 ppm either way, and the loop sits within 0.03 samples of the peaks without a slip. On the short preamble at 1000 ppm the loop is still pulling in when the body begins, so it wanders 6 samples through the body, but it does not slip.

### When to learn, and when to stop

An integrator left running is only right while it is fed a signal. On a line with no signal, the timing error is not zero on average, and an integrator running through minutes of noise walks to a rate the next preamble cannot pull in from. So the receiver holds it at zero while it searches, and starts it at the first preamble candidate acquisition finds.

Through a fade it causes trouble too. A fading path changes the shape of the received pulse, so the timing error carries a bias that follows the path gains, and the integrator turns that bias into a rate. A clock offset is a crystal's and does not change during a burst, so the first design I measured learned the rate in the preamble and froze it for the body: track from the first candidate, and once 2880 symbols and the end of the preamble had passed, freeze. On the front end alone, at 20 dB, it worked. In the receiver's qualification (the whole receiver, decoding, on white noise through the test bench's channel at the signal-to-noise ratio where each mode is meant to work, many runs per condition, each with its own seed: the number that starts the random-number generator, so a run's noise and fading can be repeated exactly) it did not. At 1000 ppm either way, on the short-interleave modes, the receiver with the freeze lost about two thirds of the bursts that the receiver with the integrator left running decoded. Step 13, `learning_time()`, shows why. It runs the integrator on 600 bps with the short interleave and the clock 1000 ppm off, at falling signal-to-noise ratios:

```
600S, the clock +1000 ppm off, the integrator running from the first candidate, three seeds
SNR (dB)  rate error at 2880 symbols (ppm)  within 100 ppm from symbol
      20                    +1   -30   -34          2627   2735   2715
      10                  +391  +207   -37          4127   4102   3859
       7                    -6   -52  +213          4091   4049   6266
       4                  +845  +580  +835          4401   6124   8606
```

At 20 dB the integrator is within 100 ppm of the clock about 2700 symbols after the first candidate, inside the 2880-symbol window. At 4 dB it takes 4400 to 8600 symbols, and when the window closes it is still 580 to 845 ppm off. The timing error is divided by the energy of the three samples it reads, and at 4 dB much of that energy is noise, so each symbol moves the loop and the integrator less. A fixed window that fits 20 dB freezes a wrong rate near the modes' working SNR.

Leaving the integrator running has a cost of its own on long messages. Through pathsim, on a slow fade (two paths 1 ms apart with a 0.5 Hz fading bandwidth) at the mode's Table 10.1 signal-to-noise ratio, the running integrator lost every one of a set of five-minute 75 bps messages, where the type-1 loop lost at most one; the test bench's own channel gave the same picture. And on white noise at each mode's Table 10.1 signal-to-noise ratio, the type-1 loop alone decoded with the clock up to 250 ppm off, though 75 bps at +250 ppm was marginal, and it lost every long-interleave mode at 500 ppm. MIL-STD-188-110B holds signaling rates to 0.01 percent of nominal (4.2.1), so two modems within the standard are at most 200 ppm apart: the integrator is for clocks worse than the standard allows.

So the receiver judges the integrator rather than freezing it. It tracks from the first candidate; at 20, 30 and 40 seconds it takes the mean of the integrator's rate since 5 seconds, when the pull-in is over, and if that mean is below a threshold rising from 125 to 175 ppm, it retires the integrator: holds it at zero, the type-1 loop, for the rest of the burst. An integrator that passes the last judgement runs to the end, and a burst over before 20 seconds keeps it throughout. The front end keeps the sum in fixed point, the rate times 2^20 per symbol, so 10^5 symbols lose nothing to float rounding. Step 14, `long_burst_rule()`, shows why the judgement is made on the mean. It runs 25 s of 2400L data, a 28.8 s body, through the same slow fade, with no clock offset and with 350 ppm:

```
2400L, 25 s of data through Watterson fading, 1 ms and 0.5 Hz, 20 dB SNR; the integrator from 5 s to 20 s
clock (ppm)  seed  largest momentary rate  mean rate  at 20 s (threshold 125 ppm)
         +0     1                 712 ppm     27 ppm  retired
         +0     2                 643 ppm     83 ppm  retired
       +350     1                 837 ppm    323 ppm  kept
       +350     2                 924 ppm    265 ppm  kept
```

Through the fade the integrator's momentary rate swings to 640 to 920 ppm whether there is a clock or not; the strongest path moves, and the integrator follows it. Over 15 seconds the swings average out: 27 and 83 ppm with no clock, 265 and 323 ppm with one. The mean tells them apart where the momentary rate cannot.

One case none of this fixes. When two fading paths are about as strong as each other, the loop can walk from one to the other, a few samples apart, whatever rate it holds. That is the equalizer's problem, and it comes with the equalizer.

## Testing, part one: what the standard asks, and how long it takes

This article starts a testing thread that runs through the rest of the series, a piece in each article, to a full treatment of compliance testing in Part 9.

MIL-STD-188-110B sets the serial-tone performance in 5.3.2.5, Table XX: a coded bit-error rate at a signal-to-noise ratio measured in a 3 kHz band, on Watterson-model channels per ITU-R F.520-2 (for the fading rows, two independent fading paths of equal average power), with the longest interleaver. The JITC conformance test plan (2004) turns that into the test conditions of its Table 10.1 (Subtest 10), and its criteria say how long: at least an hour on the non-fading channel, and at least five hours on each fading one.

The hours come from the statistics. **A bit-error rate of 10^-5 is one error in 100,000 bits.** With no errors in n bits, the 95 percent upper bound on the rate is about 3/n, so showing 10^-5 takes about 300,000 error-free bits: two minutes at 2400 bps, over an hour at 75 bps. That bound assumes the errors are independent, and a decoder's are not: when it fails, it fails over a run of bits, so for a coded modem the bound is optimistic. A fading channel needs longer still, because the errors come in the deep fades, which are rare; the test has to run long enough to meet a great many of them.

None of that tests acquisition directly. The serial-tone section sets no acquisition requirement, and Appendix C's acquisition test, for the higher-rate waveforms, reads "Not yet standardized" (C.6.2). The acquisition tests in this article are mine: every mode found, the carrier gate, waking mid-preamble, a mode not implemented, and nothing in ten seconds of noise.

A test result is only as good as the channel it ran through, so this work uses three channel simulators, and measures each before trusting its numbers:

- **The test bench's own model**, inside the unit tests: a two-path Watterson model, an engineering model. Its noise and fading come from a random-number generator started from a seed, so two designs run on the same seeds meet exactly the same noise and the same fades, and any difference between them comes from the designs, not from luck. That makes it good for comparing designs, as Steps 12 to 14 do, and not for absolute numbers.
- **pathsim** ([github.com/bubnikv/pathsim](https://github.com/bubnikv/pathsim)): Moe Wheatley AE4JY's PathSim, reworked by Vojtech Bubnik OK1IAK, under the GPL-3.0. The seeded campaigns run on it; my build adds a seed, so a run repeats.
- **The IONOS HF channel simulator** ([github.com/ARSFI/HFSimulator](https://github.com/ARSFI/HFSimulator)): a hardware box by Rick Muething KN6KB, Tom Lafleur KA6IQA and Tom Whiteside N5TW, from the Amateur Radio Safety Foundation, under the MIT license. It puts a real audio path between the transmitter and the receiver, so it checks the software simulators independently. Measured the same way as pathsim, it delivered more signal-to-noise ratio than its setting on a non-fading channel, by an amount that varied with the signal level, and its two-path settings stop at 4 ms and 2 Hz, short of the 5 ms, 5 Hz channel Table 10.1 sets for the lowest rates.

A commercial laboratory channel simulator costs many thousands of dollars. Whichever kind is used, MIL-STD-188-110D's Appendix E applies to a simulator used to test a modem's compliance, and says what it should meet: a frequency response within 0.5 dB; a signal-to-noise ratio within 0.25 dB of its setting on a non-fading path, averaged over at least 20 minutes, and within 0.5 dB on fading paths, averaged over at least two hours; and a Doppler spectrum measured from at least three hours of a tone (E.7). I ran the same kinds of measurement, shorter than the appendix asks, with the same probes on every simulator.

Two other software modems decode the same captured audio beside this one: MS-DMT, N2CKH's free, closed-source 110A modem, and ION2G, another 110A software modem. They are a second opinion on the test set-up, not a reference for this receiver: when all three decode a clean burst exactly, the audio path is good, and a disagreement points at the set-up before it points at a decoder.

The test audio comes from my waveform generator and signal lab. They render a recording with CW, noise, impulses, fades and sample slips mixed in, play it from the PC or from a microcontroller board that is also a USB sound card, capture what comes back, and keep a journal to replay any run. They will be public, under the GPL-3.0, soon.

## The tests

`tests/acquisition_tests.cpp` runs every walkthrough step and checks what it returned: the 24 segments and their countdown, the reference against every segment, the block sum through the phase flip, the matches at the segment starts and nowhere else, the estimate to 1200 Hz and the gate at 120 Hz, the mode, the countdown and the body boundary, the wake, the mode not implemented, the vote, and the noise. The phase-search test checks that every start phase pulls in within the short preamble with the search on. The clock steps' tests check the loop against the grid: type 1 off the peaks by about the drift over its gain at 100 ppm and slipping at 1000, type 2 on the peaks with no slip; at 4 dB the integrator still hundreds of ppm off when 2880 symbols have passed; and on the long burst, the mean telling no clock from a real one. Around them are tests beyond the walkthrough: every mode the modem implements, at 10 dB with the carrier 75 Hz off either way, found with the right mode, start and body; the gate, which takes a burst 110 Hz off and not one 150 Hz off; and waking at five places in a long preamble, down to the last whole segment.

The test bench grew with them. The test channel takes a carrier offset, in whole hertz, so the shift is exact over any length of audio, the way the CW tone is. Its clock offset came back too, fixed: the first version kept the resampler's position in one `float`, which past a fraction of a second could no longer take the offset's tiny step, so a whole burst got the offset only at its start. It now keeps a whole sample plus a fraction, the way the front end keeps time. And the bench has its own answer for where a burst sits in the front end's output, `locate_burst()`: it lines the burst's last 2000 symbols up with what was sent, eight at a time so a turning carrier cannot cancel the sum. Acquisition is checked against that, and never against itself.

The acquisition code reached this repository rearranged. `acquire_body_preamble()` had grown into one function of 400 lines with a lambda inside it, a fix at a time; it is now the short sequence above, one named function a step. Before the rearrangement went anywhere, a scratch program ran the old and the new code on the same inputs, every combination of six modes, four carrier offsets, three signal-to-noise ratios, six channels (one path; two paths at three spacings; two with the first path faded) and three acquisition settings, and compared every field of every result bit for bit. They were identical.

## Run it on your own

Each demo walks one stage on data I chose. The runner runs every stage published so far, in order, on what you give it: your message through Part 2's transmitter, a channel you set, Part 3's front end, and this article's acquisition. It prints what each stage found. Each article adds its stage, so it grows with the series.

```bash
./build/runner "Hi" --snr 10 --carrier 37
```

```text
== Part 2  transmit: "Hi" in 600L (600 bps, long interleave)
           2 bytes -> 11520 preamble symbols (24 segments) + 11520 body symbols, 9.60 s of audio
== Channel: 10 dB SNR, carrier +37 Hz; 0.5 s of line before the burst, seed 4
== Part 3  front end, Part 3's loop: 10.10 s of audio -> 24232 symbols
           through the body, against the transmitter's grid: 0.04 samples off the symbol peaks, in step with every symbol
== Part 4  acquisition, with the phase search on (it moved the loop 0 times): 600 bps, long interleave
           read at symbol 3116: countdown 19, correlation 0.966, carrier +37.7 Hz
           preamble from symbol 1196, body from symbol 12716 (the bench: 1196 and 12716)
           committed at symbol 1676: the phase search off, the rate integrator tracking
           the clock: the integrator learned 0 ppm through the body
           through the body, against the transmitter's grid: 0.04 samples off the symbol peaks, in step with every symbol
== next: the first path (Part 5)
```

That is the line from Steps 4 and 6: the same message, noise and seed, so the same burst. Step 6 showed the first segment through the gates, with countdown 23; the runner shows what `acquire_body_preamble()` returns, the group's best-correlated segment, here the fifth, with countdown 19. Both put the preamble at symbol 1196 and the body at 12716, where the bench's own answer puts them.

The runner runs the front end three times over the audio, once for each way a receiver can run it. The Part 3 line is Part 3's loop and nothing else. Acquisition runs on the front end with the phase search on, the way a receiver runs it while it looks for a preamble. Then the runner commits at the first candidate, a segment into the preamble: from there the search is off and the rate integrator tracks, the way a receiver listening live would have run it from that point. (A live receiver does all of that in one pass; the runner has the whole recording, so it finds the candidate first and then runs the front end again.)

Give the transmitter's clock a 1000 ppm error and the two loops part:

```bash
./build/runner "Hi" --snr 20 --clock 1000
```

```text
== Part 3  front end, Part 3's loop: 10.09 s of audio -> 24210 symbols
           through the body, against the transmitter's grid: the loop slipped 11 symbols, net 11 lost
== Part 4  acquisition, with the phase search on (it moved the loop 0 times): 600 bps, long interleave
           ...
           committed at symbol 1675: the phase search off, the rate integrator tracking
           the clock: the integrator learned +1000 ppm through the body
           through the body, against the transmitter's grid: 0.04 samples off the symbol peaks, in step with every symbol
```

Part 3's loop slips 11 symbols through the 4.8 s body, every one a symbol stepped over, about what the clock drifts across it, as Step 12's type-1 row does on 2400L. (The runner counts every slip, either way, and gives the net beside it, so a slip one way and one back cannot pass for a loop in step.) With the integrator tracking from the commit, the loop learns the clock and stays on the peaks. Try `--fade 2,1` for two-path fading, `--mode 2400S`, or a message of your own; `--help` lists the rest.

The runner also reads a WAV file, 48 kHz, 16-bit or float. `--save heard.wav` writes the audio it hands the receiver, and `--wav heard.wav` runs the receive stages on it; a recording of a real transmission works the same way, with no bench answer to compare against. `tests/runner_tests.cpp` checks the chain: the WAV file out and back in, the walkthrough's burst found where the bench put it, a 1000 ppm clock learned, and a saved recording read back to the same burst.

## Go read the code, and the standard

1. Table XVI lists the two-bit count values in an order that does not match the rule in the text of 5.3.2.3.7.2.1 ("adding a '1' before the two-bit value"). Which reading does `characterize_body_candidate()` follow, and how does the note's example, 23 sent as 5, 5, 7, decide it?
2. The prefix is the same at the start of every segment because the sync randomizer's sequence is 32 long. What would a receiver have to do if a segment were not a whole number of its periods?
3. Work out the block correlation's floor with no match: nine blocks of 32 random unit products, magnitudes added, normalized as `body_prefix_correlation()` does. Then work it out for one sum of 288.
4. The carrier estimate wraps at 1200 Hz. Add 2500 Hz to Step 5's list: what does it print, and why?
5. A receiver hears a long-interleave segment with countdown 10 at symbol 9000 of its recording. Where does the body start, and where did the preamble start?
6. The channel-symbol decode compares the eight patterns by magnitude. What would comparing the real part need that the decoder does not have?
7. Why does a consensus group's representative have to be its best-correlated member, and not its first?
8. `vote_mode_and_boundary()` needs two segments to agree before it overrules the winner. What could go wrong with one?
9. Find the D1, D2 combinations `recognize_body_designators()` treats as recognized but not implemented. Which does the standard reserve, and which is a mode?
10. Set the walkthrough's floor to 0.25 and run Step 10 again. What gets through the first two gates now, and what stops it?
11. Run Step 9 at -8 dB. How many segments read right on their own, and does acquisition still find the burst?
12. Work out the type-1 loop's offset at 100 ppm from the drift and the gain, and compare it with Step 12. What sets the slope that turns the gain into samples?
13. Step 13: at 4 dB the integrator is still hundreds of ppm off after 2880 symbols, with the same gains as at 20 dB. Look at what the timing error is divided by, and explain why noise slows the loop down.
14. Why must the integrator be held at zero while the receiver searches? Run the type-2 loop through Step 10's ten seconds of noise and watch `timing_rate()`.
15. Step 14 takes the mean from 5 s, not from the first candidate. What would the first 5 s add to the mean at 4 dB, and to the judgement?
16. The phase search moves the loop only after two windows in a row read more energy off time. Change it to one, and run Step 11 at 4 dB: what does a loop already on the peaks do now?
17. Table 10.1's fading conditions run for at least five hours. Roughly how many fades of a 1 Hz fading bandwidth is that, and why does a fading test need so many more bits than a non-fading one?

## Next

Next week: the first path. On a two-path channel, each path delivers its own copy of the preamble, a few symbols apart, and each copy forms its own group. When the two paths are of similar strength, the strongest group is often the later one, the echo, and an equalizer that starts on the echo cannot reach the path in front of it. Earliest-path anchoring finds the first copy, even when it faded through the segments the scan saw first, and moves the anchor to it.
