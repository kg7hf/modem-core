# The audio front end: finding the symbols

*Part 3 of "Building a MIL-STD-188-110 HF Modem in Modern C++." Start with the [overview](01-introduction.md), Part 1, [the rules I build by](02-embedded-cpp-philosophy.md), or Part 2, [the transmit chain](03-transmit-chain.md). The code for this article is on GitHub: [kg7hf/modem-core](https://github.com/kg7hf/modem-core/tree/part-3).*

Part 2 ended with clean symbols on the air. The transmitter knew everything about its signal: the carrier, the symbol rate, and where every symbol starts. The receiver starts with none of that. It gets 48,000 audio samples a second and has to turn them back into symbols: one complex value per symbol, taken at the right instant. That is the job of the audio front end, the first stage of the receiver.

The standard says what a receiver has to achieve, not how it does it. Everything in this article is an implementation choice, so for each one I show what the code does, why, and the step of the demo that shows it running.

Part 2 had `tx_demo` to walk its message through the transmitter; this article has `front_end_demo`. Build it, run it, and keep its output open beside this article, the way Part 2 asked you to keep `tx_demo`'s:

```bash
./build/front_end_demo
```

It runs nine steps in the order this article takes them, and every number below comes out of one of them. Each step is a function in `tests/walkthrough/front_end_steps.cpp` that `main()` calls by name, and each section below names its step. So the best way to read a section is in the debugger: in VS Code, press F5 and choose *front_end_demo (gdb)*. It stops at the top of `main()`; step into each step in turn, or set a breakpoint in the function for the section you are reading and continue to it.

| Step | Function | Section |
|---|---|---|
| 1 | `carrier_table()` | Mixing down to baseband |
| 2 | `tone_mix()` | Mixing down to baseband |
| 3 | `tribit6_mix()` | Mixing down to baseband |
| 4 | `receive_filter()`, `pulse_neighbors()` | The matched filter |
| 5 | `timing_error()` | Finding the sampling instant |
| 6 | `float_position()` | Keeping time for hours |
| 7 | `block_sizes()` | Any block size, the same symbols |
| 8 | `carrier_tracking()` | Carrier tracking, part one |
| 9 | `walk()` | Walk the symbols back out |

This article also starts the repository's unit tests, and they call the same nine functions: `tests/front_end_tests.cpp` runs each step and checks what it returned. So the walkthrough you read is also under automated regression. `ctest` runs it all, and the Testing panel in VS Code runs or debugs any one test.

## Four jobs

The front end does four things, in order:

1. **Mix down.** Move the signal from the 1800 Hz carrier to 0 Hz, so each symbol becomes a plain complex number.
2. **Filter.** Run a matched filter, the receive-side counterpart of the transmitter's pulse shaping, to get the best signal-to-noise ratio at each symbol instant.
3. **Find the sampling instant.** Work out where in the stream of filtered samples each symbol is best read.
4. **Keep it there.** Track that instant as it moves, for as long as the signal lasts.

A fifth job, removing what is left of the carrier's phase and frequency, starts here and finishes in the equalizer.

[TODO: figure, the front end as a block diagram; Mermaid for GitHub, a PNG for dev.to.]

## Mixing down to baseband

The audio is real: one number per sample. The symbols are complex: a phase on the 1800 Hz carrier. To get them back, the front end multiplies each sample by a phasor that turns at minus 1800 Hz. The signal lands at 0 Hz, along with a copy at 3600 Hz that the matched filter removes. The product is doubled because a real signal carries half its energy at plus 1800 Hz and half at minus 1800 Hz, and the mix keeps only one half.

The phasor comes from a table. At 48,000 samples a second, 1800 Hz turns 3/80 of a cycle per sample, 13.5 degrees, so after 80 samples it has made exactly three cycles and starts over. Eighty constants hold the whole carrier, and the mixer reads entry n mod 80 for sample n. It keeps that index as a counter that wraps at 80, because a remainder of the 64-bit sample count would be a library call on a part with no 64-bit divider. Nothing about the carrier is computed while the modem runs, so nothing about it can drift, however long the audio lasts.

Step 1, `carrier_table()`, walks the table. It compares every entry with the exact phasor and prints the first few entries and the quadrants; its test checks that every entry is within half a float step of exact, and that the four quadrant entries are exact:

```
  n   phase (deg)      cos        sin
  0        0.0   +1.000000  +0.000000
  1       13.5   +0.972370  +0.233445
  2       27.0   +0.891007  +0.453990
  3       40.5   +0.760406  +0.649448
  4       54.0   +0.587785  +0.809017
  5       67.5   +0.382683  +0.923880
  6       81.0   +0.156434  +0.987688
  7       94.5   -0.078459  +0.996917
 20      270.0   +0.000000  -1.000000
 40      180.0   -1.000000  +0.000000
 60       90.0   +0.000000  +1.000000
```

Step 2, `tone_mix()`, feeds the front end a pure 1800 Hz tone, and a constant comes out: 2.2505, symbol after symbol. A tone at 1810 Hz comes out turning by 0.026179 radians per symbol, which is 10 Hz at 2400 symbols per second, with a faint ripple from the mix's mirror image near 3600 Hz, far down the filter's stopband.

Then a real symbol. Part 2 ended on the randomizer's sixth value, 6, and its worked symbol was tribit 6, at 270 degrees; in the "Hi" burst they meet at body symbol 5. Step 3, `tribit6_mix()`, prints the audio around that symbol's peak, the carrier's phase at each sample, and what the mix makes of it:

```
   sample     audio   carrier (deg)   mixed I   mixed Q
   230496   -0.2831          216.0   +0.4580   -0.3328
   230497   -0.4058          229.5   +0.5270   -0.6171
   230498   -0.5029          243.0   +0.4566   -0.8961
   230499   -0.5652          256.5   +0.2639   -1.0991
   230500   -0.5872          270.0   -0.0000   -1.1744
   230501   -0.5693          283.5   -0.2658   -1.1072
   230502   -0.5165          297.0   -0.4689   -0.9203
   230503   -0.4360          310.5   -0.5664   -0.6631
   230504   -0.3384          324.0   -0.5476   -0.3979
after the matched filter: on-time (-0.016, -2.391), 269.6 degrees, tribit 6
```

One mixed sample is not the symbol. It is a real number times the mixing phasor, so it can only point along that phasor, one way or the other; put another way, it is the symbol plus a copy of it turning the other way at 3600 Hz, 27 degrees a sample. At the peak, sample 230,500, the two line up and the mixed sample points straight down. On either side the copy swings it left and right. The matched filter averages over many samples, the turning copy cancels, and what comes out is (-0.016, -2.391): 269.6 degrees, tribit 6. Set a breakpoint in `tribit6_mix()` and step through it; the numbers are the ones above.

### The compiler changed the math

At the end of Part 2 I promised to explain why the transmitter's audio can differ in the last bit between two builds of the same source, even though every symbol matches.

The transmitter computed the carrier's cosine and sine for every sample. With optimization on, GCC noticed that the code asked for the cosine and the sine of the same angle and replaced the two calls with one `sincos` call. On the toolchain I measured, that function rounds a few results differently in the last bit. The symbols are integers, so they never change; the float audio does, by a tiny fraction of the smallest step a 16-bit sample can show.

IEEE 754, the standard for float arithmetic, requires addition, subtraction, multiplication, division and square root to be correctly rounded, so they give the same bits on every machine that follows it. Sine and cosine have no such requirement. Each library computes them its own way, and the compiler can swap one call for another. No build setting controls that across every library, so the fix is to stop calling them where it matters. Two changes do it, and both are in the code now:

- The carrier table above, in the transmitter and in the receiver's mixer.
- The build forbids the compiler to fuse a multiply and an add into one instruction (`-ffp-contract=off`), because a fused result is rounded once instead of twice, and whether it fuses depends on the compiler and the target.

Measuring the change turned up a bigger effect than the one I went looking for. The old transmitter kept the carrier's phase in a `float` and added one step to it for every sample. Each addition rounds, and the rounding piles up: by sample 458,610, near the end of the "Hi" burst, the phase had walked about 1.4 degrees from where it belonged. The symbols still came out right, because the receiver tracks the carrier, but it is the same story as the timing position later in this article: a `float` that keeps adding walks off. With the table there is no running sum to walk.

What is left is plain arithmetic, and a new library cannot change a result it never computes. On Linux, with GCC and Clang, in Debug and Release, all four builds now produce the same audio, bit for bit, and the same symbols as before. I have not checked Windows yet, and the filter taps are still computed with the library's cosine when the modem starts, so a different library can still move the last bit of a tap. Making the taps constants too is the next step, and then the golden test can check the whole audio instead of five decimal places.

## The matched filter

The receiver filters with a root-raised-cosine pulse, the same family the transmitter shaped with: 161 taps, eight symbols long. Each symbol is spread across neighboring symbols on the air, and the filter is shaped so that, at each symbol's own instant, the neighbors add almost nothing.

### Why the receive filter is wider than the transmit pulse

The transmitter shapes with a rolloff of 0.25; the receiver filters with 0.35. A matched filter is, strictly, the transmit pulse itself, so this is a mismatch, and nothing in the history records why: the code says only that it copies the filter of an earlier reference receiver. So I measured it both ways on the modem's own signal (a 2400 bps burst through the whole front end, five random seeds per condition, signal-to-noise measured in a 3 kHz band):

- **Signal-to-noise:** almost no cost. The filter's peak response drops from 0.9997 to 0.9973, about 0.02 dB.
- **Interference between symbols:** a real cost on a clean channel. With the matched 0.25 filter, the largest leftover at a neighboring symbol's instant is 50.5 dB below the peak; with 0.35 it is 43 dB below. On a clean burst the symbol error floor is -42.6 dB with 0.25 and -37.7 dB with 0.35. With noise added the gap shrinks to about a tenth of a dB at 20 dB signal-to-noise, and to nothing at 10 dB.
- **Timing:** no measurable difference. The wider filter gives the timing detector a slightly stronger error signal, which shows as a slightly smaller standing offset under a 100 ppm clock error (0.74 against 0.79 samples), but the loop's jitter is the same within the noise.
- **Behind an equalizer:** none. With noise or fading and an ideal equalizer after the front end, both filters leave the same symbol error, within 0.2 dB.

Step 4, `receive_filter()` and `pulse_neighbors()`, prints the filter and the first two of those numbers for both rolloffs: the transmit pulse through the receive filter, its peak, and the largest leftover at any other symbol's instant.

```
receive filter: 161 taps, energy 1.000000, peak 0.2450 at tap 80
transmit 0.25, receive 0.25: peak 0.9997, largest neighbor 0.00298 (50.5 dB below)
transmit 0.25, receive 0.35: peak 0.9973, largest neighbor 0.00706 (43.0 dB below)
```

On this evidence the wider filter buys nothing the rest of the receiver needs, and the matched one is cleaner. The modem keeps 0.35 for now: the receiver's conformance results were measured with it, and changing a filter the receiver was qualified with means qualifying it again first.

### Computing only what is needed

The timing loop reads three points per symbol, and each point is interpolated between two filtered samples. So the front end runs the filter only at those six positions, not at all 20 samples of every symbol: 966 tap products per symbol instead of 3220.

The raw samples wait in a ring buffer of 1024 entries. The deepest read reaches 661 samples back, and a `static_assert` beside the ring checks that the buffer is at least that deep, so a change that outgrows it fails the build.

## Finding the sampling instant

Each symbol has one best instant to read it: the peak of the filtered pulse. Between peaks the signal passes through transitions, where it carries little information about the symbol. At 20 samples per symbol, the peak can fall anywhere between two samples, so the loop keeps a fractional position and reads the filtered signal there by linear interpolation between the two nearest samples.

The loop decides where to read next with an early-late timing detector. At each symbol it reads three points: half a symbol early, on time, and half a symbol late. Then:

```cpp
const auto timing_error = std::real((late - early) * std::conj(on_time));
```

If the late sample is stronger than the early one, the peak is ahead, and the loop moves later; if the early one is stronger, it moves earlier. Multiplying by the conjugate of the on-time sample makes the result independent of the carrier's phase, so the loop works before the carrier is tracked. The error is divided by the energy of the three samples, so a loud signal and a quiet one move the loop by the same amount, and the step is limited to a fifth of a sample per symbol.

Step 5, `timing_error()`, reads the receive pulse at those three points, with the on-time point on the peak and four samples either side of it:

```
on the peak +0.000000, 4 samples early +0.031043, 4 samples late -0.031043, early and turned by 1 rad +0.031043
```

Zero on the peak. Reading early, the error is positive, so the loop moves later; reading late, it is the same size with the other sign. And turning the carrier by a radian changes nothing.

The on-time sample is the symbol. The early sample goes along with it: two samples per symbol, which the equalizer uses in a later article.

Here's a sea story. Back in the day, when I arrived at US Navy boot camp, we learned to form up into a column, and we marched everywhere as a unit. You've probably seen movies where someone in the group, maybe the drill instructor or the company commander, is "calling cadence". Calling cadence was usually assigned each week as a special duty, a reward. The cadence is rhythmic, and it keeps everyone in sync: left, left, left right left. Then you can throw in some colorful rhymes. It keeps all the steps in sync, and when it doesn't, the column stumbles and falls out of formation, so keeping a good rhythm is important for marching. Well, I was awarded the job of calling cadence, and wow, was I excited. Unfortunately, my internal clock has a lot of jitter; one could say I don't have good rhythm. My cadence lasted about one day of lllleft, rrrright, rrright, leeft, all at random times, and everyone's feet were jumbled up trying to get onto the foot I was calling at the time. Needless to say, the formation lost sync and collapsed, and my day of calling cadence ended when I was replaced by a more stable oscillator.

A timing loop has the column's problem. Every symbol, the detector calls the beat, and noise makes some calls early and some late. If the loop jumped to each call, its samples would stumble the way my column did, so it moves only a fraction of what each call asks for, at most a fifth of a sample per symbol, and keeps a steady step through the noise. How steady it stays is its jitter, and jitter is where the two detectors below differ.

### Early-late or Gardner

The overview called this Gardner symbol timing. The detector in the code is the early-late form. The two are related; here is how they differ.

Both are textbook detectors, and both multiply a difference by a third sample:

- **Early-late:** the difference of the two samples half a symbol either side of this symbol, weighted by the on-time sample. It looks for the peak.
- **Gardner** (Floyd Gardner, 1986): the difference of this symbol's on-time sample and the previous one, weighted by the sample halfway between them. It looks for the zero crossing between two symbols.

I measured both on the modem's own pulses. On average they behave the same: the same zero point, the same strength. Symbol by symbol they do not. Gardner's output is steadier: on a clean signal its spread is under half of early-late's, the gap narrows as noise rises, and by a 5 dB signal-to-noise ratio they are close. Gardner is also cheaper here, because the previous symbol's on-time sample is already computed: two interpolations per symbol instead of three.

On a dirty channel the difference does not show in the symbols. I ran both loops over the same transmitted bursts (2400 bps, long interleave) with five fixed random seeds per condition: noise at 20, 10 and 6 dB signal-to-noise in a 3 kHz band, a CW tone, impulse noise, a strong signal in the adjacent channel, two-path multipath, Watterson fading, and clock offsets. Wherever both loops held lock, Gardner's timing jitter was a quarter to a half lower, yet the symbol error came out within a few tenths of a dB either way. Interference cost both the same: a CW tone 10 dB below the signal cost nothing extra, and one as strong as the signal ruined both. Multipath and fading hurt through the echoes, which is the equalizer's job, and after an ideal equalizer the two were indistinguishable. (The test kit's fading model is not conformance-qualified, so those rows are engineering evidence, not conformance results.) At a 1000 ppm clock offset both lost lock: the limit there is the loop, not the detector, and that is next week's subject.

Why does the code use early-late, then? There was no decision. The loop arrived labelled "Gardner-style", and the label stayed until I checked the formula for this article. For now the modem keeps early-late. Its conformance results were measured with it, and the front end shows no symbol-quality reason to switch; if I do switch, it will be because the full receiver's conformance runs say so.

## Keeping time for hours

The overview and Part 1 both mentioned a sample counter that walks off the grid after about twenty minutes. This is that story.

The timing loop's position is a sample count, and it grows for as long as the audio does. A `float` holds 24 bits of precision. Past 2^24 samples, about six minutes of audio, it can no longer hold the fraction of a sample the loop needs. Past 2^26 samples, about 23 minutes, it cannot even add the 20 samples of one symbol: 67,108,864 plus 20 comes out as 67,108,880, a step of 16. The loop slides four samples per symbol, and a long capture decoded at chance.

The whole-buffer version of the loop fixed it with a `double` position, which is exact for longer than anyone will record. The streaming front end, the one built for the microcontroller, splits the position into a whole number of samples (a 64-bit integer) and a fraction (a `float` between 0 and 1). The fraction keeps its full precision forever, and there is no `double` on the M33, where it would be emulated in software.

Step 6, `float_position()`, shows both:

```
float: 2^26 + 20 = 67108880.0; 2^24 + 20.5 = 16777236.0
whole + fraction: 67108885 + 0.0
```

And it holds over a long run. One test in the bench, "Thirty minutes of continuous traffic: still exact past 2^26 samples", sends 188 bursts through one front end, 86,630,400 samples, well past 2^26, and every decision in every burst comes out right. It takes a few seconds, so it runs only when asked for: `ctest -L long`.

## Any block size, the same symbols

On a microcontroller the audio arrives in whatever blocks the DMA or the interrupt hands over. The streaming front end keeps all of its state between calls: the ring, the carrier table's index, the timing position. So the symbols do not depend on how the audio is cut up: feed it one sample at a time or 480 at a time, and the output is identical, bit for bit. The carrier index is part of that state for the same reason: it counts samples, not blocks.

Step 7, `block_sizes()`, checks it: the "Hi" burst goes through one front end 480 samples at a time and through another in blocks of 1, 7, 33, 160 and 480 samples in turn, and all 23,032 symbols come out identical, bit for bit.

## Carrier tracking, part one

Two radios never agree exactly on 1800 Hz, and the path between them can shift it further. After the mix, that difference shows up as a slow rotation of every symbol. The carrier tracker removes it with a second-order loop:

- It turns each incoming symbol back by its current phase estimate.
- It measures the remaining angle against a decision, the constellation point the symbol should be.
- It adds a fraction of that angle to its frequency estimate (the integral part), and steps its phase by the frequency plus another fraction of the angle (the proportional part).

The frequency estimate is limited to 0.1 pi radians per symbol, 120 Hz at 2400 symbols per second. The decisions it learns from come from the equalizer, in a later article.

Step 8, `carrier_tracking()`, uses the receiver's gains. Given symbols turned by a fixed 0.5 radians, it learns 0.5000; given symbols turning at 5 Hz, it learns 5.000 Hz, and each corrected symbol lands on its decision. It also calls a sine and cosine for every symbol (`std::polar`), the same kind of call the float section is about.

## Walk the symbols back out

Part 2 walked the message "Hi" into the transmitter and ended with the body randomizer's first values: 0 2 4 3 3 6. The first symbols of the burst's body carry exactly those, so here they are coming back out of the front end. Step 9, `walk()`, prints them from the real code:

```
body  on-time I, Q       angle  tribit  sent  randomizer  tribit - randomizer  coded bit
   0   +2.369  +0.001     0.0       0     0           0                    0          0
   1   +0.014  +2.403    89.7       2     2           2                    0          0
   2   -2.409  -0.009   180.2       4     4           4                    0          0
   3   -1.731  +1.690   135.7       3     3           3                    0          0
   4   -1.661  +1.682   134.6       3     3           3                    0          0
   5   -0.016  -2.391   269.6       6     6           6                    0          0
 ...
  80   +1.687  -1.685   315.0       7     7           3                    4          1
```

Each on-time sample sits within a degree of one of the eight phases, at the same magnitude, about 2.4: the loop is on the symbol peaks. The nearest phase gives the tribit, and every one matches what the transmitter sent. Subtract the randomizer, the same sequence Part 2 added, and what is left is the BPSK data: tribit 0 for a coded 0, tribit 4 for a coded 1. The first six body symbols happen to carry coded zeros, so on the air they are the randomizer itself. The first coded 1 arrives at body symbol 80: it comes in as tribit 7, and taking away the randomizer's 3 leaves 4.

These coded bits are still in the interleaver's order, so they are not yet the bits of "H". Undoing the interleaver and the code is the work of later articles.

## Go read the code, and the standard

1. The mix doubles each product. Why? (A real cosine is the sum of two phasors.)
2. The ring buffer holds 1024 samples. Find the `static_assert` beside it and work out how many the code actually needs, and why.
3. Feed the matched filter a single impulse. What comes out, and why?
4. The timing detector multiplies by the conjugate of the on-time sample. What would happen without the conjugate while the carrier phase rotates?
5. In `float`, add 20 to 67,108,864. What do you get, and what does that do to a timing loop 23 minutes into a recording?
6. The mixer's carrier index wraps at 80 instead of taking the sample count mod 80. Why not just use `%` on the 64-bit count? (Look up what a 64-bit division costs on a Cortex-M33.)
7. Work out the filter's cost per symbol if it ran at every sample, against running it only where the loop reads.
8. Step 5, `timing_error()`, reads the early-late detector on the receive pulse. Write Gardner's detector next to it and run both on the same pulse. Where do they agree, and how many interpolated samples does each need per symbol? Then put it in the loop in place of the early-late error and run the 600S tests. Does Gardner escape the half-symbol start? Why does every timing detector have a point like that?
9. Step 4, `pulse_neighbors()`, prints the leftover at the neighboring symbols for a receive rolloff of 0.25 and of 0.35. Why is the 0.25 leftover smaller, and what would you have to check before switching a qualified receiver to it?
10. MIL-STD-188-110B holds the 1800 Hz carrier to within 1 Hz (paragraph 5.3.2.3.9) and every signaling rate to within 0.01 percent of nominal (paragraph 4.2.1). Put two modems at opposite ends of those limits: how far apart can their carriers be, and their symbol clocks, in parts per million? Compare that with the carrier tracker's limit of 0.1 pi radians per symbol, and with the timing loop's standing offset at 100 ppm. Then find what else in a real link moves the frequency further than the modems do.

## Next

Next week: acquisition. This week's loop tracks a symbol instant it has been handed; acquisition finds a burst in noise, reads the mode from its countdown, and tells the receiver where the body starts.

There is also one thing this week's loop cannot do. Two sample clocks never agree exactly, so part of the timing error is steady. At a 100 ppm clock offset the loop sits 0.74 samples off the eye, and at 1000 ppm it loses lock altogether (20 dB signal-to-noise). A proportional loop can only make a steady correction from a steady error, so it sits off the eye by the drift divided by its gain, and at 1000 ppm the offset it would need is past the peak of the detector's output. The fix is a second loop with an integrator, the same structure as the carrier tracker's, and next week I measure what it fixes and what it costs.
