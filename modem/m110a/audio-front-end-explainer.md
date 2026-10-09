# Audio Front End Explainer

## 1. Scope

This note explains the receiver's first stage:

- `modem/m110a/body_audio_stream_frontend.hpp` / `.cpp`: **`BodyAudioStreamFrontend`**,
  the streaming front end. It mixes the 1800 Hz carrier down, runs the root-raised-cosine
  matched filter, and finds and tracks each symbol's sampling instant, on audio blocks of
  any size.
- `CarrierTracker`, in `modem/common/dsp/timing_recovery.hpp` / `.cpp`: the second-order
  loop that removes what is left of the carrier's phase and frequency once a later stage
  supplies decisions.

The carrier itself is the 80-entry `body_carrier_table` in `modem/m110a/body_waveform.hpp`,
the same table the transmitter uses. The unit tests in `tests/front_end_tests.cpp` check
each piece, and the article series explains the design in Part 3, "The audio front end".

---

## 2. 50,000-foot view

Audio arrives as 48,000 real samples a second, in blocks of whatever size the codec or
the DMA hands over. The front end turns it into one complex number per symbol, 2400 a
second, taken at the instant each symbol is strongest, plus a second sample half a symbol
earlier. It does four things, in order:

1. **Mix down.** Multiply each sample by the conjugate of the carrier, so the signal sits
   at 0 Hz and each symbol becomes a plain complex number.
2. **Filter.** Run a 161-tap root-raised-cosine matched filter, the receive-side partner of
   the transmitter's pulse shaping.
3. **Find the instant.** An early-late detector compares the filtered signal half a symbol
   either side of the current guess.
4. **Keep it there.** A proportional loop moves the guess by a fraction of what the
   detector asks for, never more than a fifth of a sample per symbol.

All of its state lives in the object between calls, so the symbols are identical, bit for
bit, however the audio is split into blocks. The carrier tracker comes after it: given a
symbol and the constellation point it should be, it turns the symbol upright and learns
the frequency offset between the two radios.

---

## 3. 5th-grade version

The radio's sound is a tune played on an 1800 Hz whistle. The front end first takes the
whistle away, so only the tune's changes are left. Then it listens through a filter shaped
exactly like one note, so each note stands out from its neighbors. Then it taps along: it
listens a little before and a little after each beat, and if the "after" side is louder, it
taps a little later next time. It never jumps; it moves its tap a little at a time, so one
bad beat cannot throw it off.

The carrier tracker is someone turning a dial so every note lands upright. If they keep
turning it the same way, they learn the drift and start turning ahead of it.

---

## 4. Where this module sits in the pipeline

```text
  48 kHz real audio, in blocks of 1 to 480 samples
     │
     ▼  BodyAudioStreamFrontend::process()
  ┌───────────────────────────────────────────────────────────────────────┐
  │ mix:     2 · x[n] · conj(body_carrier_table[n mod 80])  ──► ring (1024) │
  │ filter:  161-tap RRC, rolloff 0.35, evaluated only where it is read   │
  │ timing:  early (-10), on time, late (+10), linear interpolation       │
  │          e = Re{(late - early) · conj(on_time)} / energy              │
  │          position += 20 + clamp(0.08 · e, ±0.2)                       │
  │          (position = whole samples (uint64) + fraction (float))       │
  └───────────────────────────────────────────────────────────────────────┘
     │ BodyFrontendSymbol {early, on_time}, 2400 per second
     ▼
  acquisition: find the preamble, the mode, the start of the body   (next stage)
     ▼
  equalizer and CarrierTracker: decisions turn each symbol upright  (later stages)
```

**Inputs:** blocks of real 48 kHz audio, at most 480 samples each. **Outputs:**
`BodyFrontendSymbol` values, one per symbol. **Depends on:** `make_root_raised_cosine_taps`
(the filter), `body_carrier_table` (the carrier), and the conversion helpers in
`modem/common/convert.hpp` (`ring_slot`, `clamp_to_size`, `floor_to`).

---

## 5. Important data structures

| Name | Role |
|---|---|
| `BodyFrontendSymbol` | `{early, on_time}`. The on-time sample is the symbol; the early sample, half a symbol before it, completes the two-samples-per-symbol pair a later equalizer uses. |
| `BodyAudioStreamFrontend` | The filter taps (161 floats), the baseband ring (1024 complex samples), the count of samples written, the timing position (a 64-bit whole-sample count plus a float fraction), the loop gain and step limit, the carrier index, and a clip counter. About 8.6 KiB of state, all inside the object; no heap. |
| `body_carrier_table` | 80 phasors; entry n is exp(j·2π·3n/80), the carrier at sample n. Written as hex-float constants so every compiler reads back the same bits; the quadrant entries (0, 20, 40, 60) are exact. |
| `CarrierTracker` | The proportional and integral gains, the frequency limit, and the state: a phase in radians and a frequency in radians per symbol. |

The constants, with the reason for each:

| Constant | Value | Why |
|---|---|---|
| `matched_filter_taps` | 161 | Eight symbols at 20 samples per symbol, plus one, so the filter has a center tap. |
| `matched_filter_rolloff` | 0.35 | The default receive rolloff; `initialize` takes another. The transmitter shapes with 0.25. |
| `default_timing_loop_gain` | 0.08 | How much of each normalized timing error the loop acts on. |
| `default_maximum_timing_step` | 0.20 | The most the loop moves in one symbol, in samples. |
| `maximum_block_samples` | 480 | The largest block one `process` call takes (10 ms). |
| `maximum_block_symbols` | 26 | 480 / 20, plus two: timing corrections can slide an extra symbol into a block. |
| `raw_ring_size` | 1024 | The deepest read is 661 samples back: 160 (filter) + 10 (half a symbol) + 11 (how far production can trail) + 480 (a whole block). A `static_assert` checks it. |
| `timing_start_sample` | 170 | The first on-time point: the filter's full length (160) plus half a symbol, so the first early point has a full filter of history behind it. |
| `CarrierTracker` limit | 0.1π rad/symbol | 120 Hz at 2400 symbols per second. |

---

## 6. Function-by-function walkthrough

### `BodyAudioStreamFrontend::initialize(timing_loop_gain, maximum_timing_step, receive_rolloff)`
- **50K:** Build the filter and set the loop.
- **Detailed:** Rejects a negative (or NaN) gain and a step limit that is not positive,
  with `invalid_argument`. Builds the 161 root-raised-cosine taps for the given rolloff
  with `make_root_raised_cosine_taps` (unit energy), then calls `reset()`.
- **5th-grade:** Cut the listening filter to shape and decide how fast you are allowed to
  move your tap.

### `BodyAudioStreamFrontend::reset()`
- **50K:** Start over, ready for a new stream.
- **Detailed:** Clears the ring, the sample and symbol counts, the clip counter and the
  carrier index, and puts the first on-time point at sample 170.
- **5th-grade:** Wipe the slate and put your finger back at the start.

### `BodyAudioStreamFrontend::process(captured, output, produced)`
- **50K:** Audio in, symbols out, the same symbols whatever the block sizes.
- **Detailed:** (1) Checks: not initialized returns `unavailable`; a block of more than 480
  samples, or an output with room for fewer than block / 20 + 2 symbols, returns
  `invalid_argument`. (2) For each sample: count it if its magnitude is 0.999 or more
  (clipping); write `2 · x · conj(body_carrier_table[index])` into the ring; advance the
  index, wrapping it at 80. (3) While the late point's lookahead exists (position + 12 ≤
  samples written) and the output has room: interpolate the early, on-time and late points
  at position - 10, position and position + 10, all with the same fraction; emit
  `{early, on_time}`; compute the early-late error, divide it by the energy of the three
  points (never by less than 1e-6), scale it by the gain, clamp it to the step limit; add
  20 plus that correction to the fraction, and move its whole part into the sample count.
- **5th-grade:** Take the whistle out of every sample as it comes in; whenever there is
  enough sound past the next beat, read the beat and decide whether to tap earlier or
  later.

### `filtered_at(sample)` / `interpolate_at(sample, fraction)` (internal helpers)
- **50K:** The matched filter, computed only where the loop reads it.
- **Detailed:** `filtered_at` is the filter's output at one sample: the sum of up to 161
  tap products over the ring, skipping taps that would reach before sample 0.
  `interpolate_at` draws a straight line between `filtered_at(n)` and
  `filtered_at(n + 1)`. A symbol reads three points, each built from two filter outputs:
  6 × 161 = 966 tap products, against 20 × 161 = 3220 to filter every sample.
- **5th-grade:** Only listen closely at the moments you are going to check.

### `early_late_timing_error(early, on_time, late)`
- **50K:** Which way is the peak?
- **Detailed:** `Re{(late - early) · conj(on_time)}`. Positive when the pulse peak lies
  after the on-time point, so the loop moves later; negative when it lies before. Taking
  the conjugate of the on-time sample makes the result independent of the carrier phase, so
  the loop works before the carrier is tracked.
- **5th-grade:** Compare the sound just before and just after the beat; move toward the
  louder side.

### `CarrierTracker::configure` / `reset` / `restore` / `update` / `advance`
- **50K:** A second-order, decision-directed carrier loop.
- **Detailed:** `configure(Kp, Ki, limit)` rejects gains that are negative or not finite,
  and a limit outside (0, π]. `reset(phase, frequency)` wraps the phase and clamps the
  frequency. `restore` installs a captured state exactly as given, so a replay is bit for
  bit. `update(input, decision)` turns the input back by the current phase (and returns that
  corrected symbol), measures the remaining angle against the decision, adds Ki times that
  angle to the frequency (clamped to the limit), and steps the phase by the frequency plus
  Kp times the angle. `advance()` steps the phase by the frequency alone, for a symbol with
  no decision.
- **5th-grade:** Keep turning the dial so each note lands upright, and learn how fast you
  keep having to turn it.

---

## 7. Worked example

Real output from the unit tests, run on the two-character message "Hi" in the 600L mode.

### One symbol, mixed down (`"Mixing Part 2's tribit 6 down from 1800 Hz"`)

Body symbol 5 of the burst is tribit 6, at 270 degrees. Around its peak:

```text
   sample     audio   carrier (deg)   mixed I   mixed Q
   230498   -0.5029          243.0   +0.4566   -0.8961
   230499   -0.5652          256.5   +0.2639   -1.0991
   230500   -0.5872          270.0   -0.0000   -1.1744
   230501   -0.5693          283.5   -0.2658   -1.1072
   230502   -0.5165          297.0   -0.4689   -0.9203
after the matched filter: on-time (-0.016, -2.391), 269.6 degrees, tribit 6
```

Each mixed sample is the symbol plus a copy of it turning the other way at 3600 Hz. The
filter averages the copy away.

### The detector on the receive pulse (`"The early-late error is zero on the peak..."`)

```text
on the peak +0.000000, 4 samples early +0.031043, 4 samples late -0.031043,
early and turned by 1 rad +0.031043
```

### The whole burst

```text
"Hi" burst: 460800 samples, 23032 symbols, identical bit for bit in blocks of 480
            and in blocks of 1, 7, 33, 160 and 480 in turn
locked by symbol 2500; 20536 of 20536 decisions match
```

### The carrier tracker, with the receiver's gains (Kp 0.02, Ki 0.0002)

```text
fixed 0.5 rad offset: learned 0.5000 rad, last symbol 0.00000 rad from its decision
5 Hz offset: learned 5.000 Hz, last symbol 0.00000 rad from its decision
```

---

## 8. Equations and formulas

```text
carrier:        c[k] = exp(j · 2π · 3k / 80)                  k = n mod 80, 13.5° per sample
mix:            b[n] = 2 · x[n] · conj(c[n mod 80])
filter:         y[n] = Σ h[k] · b[n - k],  k = 0 .. min(n, 160)
interpolate:    y(n + μ) = y[n] + μ · (y[n + 1] - y[n])        0 ≤ μ < 1
timing error:   e = Re{ (L - E) · conj(O) }                    E, O, L at τ - 10, τ, τ + 10
loop:           τ ← τ + 20 + clamp( g · e / (|E|² + |O|² + |L|²),  ±s )     g = 0.08, s = 0.2

carrier loop:   corrected = r · exp(-jθ)
                e = arg( corrected · conj(decision) )
                f ← clamp( f + Ki · e,  ±f_max )
                θ ← wrap( θ + f + Kp · e )
                Hz = f · 2400 / 2π                             f_max = 0.1π → 120 Hz
```

---

## 9. What can go wrong

| Symptom | Cause | What the code does, or what to do |
|---|---|---|
| The constellation points bend or smear | The input clips. | `clipped_samples()` counts samples at magnitude 0.999 or more. Lower the audio level. |
| `invalid_argument` from `process` | A block of more than 480 samples, or an output too small for it. | Split the audio into blocks of at most 480; size the output by `maximum_block_symbols`. |
| Symbols rotate slowly | The two radios' carriers differ; the front end does not remove that. | `CarrierTracker`, once decisions are available. Its limit is 120 Hz. |
| Symbols sit off the peak under a clock offset | A proportional loop needs a standing error to make a standing correction. | At 100 ppm the loop sits 0.74 samples off the eye; at 1000 ppm (20 dB signal-to-noise) it loses lock. A loop with an integrator fixes it. |
| Errors in noise | Noise, not timing. | White noise at 10 dB signal-to-noise (in a 3 kHz band): about 5.5% of symbols wrong, rms phase error about 11.7 degrees. |
| Errors with a tone on the carrier | Interference inside the band. | A CW tone 10 dB below the signal, at 20 dB signal-to-noise: 4.66% of symbols wrong. |
| Most decisions wrong in fading | Echoes, which no front end removes. | Two-path Watterson fading (2 ms, 1 Hz, 20 dB): the front end alone gets 3489 of 19736 decisions right. That is the equalizer's job. |
| Timing walks off after many minutes | A `float` position cannot hold a fraction past 2^24 samples (about six minutes), nor add 20 past 2^26 (about 23 minutes). | The position is a 64-bit whole-sample count plus a float fraction. The 30-minute test (188 bursts, 86,630,400 samples) gets every decision right. |

---

## 10. Debugging signals to watch

| Signal | What it tells you |
|---|---|
| `clipped_samples()` | Nonzero means the input level is too high. |
| `symbols_produced()` | Should grow by 2400 a second of audio. |
| On-time magnitude | Settles to a steady value once the loop has locked (about 2.4 on the test signal). The test bench calls the loop locked when a 200-symbol average reaches 99% of the settled value. |
| On-time angle | Clusters at the eight phases (multiples of 45 degrees), turned by any carrier offset. On a clean signal each sample sits within a degree of its phase. |
| Timing error | Averages to about zero once locked; a steady nonzero average means a clock offset. |
| `CarrierTracker::frequency_radians_per_symbol()` | Times 2400 / 2π gives hertz. Sitting at the limit means the offset is out of range. |

Healthy: the magnitude steady, the angles in eight tight clusters, the timing error
centered on zero. Unhealthy: the magnitude pumping up and down (fading, or a loop that has
not locked), the clusters smeared into a ring (a carrier offset not yet tracked), or the
clip count climbing.

---

## 11. How to read this module

1. `body_audio_stream_frontend.hpp`, top to bottom: the constants and their reasons, and
   the `static_assert` on the ring.
2. `process()` in `body_audio_stream_frontend.cpp`: the mixing loop first, then the loop
   that produces symbols.
3. `filtered_at` and `interpolate_at`, then `early_late_timing_error` in the header.
4. `CarrierTracker::update` in `modem/common/dsp/timing_recovery.cpp`: five lines.
5. `front_end_demo`, then `tests/walkthrough/front_end_steps.cpp`: the demo runs the
   article's steps in order and prints every number it quotes, and each step is one function
   there. `tests/front_end_tests.cpp` calls the same functions and checks what they return.
   Set a breakpoint in a step and step through it from either.

---

## 12. Maintainer notes

- **The block-split invariant is a contract.** Every piece of state lives in the object,
  and the carrier index counts samples, not blocks. The test "Any block size gives the same
  symbols bit for bit" checks it; never reset anything per block.
- **The ring is sized at compile time.** Growing `maximum_block_samples` or the filter must
  keep the `static_assert` beside the ring true.
- **The filter runs only where it is read.** A detector that needs more points costs 322 tap
  products per extra point; count them before adding one.
- **The carrier comes from the table.** Do not bring back a running float phase or a sine
  and cosine per sample: the float sum drifts (about 1.4 degrees over the "Hi" burst), and
  math libraries differ in the last bit.
- **The filter taps still come from the library's cosine**, computed once by `initialize`.
  Constant taps would make them the same on every platform.
- **The rolloff, the gain and the step limit are qualified values.** The receiver's
  conformance results were measured with 0.35, 0.08 and 0.2; change one and qualify the
  receiver again.
- **Keep the position as whole samples plus a fraction.** A `double` would also work on a
  host, but it is emulated in software on the Cortex-M33.

---

*Companion to the transmit-stage explainers in `modem/m110a/`. The carrier table it shares
with the transmitter is walked through in `modem/m110a/body-waveform-and-scrambler-explainer.md`.*
