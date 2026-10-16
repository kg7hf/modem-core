# Synchronization and Acquisition Explainer

## 1. Scope

This note explains the receiver's second stage:

- `modem/m110a/synchronization.hpp` / `.cpp`: **acquisition**. Given symbol-rate complex
  baseband from the front end, `acquire_body_preamble` finds a MIL-STD-188-110B
  serial-tone burst, measures its carrier offset and phase, reads its mode and countdown,
  and works out the symbol where the data body begins.
- The candidate primitives it is built from (`make_body_prefix_reference`,
  `estimate_body_prefix_frequency_radians`, `body_prefix_correlation`,
  `characterize_body_candidate`, `body_candidate_is_valid`), public so a streaming
  receiver can test one position at a time instead of scanning a whole buffer.

Two additions to the front end belong with this stage, because a receiver turns them on
and off around acquisition: the **phase search** and the **rate integrator** in
`BodyAudioStreamFrontend`. They are explained in
`modem/m110a/audio-front-end-explainer.md`, sections 6 and 9, and walked through in Steps
11 to 14 below.

The unit tests are in `tests/acquisition_tests.cpp`, and the article series explains the
design in Part 4, "Acquisition: finding the burst". Nothing here decodes a payload bit:
the output is the reference the equalizer and the decoder start from.

---

## 2. 50,000-foot view

The front end hands over one complex number per symbol, 2400 a second, whether a burst is
there or not. Every burst begins with a **preamble** the receiver knows in advance. From
the samples alone, acquisition answers:

```text
Is a burst present?                  -> correlation against the known prefix
Where does it start?                 -> the start that matches
How far off is the carrier?          -> differential phase over the prefix
What is the carrier phase?           -> the angle of the coherent correlation
Which mode is it?                    -> the D1/D2 designators
How many segments until the body?    -> the C1/C2/C3 countdown
Where does the body begin?           -> the countdown, counted forward
```

The preamble is made of **segments** of 480 symbols (0.2 s), each 15 "channel symbols"
of 32 chips. The first nine channel symbols, 288 chips, are the fixed **acquisition
prefix**, the same in every segment of every burst. Then come the two designators and the
three countdown digits, and one channel symbol of zeros. A short-interleave preamble has 3
segments (0.6 s); a long one has 24 (4.8 s). The countdown is what lets a receiver that
wakes up in the middle of the preamble know exactly how far away the body is.

The scan runs every start position through three gates, cheapest first (the carrier
offset in range, the correlation over a floor, a valid mode and countdown), groups the
survivors by the burst they describe, takes the group the most segments support, and lets
every segment of the preamble vote on the mode and the boundary. The result is one
`BodyAcquisition`.

---

## 3. 5th-grade version

Before every secret message, your friend sings the same jingle on a walkie-talkie, over and
over, with a short code and a countdown after each one: "jingle, code 4-6, twenty-three
left; jingle, code 4-6, twenty-two left; ..." You know the jingle by heart.

So you listen for it everywhere. When you hear it clearly, you learn several things at
once: a message is coming, how far off your friend's pitch is today, which code they are
using, and how many jingles are left before the real words. Even if you only tuned in
halfway through, the countdown tells you when the words start.

The room is noisy, so you might mishear one countdown. That is fine: you heard many of
them, and they all have to agree on when the words start. You go with what most of them
said.

---

## 4. Where this module sits in the pipeline

```text
  48 kHz real audio
     │
     ▼  BodyAudioStreamFrontend (Part 3; the explainer next to this one)
  ┌──────────────────────────────────────────────────────────────────────┐
  │ mix down, matched filter, early-late timing                          │
  │ + phase search  (on while looking for a preamble, off at commit)     │
  │ + rate integrator (learns the clock offset from the first candidate) │
  └──────────────────────────────────────────────────────────────────────┘
     │ one complex symbol per 2400-baud symbol (the on-time samples)
     ▼  acquire_body_preamble()                        THIS MODULE
  ┌──────────────────────────────────────────────────────────────────────┐
  │ 1. scan_for_consensus   every start through three gates:            │
  │      estimate_body_prefix_frequency_radians   |carrier| <= 120 Hz    │
  │      body_prefix_correlation                  >= minimum_correlation │
  │      characterize_body_candidate              a valid mode/countdown │
  │    survivors grouped by the burst they describe                      │
  │ 2. strongest_group      the group the most segments support          │
  │ 3. vote_mode_and_boundary  every segment votes on mode and boundary  │
  └──────────────────────────────────────────────────────────────────────┘
     │ BodyAcquisition {preamble start, body start, carrier, phase, mode}
     ▼
  equalizer, carrier tracker, deinterleaver, decoder   (later articles)
```

**Inputs:** symbol-rate complex baseband, at least one 480-symbol segment of it, and a
`BodyAcquisitionConfig`. **Outputs:** a `BodyAcquisition`, or a `Status` saying why not
(`invalid_argument` for a bad request, `unavailable` when no start passed the gates).
**Depends on:** `body_waveform.hpp` (the preamble's structure, `body_channel_symbol_tribit`,
`recognize_body_designators`, `body_preamble_segments`), `scrambler.hpp`
(`SyncRandomizer`), `demapper.hpp` (`psk8_symbol`).

---

## 5. Important data structures

| Name | Role |
|---|---|
| `BodyAcquisitionConfig` | The symbol rate (2400), the correlation floor (`minimum_correlation`, 0.75), the carrier gate (`maximum_frequency_offset_hz`, 120), and how far into the buffer a start may be (`maximum_start_symbol`, 0 for anywhere). |
| `BodyAcquisition` | The result. The preamble's first symbol, the segment the decision was read from, the body's first symbol, the countdown read there, whether the preamble's start is inside the buffer, the carrier offset (Hz) and phase, the normalized correlation, and the mode (`recognition`). |
| `BodyModeRecognition` | From `body_waveform.hpp`: `supported` with the data rate and interleave, `recognized_unsupported` (a legal pair of designators for a mode this modem does not implement), or `invalid`. |
| `AcquisitionConsensus` (internal) | One group: the candidates that agree on the body's first symbol and the mode. It keeps a representative (the member with the **highest** correlation), the number of supporting segments, the sum of their correlations and the best one. |
| `ConsensusScan` (internal) | Up to 32 groups. A start that would open a 33rd is dropped. |

### One preamble segment

```text
One segment = 15 channel symbols x 32 chips = 480 symbols (0.2 s)

  channel symbol:  0 1 2 3 4 5 6 7 8 │ 9  10 │ 11 12 13 │ 14
                   0 1 3 0 1 3 1 2 0 │ D1 D2 │ C1 C2 C3 │ 0
                   acquisition prefix  mode    countdown
                   (288 chips, the same
                    in every segment)

  Each chip is one 8-PSK symbol: the channel symbol's spreading pattern plus the sync
  randomizer, mod 8. Each countdown digit is sent as 4 | digit, so a decoded value
  below 4 is not a credible segment.

  countdown = ((C1 & 3) << 4) | ((C2 & 3) << 2) | (C3 & 3)
  the first segment carries segments - 1, the last carries 0,
  and the body begins one segment after the last.
```

### Time and size cheat sheet

```text
symbol rate ................. 2400 symbols/s
chips per channel symbol .... 32
acquisition prefix .......... 9 channel symbols = 288 chips = 120 ms
segment ..................... 15 channel symbols = 480 symbols = 200 ms
short / long preamble ....... 3 segments (1440 symbols) / 24 segments (11520 symbols)
carrier gate ................ 120 Hz = 0.314 rad per symbol
estimator's range ........... ±1200 Hz (±π rad per symbol); past it the estimate wraps
```

---

## 6. Function-by-function walkthrough

### `make_body_prefix_reference(reference)`
- **50K view:** Writes the 288-chip prefix every correlation compares against.
- **Detailed view:** For each of the nine prefix channel symbols `{0,1,3,0,1,3,1,2,0}`, it
  spreads the symbol over 32 chips with `body_channel_symbol_tribit`, adds the next
  `SyncRandomizer` tribit, and maps the sum (mod 8) to 8-PSK with `psk8_symbol`. The
  scanner builds it once, on first use, into static storage (`prefix_reference()`), so it
  costs no stack and no time after the first call.
- **5th-grade view:** Writing the jingle down so you can compare what you hear with it.

### `estimate_body_prefix_frequency_radians(input, start, reference)`
- **50K view:** The carrier offset over the prefix at one start, in radians per symbol.
- **Detailed view:** A differential estimator. Each product
  `r[k]·conj(r[k-1])·conj(c[k])·c[k-1]` strips the known modulation and leaves the phase
  the carrier turned through in one symbol; the angle of their sum is the estimate. It is
  exact to ±π per symbol (±1200 Hz); past that it wraps (Step 5: +1300 Hz reads as
  -1100 Hz). The scanner gates it at `maximum_frequency_offset_hz`, the limit of the
  carrier tracker downstream.
- **5th-grade view:** Comparing each note with the one before it to hear how far the pitch
  has drifted.

### `body_prefix_correlation(input, start, reference, frequency_radians, phase)`
- **50K view:** A score from 0 to 1 for "is the prefix here?", and the carrier phase.
- **Detailed view:** Each 32-chip block is correlated coherently against the reference,
  turned by the frequency estimate, and the **magnitudes** of the nine block sums are
  added; the total is divided by the square root of the received energy times 288. On a
  steady channel this equals one long coherent sum. When a fade turns the phase partway
  through the prefix, one long sum cancels and the block magnitudes do not (Step 3: 0.111
  against 1.000 with the last 128 chips turned 180 degrees). The phase returned is the
  angle of the whole coherent sum.
- **5th-grade view:** Matching the jingle in nine short pieces, so a wobble in the middle
  spoils one piece instead of the whole match.

### `decode_channel_symbol(input, start, channel_index, frequency_radians)` (internal)
- **50K view:** Reads one designator or countdown digit: which of the eight 32-chip codes
  is it?
- **Detailed view:** For each candidate 0 to 7 it rebuilds the expected 32 chips (code,
  sync randomizer, and the frequency ramp within the block) and correlates. The eight
  codes are orthogonal, so the largest magnitude is the best decision with the phase
  unknown. Each candidate is scored as `|lag 0|² + max over lags 3..15 of |lag|²`, the
  direct path plus its best echo, so a channel with a faded direct path and a strong echo
  up to 15 symbols late can still be read.
- **5th-grade view:** Holding up all eight letters to see which one the sound matches, and
  letting the echo help when the direct sound is weak.

### `characterize_body_candidate(input, candidate, config)`
- **50K view:** Turns "a prefix may start here" into a described candidate: the mode, the
  countdown, the body's first symbol, and the preamble's first symbol.
- **Detailed view:** Reads D1, D2, C1, C2 and C3 (channel symbols 9 to 13). A countdown
  digit below 4 marks the candidate invalid. It packs the countdown, asks
  `recognize_body_designators` for the mode (long interleave first, then short), and for a
  supported mode counts forward and back: the body begins `(countdown + 1) × 480` symbols
  after this segment, and the preamble began `(segments - countdown - 1) × 480` symbols
  before it. If that start is inside the buffer it moves `first_preamble_symbol` there and
  turns the carrier phase back to match.
- **5th-grade view:** Hearing "code 4-6, twenty-three left" and working out when the words
  start and when the jingles began.

### `body_candidate_is_valid(candidate)`
- **50K view:** True for a supported mode whose countdown fits its preamble, with the body
  at least one segment ahead.

### `evaluate_start` (internal)
- One start through the three gates, cheapest first: the carrier estimate within the gate,
  the correlation over the floor, then the decode.

### `scan_for_consensus` (internal, Step 1 of `acquire_body_preamble`)
- **50K view:** Every start through the gates; survivors grouped by the burst they
  describe.
- **Detailed view:** A survivor joins the group with the same body start and mode, or
  opens a new one. The group's representative is the member with the highest correlation,
  not the earliest: the representative supplies the carrier estimate the receiver trains
  with, and a fade can push an early segment over the floor with a wrong one.

### `strongest_group` (internal, Step 2)
- The group with the most supporting segments; a tie goes to the larger correlation sum,
  then to the larger single correlation.

### `vote_mode_and_boundary` (internal, Step 3)
- **50K view:** Every segment from the winner's on votes on the mode and the body start.
- **Detailed view:** The winner's mode and countdown came from one segment's decode, and at
  low signal-to-noise one decode can be wrong. Every later segment carries the same
  designators and a countdown that names the same body start, so each is read again with
  the winner's carrier estimate, and the votes are grouped like the scan's candidates.
  Segments past the body read as invalid and abstain. If at least two segments agree on
  something other than the winner, the majority's mode, body start and preamble start are
  adopted, and the winner's carrier estimate is kept.
- **5th-grade view:** Going with what most of the countdowns said.

### `acquire_body_preamble(symbol_rate_input, config)`
- Checks the request, takes the static reference, and runs the three steps. It returns
  `unavailable` when no start passes all three gates.

---

## 7. Worked example

Real output from `acquisition_demo` (the article's 14 steps), on the 600L mode: the message
"Hi" through the transmitter, then a channel at 10 dB signal-to-noise with the carrier
37 Hz off. The burst starts at symbol 1196 of what the receiver hears. The walkthrough
acquires at a 0.35 correlation floor, the floor the streaming receiver later in the series
runs at; the library's default is 0.75.

### The comb (Step 4)

Every start that reaches the floor, counted from the burst's start:

```text
  start  correlation  carrier (Hz)
   -256        0.354         +46.6
   -160        0.362         +29.1
    -96        0.396         +58.1
      0        0.959         +35.2
    480        0.959         +36.2
    960        0.965         +37.6
largest correlation anywhere else, inside the carrier gate: 0.308, at start 938
```

One clean match a segment, each with its own carrier estimate. The three weak starts before
the burst are the burst too: whole channel symbols early (3, 5 and 8 of them), they lay part
of the reference over the burst's first prefix, and because the prefix repeats itself (0, 1,
3, then 0, 1, 3 again) some of the overlapping blocks match. Reading the mode and the
countdown takes them out.

### Reading the first valid segment (Step 6)

```text
start 940: correlation 0.354, carrier +46.6 Hz, but not a valid segment
start 1036: correlation 0.362, carrier +29.1 Hz, but not a valid segment
start 1100: correlation 0.396, carrier +58.1 Hz, but not a valid segment
first start through all three gates: symbol 1196 (the burst starts at 1196)
  correlation 0.959, carrier +35.2 Hz
  D1 4, D2 6: 600 bps, long interleave
  countdown 23: the body starts 24 segments on, at symbol 12716 (really 12716)
  so the preamble began at symbol 1196
```

The countdown digits are C1 5, C2 5, C3 7: `(1 << 4) | (1 << 2) | 3 = 23`, and
`1196 + (23 + 1) × 480 = 12716`.

### Waking up mid-preamble (Step 7)

```text
the receiver starts 4937 symbols into the burst (segment 10, symbol 137)
  it read the countdown from burst segment 13: 10, so 11 segments to the body
  body starts at symbol 6583 of what it heard (really 6583)
  preamble start in what it heard: no
```

### The vote (Step 9)

At -4 dB, each segment read on its own; segment 20 read a countdown of 19:

```text
right 15, wrong 1, not valid 8
grouped by the burst they describe:
  600 bps, body at 12716: 15 segments
  600 bps, body at 20396: 1 segment
acquisition: right
```

### Noise alone (Step 10)

```text
10 s of noise: largest correlation 0.264 against the 0.35 floor
  starts through both gates 0, of those reading as a valid segment 0; acquired: no
```

### The front end's part (Steps 11 to 14)

```text
2400S, 10 dB, the burst started at each of the 20 sample phases of a symbol:
pulled in before symbol 1200: 17 of 20 plain, 20 of 20 with the phase search

20 dB, 10 s of 2400L data (a 14.4 s body):
                            +1000 ppm, type 1: 30 symbols slipped
                            +1000 ppm, type 2: 0 slipped, the integrator 6 ppm off
```

---

## 8. Equations and formulas

```text
Rs = 2400 symbols/s,  Np = 288 prefix chips,  Ns = 480 symbols per segment

carrier estimate:   ω = arg( Σ_{k=1}^{Np-1}  r[k] · conj(r[k-1]) · conj(c[k]) · c[k-1] )
                    f = ω · Rs / 2π                 gate: |f| ≤ 120 Hz

correlation:        b_m = Σ_{k in block m} r[k] · conj( c[k] · exp(jωk) )     m = 0 .. 8
                    ρ   = Σ_m |b_m|  /  sqrt( Np · Σ_k |r[k]|² )             0 ≤ ρ ≤ 1
                    phase = arg( Σ_m b_m )

channel symbol:     Λ_ℓ(n) = Σ_{chip} r[start + 32i + chip + ℓ] · conj( expected_n[chip] )
                    n̂ = argmax_n ( |Λ_0(n)|² + max_{3 ≤ ℓ ≤ 15} |Λ_ℓ(n)|² )      n = 0 .. 7

countdown:          D = ((C1 & 3) << 4) | ((C2 & 3) << 2) | (C3 & 3)
body start:         detected_segment + (D + 1) · Ns
preamble start:     detected_segment - (segments - D - 1) · Ns
```

---

## 9. What can go wrong

| Symptom | Cause | What the code does, or what to do |
|---|---|---|
| Nothing acquired, the carrier far off | The estimate is past the 120 Hz gate, or past ±1200 Hz, where it wraps. | Nothing: 120 Hz is the carrier tracker's limit. Step 5 shows the gate. |
| A wrong carrier estimate on the winner | A fade inside the prefix biases the estimate of one segment. | The group's representative is its highest-correlation member, not its earliest. |
| One segment reads the wrong countdown or mode | Noise, at the lowest signal-to-noise the standard allows. | `vote_mode_and_boundary`: every segment votes. Step 9. |
| A fade turns the phase within the prefix | The channel changes inside 120 ms. | The correlation adds nine block magnitudes, not one coherent sum. Step 3. |
| A mode is recognized but not acquired | The designators are legal but this modem does not implement the mode (D1 7, D2 7 is 2400 bps secure voice). | `recognized_unsupported`; the candidate is invalid. Step 8. |
| On a two-path channel the boundary is one echo late | The strongest group can be the later path. | Choosing the earliest path belongs with the equalizer, a later article. |
| The front end hangs half a symbol off the peaks | The early-late detector's output is also zero there. | The phase search (front end), on while acquiring. Step 11. |
| Symbols slip under a clock offset | A proportional loop needs a standing error for a standing correction. | The rate integrator (front end). Steps 12 to 14. |
| A rich scan loses a group | More than 32 groups. | The 33rd is dropped; on a real channel the count stays small. |

---

## 10. Debugging signals to watch

| Signal | What it tells you |
|---|---|
| `normalized_correlation` | Near 1 on a clean channel (0.96 at 10 dB in Step 4); near the floor means marginal. |
| `frequency_offset_hz` | Close to the gate means the edge of the range; segments of one burst should agree within a few hertz. |
| `first_body_symbol - detected_segment_symbol` | Must be `(preamble_countdown + 1) × 480`. |
| `preamble_start_in_buffer` | False when the receiver woke up after the preamble began (Step 7). |
| `recognition.state` | `supported`, `recognized_unsupported` or `invalid`. |
| The `Status` | `invalid_argument` for a bad config or under one segment of input; `unavailable` when nothing passed the gates. |

Healthy: one match a segment, 480 symbols apart, correlations near 1, carrier estimates
within a few hertz of each other, every segment voting for the same body start.
Unhealthy: matches that are not 480 apart, carrier estimates that scatter by tens of
hertz, or votes split between body starts.

---

## 11. How to read this module

1. `synchronization.hpp`: the two structs and the public functions.
2. In `synchronization.cpp`, the three primitives in order: `make_body_prefix_reference`,
   `estimate_body_prefix_frequency_radians`, `body_prefix_correlation`.
3. `decode_channel_symbol` and `characterize_body_candidate`: how a match becomes a mode
   and a body start.
4. `acquire_body_preamble` and its steps: `evaluate_start`, `scan_for_consensus`,
   `strongest_group`, `vote_mode_and_boundary`. Each is one short function.
5. `acquisition_demo`, then `tests/walkthrough/acquisition_steps.cpp`: the demo runs the
   article's steps in order and prints every number it quotes, and each step is one
   function there. `tests/acquisition_tests.cpp` calls the same functions and checks what
   they return. Set a breakpoint in a step and step through it from either.
6. `runner`: the receive chain end to end, on a message of your own or a WAV file.

---

## 12. Maintainer notes

- **The representative is the highest-correlation member, never the earliest.** It
  supplies the carrier estimate the receiver trains with. A fade-distorted early segment
  that clears the floor with a wrong estimate (measured: +7.6 Hz, where a later member of
  the same group read 1.0 Hz and decoded exactly) otherwise displaces a clean one.
- **Noncoherent where the channel moves.** The correlation adds block magnitudes, and the
  channel-symbol decision adds the direct and echo energies. A refactor to one coherent sum
  brings back the loss under fast fading.
- **The reference lives in static storage.** Built on the stack it costs 2.3 KB in the
  scanner's frame and puts `acquire_body_preamble` over the 4 KB stack gate.
- **The vote keeps the winner's carrier estimate.** It changes the mode and the boundary,
  never the frequency, and it needs two votes to overrule.
- **The floor is a trade.** 0.75 (the default) keeps noise out with margin (10 s of noise
  peaks near 0.26). The walkthrough runs at 0.35, the streaming receiver's floor, because
  the floor is only the second of three gates: a start in noise must also pass the carrier
  gate and read as a valid segment.
- **The primitives are the streaming API.** A receiver that tests one rolling position a
  step uses them directly, with the same conventions as the batch scan.

---

*Companion to `modem/m110a/audio-front-end-explainer.md`, the stage before this one. The
preamble's structure, the designators and the countdown, as the transmitter builds them,
are in `modem/m110a/body-waveform-and-scrambler-explainer.md`.*
