# Body Waveform, Framing, and Scrambler Explainer

## 1. Scope

This note explains six source files that define **how a serial-tone body burst is
built on the air** (and so what a receiver must expect):

- `modem/common/waveform/waveform.hpp` / `waveform.cpp`: the **waveform tables**, the
  authoritative map from a `WaveformConfig` (family + rate + interleave) to a
  `WaveformDescriptor` (modulation, FEC, spreading) and an `InterleaverSpec`.
- `modem/m110a/body_waveform.hpp` / `body_waveform.cpp`: the **serial-tone body
  waveform** (the block plan, preamble generation, per-block encode, the probe
  framing, EOM framing, and audio rendering). The header also declares the
  receive-side counterparts; their implementations arrive with the receiver series.
- `modem/m110a/scrambler.hpp` / `scrambler.cpp`: the two **scramblers** (the body data
  randomizer for whitening, and the sync sequence for the preamble) plus the
  modified-Gray constellation mapping.

Where the other explainers cover *stages* (interleaving, symbol mapping, transmit
orchestration), this one covers *structure*: the exact layout of symbols the
transmitter emits and a receiver expects.

---

## 2. 50,000-foot view

A serial-tone body transmission has a fixed anatomy:

```text
┌──────────────────┬───────────────────────────────────────────────┬───────┬────────┐
│  PREAMBLE        │  BODY  (a sequence of DATA BLOCKS)             │  EOM  │ FLUSH  │
│  N × 480 symbols │  each block = data frames of                  │  32   │  144   │
│  (countdown)     │  [unknown DATA symbols | known PROBE symbols] │  bits │  bits  │
└──────────────────┴───────────────────────────────────────────────┴───────┴────────┘
   lets a receiver       carries the payload; probes let a               end-of-message
   sync + read the mode  receiver re-learn the channel each frame         + trellis flush
```

- The **preamble** is `N` repeats of a 480-symbol segment carrying the mode
  designators and a countdown.
- The **body** is a stream of fixed-size **blocks**. Each block interleaves **unknown
  data symbols** (the payload, FEC-coded and whitened) with **known probe symbols**
  (a fixed pattern a receiver trains on to track the fading channel). For most rates
  a frame is 20 data + 20 probe symbols; for 2400/4800 it is 32 data + 16 probe. Every
  short block is exactly **1440 symbols (0.6 s)**; a long-interleave block is ×8.
- The payload ends with a 32-bit **EOM** word (`0x4B65A5B2`) and a 144-bit **flush**.

This file is the shared contract: the transmitter builds exactly this, and every
receiver stage is sized and framed by the `BodyBlockPlan` computed here.

---

## 3. 5th-grade version

Think of sending a long letter by radio. You agree on a strict format:

- First a **doorbell jingle** that repeats with a countdown ("...3, ...2, ...1, go") so the
  listener knows a letter is coming and exactly when it starts. (preamble)
- Then the letter itself, but every few words you insert a **known checkpoint phrase**
  both of you memorized. When the radio gets scratchy, the listener uses those known
  phrases to re-tune their ears. (data + probe frames)
- You also **shuffle the letters' "voice"** with a shared secret pattern so the signal
  stays lively and even (no long boring stretches that confuse the radio). (scrambling)
- At the very end you say a special **"THE END" codeword** so the listener knows to
  stop. (EOM)

This file is the rulebook for that exact format: writing it (transmit) and, with the
receiver series, reading it back (receive).

---

## 4. Where this module sits in the pipeline

```text
  TRANSMIT
  ────────
  payload octets
     │ unpack (LSB-first) + append_body_eom_and_flush
     ▼
  body_block_plan(mode)
     │
     ▼ encode_body_block (per block):
       FEC encode ─► interleave ─► map+whiten    <── waveform tables (interleaver_for)
       + insert known probes
     ▼
  body_tribits_to_audio (RRC + 1800 Hz)
     ▼
  audio out
```

**Inputs (encode):** payload bits + a `BodyBlockPlan`. **Outputs:** transmitted
tribits / audio. The receive mirror is declared in `modem/m110a/body_waveform.hpp`
(received symbols + the same plan in; soft metrics, information bits, or payload
octets out) and arrives with the receiver series.
**Depends on:** `modem/common/fec/convolutional_k7.hpp` (the K=7 encoder),
`modem/m110a/interleaver.hpp`, `modem/m110a/demapper.hpp` (`psk8_symbol`),
`modem/m110a/scrambler.hpp` (whitening/sync/Gray), and
`modem/common/dsp/timing_recovery.hpp` (`make_root_raised_cosine_taps`, the RRC taps
for audio). It is the integrator that wires these into a burst.

---

## 5. Important data structures

| Type | Purpose |
|---|---|
| `WaveformDescriptor` | One row of the mode table: family, rate, modulation, FEC profile, bits/symbol, spreading, frame geometry. 16 rows (7 serial-tone body + 5 Appendix C + 9600-and-up Appendix F). |
| `InterleaverSpec` | The interleaver matrix/permutation for a mode (see `modem/m110a/demapper-and-deinterleaver-explainer.md`). |
| `BodyMode` / `BodyDesignators` | The rate+interleave, and its two on-air D1/D2 designator symbols. |
| `BodyBlockPlan` | **The per-mode geometry** every receiver stage is sized by: information/coded bits, data/probe symbols per frame, bits per symbol, FEC repetitions, transmitted symbols per block. |
| `BodyEncodeState` / `BodyDecodeState` | The running K=7 encoder state threaded **across blocks** so the FEC is one continuous stream (not restarted per block). `BodyDecodeState` is the receive-side twin, for the declared `decode_body_block`. |
| `BodyAudioStreamModulator` | Stateful RRC pulse-shaper + 1800 Hz upconverter for rendering a long transmission in codec-sized windows. |
| `BodyDataRandomizer` / `SyncRandomizer` | The two scramblers (§6). |

### The body block plan, by rate (short interleave; ×8 for long)

| Rate | info bits | coded bits | data symbols | frame data/probe | bits/sym | FEC |
|---|---:|---:|---:|:---:|:---:|:---|
| 75 | 45 | 90 | 45 (Walsh-spread) | 32-chip orthogonal | 2 (dibit) | rate 1/2 + spread |
| 150 | 90 | 720 | 720 | 20 / 20 | 1 | rate 1/2 ×4 |
| 300 | 180 | 720 | 720 | 20 / 20 | 1 | rate 1/2 ×2 |
| 600 | 360 | 720 | 720 | 20 / 20 | 1 | rate 1/2 |
| 1200 | 720 | 1440 | 720 | 20 / 20 | 2 | rate 1/2 |
| 2400 | 1440 | 2880 | 960 | 32 / 16 | 3 | rate 1/2 |
| 4800 | 2880 | 2880 | 960 | 32 / 16 | 3 | uncoded (bypass interleave) |

Every short block totals **1440 transmitted symbols**; a long block is ×8 (11,520).

### Repetition modes vs. Walsh

- **150 (×4) and 300 (×2) are the repetition modes:** rate-1/2 coded, then each coded
  pair is emitted 4× / 2× and summed back on receive by `body_block_soft_metrics`
  (repetition-combine; declared in the header, arriving with the receiver series).
- **600 / 1200 / 2400 are plain rate 1/2** (no repetition); **4800 is uncoded**.
- **75 is the Walsh mode:** each information dibit is spread across a 32-chip
  orthogonal sequence for large processing gain.

---

## 6. Function-by-function walkthrough

### `waveform.cpp`: the tables

- **`descriptor_for` / `validate` / `interleaver_for`:** Look up the `WaveformConfig`
  in the 16-row descriptor table, validate that the interleaver type matches the family
  (and the 4800-bps bypass rule), and return the `WaveformDescriptor` or the
  `InterleaverSpec`. The body interleaver is a 40×N matrix (load step 9, fetch step 17;
  75 bps is 20×36 / 10×9 with step 7); for the Appendix C/F rows the spec is a
  multiplicative permutation with a depth scale (those rows are table data here; this
  repository implements the serial-tone body). **50K:** the single source of truth for
  "what does this mode look like." **5th-grade:** the settings dial that tells every
  other part which mode you're in. The `consteval` self-checks (uniqueness,
  bits/symbol, interleaver coprimality) make a malformed table a *compile* error.

### `scrambler.hpp` / `scrambler.cpp`: the two scramblers + Gray map

- **`BodyDataRandomizer`** (**whitening**): a 12-bit LFSR, seed `0xBAD`
  (MIL-STD-188-110B Figure 6), 8 clocks per output tribit, output = the low 3 bits,
  reset every 160 symbols. Its tribit is **added mod 8** to every transmitted body
  symbol, data and probe alike (and subtracted on receive by derotating). This
  "whitens" the signal so the spectrum stays even regardless of the data.
  **5th-grade:** a shared shuffling pattern that keeps the voice lively.
- **`SyncRandomizer`** (**preamble**): a fixed 32-entry sequence cycled by index; it
  is the sync scramble applied to every preamble symbol. It's periodic and
  segment-aligned, which is why a fresh randomizer can regenerate any segment-aligned
  suffix (used by `generate_body_preamble_tail`).
- **`modified_gray_decode(value, width)`** (**constellation mapping**): 1 bit → {0,4};
  2 bits → `{0,1,3,2}`; 3 bits → `{0,1,3,2,7,6,4,5}` (Gray-ordered). `mapped_tribit`,
  next to it in `scrambler.hpp`, adds the 2-bit shift; the transmit mapper in
  `encode_body_block` calls it (and the soft demappers declared in
  `modem/m110a/demapper.hpp` are defined over the same mapping), so adjacent points
  differ in one bit everywhere.

### `body_waveform.hpp` / `body_waveform.cpp`: the body burst

- **`body_block_plan(mode)`:** Fills the `BodyBlockPlan` table above from the mode
  (`constexpr`, in `body_waveform.hpp`, so the whole table folds at compile time).
  Everything downstream is sized by this. **This is the function to read first.**
- **`body_designators` / `recognize_body_designators`:** The map between a mode and its
  on-air D1/D2 symbols (e.g. 600 short = {6,6}, 600 long = {4,6}). `body_designators`
  is the `constexpr` forward direction; the reverse, `recognize_body_designators`, is
  declared in the header and arrives with the receiver series.
- **`body_channel_symbol_tribit` / `channel_symbol_patterns`:** The 8 mutually
  **orthogonal** spreading patterns (values 0/4 = antipodal BPSK points) used
  throughout the preamble and in the probes. Orthogonality is what lets a receiver
  decode a channel symbol by picking the max-correlation code.
- **`body_75_spread_tribit`:** The 75-bps Walsh spreading, in which each information
  dibit maps (via modified-Gray) to one of the orthogonal 32-chip sequences. 75 bps
  trades data rate for huge processing gain.
- **`emit_body_preamble_segment` / `generate_body_preamble[_tail]`:** Build a
  15-channel-symbol preamble segment (9 prefix + D1 + D2 + 3 countdown + 1 zero, each
  spread ×32 and sync-scrambled), and the whole/tail preamble.
- **`encode_body_block`:** The transmit block: FEC-encode (`encode_coded_bits`, threading
  `BodyEncodeState`) → interleave → map each data symbol (modified-Gray) and **add the
  data-randomizer whitening**, inserting the fixed **probe symbols** (whitened the same
  way) every frame; the last two frames' probes carry D1 then D2 so the receiver can
  re-identify the mode mid-body. 75 bps takes the Walsh path.
- **`decode_body_block` / `body_block_soft_metrics` (declared):** The receive mirror of
  `encode_body_block`. `decode_body_block` turns one block of received symbols back into
  information bits; `body_block_soft_metrics` stops short of the Viterbi and returns the
  block's combined rate-1/2 soft metrics, so a receiver can concatenate blocks and run
  **one continuous** Viterbi over the whole transmission. Both are declared in
  `body_waveform.hpp`; the implementations arrive with the receiver series.
- **`append_body_eom_and_flush`:** Frame the payload with the 32-bit EOM word
  `0x4B65A5B2` (MSB-first) + 144-bit flush, padded to a block multiple. The receive-side
  search, `find_body_eom` (with `body_eom_errors_at`), is declared in the header; it
  finds the EOM by minimum Hamming distance and arrives with the receiver series.
- **`pack_body_payload` (declared):** Packs decoded bits into octets LSB-first (the DTE
  byte order the transmitter unpacks with); it arrives with the receiver series.
- **`body_tribits_to_iq` / `body_tribits_to_audio` / `BodyAudioStreamModulator`:** Render
  symbols to complex IQ, or to real 48 kHz audio: RRC pulse-shape (rolloff 0.25, 20
  samples/symbol), scale to 0.98 of the worst polyphase peak (headroom), and upconvert to
  the **1800 Hz** carrier, retaining phase across windows.

---

## 7. Worked example: one 600-bps short block

Deliberately compact, **illustrative, not a real capture, not a compliance trace.**

### Transmit

```text
1. body_block_plan(600, short): info=360, coded=720, data=720 sym, frame 20/20, reps 1
2. FEC encode 360 info bits -> 720 coded bits (rate 1/2, K=7), threading BodyEncodeState
3. body_interleave 720 coded bits (40×18 matrix, load 9 / fetch 17)
4. map + whiten, framing 720 data symbols as 36 frames of [20 data | 20 probe]:
     data symbol s: tribit = modified_gray_decode(bit,1) ; emit (tribit + randomizer.next) mod 8
     probe symbol : known pattern (last two frames carry D1, then D2) + whitening
   -> 36×(20+20) = 1440 transmitted symbols
5. body_tribits_to_audio: RRC shape -> ×0.98 -> upconvert to 1800 Hz -> 1440×20 = 28,800 audio samples (0.6 s)
```

### Receive (a teaser)

The receive side is declared here, not yet implemented. Given this block's 1440
received symbols and the same `BodyBlockPlan`, `decode_body_block` will return its 360
information bits; across the concatenated blocks, `find_body_eom` and
`pack_body_payload` will turn those bits back into payload octets. Those
implementations, and everything a receiver must do before them, arrive with the
receiver series.

### One-screen summary

```text
plan -> FEC -> interleave -> map+whiten+probes -> render     (transmit)
decode_body_block -> find_body_eom -> pack_body_payload      (receive: declared, receiver series)
```

---

## 8. Equations, tables, and constants

```text
symbol rate .............. 2400 baud       carrier .......... 1800 Hz     audio .......... 48 kHz (20 sample/sym)
preamble segment ......... 15 ch-sym × 32 = 480 sym       segments ....... 3 (short) / 24 (long)
short block .............. 1440 symbols (0.6 s)           long block ..... ×8
EOM word ................. 0x4B65A5B2 (32 bits, MSB-first)   flush ......... 144 bits
whitening LFSR ........... 12-bit, seed 0xBAD, taps bits{6,4,1}, 8 clocks/tribit, reset @160 sym
sync sequence ............ fixed 32-entry table, cycled by index
modified-Gray ............ 1b:{0,4}  2b:{0,1,3,2}  3b:{0,1,3,2,7,6,4,5}
whiten transmit .......... emit = (mapped_tribit + randomizer_tribit) mod 8
dewhiten receive ......... z' = z · conj(psk8_symbol(randomizer_tribit))
```

---

## 9. What can go wrong in bad fading

| Symptom | Cause | Handling |
|---|---|---|
| Receiver loses the channel mid-block | Fading between training updates. | The **probe symbols** every frame (20/20 or 32/16) are known references a receiver re-trains on continuously. |
| Errors pile up at block boundaries | Per-block FEC termination pins the next block's start state. | The transmit FEC is one continuous stream (`BodyEncodeState`), so a receiver can run **one continuous Viterbi** across blocks; the declared `body_block_soft_metrics` exists for exactly that. |
| Payload byte order wrong | LSB vs. MSB confusion. | Payload octets go on the air LSB-first (DTE order; `generate_body_transmission_audio` unpacks them that way); the EOM word is MSB-first. Both are fixed by contract. |
| Spectrum uneven / carrier leakage | Long runs of identical symbols. | The **data randomizer** whitens every body symbol; audio is scaled to 0.98 of the worst polyphase peak for headroom. |
| Mode mis-identified mid-body | The preamble decode was marginal. | The last two frames' **probes carry D1/D2**, letting a receiver re-confirm the mode during the body. |
| EOM missed or false | A fade corrupts the EOM word. | The declared `find_body_eom` reports the best match by minimum **Hamming distance**; the header's inline `body_payload_bits` then accepts it within a tolerance, not only on an exact match. |

---

## 10. Debugging signals to watch

| Signal | What it tells you |
|---|---|
| `BodyBlockPlan` fields vs. mode | The single most common integration bug is a plan/mode mismatch: check `transmitted_symbols == 1440×mult` and the frame split. |
| Whitening alignment | If decoded bytes are garbage but structured, suspect a data-randomizer phase slip (reset every 160 symbols). |
| `BodyEncodeState` continuity | If the FEC state isn't threaded across blocks, boundary bits corrupt; verify it's carried, not re-zeroed. |
| Audio peak / clip | `mAmplitudeScale` targets 0.98; clipping means the scale or the symbol source is wrong. |

---

## 11. How to read this module

1. Read `body_block_plan` first: it defines the geometry every other function and
   every receiver stage obeys. Keep the block-plan table (§5) open.
2. Read `encode_body_block` next (map + whiten + probes); its exact receive-side mirror
   arrives with the receiver series.
3. Read the scramblers in `scrambler.cpp` (small and self-contained); note the two are
   *different* mechanisms for two purposes (whitening, sync).
4. Read `append_body_eom_and_flush` for the framing, and the LSB-first unpacking in
   `generate_body_transmission_audio` (`modem/m110a/transmitter.cpp`) for byte order.
5. `BodyAudioStreamModulator::render_window` is the only heavy DSP here (RRC + carrier);
   read it when working on TX audio, and cross-reference `make_root_raised_cosine_taps`
   in `modem/common/dsp/timing_recovery.cpp` for the taps.
6. `waveform.cpp` is a lookup table with compile-time self-checks: skim the descriptors,
   trust the `static_assert`s.

---

## 12. Maintainer notes

- **`body_block_plan` is the contract.** The symbol-mapping width, the interleaver
  size, the FEC repetition, and every receiver stage's geometry all derive from it.
  Change a field here and you must re-verify every downstream stage; this is the
  riskiest file to edit.
- **Encode and its receive mirror must stay mirror-exact.** `encode_body_block` fixes
  the frame layout, whitening walk, and probe positions that the receive side
  (arriving with the receiver series) walks in exact reverse. Any change to one without
  the other silently corrupts every decode.
- **The whitening seed and taps are standard-fixed** (`0xBAD`, Figure 6). The comment in
  `scrambler.cpp` records an independent comparison over all 160 output symbols; don't
  "tidy" the LFSR.
- **Continuous FEC state is deliberate.** `BodyEncodeState` threads the encoder register
  across blocks, because MIL-STD-188-110B routes the data, EOM, and flush through one
  continuous K=7 encoder; the declared `BodyDecodeState` and `body_block_soft_metrics`
  exist so a receiver can run one Viterbi over the whole transmission. Keep the
  threading.
- **Byte order is asymmetric on purpose:** EOM MSB-first, payload octets LSB-first
  (DTE). Both are load-bearing for interop; neither is a bug.
- **The waveform table's `consteval` checks are a feature.** They make a malformed
  descriptor or interleaver a compile error. Preserve them when adding modes.

---

## 13. Potential follow-up issues

Observations for later; **no code changed**.

- **Magic frame constants (20/20, 32/16, the `symbol >= 16` probe-zeroing)** are correct
  per the standard but appear as inline literals. A named constant block citing the table
  would make the probe framing self-documenting.

---

*Companion to the other explainers in `modem/m110a/`. This module is the structural
backbone the others plug into: `modem/m110a/transmitter-and-dsp-explainer.md` covers the
orchestrator that drives it, `modem/m110a/demapper-and-deinterleaver-explainer.md` covers
the 8-PSK mapping and interleaver it calls, and `modem/m110a/standard-interpretations.md`
records where the standard needed a judgment call. The `BodyBlockPlan` computed here also
sizes every receiver stage; the receiver series covers those.*
