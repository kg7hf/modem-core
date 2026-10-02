# Demapper and Deinterleaver Explainer

## 1. Scope

This note explains four source files:

- `modem/m110a/demapper.hpp` / `modem/m110a/demapper.cpp`: the 8-PSK symbol **mapper**
  (`psk8_symbol`), plus the declarations of the hard and soft **demappers** that turn a
  received symbol back into bit evidence.
- `modem/m110a/interleaver.hpp` / `modem/m110a/interleaver.cpp`: the MIL-STD-188-110B
  matrix **interleaver** (`body_interleave` and its load/fetch addressing), plus the
  declarations of the **deinterleavers** and the soft (LLR) variants.

This repository is the transmit side, so the mapper and the interleaver are what is
implemented here. The demappers and deinterleavers are declared in the headers with
their design comments; their implementations arrive with the receiver series.

Together these two files sit between the FEC encoder and the modulator. The encoder
produces coded bits in *encoder order*; the interleaver reorders them into *channel
order*; each group of 1, 2, or 3 channel-order bits becomes a tribit through the
modified-Gray mapping and the scrambler (`modem/m110a/scrambler.hpp`); and
`psk8_symbol` places that tribit on the 8-PSK circle. On receive the same two files
run the other way: symbol→soft-bit, then channel-order→encoder-order.

---

## 2. 50,000-foot view

Two small but pivotal jobs sit between "the FEC has encoded the bits" and "the symbol
goes on the air":

1. **Interleaving.** The transmitter deliberately **scrambles the time order** of the
   coded bits before sending them, using a matrix. Why? So that a fade, which wipes
   out a *contiguous* run of received symbols, damages *scattered* coded bits once the
   receiver unscrambles them, not one long consecutive stretch. Convolutional codes
   recover easily from scattered errors but poorly from long bursts.

2. **Mapping.** A symbol is a point on the 8-PSK circle. The transmitter puts 1, 2, or
   3 bits on each symbol (depending on rate), choosing the point through a
   modified-Gray code so that neighboring points differ in a single bit.

The receiver runs both jobs backwards. Its **demapper** asks, for each of those bits:
*how strongly does this received point favor 0 versus 1?* The answer is **soft** (a
log-likelihood ratio, LLR), not a hard 0/1, because the FEC decoder needs
reliabilities, not guesses. Its **deinterleaver** undoes the scramble, restoring
**encoder order** so the decoder sees the codeword the way the transmitter built it.
Both are declared in the headers here and implemented in the receiver series.

```text
  transmit:            coded bits (encoder order)  ──►  INTERLEAVE  ──►  coded bits (channel order)  ──►  MAP  ──►  8-PSK symbol
  receive (declared):  received symbol  ──►  DEMAP  ──►  soft bits (channel order)  ──►  DEINTERLEAVE  ──►  soft bits (encoder order)
```

---

## 3. 5th-grade version

**Interleaving:** Imagine writing a secret note, then cutting it into little squares
and shuffling them in a fixed pattern before mailing, so that if the mail truck loses
a corner of the envelope, you lose *scattered* squares (easy to guess the missing
ones) instead of a whole sentence. The interleaver is that shuffle. On the other end,
the deinterleaver puts the squares back in order using the same fixed pattern.

**Mapping:** You're playing darts. Each of 8 spots on a circle stands for a little
pattern of 1, 2, or 3 bits, laid out so that neighboring spots differ in just one bit.
The transmitter aims at the spot for each pattern. On the receiving end the dart (the
received symbol) lands *near* a spot but not exactly on it. Instead of just saying
"closest spot is #3," the receiver's demapper says for each bit: "I'm *pretty* sure
this bit is 0" or "barely leaning 1." That confidence is the soft value.

---

## 4. Where this module sits in the pipeline

```text
  K=7 FEC encoder (convolutional_k7.cpp)
        │  coded bits, ENCODER order
        ▼
  ┌──────────────────────────────────────────────────────────────┐
  │  *** INTERLEAVER (interleaver.cpp) ***                       │
  │    body_interleave(input, scratch, output, spec)             │
  │    spec = interleaver_for(config)   (waveform.cpp)           │
  └──────────────────────────────────────────────────────────────┘
        │  coded bits, CHANNEL order (the order they will fly)
        ▼
  mapped_tribit (modified Gray), then + randomizer tribit, mod 8
        │  transmitted tribit 0..7   (scrambler.hpp, body_waveform.cpp)
        ▼
  ┌──────────────────────────────────────────────────────────────┐
  │  *** MAPPER (demapper.cpp) ***                               │
  │    psk8_symbol(tribit)  -> e^{j·tribit·π/4}                  │
  └──────────────────────────────────────────────────────────────┘
        │  8-PSK symbol
        ▼
  pulse shaping + 1800 Hz up-conversion (body_waveform.cpp)  ─►  audio
```

Two body rates bend this path: 4800 bps is uncoded and runs the interleaver in
`bypass` (a plain copy), and 75 bps keeps the FEC and the interleaver but replaces the
tribit mapping with Walsh spreading (see
`modem/m110a/body-waveform-and-scrambler-explainer.md`).

On **receive** the same two blocks run in reverse: the demapper turns each received
symbol into soft bits in channel order, and the deinterleaver restores encoder order
for the FEC decoder. Both are declared in `modem/m110a/demapper.hpp` and
`modem/m110a/interleaver.hpp`; the receiver series implements them.

**Inputs:** a tribit 0..7 for `psk8_symbol`; a coded-bit vector, scratch, and an
`InterleaverSpec` for `body_interleave`.
**Outputs:** one unit-magnitude constellation point; the permuted (channel-order) bit
vector.
**Depends on:** `modem/m110a/scrambler.hpp` (`modified_gray_decode` and
`mapped_tribit`, the body bit→tribit mapping), `modem/common/waveform/waveform.hpp`
(`InterleaverSpec`). Allocation-free; the caller owns the scratch matrix.

---

## 5. Important data structures

### Mapper and demappers

There is no state; `psk8_symbol` and the declared demappers are pure functions over one
symbol. The key facts:

| Concept | Meaning |
|---|---|
| **tribit** | A 3-bit index 0..7 selecting one of the 8 points `e^{j·tribit·π/4}`. |
| **source_bits** | How many bits one symbol carries: 1 (BPSK-like), 2 (QPSK), or 3 (8-PSK). Rate-dependent; the transmit side reads it from the block plan as `information_bits_per_channel_symbol`. |
| **modified-Gray sub-mapping** | 1 bit → points {0,4}; 2 bits → `gray<<1` → {0,2,4,6}; 3 bits → all 8 via Gray (`mapped_tribit` in `modem/m110a/scrambler.hpp`). Adjacent points differ in one bit so a small symbol error flips at most one bit. |
| **soft value / LLR** | Receive side (declared): `+` favors bit 0, `−` favors bit 1, magnitude = confidence. |

### Interleaver: `InterleaverSpec` (from `modem/common/waveform/waveform.hpp`)

| Field | Meaning |
|---|---|
| `rows`, `columns` | Body matrix dimensions; `size_bits = rows·columns`. |
| `load_row_increment` | Transmit **load** walk: within each column, step rows by this increment. |
| `fetch_column_decrement` | Transmit **fetch** walk: across each row, offset columns by this decrement. |
| `increment` | Not read by the body matrix interleaver (it serves other waveform families); `interleaver_for` sets it to 0 for every body mode. |
| `bypass` | Zero-interleave mode: pass-through, no permutation. |
| `size_bits`, `input_bits` | Total bits in the matrix / meaningful input bits. |

---

## 6. Function-by-function walkthrough

### `psk8_symbol(tribit)`
- **50K view:** Map a tribit 0..7 to its 8-PSK constellation point `e^{j·tribit·π/4}`.
- **Detailed view:** Returns from a precomputed 8-entry table (filled with the same
  `std::polar` expression) so the hot path is a table read, not a `sinf/cosf`; this
  matters on cores without hardware trig (Cortex-M33). Bit-identical to computing it
  each call. On transmit, `body_tribits_to_iq` and `BodyAudioStreamModulator` (both in
  `modem/m110a/body_waveform.cpp`) call it to turn each transmitted tribit into its
  point.
- **5th-grade view:** Look up which of the 8 dartboard spots a 3-bit pattern means.

### Declared for the receiver: the demappers
- **50K view:** The receive-side inverse of the mapping: turn a received symbol back
  into bit evidence.
- **Detailed view:** Declared in `modem/m110a/demapper.hpp`; the implementations
  arrive with the receiver series. `psk8_hard_demapper` returns the nearest tribit (a
  hard decision). `psk8_soft_demapper` returns one soft value (LLR) per source bit, so
  the FEC decoder gets reliabilities instead of guesses. `psk8_soft_demapper_apriori`
  is a variant that also takes prior information about the bits from the decoder.
- **5th-grade view:** "Which spot did the dart land closest to?" (hard), or "how sure
  am I about each bit?" (soft).

### Interleaver addressing: `body_load_address` / `body_fetch_address`
- **50K view:** The two halves of the MIL-STD-188-110B matrix permutation.
- **Detailed view:** The interleaver is a `rows × columns` matrix. **Load** (transmit
  scatter) writes the input *down columns*, stepping the row by `load_row_increment`:
  `row = (index%rows · load_row_increment) mod rows`, `column = index/rows`. **Fetch**
  (transmit gather) reads *across rows*, offsetting the column by
  `fetch_column_decrement`: `row = index%rows`, `column = (index/rows −
  row·fetch_column_decrement) mod columns`. The staggered read is what turns a
  contiguous fade into scattered coded-bit errors.
- **5th-grade view:** Fill the grid going down the columns in a hop pattern; read it
  back going across the rows in a different hop pattern. The mismatch shuffles
  everything.

### `body_interleave` (and its declared inverses)
- **50K view:** Apply the permutation: encoder order in, channel order out.
- **Detailed view:** Interleave = scatter by `load`, gather by `fetch`, through the
  caller's scratch matrix. `bypass` short-circuits to a copy. The header also declares
  `body_deinterleave` (scatter by `fetch`, gather by `load`: the exact inverse, so
  `deinterleave(interleave(x)) == x`) and `_soft` variants that do the identical index
  math on `float` LLRs; those arrive with the receiver series.
- **5th-grade view:** Shuffle the squares of the note with a fixed pattern; the
  receiver un-shuffles them with the same pattern.

---

## 7. Worked example: a tiny interleave, then one symbol onto the circle

Deliberately tiny, **illustrative: not a real capture, not a compliance trace.** The
addresses and the channel order below are what the shipped `body_load_address`,
`body_fetch_address` and `body_interleave` return for this spec.

### Step 1: Load the coded bits into the matrix

Take a tiny `rows=3, columns=4` matrix with `load_row_increment=1`,
`fetch_column_decrement=1` (illustrative). Twelve coded bits leave the FEC encoder in
encoder order, `e0` to `e11`. `body_interleave` first **scatters** them by the
**load** address, which fills the matrix down the columns:

```text
load addresses (where encoder bit i is written):  0,4,8,1,5,9,2,6,10,3,7,11

           col 0   col 1   col 2   col 3
  row 0:    e0      e3      e6      e9
  row 1:    e1      e4      e7      e10
  row 2:    e2      e5      e8      e11
```

### Step 2: Fetch them out in channel order

Then it **gathers** across the rows by the **fetch** address. Successive channel bits
step down the rows while sliding back `fetch_column_decrement` columns per row (one,
here), wrapping around:

```text
fetch addresses (where channel bit j is read from):  0,7,10,1,4,11,2,5,8,3,6,9

channel index:  0    1    2    3    4    5    6    7    8    9    10   11
carries:        e0   e10  e8   e3   e1   e11  e6   e4   e2   e9   e7   e5
```

### Step 3: What a fade does now

A short fade that corrupts channel indices 3, 4 and 5 lands on encoder bits `e3`,
`e1` and `e11`. Neighbors on the air are no longer neighbors in the codeword, and
scattered errors are what a convolutional decoder tolerates. **Scattering is the
entire product**; the interleaver adds no redundancy and removes none.

### Step 4: Map one symbol onto the circle

The channel-order bits are then taken `source_bits` at a time. At 3 bits per symbol:

```text
one channel-order group with value 2 (binary 010):
  mapped_tribit(2, 3)  -> tribit 3            (modified-Gray table 0,1,3,2,7,6,4,5)
  tribit_add(3, r)     -> t = (3 + r) mod 8   (r = the body randomizer's next tribit)
  psk8_symbol(t)       -> e^{j·t·π/4}         (t = 3, for example, is the point at 135°)
```

Read around the circle from tribit 0 to tribit 7, the 3-bit values are 000, 001, 011,
010, 110, 111, 101, 100: each differs from both neighbors (the wrap from 7 back to 0
included) in exactly one bit. That is the Gray property in action. `tests/tx_demo.cpp`
prints the same chain, with a real randomizer value, for a message at one bit per
symbol (its steps 3 to 5).

### The receive direction (declared)

On receive the chain runs backwards. The soft demapper turns each received point into
one LLR per bit, in channel order, and the deinterleaver (scatter by fetch, gather by
load) puts those LLRs back in encoder order, so the fade from Step 3 reaches the FEC
decoder as three isolated weak bits rather than a run. Both are declared in
`modem/m110a/demapper.hpp` and `modem/m110a/interleaver.hpp`; their implementations
arrive with the receiver series.

### One-screen summary

```text
1. interleaver_for(config)        mode -> InterleaverSpec (matrix geometry, or bypass)
2. body_interleave()              encoder order -> channel order (scatter load, gather fetch)
3. mapped_tribit(), tribit_add()  bits -> modified-Gray tribit -> + randomizer, mod 8
4. psk8_symbol()                  tribit -> e^{j·tribit·π/4}
   receive (declared):            demap, then deinterleave (scatter fetch, gather load)
```

---

## 8. Equations and formulas

**8-PSK point:** `s(tribit) = e^{j·tribit·π/4}`.

**Transmitted tribit:** `t = (mapped_tribit(value, width) + randomizer) mod 8`
(`tribit_add`).

**Body matrix interleaver:**
```text
load(i)  : row = (i mod rows)·load_row_increment mod rows,  col = i / rows
fetch(i) : row = i mod rows, col = (i/rows − row·fetch_column_decrement) mod columns
address  = row·columns + col
interleave   = scatter by load,  gather by fetch
deinterleave = scatter by fetch, gather by load        (exact inverse; declared, receive side)
```

---

## 9. What can go wrong in bad fading

| Symptom | Cause | Mitigation in this module |
|---|---|---|
| One symbol error flips several bits | Bad bit→point mapping puts far-apart bit patterns on adjacent points. | Modified-Gray sub-mapping: adjacent points differ in one bit. |
| A fade wipes a run of symbols | Contiguous burst errors defeat the convolutional code. | The interleaver spreads bursts into scattered coded-bit errors (its entire reason to exist). |
| Interleaver buffer/spec mismatch | Wrong `rows·columns` vs. block size → wrong permutation. | `body_interleave` requires `input.size() == size_bits` and scratch and output at least that large, and the address functions reject zero dimensions or an out-of-range index, all before touching memory. |

Deep fade caveat: the interleaver here, and the demapper and deinterleaver on receive,
only **preserve and reorganize** evidence; they never create it. If the channel
destroyed the information, no soft value or shuffle can bring it back.

---

## 10. Debugging signals to watch

| Signal | What it tells you |
|---|---|
| `psk8_symbol` output | Unit magnitude at `tribit·45°`, every time; `tests/tx_demo.cpp` prints one with its I/Q and angle (step 5). Anything else means the constellation table is broken. |
| Load and fetch addresses over a whole block | Each walk must hit every matrix cell exactly once. A repeat or a gap means an `InterleaverSpec` field (dims or steps) is wrong for the mode; a walk that is not a permutation loses bits. |
| Encoder positions of adjacent channel bits | Should be scattered, as in §7, Step 3. Clustering means the wrong spec for the mode, or `bypass` where the mode should interleave. |

---

## 11. How to read this module

1. Read `psk8_symbol` first: the constellation as an 8-entry table, and why it is a
   table.
2. Read `mapped_tribit` and `modified_gray_decode` in `modem/m110a/scrambler.hpp`:
   which bit pattern lands on which point.
3. For the interleaver, read `body_load_address` and `body_fetch_address` together and
   trace a 3×4 example by hand (see §7). Once those two click, `body_interleave` is
   just a scatter/gather wrapper, and the declared receive-side inverses are the same
   two walks swapped.
4. Finish with `encode_body_block` in `modem/m110a/body_waveform.cpp`, where the
   interleaver and the bit→tribit mapping meet in the transmit chain.

---

## 12. Maintainer notes

- **`psk8_symbol`'s table must stay bit-identical to `std::polar`.** The comment
  guarantees the M33 speedup changes cost, not results. Any constellation edit must
  update the table generator and the bit→tribit mapping in
  `modem/m110a/scrambler.hpp` together.
- **The load and fetch walks are a matched pair.** Interleave scatters by load and
  gathers by fetch; the declared deinterleave is its exact inverse by contract
  (scatter by fetch, gather by load). Don't "optimize" one walk without the other.
- **`modified_gray_decode` lives in `scrambler.hpp`.** The transmit mapping uses it
  through `mapped_tribit`, and the demappers declared beside `psk8_symbol` must invert
  it; it is documented in `modem/m110a/body-waveform-and-scrambler-explainer.md`. If
  the body sub-mapping ever changes, the mapping and the demappers must change
  together.

---

## 13. Potential follow-up issues

Observations for later; **no code changed**.

- **`modified_gray_decode` placement.** It is the constellation bit→tribit mapping but
  lives in `scrambler.hpp`, which is a surprising home. A future cleanup might move it
  to `demapper.hpp`/`waveform.hpp` where the mapping conceptually belongs (mechanical,
  but it touches includes; out of scope here).
- **Interleaver address results are `Result<std::size_t>` inside tight loops.** Each
  index re-validates dims. Fine for correctness; if a profile ever shows this hot, a
  pre-validated fast path (validate once, then raw index math) is available, but only
  behind measurement.

---

*Companion to the transmit-chain walkthroughs. Upstream: the K=7 encoder
(`modem/common/fec/convolutional_k7.cpp`) produces the coded bits this module
interleaves. Alongside: `modem/m110a/body-waveform-and-scrambler-explainer.md` (block
framing, the scrambler, and where `body_interleave` and the mapping are called). The
whole burst, planned and rendered to audio:
`modem/m110a/transmitter-and-dsp-explainer.md`. The receive-side demapper and
deinterleaver arrive with the receiver series.*
