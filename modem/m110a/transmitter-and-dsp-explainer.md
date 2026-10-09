# Transmitter Explainer

## 1. Scope

This note explains two small but foundational source files:

- `modem/m110a/transmitter.hpp` / `modem/m110a/transmitter.cpp`: the **transmit-side
  orchestrator**, which plans a complete DTE-octet transmission and renders it to
  48 kHz audio (the mirror image of the receive chain).

They sit at the top of the TX stack. The DSP they drive at the end (root-raised-cosine
pulse shaping and upconversion to the 1800 Hz carrier) lives in
`modem/m110a/body_waveform.cpp` and `modem/common/dsp/timing_recovery.cpp`.

---

## 2. 50,000-foot view

**Transmitter.** Given a message (a run of DTE octets) and a mode, the transmitter:
plans the whole transmission (how many body blocks, how many symbols, how many audio
samples), frames the payload with an EOM marker and a coder flush, prepends the
preamble, encodes each body block, and renders the symbol stream to real 48 kHz
audio... all into caller-owned buffers, no allocation. It is exactly the receive chain
run backwards, and it reuses the same `body_waveform` building blocks.

---

## 3. 5th-grade version

**Transmitter:** It's the "send" button. You hand it a note; it wraps the note in the
agreed envelope (jingle in front, "THE END" at the back), writes it in the secret
code, and turns it into the sound the radio plays. It's the same steps as reading a
message, just done in reverse to *make* one.

---

## 4. Where these sit in the pipeline

```text
  TRANSMIT (transmitter.cpp)
  --------------------------
  DTE octets
     │ body_transmission_plan()
     ▼
  frame: payload + EOM + flush + block padding
     │
     ▼ generate_body_transmission_audio():
       generate_body_preamble()
       encode_body_block() × body_blocks  (threads BodyEncodeState)
       body_tribits_to_audio()  (RRC + 1800 Hz)
     ▼
  48 kHz mono audio  ─►  DAC / codec / WAV
```

**Transmitter inputs:** a `BodyMode`, payload octets, caller scratch. **Output:** a
mono 48 kHz waveform. **Depends on:** `body_waveform` (plan, preamble, encode, render).

---

## 5. Function-by-function walkthrough

### `body_transmission_plan(mode, payload_bytes)`
- **50K:** Compute every size the transmission needs before touching a buffer.
- **Detailed:** From the `BodyBlockPlan`, derives `payload_bits` (octets × 8),
  `framed_bits` (payload + EOM + flush, rounded up to a whole number of body blocks),
  `body_blocks`, `preamble_symbols`, `transmitted_symbols`, and `audio_samples`, with
  an overflow guard at every multiply. The caller uses the counts to size scratch.
- **5th-grade:** Measure out exactly how much paper, ink, and tape you'll need first.

### `generate_body_transmission_audio(plan, payload, scratch, audio)`
- **50K:** Render one complete transmission to audio, allocation-free.
- **Detailed:** (1) Unpack payload octets **LSB-first** into bits. (2)
  `append_body_eom_and_flush` frames them. (3) `generate_body_preamble` writes the
  preamble symbols. (4) For each body block, `encode_body_block` (threading one
  continuous `BodyEncodeState`) writes the block's transmitted symbols after the
  preamble. (5) `body_tribits_to_audio` pulse-shapes and upconverts the whole symbol
  stream to 48 kHz. Every buffer is validated against the plan first.
- **5th-grade:** Wrap, encode, and play the note as sound, start to finish, using no
  scratch paper beyond what you measured.

---

## 6. Worked example

Deliberately tiny, **illustrative, not a real capture.**

### Transmit a 2-byte message at 600 bps short

```text
body_transmission_plan(600-short, 2 bytes):
  payload_bits = 16;  framed = 16 + 32(EOM) + 144(flush) = 192 -> round up to 1 block (360 info bits)
  body_blocks = 1;  preamble = 3×480 = 1440 sym;  transmitted = 1440 + 1440 = 2880 sym
  audio_samples = 2880 × 20 = 57,600 (1.2 s)

generate_body_transmission_audio:
  unpack 0x41 0x42 -> bits (LSB first): 1,0,0,0,0,0,1,0, 0,1,0,0,0,0,1,0
  append EOM 0x4B65A5B2 (MSB first) + 144 flush + pad to 360
  generate_body_preamble -> 1440 preamble symbols (countdown 2,1,0)
  encode_body_block -> 1440 body symbols
  body_tribits_to_audio -> 57,600 samples on the 1800 Hz carrier
```

---

## 7. What can go wrong / debugging

| Signal | What it tells you |
|---|---|
| `body_transmission_plan` overflow status | Payload too large for `size_t` arithmetic; every multiply is guarded, so a failure is a genuine size problem, not a silent wrap. |
| Buffer-too-small from `generate_body_transmission_audio` | A scratch span doesn't match the plan; size scratch from the plan's counts, not by guesswork. |
| Byte order | Payload unpacks **LSB-first**; the EOM is MSB-first. Both are fixed by contract. On receive, `pack_body_payload` packs decoded bits back into octets in the same LSB-first order; it is declared in `modem/m110a/body_waveform.hpp`, and its implementation arrives with the receiver series. |

Fading note: the transmitter never sees the channel. Its correctness is about buffer
contracts and bit order, not channel robustness.

---

## 8. How to read these modules

1. Read `modem/m110a/transmitter.cpp` top to bottom; it is short and linear: plan,
   then the 5-step render. Cross-reference `modem/m110a/body_waveform.cpp` for each
   step it calls.
2. Then follow step (5), the render, into `modem/m110a/body_waveform.cpp`.
   `body_tribits_to_audio` is a one-shot wrapper over `BodyAudioStreamModulator`:
   `initialize` builds the root-raised-cosine pulse with `make_root_raised_cosine_taps`
   (in `modem/common/dsp/timing_recovery.cpp`), and `render_window` shapes the 8-PSK
   symbols with it and upconverts them to the 1800 Hz carrier, which it reads from the
   80-entry `body_carrier_table` in `body_waveform.hpp`.

---

## 9. Maintainer notes

- **The transmitter is the receiver's mirror.** Its byte order, framing, preamble,
  encode, and rendering all come from `body_waveform`; keep it a thin orchestrator, not
  a place to re-implement any of them.
- **The transmitter is mode-agnostic.** The rate-1/2 FEC, the 150/300 repetition
  copies, the 75-bps Walsh spreading, and the probe framing all live in `body_waveform`
  (`encode_body_block` / `body_block_plan`), not here; this file only sizes and drives
  the render, so every body rate (75 Walsh, 150/300 repetition, 600/1200/2400 rate 1/2,
  4800 uncoded) flows through the same code path. Add a mode by extending
  `body_block_plan`, not this file.
- **Overflow guards are deliberate.** `multiply_would_overflow` wraps every size
  multiply so a huge payload fails cleanly instead of wrapping into a small buffer.

---

*Companion to the other transmit-stage explainers in `modem/m110a/`. The transmitter
composes the encode and render functions walked through in
`modem/m110a/body-waveform-and-scrambler-explainer.md`.*
