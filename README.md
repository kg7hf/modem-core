# modem-core

[![CI](https://github.com/kg7hf/modem-core/actions/workflows/ci.yml/badge.svg)](https://github.com/kg7hf/modem-core/actions/workflows/ci.yml)

A real, standards-compliant **MIL-STD-188-110A/B serial-tone HF modem** written in
modern C++ (C++23), small enough to cross-compile onto a microcontroller. This
repository is the **transmit side** and the first stages of the **receiver**, published
as the companion code to the *Signal Path* article series, "Building a MIL-STD-188-110
HF Modem in Modern C++."

It is the easy half: deterministic, fully specified by the standard, and the
reference the receiver is measured against. A transmitter that another vendor's
modem can decode is where building a receiver starts.

## What's here

The complete transmit chain for the serial-tone body waveform, from user octets to
an 1800 Hz, 2400 symbol/second passband signal:

- **Framing and planning** (`m110a/transmitter`, `m110a/body_waveform`): size the
  whole burst up front, allocate nothing on the signal path.
- **FEC** (`common/fec/convolutional_k7`): the K=7, rate-1/2 convolutional encoder
  (generators 133/171 octal), plus the repetition and tail-biting variants.
- **Interleaving** (`m110a/interleaver`): the 0.6 s and 4.8 s block interleavers.
- **Scrambling and mapping** (`m110a/scrambler`, `m110a/demapper`): the body
  randomizer and the 8-PSK / QPSK / BPSK Gray constellation.
- **Modulation** (`m110a/body_waveform`, `common/dsp/timing_recovery`): the
  square-root raised-cosine pulse shaping and the 1800 Hz up-conversion.

And the first stages of the receiver:

- **The audio front end** (`m110a/body_audio_stream_frontend`): mixes the audio down
  from 1800 Hz, runs the root-raised-cosine matched filter, and finds and tracks each
  symbol's sampling instant, in audio blocks of any size, with a phase search that frees a
  loop hung between the peaks and a rate integrator that learns the clock offset.
- **Acquisition** (`m110a/synchronization`): finds a burst in noise by its preamble,
  measures the carrier offset, reads the mode and the countdown, works out where the data
  begins, and anchors on the first path of a burst that arrives by two, not the strongest.
- **Carrier tracking** (`common/dsp/timing_recovery`): the second-order loop that
  removes a carrier phase and frequency offset.
- **The test bench** (`tests/`): Catch2 unit tests that run the real transmitter's
  signals through the receive stages, each one small enough to step through, and the
  runner, which runs every stage so far end to end on your own message or a WAV file.
  Part 5's walkthrough runs on recordings published with the repository as a release
  (configure with `-DMODEM_ARTICLE_DATA=ON`).

The main stages have a detailed `*-explainer.md` next to their source; [docs/](docs/)
lists them.

## Documentation

New here? Start with [docs/start-here.md](docs/start-here.md): the big picture, a
guided tour of the transmit chain, and a reading order. [docs/](docs/) also holds the
article series this code accompanies and links to the per-stage code walkthroughs.

## Build and run

Requires a C++23 compiler (GCC 14+, Clang 19+) and CMake 3.25+. Catch2, the
unit-test framework, is a git submodule in `external/catch2`, so clone with it:

```bash
git clone --recurse-submodules https://github.com/kg7hf/modem-core.git
cd modem-core
cmake -B build
cmake --build build
./build/tx_demo "Hi"      # prints the walkthrough values, writes tx_out.wav
ctest --test-dir build    # the golden tests and the unit tests
./build/transmit_chain_tests "[walkthrough]" --order decl   # every number in Part 2
./build/front_end_demo                                      # Part 3, step by step
./build/acquisition_demo                                    # Part 4, step by step
./build/first_path_demo                                     # Part 5, on the article's recordings (-DMODEM_ARTICLE_DATA=ON)
./build/runner "Hi" --snr 10 --carrier 37                   # every stage so far, on your message
```

In a clone made without `--recurse-submodules`, run `git submodule update --init`
once. To build only the modem and `tx_demo`, configure with `-DMODEM_BUILD_TESTS=OFF`.

Prefer an editor? Open the folder in VS Code: the CMake presets, build and run tasks,
and debug configurations for `tx_demo`, `front_end_demo`, `acquisition_demo`, `first_path_demo`, the runner and
the transmit-chain tests are included, and the Testing panel lists every unit test to run or debug (see
[CONTRIBUTING.md](CONTRIBUTING.md#vs-code)).

`tx_demo` walks the message through the 600 bps long-interleave mode ("600L"),
prints the intermediate values the transmit-chain article steps through, and
renders the whole burst to a 48 kHz WAV you can listen to.

## Quality

CI builds on GCC and Clang (warnings-as-errors), runs golden tests that check every
transmitted symbol and every sample of the audio, and the receiver's unit
tests, and checks formatting (clang-format), the C++ Core Guidelines (clang-tidy), cyclomatic
complexity (lizard), coverage (gcov), and bounded stack usage (`-fstack-usage`).
Each gate backs a claim the articles make. See [CONTRIBUTING.md](CONTRIBUTING.md)
to run any of them locally.

## What's coming

The receiver is the hard half, and it is its own series; the audio front end and
acquisition and the earliest-path anchor are its first stages. You will find the rest of the receiver **declared** in
the headers here (the Viterbi decoder, the soft demapper, the block decoder) with their
design notes intact, but not yet implemented; they arrive with the receiver articles,
which cover the equalizer and the burst decoder next. This
repository grows with the series, and so does the runner: each article adds its stage.

## License

GPL-3.0-or-later. See [COPYING](COPYING).

Copyright (C) 2026 Paul R. Decker (KG7HF).
