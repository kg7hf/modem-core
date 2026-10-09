# Documentation

## Start here

[Start here](start-here.md): the map of the repository, a guided tour of the transmit
chain, a reading order, and a look ahead at the receiver's next stage.

## The series

This code accompanies the *Signal Path* article series, "Building a MIL-STD-188-110
HF Modem in Modern C++":

- [An Introduction](articles/01-introduction.md): what the waveform is and why it still matters.
- [The rules I build by](articles/02-embedded-cpp-philosophy.md): the embedded-first engineering philosophy behind the code.
- [The transmit chain](articles/03-transmit-chain.md): the companion to this repository; from user bytes to an 1800 Hz waveform, one stage at a time.
- [The audio front end](articles/04-audio-front-end.md): the receiver's first stage; mixing down, the matched filter, and finding and keeping each symbol's sampling instant, with the unit tests that check each piece.

The next article, acquisition, joins here when it publishes.

## Code-level walkthroughs

The transmit stages have detailed explainers next to their source:

- [`transmitter-and-dsp-explainer.md`](../modem/m110a/transmitter-and-dsp-explainer.md): the transmit orchestrator, from octets to audio.
- [`body-waveform-and-scrambler-explainer.md`](../modem/m110a/body-waveform-and-scrambler-explainer.md): the body burst; framing, encoding, scrambling, modulation.
- [`demapper-and-deinterleaver-explainer.md`](../modem/m110a/demapper-and-deinterleaver-explainer.md): the 8-PSK constellation and the body interleaver.
- [`standard-interpretations.md`](../modem/m110a/standard-interpretations.md): where the standard needs interpretation, and the call made.

The receiver's first stage has one too:

- [`audio-front-end-explainer.md`](../modem/m110a/audio-front-end-explainer.md): the streaming front end (mix, matched filter, symbol timing) and the carrier tracker.

The unit tests in [`tests/front_end_tests.cpp`](../tests/front_end_tests.cpp) show each
piece of the front end working. The rest of the receiver is declared in the headers and
arrives with the receiver series.
