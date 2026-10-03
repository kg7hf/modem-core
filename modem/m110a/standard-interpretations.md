# MIL-STD-188-110A: standard interpretations

Where this implementation had to decide something the printed standard does not
settle outright, or where the edition boundary is not where a reader would guess.

## STD-A-001: this folder is 110A *and* the 110B body

The serial-tone waveform in this folder is not "the 110A version" of anything.
110B section 5.3.1.1 presents the same fixed-frequency PSK serial waveform at
75/150/300/600/1200/2400 bps as its mandatory joint-service-interoperable HF
capability, the same ladder 110A defined in 1991. New 110B capability is
concentrated in the appendices; between these editions, interoperability
questions are appendix questions, not body-waveform questions.

So one implementation serves both editions. It is filed under 110A because that
is the edition that introduced it, not because 110B replaced it. Fielded
equipment corroborates this: at least one commercial HF modem auto-detects
between the 110B QAM waveforms and the 110A serial-tone waveforms, treating the
latter as its legacy subset.

## STD-A-002: uncoded 4800 bps lives here, not with the 110B appendices

**The judgement most likely to look wrong at a glance.** The tidy rule of thumb
(*110A is 75-2400; 110B is above 2400 plus ISB*) would put 4800 next door. It
does not go there.

110B section 5.3.1.1 lists uncoded 4800 bps as a **design objective of the same
serial-tone ladder**, and the 110A fixed-frequency rate table carries 4800:
uncoded, no FEC, 8-ary tribits, interleaver bypassed, framing 32/16 (the same
32/16 as 2400), not applicable to frequency hopping. A conformant 110A
implementation stays on-air compatible at 75-2400 (and uncoded 4800) bps.

What 110B adds above 2400 is the **coded** Appendix C rates and Appendix F ISB.
The descriptor table in `modem/common/waveform/waveform.cpp` therefore carries
two different 4800 rows, and they are two different waveforms:

```
{serial_tone, bps4800, psk8, uncoded}                       -> this folder
{appendix_c,  bps4800, psk8, k7_tail_biting_punctured_3_4}  -> the 110B Appendix C waveform
```

There is also an implementation fact that would have forced this even without the
standards answer: the uncoded 4800 path is `if (rate == bps4800)` branches woven
through `body_waveform.cpp` and the receiver. It has no seam.

## STD-A-003: continuity is corroborated, not transcribed

This caveat bounds every claim above. The 110A source used for this work is a
page-image scan, so no direct 110A-versus-110B text diff was performed; the
body-waveform continuity here is corroboration, not verified transcription
against the 110A text. Other evidence points the same way (the conformance test
plan's Table 10.1 low-rate rows are unchanged across the editions), but none of
it is a transcription. Treat any *numeric* 110A-only claim as needing its own
check.
