# Receiver Core Equalizer Explainer

## 1. Scope

This note explains the receiver's third stage, the **channel equalizer**: the stage that
undoes the smearing an HF channel does to the symbols, after acquisition has found the
burst and before the symbols are turned back into bits. It covers six files:

- `modem/m110a/equalizer.hpp` / `.cpp`: the **adaptive** family. `estimate_flat_channel`
  (a one-tap channel gain), `LmsEqualizer` (a plain T-spaced NLMS transversal filter, a
  textbook baseline the receiver does not use) and `ProbeDfe`, the probe-trained
  **decision-feedback equalizer** the receiver runs, with NLMS adaptation (RLS is an option
  the receiver never selects).
- `modem/common/cir_equalizer.hpp` / `.cpp`: the **model-based** family. Measure the
  channel impulse response (CIR) from known symbols by regularized least squares, then
  **solve** the finite-length MMSE decision-feedback equalizer from that estimate. The
  data-symbol filter is computed, not adapted.
- `modem/m110a/receiver.hpp` / `.cpp`: `BodyAdaptiveReceiver`, the engine that uses them.
  It trains on the preamble (with a frequency race between up to three carrier
  estimates), runs under the window its caller selects from the measured delay spread
  (`select_body_receiver_geometry`), and walks the body frame by frame with one of two
  engines: `adaptive` (the default, and the only one in an embedded build) or `cir_mmse`.

Neither engine wins everywhere. On the standard's two-path channels the model-based
CIR-MMSE engine does better at 2400 bps and the adaptive one at 600 bps and below; on the
article's 300L recording through a 5 ms channel (section 7) the adaptive one gets fewer
data symbols wrong. That is why both are here, and why the choice is a runtime setting.

The tests are `tests/equalizer_tests.cpp` (the solvers and the receiver),
`tests/delay_spread_tests.cpp` (the measurement and the window it selects) and
`tests/equalizer_walkthrough_tests.cpp` (the article's steps), and the article series
explains the design in Part 6, "The equalizer". The output is equalized symbols, one per
transmitted symbol, scaled for the demapper; turning them back into the message (soft
demapping and the decoder) is Part 7's, and so is the whole-burst turbo equalizer.

---

## 2. 50,000-foot view

The channel takes each transmitted symbol and (a) spreads its energy over several
neighbouring symbol slots (**multipath, or inter-symbol interference, ISI**), (b) scales
and rotates it (**fading**), and (c) adds **noise**. The equalizer's job is a clean
estimate `z` of each transmitted symbol.

A **decision-feedback equalizer (DFE)** does it with two filters:

```text
        feed-forward filter (on received T/2 samples)
                    │   handles the PRECURSOR smear and the noise
                    ▼
   z = Σ w[t]·window[t]  −  Σ b[j]·decision[k−1−j]
                              ▲
                    feedback filter (on PAST DECISIONS)
                    cancels the POSTCURSOR smear those symbols leave behind
```

- The **feed-forward** filter works on the received samples, at **T/2** (two samples per
  symbol), around the symbol being decided.
- The **feedback** filter works on symbols already decided. A past symbol leaves a
  trailing echo on the current one; with the past symbol decided, its echo can be
  subtracted. Feedback uses **decisions, not samples**, so it cancels ISI without
  amplifying noise.

The two families differ only in **how the taps `w` and `b` are found**:

- `ProbeDfe` **adapts** them, a small step per symbol toward a smaller error.
- `cir_mmse_dfe_solve` **solves** them in one step from a measured channel.

Around the filters, the receiver does three things in order:

```text
Train on the preamble       -> the taps and the carrier, at the best of up to 3 frequencies
Size the window             -> the caller measures the delay spread, picks the geometry,
                               and trains again only if the geometry changed
Walk the body               -> per frame: the unknown data symbols, then the known probe;
                               the probe retrains (adaptive) or re-measures (CIR-MMSE)
```

A frame is the unknown data symbols followed by a short **probe** of known symbols (at
300 bps, 20 and 20). The probe is what keeps either engine on the channel as it fades.

---

## 3. 5th-grade version

Someone shouts words to you across a canyon. You hear each word, plus a delayed echo of
the word before it bouncing off the far wall, plus wind noise.

- The **feed-forward** filter is you cupping your ears and leaning just right to hear
  the current word as clearly as possible.
- The **feedback** filter is you remembering the previous word you already worked out,
  so when its echo arrives you subtract it: "that part is just the echo of the last word."

Every so often the shouter says a phrase you both agreed on in advance. That is the probe:
you know what it should sound like, so it tells you how far off you are.

Two ways to set yourself up:

- **Adaptive:** after every word, correct your listening a little in the direction that
  would have made it clearer. You keep up with an echo that changes slowly and steadily.
- **Model-based:** each time the agreed phrase comes, measure the canyon from it (how
  loud and how late each echo is), then work out the best way to listen in one go. You
  start each stretch fresh, but if the canyon changes a lot between phrases, your
  measurement is already old.

Neither is better in every canyon.

---

## 4. Where this module sits in the receive pipeline

```text
  acquisition (synchronization.cpp)
        │  BodyAcquisition: start, carrier, phase, mode, first_body_symbol
        ▼
  ┌──────────────────────────────────────────────────────────────────────┐
  │ BodyAdaptiveReceiver                                 THIS MODULE     │
  │                                                                      │
  │ select_data_equalizer(adaptive | cir_mmse)       before training     │
  │ configure_fractionally_spaced(default window: 24 taps, ref 12, fb 8) │
  │ train_preamble_fractional                                            │
  │   flat gain over the prefix -> ProbeDfe::initialize_cursor_gain      │
  │   3 supervised passes over the preamble, NLMS                        │
  │   residual over the tail above the floor? the frequency race:        │
  │     whole-preamble estimate, channel-fit grid (cir_polyphase_solve)  │
  │   cir_mmse only: seed_cir_state (first CIR, first solved taps)       │
  └──────────────────────────────────────────────────────────────────────┘
        │  selected_training_frequency_radians
        ▼
  estimate_body_preamble_delay_spread (synchronization.cpp)
        │  delay spread, in symbols
        ▼
  select_body_receiver_geometry:  0..6 -> {24, 12, 8}
                                  7+   -> {32,  6, 14}   (reaches a 5 ms echo)
        │  a different geometry? configure and train again on it
        ▼
  ┌──────────────────────────────────────────────────────────────────────┐
  │ equalize_block_fractional, frame by frame                            │
  │   adaptive:  ProbeDfe::filter + adapt on reliable decisions,         │
  │              ProbeDfe::train on the probe, collapse retrain          │
  │   cir_mmse:  solve the CIR on each probe (cir_polyphase_solve),      │
  │              solve the taps (cir_mmse_dfe_solve), interpolate them   │
  │              across the frame                                        │
  │   data symbols scaled by 1 / (local probe residual)                  │
  └──────────────────────────────────────────────────────────────────────┘
        │  equalized symbols z, one per transmitted symbol
        ▼
  soft demapper, deinterleaver, decoder                     (Part 7)
```

In this repository the caller is the walkthrough's `train` step in
`tests/walkthrough/equalizer_steps.cpp`: it trains on the default window, measures the
delay spread with the frequency the training kept, and configures and trains again when
the measurement selects the long window. The receiver itself never reads the mode label
to choose a window.

**Build gating.** `cir_equalizer.cpp` compiles in every target, but the engine that calls
it in the body walk, the receiver's `cir_mmse` engine, is host-only: the
`M110_BODY_RECEIVER_CIR_ENGINE` option in `CMakeLists.txt` is ON by default for the
native build and OFF for an embedded one, which drops about 14 KiB of engine state from
every receiver. `receiver.hpp` defines the macro to 0 when the build does not set it.
The adaptive family has no such gate; on an embedded build it is the only body equalizer.
The preamble's channel-fit grid uses `cir_polyphase_solve` in every build.

**Inputs:** the received T/2 samples (early, then on time, for each symbol), the
`BodyAcquisition`, the preamble's expected tribits, and the geometry. **Outputs:**
equalized symbols, scaled by the local channel quality; diagnostics (the race, the
residuals, the taps). **Depends on:** `types.hpp`, `status.hpp`, `scrambler.hpp`
(`BodyDataRandomizer`), `demapper.hpp` (`psk8_symbol`, `psk8_hard_demapper`),
`body_waveform.hpp` (`BodyBlockPlan`), `timing_recovery.hpp` (`CarrierTracker`). No heap:
every buffer is a fixed `std::array` sized by the `maximum_*` constants.

---

## 5. Important data structures

### From `equalizer.hpp` (adaptive)

| Type | Purpose |
|---|---|
| `ChannelEstimate` | A **flat** (one-tap) channel: the complex `gain` and the `mean_squared_error` of the fit. |
| `LmsEqualizer` | A T-spaced **NLMS** transversal equalizer (up to 32 taps): a tapped delay line and a normalized-LMS update with optional leakage. The textbook baseline; the receiver does not use it, and the unit tests check it. |
| `DfeAdaptation` | `nlms` or `rls`: which adaptation `ProbeDfe` uses. The receiver uses `body_receiver_adaptation`, which is `nlms`. |
| `ProbeDfe` | The decision-feedback equalizer: up to 32 feed-forward taps (T or T/2) around a `cursor` (the reference tap), up to 16 feedback taps on past decisions. Trains on known symbols, then filters data. |
| `ProbeDfe::NlmsState` | Exactly what NLMS training changes (the feed-forward and feedback taps and the decision history), so a caller can capture and restore a training without the RLS matrix. |

### From `cir_equalizer.hpp` (model-based)

| Type | Purpose |
|---|---|
| `CirPolyphaseProblem` | One polyphase weighted least-squares problem: up to 48 observations against rows of known or decided symbols, each with a weight. The caller builds the rows, because it owns the framing, the scrambler and the decisions. |
| `CirSolveScratch` | Scratch for the Hermitian solves (the largest is feed-forward taps squared, complex). |
| `CirLayout` | The **polyphase lag layout** for a geometry: the CIR's support around the cursor (`precursor`, `postcursor`, `support`) and how it splits into even and odd T-spaced sub-channels. The preamble's channel-fit search and the CIR-MMSE walk index one CIR through it. |
| `CirMmseGeometry` | The equalizer's geometry (feed-forward taps, cursor, feedback taps) and the channel's support, for the MMSE solve. |

### From `receiver.hpp`

| Type | Purpose |
|---|---|
| `BodyReceiverGeometry` | Feed-forward taps, cursor and feedback taps of a window. `select_body_receiver_geometry` maps a delay spread to one. |
| `BodyDataEqualizer` | `adaptive` or `cir_mmse`: the engine that walks the body. |
| `BodyAdaptiveReceiver` | The engine: one `ProbeDfe`, one `CarrierTracker`, the preamble-fit workspace, the training state machine and, in a host build, the CIR engine's state. |
| `BodyAdaptiveReceiver::PreambleRaceDiagnostics` | The last frequency race: every frequency trained (acquisition's first), each one's residual over the preamble's tail, the winner, and whether the race ran at all (`escalated`). |
| `BodyFrameState` | What the adaptive walk carries from frame to frame inside one interleaver block (the data randomizer, the previous probe's residual, the running baseline), so a streaming caller can equalize a frame at a time. |

### Key constants

```text
ProbeDfe::maximum_feedforward_taps ........ 32    (T/2 spaced in the receiver)
ProbeDfe::maximum_feedback_taps ........... 16    (T spaced, on decisions)
LmsEqualizer::maximum_taps ................ 32
body_receiver_feedforward_taps ............ 24    default window, reference tap 12
body_receiver_feedback_taps ............... 8
body_receiver_default_delay_symbols ....... 6     a larger delay spread selects {32, 6, 14}
body_receiver_equalizer_step_robust ....... 0.10  NLMS step at 75, 150 and 300 bps
body_receiver_equalizer_step_fast ......... 0.40  NLMS step at 600 bps and up
body_receiver_carrier_proportional_gain ... 0.02
body_receiver_carrier_integral_gain ....... 0.0002
body_receiver_collapse_absolute_threshold . 0.9   probe residual that is an outage on its own
body_receiver_adaptation .................. nlms
body_receiver_rls_forgetting .............. 0.99  (used only if RLS were selected)
cir_precursor_taps ........................ 4     default precursor; the seed searches 2, 4, 6, 8
cir_maximum_taps .......................... 32    T/2 CIR support
cir_polyphase_taps ........................ 16    per sub-channel
cir_maximum_rows .......................... 48
cir_preamble_seed_symbols ................. 48    the preamble tail the CIR fits use
```

Inside `receiver.cpp`: `probe_noise_floor` 0.01 (the floor under every residual), the
collapse test's `collapse_retrain_threshold` 0.50, `collapse_retrain_baseline_ratio` 2.0
and `collapse_retrain_passes` 4, and the channel-fit grid's `preamble_grid_search_span`
0.08 and `preamble_grid_search_step` 0.008 radians per symbol (±30.6 Hz in steps of
3.06 Hz). Inside `cir_equalizer.cpp`: `shaping_rolloff` 0.25.

---

## 6. Function-by-function walkthrough

### The adaptive family (`equalizer.cpp`)

#### `estimate_flat_channel(received_probes, known_probes)`
- **50K view:** Least-squares fit of one complex gain `g` that best maps the known
  symbols onto what was received, and the mean squared error left over.
- **Detailed view:** `g = (Σ r·conj(k)) / (Σ |k|²)`; then MSE = mean `|r − g·k|²`. The sums
  are strict left-to-right folds, so the result is the same on every build. The
  receiver's preamble training computes the same gain over the 288-symbol prefix, one
  symbol at a time so the work can be sliced, and seeds the DFE's reference tap with it.
- **5th-grade view:** One number for how much the channel shrank and turned the signal,
  from comparing what you heard with what you know was sent.

#### `LmsEqualizer::configure / reset / filter / train / decision_directed`
- **50K view:** A normalized-LMS transversal equalizer. `train` adapts toward a known
  symbol, `decision_directed` toward a decision, and `filter` only filters.
- **Detailed view:** Each call shifts the input into a tapped delay line, computes
  `Σ c[i]·history[i]`, and when adapting updates
  `c ← (1−leak)·c + (μ/energy)·error·conj(history)` with `error = desired − output`.
  Dividing the step by the input energy (NLMS) makes convergence independent of the input
  level; leakage pulls coefficients toward zero so they stay bounded on noise alone.
  `reset` sets a unit first tap.
- **5th-grade view:** Turn the dials a little in the direction that shrinks the mistake,
  with smaller turns when the sound is loud, so loud and quiet inputs learn at the same
  pace.

#### `ProbeDfe::configure / reset / initialize_cursor_gain`
- **50K view:** Set up the DFE's geometry, and seed the reference tap from a flat channel
  estimate.
- **Detailed view:** `configure` checks the geometry (1 or 2 samples per symbol, the cursor
  inside the feed-forward span, at most 16 feedback taps, the step and the forgetting
  factor in range) and selects NLMS or RLS. `reset` zeroes the taps and sets a unit
  cursor. `initialize_cursor_gain(g)` re-seeds everything (feed-forward, feedback,
  decision history and the RLS matrix) with `w[cursor] = conj(g)/|g|²`, the inverse of the
  flat gain. The full re-seed matters because preamble training runs once per frequency
  in the race, and each run has to depend only on the samples and its frequency, never on
  the taps a previous run left.
- **5th-grade view:** Point your ears roughly the right way before fine-tuning, and wipe
  the slate clean each time you start over.

#### `ProbeDfe::filter / train / adapt` (and the private `output`)
- **50K view:** `filter` computes the DFE output; `train` also adapts the taps toward
  `desired` and returns the output; `adapt` adapts without returning it.
- **Detailed view:** `output = Σ w[t]·window[t] − Σ b[j]·decision_history[j]`, the DFE
  equation with **plain complex products** (the conjugation convention is in section 8).
  `train` and `adapt` run NLMS or RLS, then commit `desired` into the decision history.
  A window shorter than the feed-forward span returns zero and changes nothing.
- **5th-grade view:** Add up what your cupped ears hear of the current word, then subtract
  the remembered echoes of the words you already worked out.

#### `ProbeDfe::train_nlms` (private)
- **50K view:** One normalized-LMS step of both filters from one symbol. This is the
  receiver's adaptation.
- **Detailed view:** `error = desired − output`; the step is divided by the **joint**
  energy of the feed-forward window and the feedback history; then
  `w ← (1−leak)w + μ̃·e·conj(window)` and `b ← (1−leak)b − μ̃·e·conj(history)`. The
  feedback step is negative because the feedback sum is subtracted in the output. The
  receiver runs it with no leakage and the rate's step from
  `body_receiver_equalizer_step_for`.
- **5th-grade view:** One gentle correction to both your listening and your echo memory,
  sized to how loud things are right now.

#### `ProbeDfe::train_rls` (private, an option the receiver does not select)
- **50K view:** Exponentially weighted recursive least squares: it converges in about
  twice the tap count instead of many tens of symbols, at O(taps²) per symbol and an
  (ff+fb)² complex matrix.
- **Detailed view:** Over the joint input `u = [window; −history]`, it keeps the inverse
  correlation matrix `P`. Each symbol: `g = P·conj(u)`, `denom = λ + uᵀg`, `k = g/denom`,
  `w += k·e`, `P = (P − k·gᴴ)/λ`. A **trace cap** (`200 × joint taps`) keeps `P` bounded:
  the `1/λ` growth otherwise winds `P` up along directions the input never excites and
  overflows float within minutes; scaling `P` back leaves the excited directions'
  geometry alone. A trace that is not positive and finite restarts from `P = 100·I`.
  The matrix is in every `ProbeDfe`, selected or not.
- **5th-grade view:** A faster learner that keeps a running summary of every past word,
  with a safety valve so its bookkeeping never blows up.

#### `ProbeDfe::capture_nlms_state / restore_nlms_state / clear_decision_history / commit_decision`
- **50K view:** Save and restore a training; clear or extend the feedback register.
- **Detailed view:** `capture_nlms_state` and `restore_nlms_state` move the feed-forward
  and feedback taps and the decision history: the whole end-of-training state under NLMS,
  and not under RLS (the `P` matrix is left out), so an RLS caller retrains instead. The
  frequency race uses them to keep its winner without training it twice.
  `commit_decision` shifts a decision into `decision_history[0]`; with no feedback taps
  it does nothing.
- **5th-grade view:** Write down how you were listening so you can go back to it exactly;
  and move each word you just worked out into your echo memory.

### The model-based family (`cir_equalizer.cpp`)

#### `make_cir_layout(feedforward_taps, precursor)` / `cir_tap_index(layout, even, lag_index)`
- **50K view:** How one **T/2 channel impulse response** is laid out around the cursor and
  split into two T-spaced sub-channels, and how to index between them.
- **Detailed view:** The CIR is `c[q]`, `q` from `−precursor` to `+postcursor` half-symbols
  around the cursor sample, stored at `q + precursor`.
  `postcursor = (feedforward_taps ≥ 32 ? 31 : 19) − precursor`, so the long window's CIR
  reaches a 5 ms echo (24 half-symbols) and the pulse's tails. The even/odd split turns
  one T/2 problem into two T-spaced ones; each of those sees **white** noise, because the
  matched filter's noise autocorrelation is zero at every nonzero even half-symbol lag.
  `cir_tap_index` turns `(even or odd, lag_index)` into the assembled index `q + precursor`.
- **5th-grade view:** Decide how long a map of the canyon's echoes to keep, and how to read
  its odd and even half-beats.

#### `cir_residual_dof` / `cir_noise_inflation`
- Two small shared formulas, so every CIR fit uses one derivation: the residual's degrees
  of freedom (rows minus the support, floored at 6) and the noise inflation from a finite
  support (`1 + support / denominator`).

#### `cir_cholesky_solve(matrix, rhs, dimension, pivot_floor)`
- **50K view:** Solve a Hermitian positive-definite system `A x = b` in place; return a
  not-positive-definite `Status` (`unavailable`) when a pivot collapses, so the caller can
  regularize and retry.
- **Detailed view:** Lower Cholesky factorization `A = L Lᴴ`, then forward and back
  substitution, with each diagonal pivot checked against `pivot_floor`. The polyphase and
  MMSE solvers call the same factorization through a private path.
- **5th-grade view:** The standard fast, stable way to solve a symmetric set of equations,
  with a tripwire if they cannot be solved.

#### `raised_cosine_autocorrelation_half_lag(half_symbol_lag)`
- **50K view:** The autocorrelation of the receive matched filter (root raised cosine
  twice, rolloff 0.25) at a half-symbol lag: the **shape of the noise correlation** in
  the T/2 stream.
- **Detailed view:** 1 at lag 0, exactly 0 at nonzero **even** lags (Nyquist), and the
  raised-cosine value at **odd** lags. The T/2 stream is band-limited, so its noise is
  **coloured**, not white; the MMSE solve needs that covariance.
- **5th-grade view:** How much the wind noise at one half-beat looks like the wind noise
  half a beat later.

#### `cir_polyphase_solve(problem, lambda, epsilon, prior, scratch, taps, residual_energy)`
- **50K view:** Solve one polyphase sub-channel of the CIR by **weighted ridge least
  squares**, pulled toward a prior and kept invertible by a small ridge floor.
- **Detailed view:** Minimizes `Σ w_s|x_s − r_sᵀg|² + λ|g − prior|² + ε|g|²` through the
  weighted normal equations `(Σ w_s r_s* r_sᵀ + (λ+ε)I) g = Σ w_s r_s* x_s + λ·prior`,
  Cholesky-solved, and returns the taps and the weighted residual energy after the solve
  (the fit's quality). `λ` pulls toward the prior (the previous probe's channel, so the
  estimate stays continuous); `ε` guarantees a solution. The preamble's fits use
  `λ = 0` and a zero prior; the body walk's `λ` comes from the measured noise and the
  channel's expected change between probes, clamped to 1..16.
- **5th-grade view:** Fit the canyon map to what you measured, without straying far from
  the map you already had.

#### `cir_mmse_dfe_solve(cir, geometry, noise_variance_effective, scratch, feedforward, feedback)`
- **50K view:** From the measured T/2 CIR, compute the finite-length **MMSE-DFE**
  feed-forward and feedback taps in one solve, with a regularization retry ladder.
- **Detailed view:** It builds the feed-forward correlation matrix
  `R = Σ_{lag∉[1,fb]} h_lag h_lagᴴ + N₀·Γ + jitter·I`, where `h_lag[t]` is the channel's
  response to the symbol `lag` symbols back at feed-forward tap `t`, and `Γ` is the noise
  autocorrelation above. The cursor lag (0) is **kept** (MMSE, not zero-forcing); lags
  `1..fb` are left out because the feedback taps cancel them exactly; any lag beyond the
  feedback register stays in as interference. It solves `R·conj(w) = h₀` (the cursor's
  column), stores `feedforward = conj(w)`, and sets `feedback[j] = Σ_t w[t]·h_{j+1}[t]`,
  the postcursor the feed-forward filter leaves for the register. The **2% diagonal
  jitter is load-bearing** (section 9). On a pivot collapse it rebuilds with ten times the
  jitter, up to 4 attempts, then returns `internal_error`.
- **Noise term:** `noise_variance_effective` is the caller's `N₀`; it sets how far the
  solve trusts the channel over the noise (the balance between MMSE and zero-forcing).
  The receiver's CIR engine supplies the fit's residual per degree of freedom, clamped to
  0.001..2 (in the body walk, averaged with the previous estimate), inflated by
  `cir_noise_inflation` for the finite support.
- **5th-grade view:** From the canyon map, work out the best way to listen and the best
  echo memory in one go, with a stabilizer so the arithmetic never divides by almost zero.

### The receiver (`receiver.cpp`)

#### `select_body_receiver_geometry(delay_spread_symbols)` and `body_receiver_equalizer_step_for(rate)`
- **50K view:** The window for a measured delay spread, and the NLMS step for a rate.
- **Detailed view:** A delay spread up to `body_receiver_default_delay_symbols` (6) keeps
  the default `{24, 12, 8}`: 24 T/2 taps with the reference tap in the middle reach 6
  symbols ahead and 5.5 behind, enough for a 2 ms echo (4.8 symbols) on either side.
  Anything larger gets `{32, 6, 14}`: 3 symbols ahead, 12.5 behind, and 14 feedback taps
  that span a 5 ms echo (12 symbols). The caller measures the spread on the preamble,
  never reads it from the mode. The step is 0.10 at 75, 150 and 300 bps, whose hard
  case is the 5 ms, 5 Hz channel, where a larger step lets gradient noise push the taps off
  the channel; 0.40 at 600 bps and up, which need the faster convergence.
- **5th-grade view:** A bigger room needs a longer memory for echoes.

#### `select_data_equalizer(engine)` / `configure_fractionally_spaced(...)` / `reset` / `rearm`
- **50K view:** Choose the engine, set up the DFE and the carrier loop, and clear state.
- **Detailed view:** `select_data_equalizer` returns `unavailable` for an engine compiled
  out of this build (`body_receiver_engine_available`); select before training, because
  the CIR engine seeds itself at the end of training, and an engine selected later finds
  no seed and walks that burst with the adaptive engine. `configure_fractionally_spaced`
  resets the receiver and configures `ProbeDfe` at 2 samples per symbol with NLMS and no
  leakage, and the `CarrierTracker` with the given gains. `rearm` keeps the configuration
  and drops the training (the next burst); `reset` drops both. The engine selection and
  the collapse threshold are receiver-definition settings that neither touches.
- **5th-grade view:** Pick your listening style, and clear your head between messages.

#### `train_preamble_fractional` (with `begin_preamble_training` / `step_preamble_training`)
- **50K view:** Train the taps and the carrier on the known preamble,
  and settle the carrier frequency by measurement.
- **Detailed view:** Training runs one frequency at a time (the inner trainer):
  1. The flat gain over the 288-symbol prefix, derotated by acquisition's phase and the
     frequency under test, seeds the reference tap (`initialize_cursor_gain`).
  2. Three supervised passes over the whole training span (eight for 75 bps with the
     short interleave, whose preamble is an eighth as long); each pass clears the
     feedback register and restarts the carrier loop, then trains every symbol with
     `ProbeDfe::train` and steps the carrier loop.
  3. The residual: the mean squared error over the last 96 symbols of the last pass,
     floored at `probe_noise_floor` (0.01).

  Acquisition's frequency trains first. If its residual is at the floor, the race is
  skipped: no other frequency could score strictly lower, so skipping gives the same
  result. Otherwise up to two more frequencies train, each only when it differs from the
  ones already in by more than 0.004 radians per symbol (1.5 Hz):
  - the **whole-preamble estimate**: the angle of the lag-1 product summed over the whole
    known preamble, kept only within 0.08 radians per symbol (30.6 Hz) of acquisition's;
  - the **channel-fit grid** (T/2 only, with at least 64 known symbols): 21 frequencies
    from −30.6 Hz to +30.6 Hz around acquisition's, each scored by two zero-prior
    `cir_polyphase_solve` fits of the last 48 preamble symbols; the smallest residual
    among the interior points wins (a minimum at the edge is the signature of an alias).

  The frequency with the strictly smallest residual is kept; a tie keeps acquisition's.
  Under NLMS with the adaptive engine the winner's end state is restored from a snapshot
  instead of being trained again. `begin_preamble_training` and
  `step_preamble_training(quota)` run the same machine a bounded amount of work at a time,
  so a streaming receiver stays under its per-callback deadline and ends with the same
  receiver bit for bit; the batch call drives it to completion.
- **5th-grade view:** Try a few positions of the tuning dial and keep the one that sounds
  clearest, a little at a time so you never hog the airtime.

#### `seed_cir_state` (private, CIR engine only)
- **50K view:** The CIR engine's starting point: the first channel estimate, the first
  solved taps, and the carrier ramp and precursor the body walk starts from.
- **Detailed view:** At the end of every training run (the winner's is the one that
  stands), it fits the CIR to the last 48 preamble symbols over a joint search: the
  precursor (2, 4, 6 or 8 half-symbols) and the carrier frequency (±0.10 radians per
  symbol around the trained one, in steps of 0.01), each point two zero-prior polyphase
  solves, the smallest residual per row winning. The precursor axis is there because the
  timing can put the direct path before a short fixed precursor; the frequency axis
  because a static ramp error turns successive channel estimates by whole radians, which
  tap interpolation cannot follow. It adopts the winner's ramp and precursor, takes the
  noise variance from the residual, solves the first taps with `cir_mmse_dfe_solve`, and
  loads the feedback history with the preamble's last symbols.
- **5th-grade view:** Measure the canyon once from the agreed opening phrase, before the
  message starts.

#### `equalize_block_fractional` / `equalize_frame_fractional`: the adaptive walk
- **50K view:** One frame: the unknown data symbols, then the probe; retrain on a frame a
  fade knocked the equalizer off.
- **Detailed view:**
  - **Data symbols.** Each window is derotated by the carrier loop's phase and frequency
    and filtered. The decision is the nearest 8-PSK point (`psk8_hard_demapper`). It
    adapts the taps and steers the carrier loop only when it is reliable: the output
    carries real energy (`|z|² ≥ 0.25`) and its distance to the decision is inside the
    distance to the next 8-PSK point (`|d − z|² ≤ 2 − √2`). An unreliable decision still
    enters the feedback register, and the carrier phase advances open-loop.
  - **The probe.** Every known symbol trains the taps and the carrier loop
    (`ProbeDfe::train`). The probe's symbols are the scrambled pattern of 0, or of D1 and
    D2 in a block's last two frames, as the transmitter sends them.
  - **The collapse retrain.** If the probe's mean squared error is over 0.5 and over twice
    the block's running baseline, or over the absolute threshold (0.9 by default,
    `set_collapse_absolute_threshold`), the fade broke the equalizer: the feedback
    register is cleared and the probe is replayed 4 times. The absolute term catches the
    outage the relative test misses at low SNR, where the baseline itself sits near 0.3 to
    0.5. The baseline is a slow average (0.9 old, 0.1 new) that a collapsed frame does not
    move.
  - **Channel-state weighting.** Each data symbol is divided by the probe residual
    interpolated between the previous probe and this one (each floored at 0.01). A
    symbol near a weak stretch comes out smaller, so the soft demapper downstream gives
    its bits less confidence, with no change to its interface.

  `begin_block_frames`, `equalize_frame_fractional` and `end_block_frames` give a streaming
  receiver the same walk a frame at a time; `equalize_block_fractional` runs them over a
  block and reproduces it exactly.
- **5th-grade view:** Work out each word and keep adjusting; when the agreed phrase shows
  you have drifted badly, re-tune on it a few times before moving on.

#### `equalize_block_cir` (private): the CIR-MMSE walk
- **50K view:** Measure the channel at every probe, solve the taps, and interpolate them
  across the frame, so both probes around a data symbol inform it.
- **Detailed view:** Per frame:
  1. **Pass 1:** provisional decisions from the previous probe's taps plus a small,
     leaky decision-directed correction (step 0.25, leak 0.95).
  2. **Two channel estimates** at this frame's probe, each a ridge solve toward the
     previous channel: one with rows from the probe and the pass-1 decisions, one from
     the probe alone. Taps under the noise are zeroed, an estimate with almost no power
     is rejected, and each surviving estimate gets its MMSE taps.
  3. **Pass 2:** for each estimate, the data symbols again, the taps interpolated linearly
     from the previous probe's to this one's, with the same leaky correction on top (the
     interpolation alone cannot follow a fade's phase jump inside a frame); then the probe,
     scored. The estimate with the lower probe error wins. Rows built on decisions follow
     the channel best while the decisions hold; once they go wrong, a fit to them can go
     wrong with them, and the probe-only estimate cannot.
  4. **Outage:** if the winner fails the same collapse test as the adaptive walk (with its
     absolute term fixed at 0.9), the frame is an outage. One probe cannot re-identify the
     channel alone, so clean probe rows are stacked over consecutive outage frames and the
     channel is solved again, with no prior, once the stack is overdetermined.
  5. **Carrier:** a small frequency correction from how far the channel estimate turned
     from one probe to the next, kept phase-continuous.

  The data symbols are weighted by `2 / (previous probe residual + this one)`. A block of
  75 bps, and a burst whose seed is missing, take the adaptive walk.
- **5th-grade view:** Each time the agreed phrase comes, measure the canyon again, and
  blend the old and new measurements across the words in between.

#### `equalize_75_group_fractional`
- 75 bps has no probes: each 32-symbol group is filtered, correlated against the four
  spread patterns it could be, and the receiver retrains on the winner. It requires zero
  feedback taps (the walkthrough configures 75 bps that way).

#### `equalize_block_supervised_fractional`
- Re-equalizes a block with every symbol trained against a reference the caller supplies
  (for example, a first decode re-encoded). That is for a caller with a decoder: Part 7.

#### `last_block_probe_error` / `last_preamble_race` / `selected_training_frequency_radians` / `equalizer`
- Diagnostics: the mean probe residual of the last block (low in lock, near 1 on noise or
  a lost carrier; a streaming receiver uses it to detect loss of lock), the last race, the
  frequency training kept, and the `ProbeDfe` with its taps.

---

## 7. Worked example

Real output from `equalizer_demo`, on the article's recordings (configure with
`-DMODEM_ARTICLE_DATA=ON`). The example follows fp1: the "Hi" message at 300 bps with the
long interleave (300L), through a two-path channel with a 5 ms delay and 5 Hz fading spread
at 7 dB, simulated with pathsim. Every decision is checked against the symbol the
transmitter sent.

### Step 1: how far behind the anchor

Acquisition anchored on the direct path. The delay-spread estimate (the acquisition
explainer covers it) reads the echo 12 symbols behind it:

```text
fp1-echo-group  1.00    .    .    .    .    . 0.06    .    .    .    . 0.05 0.63    .    .    .
                delay spread 12 symbols, over 8 preamble segments
```

### Step 2: a window sized from the measurement

```text
fp1-echo-group  spread 12 -> 32 taps, reference tap 6: 3.0 symbols ahead, 12.5 behind, 14 feedback; trained again on it
```

The default window reaches 5.5 symbols behind its reference tap; the echo is at 12. The
long window reaches 12.5 behind, and its 14 feedback taps cover the echo, so the feedback
filter can cancel it once the earlier symbol is decided. The receiver is configured again
and trained again on the long window.

### Step 3: training on the preamble

```text
fp1-echo-group: acquisition's frequency +2.55 Hz
its error stayed above the floor, so the receiver trained every distinct guess:
acquisition's estimate       +2.55 Hz   error on the preamble's tail 0.623
a new guess                 +21.13 Hz   error on the preamble's tail 0.741
a new guess                  -0.51 Hz   error on the preamble's tail 0.622   <- kept
```

Acquisition's residual was far above the 0.01 floor, so the race ran. The whole-preamble
estimate (+21.13 Hz, 18.6 Hz from acquisition's) was in the 1.5 to 30.6 Hz window and
trained; it scored worse (the whole-preamble estimate can carry a fading bias of its
own). The channel-fit grid's minimum was one grid step
(3.06 Hz) below acquisition's, at −0.51 Hz, and scored 0.622 against 0.623: strictly
lower, so it was kept. On a 5 Hz fading channel the three estimates can disagree by tens
of hertz; the residual over the known tail settles it.

### Step 4: following the channel (the adaptive engine)

At 300 bps a frame is 20 data symbols and a 20-symbol probe; the body is 288 frames,
5760 data symbols.

```text
adaptive on fp1-echo-group: 951 of 5760 data symbols decided wrong before the decoder (16.5%); 36 of 288 frames without one
the first 24 frames (wrong data symbols / probe error):
 1/0.73  4/1.12  2/0.72  2/0.69  3/0.82  3/0.69  3/0.63  3/0.57
 1/0.83  6/0.72  4/0.79  3/0.71  3/0.50  4/0.53  1/0.79  2/0.74
 3/0.59  1/0.33  1/0.38  0/0.31  0/0.47  1/0.80  6/0.88 10/1.10
```

"Wrong" is counted in the mode's own alphabet. At 300 bps each data symbol carries one
bit, so the transmitter could have sent one of two 8-PSK points there (the sent one, or
the one turned half a circle); a decision is wrong when the other one is nearer. (At 1200
bps there are four candidates, at 2400 all eight.) The probe errors near 1 (frames 2 and
24) are where the 5 Hz fade took the signal down; both probes, at 1.12 and 1.10, are over
the 0.9 absolute term. The probe error the demo prints is the one the receiver's collapse
test reads (the probes are not scaled), so both frames set off the retrain. Errors this dense are left for the
decoder: at 300 bps each bit goes through a rate-1/2 code and every coded pair is sent
twice.

### Step 5: measuring the channel (the CIR-MMSE engine)

The same burst with `select_data_equalizer(BodyDataEqualizer::cir_mmse)`:

```text
CIR-MMSE on fp1-echo-group: 1286 of 5760 data symbols decided wrong before the decoder (22.3%); 53 of 288 frames without one
the first 24 frames (wrong data symbols / probe error):
 0/0.34  0/0.61  2/0.42  1/0.51  1/0.44  1/0.38  1/0.35  0/0.37
 0/0.65  0/0.49  0/0.37  0/0.38  0/0.26  0/0.20  0/0.30  0/0.39
 0/0.27  0/0.23  1/0.19  0/0.29  0/0.48  3/0.70  5/0.71  7/1.35
```

More clean frames than the adaptive engine (53 against 36), and lower probe errors through
the opening frames, but more wrong decisions overall: 22.3% against 16.5%.

### Step 6: when the channel moves

The frames where each engine trails the other most:

```text
block frame   adaptive      CIR-MMSE
CIR-MMSE behind:
    0    96    0 / 0.40     10 / 1.06
    0    97    0 / 0.29     10 / 1.50
    0   127    0 / 0.61     13 / 1.32
adaptive behind:
    0   117    7 / 0.87      1 / 0.74
    0   174    6 / 0.41      0 / 0.23
    0   212    8 / 0.69      1 / 0.56
```

The CIR-MMSE engine's errors come in bursts: in a fade its channel estimate from the last
probe is already old, its probe error climbs past 1, and half a frame goes wrong at once.
The adaptive engine spreads its errors: a few in most frames, rarely ten. On this burst
that adds up in the adaptive engine's favour; at 2400 bps on the standard's channels, the
CIR-MMSE engine comes out ahead. `tests/equalizer_walkthrough_tests.cpp` checks both
facts about this recording: the adaptive engine has fewer wrong decisions, and each
engine has frames where it trails the other.

### Inside one CIR-MMSE solve (illustrative)

The demo does not print the taps. The numbers below are **made up for illustration**: a
two-path channel like fp1's, through the long window, to show what `cir_polyphase_solve`
and `cir_mmse_dfe_solve` do.

```text
make_cir_layout(feedforward_taps = 32, precursor = 6):
  postcursor = 31 − 6 = 25 half-symbols; support = 32; 16 even and 16 odd taps

the CIR after the two polyphase solves (magnitudes, illustrative):
  q (half-symbols from the cursor):  −1     0    +1   ...  +23   +24   +25
  |c[q]|:                            0.63  1.00  0.63 ...  0.35  0.55  0.35
                                           ▲                     ▲
                                       main path            the echo, 12 symbols late
```

A single path shows on three neighbouring half-symbol taps, because the matched pulse at
half a symbol is still about 0.63 of its peak. The echo at 12 symbols is `q = +24`.

```text
cir_mmse_dfe_solve, geometry {32 taps, cursor 6, 14 feedback}:
  R = Σ_{lag ∉ [1..14]} h_lag h_lagᴴ + N₀·Γ + jitter·I        jitter = 0.02 × trace/32
  solve R·conj(w) = h₀  ->  feedforward w[0..31]
  feedback[j] = Σ_t w[t]·h_{j+1}[t],  j = 0..13

the echo is symbol lag 12: inside 1..14, so it is not in R; feedback[11] cancels it.
```

Each data symbol is then `z = Σ w[t]·window[t] − Σ b[j]·d[k−1−j]`, with the window
derotated by the carrier ramp and `d` the decisions already made. The adaptive engine
reaches taps that do the same thing by a different route: the NLMS steps over the
preamble, every probe, and every reliable decision.

### One-screen summary

```text
1. select_data_equalizer, configure_fractionally_spaced(24, 8, step, gains, 12)
2. train_preamble_fractional    flat gain -> cursor; 3 passes; the race if needed
3. estimate_body_preamble_delay_spread with the trained frequency
4. select_body_receiver_geometry; different? configure and train again
5. equalize_block_fractional per interleaver block:
     adaptive:  filter, adapt on reliable decisions, train on the probe, collapse retrain
     cir_mmse:  pass 1, two CIR estimates, pass 2 with interpolated taps, outage stacking
     every data symbol scaled by 1 / local probe residual
6. the equalized symbols go to the soft demapper (Part 7)
```

---

## 8. Equations and formulas

**Flat channel estimate:**
```text
g = ( Σ_k r[k]·conj(s[k]) ) / ( Σ_k |s[k]|² )       MSE = mean_k |r[k] − g·s[k]|²
reference-tap seed: w[cursor] = conj(g)/|g|²
```

**DFE output (the code's convention, plain products):**
```text
z = Σ_t w[t]·window[t]  −  Σ_j b[j]·d[k−1−j]
```
The code stores `w = conj(Wiener solution)`, so `Σ w·window` is `uᴴ·window`; the feedback
taps are stored as the postcursor response itself, subtracted in the sum, with no extra
sign flip. `ProbeDfe::output` and `cir_mmse_dfe_solve` follow the same convention.

**NLMS update (μ̃ = μ / joint energy, retention = 1 − leak):**
```text
e = d − z
w ← retention·w + μ̃·e·conj(window)
b ← retention·b − μ̃·e·conj(history)
```

**RLS update (joint input u = [window; −history]):**
```text
g = P·conj(u)          denom = λ + uᵀ·g          k = g/denom
w += k·e               P = (P − k·gᴴ)/λ          trace(P) capped at 200·(ff+fb)
```

**Polyphase ridge least squares (per sub-channel):**
```text
minimize   Σ_s w_s |x_s − r_sᵀ g|²  +  λ |g − prior|²  +  ε |g|²
normal eq: ( Σ_s w_s conj(r_s) r_sᵀ + (λ+ε) I ) g = Σ_s w_s conj(r_s) x_s + λ·prior
```

**MMSE-DFE normal equations:**
```text
h_lag[t] = c[t − cursor + 2·lag + precursor]        (0 outside the support)
R = Σ_{lag ∉ [1..fb]} h_lag h_lagᴴ  +  N₀·Γ  +  jitter·I
Γ[i][j] = raised_cosine_autocorrelation_half_lag(|i − j|)
jitter  = 0.02 · trace(signal + noise) / taps          (×10 per retry, up to 4 attempts)
solve  R·conj(w) = h₀            feedback[j] = Σ_t w[t]·h_{j+1}[t]
```
The cursor lag 0 is kept (MMSE, not zero-forcing); lags `1..fb` are cancelled by the
feedback; lags beyond `fb` stay as modelled interference.

**The frequency race:**
```text
residual(f) = max( 0.01,  mean over the last 96 symbols of the last pass of |d − z|² )
race runs when residual(f_acquisition) > 0.01
f_whole = arg( Σ_k r[k]·conj(s[k]) · conj( r[k−1]·conj(s[k−1]) ) )    over the preamble
grid:     f_acquisition + n·0.008,  n = −10..10      (interior points eligible)
kept:     the strict minimum of residual(f); a tie keeps f_acquisition
```

**The collapse test and the channel-state weight (adaptive walk):**
```text
e_probe  = mean over the probe of |d − z|²
collapse = (e_probe > 0.5  and  e_probe > 2·baseline)  or  e_probe > 0.9
baseline ← 0.9·baseline + 0.1·max(e_probe, 0.01)      unless e_probe ≥ 2·baseline
weight of data symbol i (1..N) = 1 / ( (1−α)·e_previous + α·e_this ),  α = i/(N+1)
```

**Reliable decision (both engines):**
```text
|z|² ≥ 0.25   and   |d − z|² ≤ 2 − √2 = 0.5858     (the squared distance to the next 8-PSK point)
```

---

## 9. What can go wrong

| Symptom | Cause | What the code does, or what to do |
|---|---|---|
| The adaptive equalizer does not converge on a probe | A 20-symbol probe is short for 46 taps (32 + 14). | The adaptive engine also trains on every reliable data decision, so the probe only has to correct it. The CIR engine solves the channel's taps instead, which the probe and the frame's decisions overdetermine. Neither is ahead on every channel. |
| The CIR-MMSE engine loses whole frames in a fade | Its estimate comes from the last probe, and the channel moved since. | The leaky correction inside the frame, the probe-only estimate when decisions go wrong, and outage stacking; on 300L through 5 ms the adaptive engine still gets fewer symbols wrong (section 7). |
| A fit to the frame's decisions spirals into a whole-block outage | Wrong decisions fitted self-consistently. | The probe-only estimate runs beside it, and the lower probe error wins. |
| Feed-forward taps swing 100 to 200% between neighbouring probes on a steady channel | The signal and the band-limited noise share the same out-of-band directions; there the solution is estimation noise over a tiny eigenvalue. | The **2% diagonal jitter** holds that part near zero, at about 0.1 dB of in-band error. It is load-bearing, not defensive. |
| The MMSE solve returns "lost positive definiteness" | A near-singular `R` (a deep null, or the third of the T/2 band with no signal at 25% excess bandwidth). | The retry ladder: up to 4 attempts, ten times the jitter each. The walk then keeps the previous probe's taps. |
| RLS output overflows after minutes | The `1/λ` growth winds `P` up along directions the input never excites. | The trace cap, and a restart from `P = 100·I` on a non-finite trace. The receiver uses NLMS, so this matters only to a caller that selects RLS. |
| One wrong decision causes more | It feeds a wrong echo cancellation into the next symbols. | The feedback span is bounded (14 or 8), the reliability test keeps a doubtful decision out of the adaptation, and the collapse retrain clears the register after a broken frame. The decoder corrects the rest. |
| An echo beyond the feedback register | A path later than the register reaches. | MMSE keeps that lag in `R` as interference for the feed-forward filter. For the standard's 5 ms channel the long window's 14 feedback taps reach past the echo at 12. |
| A 5 ms echo outside the window | The default window reaches 5.5 symbols behind its reference tap. | The caller measures the delay spread and selects `{32, 6, 14}`; trains again on it. |
| A direct path before the CIR's short precursor | The timing put the first path early. | `seed_cir_state` searches the precursor (2, 4, 6, 8), and the walk keeps the one that fit best. |
| Training at a frequency tens of hertz off | A fade biased acquisition's estimate. | The frequency race: up to three frequencies, kept by the measured residual. |
| A full outage the collapse test never declares | At low SNR the baseline is itself 0.3 to 0.5, and an outage (about 1.0) never clears twice it. | The absolute term, 0.9 (`set_collapse_absolute_threshold`, +infinity turns it off). |
| `select_data_equalizer` returns `unavailable` | The CIR engine is compiled out (`M110_BODY_RECEIVER_CIR_ENGINE` OFF, an embedded build). | Use the adaptive engine, or build for the host. |
| The CIR engine selected, but the adaptive walk ran | It was selected after training, so there was no seed; or the burst is 75 bps. | Select before configuring and training. |

---

## 10. Debugging signals to watch

| Signal | What it tells you |
|---|---|
| `last_preamble_race()` | Whether the race ran (`escalated`), every frequency trained with its residual, and the winner. On a clean burst it stops at acquisition's. |
| `selected_training_frequency_radians()` | The frequency to derotate the delay-spread estimate with. |
| The delay spread and `BodyDelaySpreadProfile` | Which window the caller selected, and why (the acquisition explainer). |
| `last_block_probe_error()` | Mean probe residual of the last block: well under 1 in lock, near 1 on noise or a lost carrier. |
| The per-frame probe error (the walkthrough prints it) | Where the fades are; an adaptive frame whose probe error is over 0.9 set off the collapse retrain. |
| `equalizer().feedforward_coefficients()` / `feedback_coefficients()` | The trained taps. With the long window on a 5 ms channel, expect the feedback energy near tap 11 (the echo at lag 12). |
| `ChannelEstimate::mean_squared_error` | Flat-fit residual: high on a dispersive channel, which the DFE is there for. |
| `cir_polyphase_solve`'s `residual_energy` | The channel fit's quality: low for a clean estimate, high when the channel moved during the rows. |
| `cir_mmse_dfe_solve`'s `Status` | `internal_error` after the retries: a near-singular `R`. |
| Decision errors against the transmitter (the walkthrough) | The ground truth `equalizer_demo` counts, in the mode's alphabet. |

Healthy:
```text
the race stops at acquisition's frequency, or keeps one with a clearly lower residual;
the delay spread matches the channel; probe errors well under 0.5;
the equalized symbols cluster on the mode's points
```

Unhealthy:
```text
probe errors near 1 frame after frame, collapse retrains in a row,
a race whose residuals are all high and close together,
the equalized symbols smeared around the circle -> fading faster than the probes follow,
or the window too short for the channel
```

---

## 11. How to read this module

1. Start with the **DFE equation** (`ProbeDfe::output`): everything else exists to fill in
   `w` and `b`. Fix the conjugation convention from section 8 in your head.
2. The **adaptive** family in `equalizer.cpp`: `LmsEqualizer` (the simplest NLMS), then
   `ProbeDfe::train_nlms`, then `train_rls`.
3. The **model-based** family in `cir_equalizer.cpp`, in this order: `cir_cholesky_solve`,
   `make_cir_layout`, `cir_polyphase_solve`, `cir_mmse_dfe_solve`, and
   `raised_cosine_autocorrelation_half_lag`, which is why the noise model is coloured and
   why the even/odd split is legitimate.
4. `receiver.hpp`: the constants (each comment gives the reason for its value),
   `select_body_receiver_geometry`, and the public interface of `BodyAdaptiveReceiver`.
5. `receiver.cpp`: `inner_trainer_step` and `finalize_hypothesis` (training and the race),
   then `equalize_frame_impl` (the adaptive walk), then `seed_cir_state` and
   `equalize_block_cir`.
6. `equalizer_demo` (`tests/equalizer_demo.cpp`), then `tests/walkthrough/equalizer_steps.cpp`: the demo runs the
   article's six steps and prints every number section 7 quotes, and each step is one
   function there (`train` is the caller's order: train, measure, select, train again).
   `tests/equalizer_walkthrough_tests.cpp` checks the same steps. Set a breakpoint in a
   step and step through it from either.
7. `tests/equalizer_tests.cpp`: the solvers on known channels, and the receiver's
   contracts (the race skipped on an ideal channel, resumable training equal to the batch
   training bit for bit, the engine selection).

---

## 12. Maintainer notes

- **Two tap conventions stay in step.** `ProbeDfe::output` and `cir_mmse_dfe_solve`
  produce taps for the same filter equation, and the CIR walk filters with them. Change
  one (sign, conjugation, cursor) and change the other.
- **The 2% jitter and the RLS trace cap are measured values.** One cures 100 to 200% tap
  swings, the other float overflow within minutes. Re-measure before changing them.
- **`capture_nlms_state` / `restore_nlms_state` are NLMS-only by contract.** They leave
  the RLS matrix out; the race snapshots its winner only for the adaptive engine under
  NLMS (`memoize_winner`) and trains the winner again otherwise.
- **`initialize_cursor_gain` re-seeds everything on purpose,** so each frequency's
  training depends only on the samples and that frequency, and the race is reproducible.
- **The race skip is outcome-identical.** It skips only at the residual floor, where no
  other frequency can score strictly lower. Raising the skip residual above the floor
  trades that for a measurement.
- **Resumable means bit for bit.** The sliced training and the frame-by-frame walk are
  the batch paths; the batch calls drive them. The grid's frequency is a running sum,
  never recomputed from an index, so slicing never changes a trial frequency.
- **The window comes from the measurement.** `select_body_receiver_geometry` takes the
  delay spread; the CIR's reach (`postcursor = (ff ≥ 32 ? 31 : 19) − precursor` in
  `make_cir_layout`) follows the window's tap count. Change them together.
- **Keep the CIR engine behind its gate.** New CIR-engine state and entry points go
  inside `#if M110_BODY_RECEIVER_CIR_ENGINE`, so the embedded receiver does not grow;
  `tests/equalizer_tests.cpp` checks the receiver's size with the engine in and out.
- **The two engines' trade is measured, not argued.** On the standard's two-path channels
  CIR-MMSE does better at 2400 bps and the adaptive engine at 600 bps and below. Re-measure
  both before changing the default.

---

*Companion to `modem/m110a/synchronization-and-acquisition-explainer.md`, the stage before
this one (it produces the `BodyAcquisition` and the delay spread this module uses). The
demapper this module feeds is in `modem/m110a/demapper-and-deinterleaver-explainer.md`.*
