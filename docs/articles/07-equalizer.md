# The equalizer

*Part 6 of "Building a MIL-STD-188-110 HF Modem in Modern C++." Start with the [overview](01-introduction.md), Part 1, [the rules I build by](02-embedded-cpp-philosophy.md), Part 2, [the transmit chain](03-transmit-chain.md), Part 3, [the audio front end](04-audio-front-end.md), Part 4, [acquisition](05-acquisition.md), or Part 5, [the first path](06-first-path.md). The code for this article is on GitHub: [kg7hf/modem-core](https://github.com/kg7hf/modem-core/tree/part-6).*

Part 5 ended with an anchor on the first path, a carrier estimate, the mode and the body boundary. From those, the position of every symbol of the body is known. The symbols still cannot be read. On Table XX's channels each received symbol is the transmitted one plus a copy of a symbol sent 4.8 or 12 symbols earlier, and both copies fade on their own. Read at the anchor, a symbol is two symbols added together, with weights that change several times a second.

Undoing that is the equalizer's job. It has help: every frame of data is followed by known symbols, the probes, so the receiver can keep checking its work against symbols it does not have to guess. There are two ways to use them. One is to follow the channel: keep a filter that undoes it, and nudge the filter's taps every time a known symbol shows how far off it is. The other is to measure the channel: estimate from each probe what the channel did to it, and solve for the filter that undoes that. This article builds both, and measures them against each other on the same recordings.

Part 5's last paragraph promised a third way, the whole-burst turbo equalizer. It needs the decoder, which is Part 7's subject, so it moves there.

Part 5 had `first_path_demo`; this article has `equalizer_demo`. It runs on the same recordings, so a build that fetched them for Part 5 has them already:

```bash
cmake -S . -B build -DMODEM_ARTICLE_DATA=ON
cmake --build build
./build/equalizer_demo
```

```text
== Equalizer walkthrough, on the article's recordings ==
```

| Step | Function | Section |
|---|---|---|
| 1 | `delay_spread()` | How far behind the anchor |
| 2 | `window()` | A window sized from the measurement |
| 3 | `training()` | Training on the preamble |
| 4 | `adaptive()` | Following the channel |
| 5 | `cir_mmse()` | Measuring the channel |
| 6 | `fade()` | When the channel moves |

As before, `tests/equalizer_walkthrough_tests.cpp` calls the same functions and checks what they returned.

## Since Part 5

- **The turbo equalizer moves to Part 7**, beside the decoder it runs with.
- **A first clang-tidy cleanup** of the modem reached the published files: constants from `std::numbers`, loop counters as wide as the values they count to, and no mutable global for the error handler. No output changed; the whole-burst audio test checks that.
- **Testing for contributors.** One command now builds and runs a named suite: `cmake -DSUITE=quick -P tools/test.cmake` runs the unit and golden tests, `article` adds the recordings' tests and the demos (fetching the exact release package if it is missing), and `full` runs everything the repository registers. A Dockerfile carries the compilers the CI uses, GCC 14 and Clang 19, for anyone who would rather not install them. The recordings' check is itself tested: a damaged, incomplete or padded package must be refused. Every pull request runs the quick suite with both compilers and the article suite in Docker. `docs/testing.md` has the commands. This article's tests join those suites.

## How far behind the anchor

The equalizer needs to know how long the channel is before it can size itself. Part 5 searched the lags ahead of the anchor for an earlier path. `estimate_body_preamble_delay_spread()` runs the same kind of search behind it. It correlates the known prefix at lags 0 to 15 symbols, in every preamble segment from the acquired one to the body. It adds the powers across segments, so a path that fades through one segment still counts. Then it subtracts the prefix's own sidelobes; the same idea as Part 5's model, built here from the symbols that follow the prefix instead of the ones before it.

What is left is a power for each lag. Lags 0 to 2 belong to the main path: a path that arrives between two symbol times splits its power over neighbouring lags, so those three are never counted as an echo. Echoes cast sidelobes too, so before the scan the estimator removes those as well (more on that below). Then, from lag 3 on, the delay spread is the largest lag whose power reaches an eighth of the strongest (about 9 dB down). If none does, the answer is 0, one clean path.

```cpp
static void cancel_echo_sidelobes(std::array<float, lag_table_size>& lag_powers, const std::array<float, lag_table_size>& sidelobe_powers, std::size_t maximum_delay_symbols,
                                  float significant_power) noexcept
{
    std::array<bool, lag_table_size> sidelobes_removed{};

    for (std::size_t pass = main_path_lobe_lags; pass <= maximum_delay_symbols; ++pass)
    {
        std::size_t echo_lag = 0U;
        float echo_power = 0.0F;

        for (std::size_t lag = main_path_lobe_lags; lag <= maximum_delay_symbols; ++lag)
        {
            if (!sidelobes_removed[lag] && lag_powers[lag] > echo_power)
            {
                echo_power = lag_powers[lag];
                echo_lag = lag;
            }
        }

        if (echo_lag == 0U || echo_power < significant_power)
        {
            return;
        }

        sidelobes_removed[echo_lag] = true;

        for (std::size_t lag = echo_lag + 1U; lag <= maximum_delay_symbols; ++lag)
        {
            lag_powers[lag] = std::max(0.0F, lag_powers[lag] - (sidelobe_powers[lag - echo_lag] / sidelobe_powers[0]) * echo_power);
        }
    }
}

// ... and in estimate_body_preamble_delay_spread(), after the main path's sidelobes are gone:
    const float significant_power = significant_path_fraction * strongest_power;
    cancel_echo_sidelobes(lag_powers, sidelobe_powers, maximum_delay_symbols, significant_power);

    for (std::size_t lag = maximum_delay_symbols; lag >= main_path_lobe_lags; --lag)
    {
        if (lag_powers[lag] >= significant_power)
        {
            delay_spread = lag;
            break;
        }
    }
```

That rule is the code's second, and changing to it uncovered a third piece. Until recently the estimator took the strongest echo, not the last one, and checked it against the same bar, while every comment around it described the largest. Nothing caught it, because on the standard's channels the two rules agree: a two-path channel has one echo, and it is both the strongest and the last. A third path tells them apart. Put a strong echo at 5 symbols and a weaker but real one at 12, and the old rule says 5, keeps the short window, and leaves the path at 12 outside it. The window has to hold every path that matters, so the rule became the largest.

Switching to it exposed a case the old rule never reached. The prefix repeats every 8 symbols, which is why the main path's sidelobes appear 8 and 9 lags behind it. An echo is a copy of the same prefix, so it casts the same sidelobes 8 and 9 lags behind itself: an echo at 5 symbols shows up again, weakly, at 13 and 14. The strongest-echo rule never considered a lag beyond the strongest echo. The largest-lag rule does, and on a burst whose direct path had faded, an echo's sidelobe sometimes cleared the bar and read as a far path. So the estimator now cancels each echo's sidelobes the way it cancels the main path's, strongest echo first, before it looks for the largest lag.

Cancelling takes the sidelobe's share off a lag, not the lag itself, so a real path sitting exactly where an echo's sidelobe falls still counts. It has to clear the bar by the sidelobe's share, though. For a path 9 symbols behind a strong echo, that means about 1 dB more than the bar, and a path closer to the bar than that is lost behind the echo. That is a limit of the method, and a test records it as one.

The tests are worth a look for how they are built. Each three-path case also checks the lag profile, to prove the channel actually separates the rules it is meant to separate: that the 5-symbol echo really is the stronger lag, and that the old code really did read the sidelobe at lag 14 at 7.4 percent of the echo. A test separates two rules only if its channel makes them give different answers. The cases are in `tests/delay_spread_tests.cpp`, from "A weaker echo at 12 symbols behind a strong one at 5 is the delay spread, not the strongest echo" to "A real path nine symbols behind the echo is the cancellation's limit".

```text
1. How far behind the anchor
   lag power behind the anchor, as a fraction of the strongest (lags 0 to 15; . under 0.05):
   fp1-echo-group  1.00    .    .    .    .    . 0.06    .    .    .    . 0.05 0.63    .    .    .
                   delay spread 12 symbols, over 8 preamble segments
   fp2-window      1.00 0.06    .    . 0.08    .    .    .    .    .    .    .    .    .    .    .
                   delay spread 0 symbols, over 21 preamble segments
   clean           1.00    .    .    .    .    .    .    .    .    .    .    .    .    .    .    .
                   delay spread 0 symbols, over 23 preamble segments
```

Each row is one burst's profile: the power at each lag behind the anchor, after the subtractions, as a fraction of the strongest. On fp1, the 300 bps recording through the 5 ms channel, the echo stands at lag 12 with 0.63 of the main path's power, and that is the delay spread. What is left at lags 6 and 11 is a few percent, well under the bar of an eighth. The clean burst has one path and reads 0. fp2, through the 2 ms channel, also reads 0, and the next paragraph is why.

The estimate only looks behind the anchor. On a burst anchored on the echo it reads 0, because no path follows the echo; the direct path is ahead of it, where this search does not look. That is one more reason the anchor goes on the first path.

## A window sized from the measurement

The equalizer is a window of taps on the received samples, half a symbol apart, plus taps on symbols it has already decided. `select_body_receiver_geometry()` picks one of two shapes from the measured spread:

```cpp
[[nodiscard]] constexpr BodyReceiverGeometry select_body_receiver_geometry(std::size_t delay_spread_symbols) noexcept
{
    if (delay_spread_symbols > body_receiver_default_delay_symbols)
    {
        return {32U, 6U, 14U};
    }

    return {body_receiver_feedforward_taps, body_receiver_feedforward_taps / 2U, body_receiver_feedback_taps};
}
```

The default holds a 2 ms channel: 24 taps with the reference tap at 12, so 6 symbols ahead of it and 5½ behind, and 8 taps on past decisions. A 2 ms echo is 4.8 symbols late, inside that reach. A spread longer than 6 symbols takes the long shape: 32 taps with the reference tap at 6, so 3 symbols ahead and 12½ behind, and 14 taps on past decisions, enough to reach a 5 ms echo at 12 symbols. Thirty-two feed-forward taps is as large as `ProbeDfe` goes.

The long shape is lopsided on purpose. With the anchor on the first path, the energy that matters arrives after it, so the window spends its taps behind the reference and few ahead. The feedback taps do the rest: an echo of a symbol already decided can be rebuilt from that decision and subtracted. The choice comes from the measurement, never from the mode: a 300 bps burst on a short channel gets the short window. The estimator can only answer 0 or 3 to 15, so in practice 0 to 6 means the default and 7 to 15 the long shape.

One rate is different. At 75 bps the receiver keeps the window but sets the feedback taps to zero, because it has nothing reliable to feed back. Feedback cancels the trailing echoes of symbols already decided, one symbol at a time, so it needs a sound decision for each symbol as it passes. At 75 bps each pair of bits is spread over a 32-symbol sequence, and the receiver only knows which sequence was sent after it has correlated all 32. A guess at one symbol on its own is made about 15 dB below the group's signal level, the gain of spreading over 32, and at the signal levels 75 bps exists for it would be wrong more often than right. Fed back, those guesses would add errors instead of removing echoes. So the feed-forward taps alone shape the channel, and the group correlation does the deciding.

```text
2. A window sized from the measurement
   fp1-echo-group  spread 12 -> 32 taps, reference tap 6: 3.0 symbols ahead, 12.5 behind, 14 feedback; trained again on it
   fp2-window      spread  0 -> 24 taps, reference tap 12: 6.0 symbols ahead, 5.5 behind, 8 feedback
   clean           spread  0 -> 24 taps, reference tap 12: 6.0 symbols ahead, 5.5 behind, 8 feedback
```

fp1 gets the long window and is trained a second time on it; the other two keep the default. The demo prints the window's reach the way the receiver uses it: with the reference tap at 6 of 32 half-symbol taps, the window covers 3 symbols ahead of the anchor and 12.5 behind, and the 14 feedback taps reach past the 12-symbol echo.

## Training on the preamble

Once the countdown is read, the whole preamble is known: every symbol the transmitter sent before the body. `train_preamble()` uses it to train the equalizer before the first data symbol arrives, in three steps.

First, a one-tap guess. Over the 288-symbol prefix it measures the channel as a single complex gain, the received symbols against the known ones, and puts that gain on the reference tap. Every other tap starts at zero.

Second, practice. It runs the whole known preamble through the equalizer three times, adapting every tap against the true symbol each time, while a carrier loop takes out the frequency error acquisition left. Each pass starts over at the beginning of the preamble, with the carrier loop and the past decisions cleared, but the taps keep what the last pass taught them. The 75 bps short-interleave preamble is shorter, so it gets eight passes. The equalizer's step size depends on the rate: 0.40 for 600 bps and up, which must converge quickly, and 0.10 for 75, 150 and 300 bps, whose 5 ms, 5 Hz channel punishes a step that large with gradient noise. The score is the mean squared error over the last 96 symbols of the last pass.

Third, a check on the frequency. Acquisition's frequency estimate is good on a clean channel and can be biased on a fading one. So the score decides. If the first training's error is already at the floor (one percent of the signal power), training is done. If not, the receiver makes up to two more guesses: a fresh estimate from the whole preamble, kept only if it differs from acquisition's by 1.5 to 30 Hz, and a 21-point search across ±30 Hz in 3 Hz steps that fits a channel to the preamble's last 48 symbols at each frequency, where the end points are never allowed to win. Each guess gets a complete training of its own, and the lowest error wins; a tie goes to acquisition. A new estimate never replaces the old one without that measurement, because on a fading channel the estimates themselves fade. Step 3 of the demo shows it: on fp1 the whole-preamble estimate comes out at +21 Hz, where acquisition said +2.55 and the search found -0.51. Taken on trust, it would have trained the equalizer twenty hertz off; scored, it has the worst error of the three and is thrown out.

Training is also why the order above is not quite the order the code runs. The spread estimate correlates blocks of 32 symbols coherently, so it needs a good frequency, and the good frequency comes out of training. Both receivers therefore train on the default window first, measure the spread with the frequency that training chose, and train again, from scratch, only if the measurement asks for the long window. A 2 ms burst trains once; a 5 ms burst trains twice.

```text
3. Training on the preamble
   fp1-echo-group: acquisition's frequency +2.55 Hz
   its error stayed above the floor, so the receiver trained every distinct guess:
   acquisition's estimate       +2.55 Hz   error on the preamble's tail 0.623
   a new guess                 +21.13 Hz   error on the preamble's tail 0.741
   a new guess                  -0.51 Hz   error on the preamble's tail 0.622   <- kept
   (the new guesses: the whole-preamble estimate and the channel-fit search, each when it differs)
```

On fp1, the error after training on acquisition's frequency stayed far above the floor, so the receiver trained two more guesses. The whole-preamble estimate, +21 Hz, did worse. The search's -0.51 Hz did better by a thousandth and was kept; had it only tied, acquisition's estimate would have stood. On the clean burst the first training reaches the floor and the race never starts (the walkthrough tests check both).

## Following the channel

`ProbeDfe` is the adaptive equalizer. Each output is the window's taps applied to the received samples, minus the feedback taps applied to the symbols already decided:

```cpp
IQSample ProbeDfe::output(IQSampleSpan feedforward_window) const noexcept
{
    IQSample value{};

    for (std::size_t tap = 0U; tap < mFeedforwardTaps; ++tap)
    {
        value += mFeedforwardCoefficients[tap] * feedforward_window[tap];
    }

    for (std::size_t tap = 0U; tap < mFeedbackTaps; ++tap)
    {
        value -= mFeedbackCoefficients[tap] * mDecisionHistory[tap];
    }

    return value;
}
```

The body comes in frames: a run of data symbols, then a probe of known ones. At 150 to 1200 bps a frame is 20 data symbols and a 20-symbol probe; at 2400 bps it is 32 and 16. (75 bps has no probes at all; the last section explained why it runs on the feed-forward taps alone.) `equalize_frame()` walks one frame at a time, and it handles the two kinds of symbol differently.

On a probe the right answer is known, so the error is exact: the known symbol minus the output. The taps move a small step against it. The update is normalized LMS: the step is divided by the energy in the window and in the feedback register, so a loud signal and a quiet one adapt at the same rate. Every probe symbol trains, whatever its error.

```cpp
IQSample ProbeDfe::train_nlms(IQSampleSpan feedforward_window, IQSample desired) noexcept
{
    const auto value = output(feedforward_window);
    const auto error = desired - value;
    float energy{};

    for (std::size_t tap = 0U; tap < mFeedforwardTaps; ++tap)
    {
        energy += std::norm(feedforward_window[tap]);
    }

    for (std::size_t tap = 0U; tap < mFeedbackTaps; ++tap)
    {
        energy += std::norm(mDecisionHistory[tap]);
    }

    const auto normalized_step = mStepSize / std::max(energy, 1.0e-6F);
    const auto retention = 1.0F - mLeakage;

    for (std::size_t tap = 0U; tap < mFeedforwardTaps; ++tap)
    {
        mFeedforwardCoefficients[tap] = retention * mFeedforwardCoefficients[tap] + normalized_step * error * std::conj(feedforward_window[tap]);
    }

    for (std::size_t tap = 0U; tap < mFeedbackTaps; ++tap)
    {
        mFeedbackCoefficients[tap] = retention * mFeedbackCoefficients[tap] - normalized_step * error * std::conj(mDecisionHistory[tap]);
    }

    commit_decision(desired);
    return value;
```

On data the receiver has no right answer, only its own decision: the output rounded to the nearest 8-PSK point. It trains on that decision only when the decision passes two tests. The output must carry real energy, at least a quarter of a symbol's, and it must sit within one neighbour's distance of the point it was rounded to. A decision that fails either test does not move the taps or steer the carrier loop; the carrier simply coasts for that symbol. It still goes into the feedback register, though, because the next symbols' echoes of it have to be cancelled with something, and the decision is the best guess there is.

```cpp
    const auto process_data_symbol = [&]() noexcept
    {
        make_equalizer_window(received_samples, received_index, samples_per_symbol, mCarrierTracker.phase_radians(), mCarrierTracker.frequency_radians_per_symbol(), mEqualizer, window);
        const auto output = mEqualizer.filter(std::span<const IQSample>{window}.first(mEqualizer.feedforward_taps()));
        const auto decision = psk8_symbol(psk8_hard_demapper(output));
        // Decision-directed adaptation between probes, gated on decision
        // reliability: the output must carry real energy and its residual must
        // stay inside the distance to the nearest wrong 8-PSK decision, so a
        // noise-only or likely-wrong decision can neither train the taps nor
        // steer the carrier loop. The rejected path still commits the decision
        // to the feedback register and advances the carrier phase open-loop.
        constexpr float squared_adjacent_decision_distance = 0.58578644F;
        const bool decision_reliable = std::norm(output) >= 0.25F && std::norm(decision - output) <= squared_adjacent_decision_distance;

        if (decision_reliable)
        {
            mEqualizer.adapt(std::span<const IQSample>{window}.first(mEqualizer.feedforward_taps()), decision);
            std::ignore = mCarrierTracker.update(output * std::polar(1.0F, mCarrierTracker.phase_radians()), decision);
        }
        else
        {
            mEqualizer.commit_decision(decision);
            mCarrierTracker.advance();
        }

        equalized_symbols[received_index++] = output;
    };
```

One limit: between probes the taps move only on the receiver's own decisions, so a fade that makes a few decisions wrong can move the taps the wrong way, and the gate catches only the worst of them. The next probe measures the damage. If a probe's mean error comes back above half the signal power and more than twice the block's usual level, or above 0.9 of the signal power whatever the usual level, the equalizer has lost the channel. It clears its feedback register and replays that probe four more times before the next frame, so it starts the frame from a filter that fits the channel again.

One more step happens on the way out, and Part 7 depends on it. Each probe measures how noisy the equalizer's output is at that moment. The receiver scales every data symbol by the inverse of that noise, interpolated between the probe before the frame and the probe after it, so a symbol sliding into a fade comes out smaller than one in a strong stretch. The scale does not change a hard decision. It changes the soft decoder's input: a smaller symbol reads as a less certain one.

The equalizer adapts with normalized LMS. `ProbeDfe` also carries a second method, recursive least squares, which converges in about twice as many symbols as it has taps, where normalized LMS takes many tens. The receiver never selects it. Its state is still compiled into every `ProbeDfe`, about 18 KiB for the matrix alone, which an embedded build pays for without using; gating it out the way the CIR engine is gated is a change for a later version.

```text
4. Following the channel
   adaptive on fp1-echo-group: 951 of 5760 data symbols decided wrong before the decoder (16.5%); 36 of 288 frames without one
   the first 24 frames (wrong data symbols / probe error):
    1/0.73  4/1.12  2/0.72  2/0.69  3/0.82  3/0.69  3/0.63  3/0.57
    1/0.83  6/0.72  4/0.79  3/0.71  3/0.50  4/0.53  1/0.79  2/0.74
    3/0.59  1/0.33  1/0.38  0/0.31  0/0.47  1/0.80  6/0.88 10/1.10
```

Each pair is one frame: how many of its 20 data symbols the equalizer decided wrong, and the mean squared error on the probe that followed. A wrong decision here means a different point of the mode's own alphabet was nearer; at 300 bps that alphabet is two points, so the demo does not count a near miss among eight. Across the whole body the adaptive equalizer decides about one data symbol in six wrong. The probes tend to rise with the errors around them: the frame with 10 wrong decisions has a probe error of 1.10. These are decisions before the decoder; Part 7's decoder works from them.

## Measuring the channel

The other way starts from a count. On the long window the adaptive equalizer has 46 taps to set, 32 feed-forward and 14 feedback, and it learns them a step at a time. The channel itself is shorter. The CIR-MMSE engine models it as 32 taps at half-symbol spacing on the long window, 20 on the default one, and splits them into two sets, one for each half-symbol phase, so each set is an ordinary symbol-spaced channel of 16 or 10 taps. The split also keeps the arithmetic simple: after the receive filter, the noise in each phase is uncorrelated from one symbol to the next, so each set is a plain least-squares problem.

A 20-symbol probe gives each set 18 or 19 equations. That is plenty for 10 unknowns and only just enough for 16, so the estimate leans on two more sources. The equations reach back into the data symbols just before the probe, using the receiver's own decisions there, and each equation counts half as much for every doubtful decision it uses. The solve is also pulled toward the previous frame's channel, more strongly when the noise is high (ridge regression, with the old estimate as the prior).

```cpp
// cir_polyphase_solve(): the normal equations, weighted by the rows' reliability,
// pulled toward the prior (lambda) and kept invertible (epsilon).
    for (std::size_t index = 0U; index < dimension; ++index)
    {
        rhs[index] = lambda * prior[index];
    }

    for (std::size_t row = 0U; row < problem.rows; ++row)
    {
        const auto weight = problem.weights[row];
        const auto& regressor = problem.regressors[row];

        for (std::size_t i = 0U; i < dimension; ++i)
        {
            const auto weighted = weight * std::conj(regressor[i]);

            for (std::size_t j = i; j < dimension; ++j)
            {
                matrix[i * dimension + j] += weighted * regressor[j];
            }

            rhs[i] += weighted * problem.observations[row];
        }
    }

    for (std::size_t i = 0U; i < dimension; ++i)
    {
        matrix[i * dimension + i] += IQSample{lambda + epsilon, 0.0F};

        for (std::size_t j = 0U; j < i; ++j)
        {
            matrix[i * dimension + j] = std::conj(matrix[j * dimension + i]);
        }
    }

    if (!cholesky_solve(matrix, rhs, dimension, 0.0F))
    {
        return {StatusCode::internal_error, "CIR polyphase system lost positive definiteness"};
    }
```

From the estimated channel the receiver solves for the filter instead of adapting one: the feed-forward and feedback taps that give the least mean squared error for that channel. The feedback taps cancel the echoes of the last few decided symbols exactly. Everything else, including any echo longer than the feedback span, is treated as interference, together with the noise as the receive filter shapes it. That is one small linear system per probe, solved by Cholesky factorization. It needs a small constant added to its diagonal: about a third of the half-symbol band carries neither signal nor modelled noise, so without that constant the system is nearly singular and the taps from one probe to the next disagree wildly.

```cpp
// cir_mmse_dfe_solve(), one attempt of its retry ladder:
        // Interference: every lag the feedback register cannot cancel. The
        // cursor lag stays (MMSE); lags 1..feedback_taps are excluded because
        // the feedback taps cancel them exactly; the postcursor tail beyond
        // the register (present when the echo outruns the feedback span)
        // remains as interference.
        for (auto lag = lag_minimum; lag <= lag_maximum; ++lag)
        {
            if (lag >= 1 && lag <= static_cast<std::ptrdiff_t>(geometry.feedback_taps))
            {
                continue;
            }

            for (std::size_t row = 0U; row < taps; ++row)
            {
                const auto column_row = channel_column(lag, row);

                for (std::size_t column = row; column < taps; ++column)
                {
                    matrix[row * taps + column] += column_row * std::conj(channel_column(lag, column));
                }
            }
        }

        float trace{};

        for (std::size_t row = 0U; row < taps; ++row)
        {
            for (std::size_t column = row; column < taps; ++column)
            {
                matrix[row * taps + column] += IQSample{noise_variance_effective * raised_cosine_autocorrelation_half_lag(column - row), 0.0F};
            }

            trace += matrix[row * taps + row].real();
        }

        const auto jitter = jitter_scale * trace / static_cast<float>(taps);

        for (std::size_t row = 0U; row < taps; ++row)
        {
            matrix[row * taps + row] += IQSample{jitter, 0.0F};

            for (std::size_t column = 0U; column < row; ++column)
            {
                matrix[row * taps + column] = std::conj(matrix[column * taps + row]);
            }
        }

        for (std::size_t row = 0U; row < taps; ++row)
        {
            rhs[row] = channel_column(0, row);
        }

        const auto pivot_floor = pivot_floor_fraction * trace / static_cast<float>(taps);

        if (cholesky_solve(matrix, rhs, taps, pivot_floor))
        {
            // ... the feed-forward taps are the solution, conjugated; the feedback taps follow from them
        }
```

Between two probes the receiver interpolates the taps linearly, so the probe before a frame and the probe after it both shape every symbol in it. Linear interpolation cannot follow everything, though. A fade can turn the channel's phase sharply inside one frame, so the engine also runs a small, leaky adaptive correction on top of the interpolated taps, the same normalized LMS as `ProbeDfe`, gated the same way, with a leak that drains any bad update within about 20 symbols. So the engine that measures the channel also follows it a little, and the measurement resets the base at every probe.

Two more pieces deal with the decisions going wrong. Each frame solves two candidate channels: one from all the equations, decisions included, and one from the equations that use probe symbols only. The first tracks better while the decisions are right; once they go wrong, the least squares can fit the wrong decisions consistently and the estimate drifts away with them. The second has no decisions in it, so it can only lag. Both run the frame, and the one with the smaller error on the probe wins. If both fail, the frame is an outage. The receiver then collects probe-only equations across consecutive outage frames until there are enough of them, and solves again with no prior at all.

There is no per-symbol carrier loop here. Each frame is derotated by a straight phase ramp, and the frequency is nudged once per frame from how far the channel estimate rotated since the last probe. The engine also picks its own starting point: at the end of the preamble it searches the frequency and the channel's alignment around the anchor together and keeps the pair that fits best, so it can start the body on a slightly different frequency than the adaptive equalizer's training chose. That is deliberate. The adaptive equalizer tracks the frequency symbol by symbol and can live with a small error in its starting point; a fixed frequency error here rotates each probe's channel estimate against the last one, and interpolating between two rotated estimates gives taps that fit neither.

Two limits. The channel model reaches only a few symbols ahead of the anchor, so CIR-MMSE needs the anchor on the first path. In the measurements for this article, with the anchor left on the echo of a 2 ms channel, it lost every one of those bursts. And 75 bps has no probes, so at that rate the receiver uses the adaptive equalizer whichever engine was asked for.

One difference is not about the models. Both engines scale their outputs by the measured noise for the soft decoder, but the adaptive equalizer interpolates that weight symbol by symbol across the frame, while CIR-MMSE uses one weight for the whole frame. Any comparison after the decoder includes that difference too.

The cost is memory and arithmetic: about 14 KiB of working state per receiver, and two least-squares solves and two filter solves per probe. Host builds compile it in. Embedded builds leave it out with one build option, so the adaptive equalizer is the only one there.

```text
5. Measuring the channel
   CIR-MMSE on fp1-echo-group: 1286 of 5760 data symbols decided wrong before the decoder (22.3%); 53 of 288 frames without one
   the first 24 frames (wrong data symbols / probe error):
    0/0.34  0/0.61  2/0.42  1/0.51  1/0.44  1/0.38  1/0.35  0/0.37
    0/0.65  0/0.49  0/0.37  0/0.38  0/0.26  0/0.20  0/0.30  0/0.39
    0/0.27  0/0.23  1/0.19  0/0.29  0/0.48  3/0.70  5/0.71  7/1.35
```

On the same body, CIR-MMSE starts out cleaner: most of its first 20 frames have no wrong decision at all. Over the whole body it does worse, 22.3% against 16.5%, although it has more frames without a single error, 53 against 36. Its errors come in bunches.

## When the channel moves

```text
6. When the channel moves
   fp1-echo-group: where each engine trails the other most (wrong data symbols / probe error)
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

The bunches are fades. In frames 96, 97 and 127, CIR-MMSE decides 10 to 13 of 20 symbols wrong and its probe error is above 1, more than the signal's own power: both of its channel estimates failed the probe, the frame was an outage, and the engine had to collect probe-only equations over the following frames before it could solve again. The adaptive equalizer went through the same frames with no wrong decision, its taps moving a little at every symbol. The other three rows run the other way. In frames 117, 174 and 212 the adaptive equalizer fell behind for a few symbols, and CIR-MMSE, which solves afresh from each probe, did not.

Which of those patterns wins depends on the channel and the rate, and that takes more than one recording to measure.

## What it measured

I measured the two equalizers on the six long-interleave rows of Table 10.1, each through pathsim on its row's channel at its row's signal-to-noise ratio, 500 bursts a row. Every burst was rendered once and then decoded by every arm of the test, so each comparison is between two receivers on the same bursts, not on two runs of the channel's luck.

All of these numbers come from the host's whole-burst receiver, the one the modem's test tools drive. The streaming receiver, the path an embedded build runs, is Part 8's subject, and I measure it there.

Each equalizer ran three ways. Alone means the code in this article: equalize, demap, decode, nothing fed back. With the decoder means the receiver also takes each decoded block, encodes it again, and re-equalizes the block trained on those symbols; that pass needs the decoder, so it belongs to Part 7, but it shows how much of the gap better decisions close. The turbo receiver is Part 7's whole-burst decoder, for scale.

| Row | Channel | SNR | Adaptive alone | CIR-MMSE alone | Adaptive with the decoder | CIR-MMSE with the decoder | Turbo receiver |
|---|---|---|---|---|---|---|---|
| 2400 bps | 2 ms, 1 Hz | 18 dB | 310 | 454 | 496 | 500 | 500 |
| 1200 bps | 2 ms, 1 Hz | 11 dB | 471 | 470 | 500 | 500 | 500 |
| 600 bps | 2 ms, 1 Hz | 7 dB | 488 | 399 | 500 | 483 | 500 |
| 300 bps | 5 ms, 5 Hz | 7 dB | 346 | 157 | 498 | 455 | 500 |
| 150 bps | 5 ms, 5 Hz | 5 dB | 499 | 293 | 500 | 467 | 500 |
| 75 bps | 5 ms, 5 Hz | 2 dB | 498 | 498 | 499 | 499 | 499 |

*Bursts of 500 decoded exactly. At 75 bps every arm runs the adaptive equalizer, since that rate has no probes.*

Read alone, the two equalizers trade places with the data rate. At 2400 bps measuring the channel wins: on the bursts where the two disagreed, CIR-MMSE decoded 157 that the adaptive equalizer did not, and the adaptive equalizer 13 that CIR-MMSE did not. At 1200 bps they tie, 17 against 18. Below that, following the channel wins: 90 against 1 at 600 bps, 217 against 28 at 300 bps, and 206 against none at 150 bps. A sign test on the bursts where the two disagree puts every one of those rows far beyond chance except 1200 bps, which is a tie.

In the overview I wrote that measuring the channel and solving the filter is the stronger move. On these channels that holds at 2400 bps and not below it, and the 5 ms, 5 Hz rows, where I expected the model to win, are where it loses by the most.

I have not measured why, and I will not guess here. One difference is not about the models at all, though. Both engines scale their outputs by the measured noise before the decoder, but the adaptive equalizer interpolates that weight symbol by symbol across a frame, and CIR-MMSE uses one weight per frame. A comparison after the decoder includes that difference along with the equalizers themselves, and on the 5 Hz rows, where the noise changes fastest inside a frame, it can matter most.

With the decoder's help, the order changes again. The adaptive equalizer then decodes all but 4 bursts across the table, every one of the 4 at 2400 bps, while CIR-MMSE still loses bursts at 600, 300 and 150 bps. The turbo receiver decodes everything but one burst at 75 bps. Part 7 builds both.

You can run a smaller version of this comparison yourself. `equalizer_compare` sends bursts of random bytes through the test bench's two-path channel on each long-interleave row, one seed per burst, and equalizes each burst twice:

```bash
./build/equalizer_compare
```

```text
== The two equalizers on the same bursts: 20 a row, the bench's two-path channel ==
   data decisions wrong before the decoder, in the mode's own alphabet

   row     channel      SNR   acquired   adaptive   CIR-MMSE   adaptive better   CIR-MMSE better   same
   2400L   2 ms 1 Hz    18 dB    20 of 20    30.71%     20.22%                 2                18      0
   1200L   2 ms 1 Hz    11 dB    20 of 20    13.90%     14.31%                 3                17      0
   600L    2 ms 1 Hz     7 dB    20 of 20     5.65%      8.82%                20                 0      0
   300L    5 ms 5 Hz     7 dB    20 of 20    19.53%     22.90%                19                 1      0
   150L    5 ms 5 Hz     5 dB    20 of 20    21.41%     26.95%                20                 0      0
```

Its numbers are decisions before the decoder, through an engineering model of the channel rather than the calibrated simulator, so they are not the table above. The shape is the same: CIR-MMSE decides better at 2400 bps, the two are close at 1200 bps (CIR-MMSE wins most bursts there but loses a few badly), and the adaptive equalizer decides better at 600 bps and below. Whether each burst's message comes out needs the decoder; Part 7 grows this program into that measurement.

The anchor matters more to one of them. CIR-MMSE cannot describe a path that arrives more than a few symbols before its anchor. On the 2400 and 1200 bps rows, 226 of the 500 bursts were acquired on the echo; with the anchor left there, CIR-MMSE lost all 226 on both rows, even with the decoder's help. The adaptive equalizer, anchored on the echo of a 2 ms channel, decodes anyway: the direct path is 4.8 symbols ahead, inside its window. Moving its anchor onto the direct path did not help it; alone at 2400 bps it then decoded 252 bursts instead of 310. That is Part 5's window rule seen from the other side: the adaptive equalizer's anchor moves only for a path 7 or more symbols early.

## Testing, part three: the genie

How good is good? A count of decoded bursts does not say whether an equalizer is close to what the channel allows or far from it. A genie answers that. It is the same receiver, given the true transmitted symbols to train on at every symbol instead of its own decisions. No real receiver can do that, since the transmitted symbols are what it is trying to find, but a test can, because the test sent them. The genie sets a bound: the gap between it and the real equalizer is how much better decisions would be worth, and a burst the genie still loses was lost to the channel, not to the receiver.

| Row | Adaptive alone | Adaptive genie | CIR-MMSE alone | CIR-MMSE genie |
|---|---|---|---|---|
| 2400 bps | 310 | 500 | 454 | 500 |
| 1200 bps | 471 | 500 | 470 | 500 |
| 600 bps | 488 | 500 | 399 | 486 |
| 300 bps | 346 | 498 | 157 | 493 |
| 150 bps | 499 | 500 | 293 | 490 |
| 75 bps | 498 | 499 | 498 | 499 |

*Bursts of 500 decoded exactly, on the same bursts as the table above.*

Neither equalizer alone reaches its genie. The adaptive equalizer's gap is 190 bursts at 2400 bps and 152 at 300 bps; CIR-MMSE's is 336 at 300 bps and 197 at 150 bps. Almost all of those bursts fail because of the equalizer's own decisions, not because the channel destroyed them. The 2 bursts at 300 bps and the 1 at 75 bps that the adaptive genie also loses are the channel's.

The two genies also differ. With true symbols the adaptive equalizer decodes every burst on the first five rows except those 2 at 300 bps. CIR-MMSE with true symbols still loses 14 bursts at 600 bps, 7 at 300 bps and 10 at 150 bps that the adaptive genie decodes (and decodes 2 at 300 bps that the adaptive genie loses). On those bursts better decisions alone would not have rescued it; the shortfall is in the model or the way it tracks the channel.

Better decisions are exactly what the decoder supplies, and the first table shows it: with the decoder's help, the adaptive equalizer comes within 4 bursts of its genie, all at 2400 bps. Part 7 builds that decoder.

One measure pointed the wrong way. Alone at 300 bps, CIR-MMSE made slightly fewer symbol errors before the decoder than the adaptive equalizer, and decoded fewer than half as many bursts. A symbol error rate counts every wrong symbol the same; the decoder does not, and what reaches the user is the message. So the tables count bursts decoded exactly, and the bit errors behind them, after the decoder. The campaign's tool counted those errors the blunt way, against all eight 8-PSK points and over every symbol, probes included. At 300 bps that inflates both engines' counts alike, so the comparison stands, but it is also why the demo counts a decision wrong only against the points the mode can send.

About the test itself: every arm decoded the same bursts, so a difference is a difference in the receivers; the disagreements are counted burst by burst and tested for chance; and every result names the revision, the seeds and the command that produced it. The campaign itself ran on my private test tools, but `equalizer_compare` keeps the same design in the public repository: the same bursts for both engines, one seed each, counted pair by pair, so you can run that part of the test yourself.

## The tests

Three suites cover this article's code.

`tests/delay_spread_tests.cpp` builds its channels from the transmitter and checks the estimate and the window it picks: one clean path, a 2 ms and a 5 ms echo, the weaker far echo behind a strong near one, an echo's own sidelobes, a real path 8 symbols behind an echo that survives the cancellation, and one 9 behind that shows the cancellation's limit. Its three-path cases also check the lag profile, so each proves its channel tells the rules apart.

`tests/equalizer_tests.cpp` checks the pieces one at a time on synthesized signals: the one-tap channel estimate and the plain LMS equalizer, the decision feedback cancelling a one-symbol echo, the CIR solvers recovering a known channel and the MMSE-DFE taps they give, training on a two-path channel, supervised re-equalization, the receiver's lifecycle (configured, trained, rearmed, reset), its probe residual through a gap in the signal, the carrier loop's clamp, the frequency race skipped on a clean channel and run on a poor one, resumable training giving the batch training's receiver bit for bit, and which engines a build can select.

`tests/equalizer_walkthrough_tests.cpp` runs the walkthrough's steps and checks what the article says about them: on the clean burst a spread of 0, the default window, no frequency race, and both engines deciding every data symbol right; on the recordings, fp1's spread of 12 and its second training on the long window, fp2's echo anchor reading 0, the race keeping a better guess than acquisition's, the adaptive equalizer ahead of CIR-MMSE through fp1's body, and frames where each engine trails the other. A build without the CIR-MMSE engine skips the cases that need it.

The first two need no recordings and run in the quick suite. The walkthrough's recording cases carry the `first-path` label with Part 5's, so the article suite runs them:

```bash
ctest --test-dir build -R "delay_spread|equalizer" --output-on-failure
ctest --test-dir build -L first-path --output-on-failure
```

## Run it on your own

The runner gains a Part 6 line: the delay spread it measured, the window it chose, the frequency training kept, and what each engine made of the body. On a recording it cannot count wrong decisions, because it does not know what was sent, so it reports each engine's mean probe error instead:

```bash
./build/runner --wav test-data/first-path-v1/fp1-echo-group.wav
```

```text
== Part 6  the equalizer: delay spread 12 symbols -> the long window, 32 taps, reference tap 6, 14 feedback; trained again on it
           training: acquisition's +2.6 Hz stayed above the floor, so the receiver raced 3 guesses and kept -0.5 Hz
           adaptive: mean probe error 0.682 over 1 block; counting wrong decisions needs the transmitted symbols
           CIR-MMSE: mean probe error 0.744 over 1 block; counting wrong decisions needs the transmitted symbols
== next: soft demapping and the decoder (Part 7)
```

On a message of your own the runner builds the burst itself, so it knows every symbol and counts the decisions:

```bash
./build/runner "Hi" --snr 10 --carrier 37
```

```text
== Part 6  the equalizer: delay spread 0 symbols -> the default window, 24 taps, reference tap 12, 8 feedback
           training: acquisition's +37.7 Hz stayed above the floor, but no other guess differed from it, so it kept +37.7 Hz
           adaptive: 0 of 5760 data symbols decided wrong before the decoder (0.0%)
           CIR-MMSE: 0 of 5760 data symbols decided wrong before the decoder (0.0%)
== next: soft demapping and the decoder (Part 7)
```

It still stops before the decoder; that is next week.

## Go read the code, and the standard

1. In MIL-STD-188-110B 5.3.2.2.2, find the frame of unknown data symbols and known probe symbols for each data rate, and check them against `body_block_plan()` in `body_waveform.hpp`. What fraction of the symbols is known at 2400 bps, and at 600 bps?
2. `estimate_body_preamble_delay_spread()` never counts lags 0 to 2 as an echo. A 1 ms echo arrives 2.4 symbols late: over which lags does its power fall, and what could the estimate report for it?
3. `cancel_echo_sidelobes()` removes the strongest echo's sidelobes first, not the nearest echo's. What could go wrong if a weaker echo were handled before a stronger one?
4. Work out the long window's reach from 32 taps half a symbol apart with the reference tap at 6. Why does `select_body_receiver_geometry()` switch above 6 symbols rather than 5, when a 2 ms echo is 4.8 symbols late?
5. The receiver trains on the default window before it measures the delay spread. How long is each coherent block in the estimate, and why does that make acquisition's frequency the wrong one to measure with?
6. `inner_trainer_begin()` seeds one tap from the prefix and then replays the preamble three times. Why does each replay clear the decision history and the carrier loop but keep the taps? Why does the 75 bps short preamble get eight passes?
7. The frequency race keeps the whole-preamble estimate only if it differs from acquisition's by more than 0.004 and less than 0.08 radians per symbol. What are those limits in hertz at 2400 Bd, and why is there a lower one?
8. In `equalize_frame_impl()`, a data decision trains the taps only if the output's power is at least 0.25 and its squared distance to the decision is at most 0.5858. Where does 0.5858 come from? Why does a rejected decision still go into the feedback register?
9. The adaptive walk scales each data output by the inverse of the probe error around it. Why does that change no hard decision, and what does it give Part 7's soft decoder?
10. `cir_polyphase_solve()` works on two problems, one for each half-symbol phase. Using `raised_cosine_autocorrelation_half_lag()`, show why the noise is uncorrelated from symbol to symbol within one phase.
11. `cir_mmse_dfe_solve()` adds 2% of the mean diagonal before it solves. What happens to consecutive frames' taps without it, and why does that matter to the interpolation between probes?
12. Run `equalizer_compare --seeds 100`. Does the 1200 bps row stay close? What does it mean when one engine decides better on most bursts but has nearly the same error rate overall?

## Next

Next week: soft demapping, the decoder, and the turbo loop. The equalizer hands over symbols and its own decisions about them. The decoder works better with less: not a decision, but how sure the receiver is of each bit. Part 7 turns symbols into those probabilities, decodes them, and then feeds the decoder's answers back to the equalizer, which is the turbo equalizer Part 5 promised.
