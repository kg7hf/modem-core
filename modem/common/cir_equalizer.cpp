// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

/*
================================================================================
cir_equalizer.cpp - the receiver's MODEL-BASED channel equalizer
================================================================================

WHAT THIS FILE IS
-----------------
The MODEL-BASED equalizer: instead of ADAPTING a filter (equalizer.cpp), it
MEASURES the channel impulse response (CIR) from the known probe symbols by
regularized least squares, then SOLVES the optimal finite-length MMSE
decision-feedback equalizer directly from that estimate. The data-symbol filter is
computed, not adapted.

  INPUT  : polyphase rows of known/decided symbols vs. observations (CirPolyphase
           Problem); a solved T/2 CIR + geometry (CirMmseGeometry); a noise variance.
  OUTPUT : the T/2 CIR taps; the MMSE-DFE feed-forward + feedback taps.
  DEPENDS: types.hpp (IQSample), status.hpp. No heap - fixed std::array scratch.

WHY MODEL-BASED
---------------
A ~20-symbol probe cannot converge a ~46-tap adaptive equalizer, but it OVER-
determines the ~13-tap T-spaced channel. Estimate the channel (well-posed), then
solve the equalizer (a linear-algebra step). Whether that beats adapting depends on
the channel and the rate; Part 6 of the article series measures both.

BUILD GATING (host vs embedded)
-------------------------------
These solvers are compiled into EVERY target, but nothing reaches them on an
embedded build: the engine that calls them, the receiver's runtime-selectable
cir_mmse (model-based) body equalizer, is host-only, gated by the
M110_BODY_RECEIVER_CIR_ENGINE build option (default ON for native, OFF for
embedded, which drops the ~14 KiB workspace). See receiver.hpp.

WHAT LIVES HERE
---------------
  cir_cholesky_solve()                 Hermitian positive-definite linear solve.
  make_cir_layout() / cir_tap_index()  T/2 CIR support + even/odd polyphase split.
  cir_polyphase_solve()                weighted RIDGE least squares -> one CIR
                                       sub-channel (leans on a prior + a ridge floor).
  cir_mmse_dfe_solve()                 SOLVE the MMSE-DFE from the CIR: build
                                       R = signal + coloured-noise + jitter, solve
                                       R*conj(w) = h0, derive the feedback taps.
  raised_cosine_autocorrelation_half_lag()  the coloured-noise covariance of the
                                       band-limited T/2 stream (RRC*RRC, rolloff 0.25).

CONVENTIONS (must match ProbeDfe::output exactly)
-------------------------------------------------
  z = sum_t w[t]*window[t] - sum_j b[j]*d[k-j]      (plain products, w stored conjugated)
  cursor lag kept (MMSE, not zero-forcing); lags 1..fb cancelled by feedback; lags
  beyond fb remain as modelled interference. The 2% diagonal jitter is LOAD-BEARING
  (signal and band-limited noise share the out-of-band nullspace) - see the inline
  note in cir_mmse_dfe_solve.

WORKED EXAMPLE / DEEP DIVE
--------------------------
    modem/m110a/receiver-core-equalizer-explainer.md
================================================================================
*/

#include "modem/common/cir_equalizer.hpp"

#include <cmath>
#include <complex>
#include <cstddef>
#include <numbers>
#include <numeric>

namespace modem::common
{

namespace
{

constexpr float shaping_rolloff = 0.25F;
constexpr float pivot_floor_fraction = 1.0e-7F; // Cholesky pivot floor as a fraction of the mean diagonal (trace / taps)

// In-place lower Cholesky factorization and solve of the Hermitian system
// held row-major in matrix (dimension x dimension active); the solution
// overwrites rhs. Returns false when a pivot collapses, so the caller can
// raise the jitter and rebuild.
// -----------------------------------------------------------------------------
// cholesky_solve  (Hermitian positive-definite linear solve, in place)
// -----------------------------------------------------------------------------
// 50K view: Solve A x = b for a Hermitian PD matrix A; return false if A is not
//   positive-definite so the caller can regularize and retry.
// Detailed view: Lower Cholesky A = L*L^H, then forward + back substitution. Each
//   diagonal pivot is guarded by pivot_floor. The public cir_cholesky_solve wraps
//   this.
// 5th-grade view: The standard fast, stable way to solve a symmetric set of
//   equations, with a tripwire if they are secretly unsolvable.
// -----------------------------------------------------------------------------
[[nodiscard]] bool cholesky_solve(std::span<IQSample> matrix, std::span<IQSample> rhs, std::size_t dimension, float pivot_floor) noexcept
{
    for (std::size_t k = 0U; k < dimension; ++k)
    {
        auto pivot = matrix[k * dimension + k].real();

        for (std::size_t m = 0U; m < k; ++m)
        {
            pivot -= std::norm(matrix[k * dimension + m]);
        }

        if (!(pivot > pivot_floor))
        {
            return false;
        }

        const auto diagonal = rmath::sqrt(pivot);
        matrix[k * dimension + k] = IQSample{diagonal, 0.0F};
        const auto inverse_diagonal = 1.0F / diagonal;

        for (std::size_t row = k + 1U; row < dimension; ++row)
        {
            auto value = matrix[row * dimension + k];

            for (std::size_t m = 0U; m < k; ++m)
            {
                value -= matrix[row * dimension + m] * std::conj(matrix[k * dimension + m]);
            }

            matrix[row * dimension + k] = value * inverse_diagonal;
        }
    }

    for (std::size_t row = 0U; row < dimension; ++row)
    {
        auto value = rhs[row];

        for (std::size_t m = 0U; m < row; ++m)
        {
            value -= matrix[row * dimension + m] * rhs[m];
        }

        rhs[row] = value / matrix[row * dimension + row].real();
    }

    for (std::size_t back = dimension; back > 0U; --back)
    {
        const auto row = back - 1U;
        auto value = rhs[row];

        for (std::size_t m = row + 1U; m < dimension; ++m)
        {
            value -= std::conj(matrix[m * dimension + row]) * rhs[m];
        }

        rhs[row] = value / matrix[row * dimension + row].real();
    }

    return true;
}

} // namespace

// -----------------------------------------------------------------------------
// make_cir_layout / cir_tap_index
// -----------------------------------------------------------------------------
// 50K view: Define how one T/2 channel impulse response is laid out (precursor/
//   postcursor around the cursor) and split into two T-spaced polyphase sub-
//   channels; index between polyphase order and the assembled CIR.
// Detailed view: postcursor = (ff>=32 ? 31 : 19) - precursor, so the long geometry
//   reaches a 5 ms echo (24 half-symbols) + shaping tails. The even/odd split lets
//   each polyphase problem see WHITE noise (Nyquist: RRC autocorr = 0 at nonzero
//   even lags). Shared identically by the preamble fit and the block walk.
// 5th-grade view: Decide how long a "map of the canyon echoes" to keep, and how to
//   read its odd and even samples.
// -----------------------------------------------------------------------------
CirLayout make_cir_layout(std::size_t feedforward_taps, std::size_t precursor) noexcept
{
    CirLayout layout{};
    layout.precursor = precursor;
    // The support length tracks the geometry - the long-delay window exists
    // to reach a 5 ms echo (24 half-symbols) plus shaping tails - while the
    // precursor/postcursor split is an alignment decision: the acquisition's
    // timing lock can place the direct path early enough that a fixed short
    // precursor silently unmodels it, so the preamble seed searches the
    // split and the block walk honors the measured winner.
    layout.postcursor = (feedforward_taps >= 32U ? 31U : 19U) - precursor;
    layout.support = layout.precursor + layout.postcursor + 1U;
    layout.even_first_lag = -static_cast<std::ptrdiff_t>(layout.precursor / 2U);
    layout.odd_first_lag = layout.even_first_lag + 1;
    layout.even_taps = layout.precursor / 2U + (layout.postcursor - 1U) / 2U + 1U;
    layout.odd_taps = (layout.postcursor + 1U) / 2U + static_cast<std::size_t>(-layout.odd_first_lag) + 1U;
    return layout;
}

std::size_t cir_tap_index(const CirLayout& layout, bool even, std::size_t lag_index) noexcept
{
    const auto lag = (even ? layout.even_first_lag : layout.odd_first_lag) + static_cast<std::ptrdiff_t>(lag_index);
    const auto q = 2 * lag - (even ? 0 : 1) + static_cast<std::ptrdiff_t>(layout.precursor);
    return static_cast<std::size_t>(q);
}

Status cir_cholesky_solve(std::span<IQSample> matrix, std::span<IQSample> rhs, std::size_t dimension, float pivot_floor) noexcept
{
    return cholesky_solve(matrix, rhs, dimension, pivot_floor) ? Status::success() : Status{StatusCode::unavailable, "cholesky solve: matrix is not positive definite"};
}

// -----------------------------------------------------------------------------
// raised_cosine_autocorrelation_half_lag
// -----------------------------------------------------------------------------
// 50K view: The autocorrelation of the receive matched filter (RRC*RRC, rolloff
//   0.25) at a half-symbol lag - the SHAPE of the noise correlation in the T/2
//   stream.
// Detailed view: 1 at lag 0, exactly 0 at nonzero EVEN lags (Nyquist), the raised-
//   cosine value at ODD lags. The T/2 stream is band-limited, so its noise is
//   COLOURED; cir_mmse_dfe_solve needs this covariance to be correct.
// 5th-grade view: How much the wind-noise at one half-beat looks like the
//   wind-noise a half-beat later.
// -----------------------------------------------------------------------------
float raised_cosine_autocorrelation_half_lag(std::size_t half_symbol_lag) noexcept
{
    if (half_symbol_lag == 0U)
    {
        return 1.0F;
    }

    if (half_symbol_lag % 2U == 0U)
    {
        return 0.0F;
    }

    const auto t = 0.5F * static_cast<float>(half_symbol_lag);
    constexpr float pi = std::numbers::pi_v<float>;
    const auto sinc = rmath::sin(pi * t) / (pi * t);
    const auto denominator = 1.0F - 4.0F * shaping_rolloff * shaping_rolloff * t * t;

    constexpr float singularity_threshold = 1.0e-5F;
    if (std::fabs(denominator) < singularity_threshold)
    {
        // Raised-cosine singular point t = 1/(2*rolloff): the limit is
        // (pi/4)*sinc(t), which the odd-lag caller never actually hits at
        // rolloff 0.25 (t = 2 is an even lag), but guard anyway.
        return 0.25F * pi * sinc;
    }

    return sinc * rmath::cos(pi * shaping_rolloff * t) / denominator;
}

// -----------------------------------------------------------------------------
// cir_polyphase_solve  (weighted ridge least squares -> one CIR sub-channel)
// -----------------------------------------------------------------------------
// 50K view: Estimate one polyphase sub-channel of the CIR, leaning on a prior (the
//   preamble estimate) and a small ridge floor.
// Detailed view: minimize sum_s w_s|x_s - r_s^T g|^2 + lambda|g - prior|^2 +
//   epsilon|g|^2. Builds the weighted Hermitian normal equations, Cholesky-solves,
//   returns taps + post-solve weighted residual energy (a fit-quality number).
//   lambda pulls toward the prior (temporal continuity across anchors); epsilon
//   guarantees invertibility.
// 5th-grade view: Fit the canyon map to what you measured, but don't stray far from
//   the map you already had, and never let the fit go crazy.
// -----------------------------------------------------------------------------
Status cir_polyphase_solve(const CirPolyphaseProblem& problem, float lambda, float epsilon, std::span<const IQSample> prior, CirSolveScratch& scratch, std::span<IQSample> taps,
                           float& residual_energy) noexcept
{
    const auto dimension = problem.taps;

    if (dimension == 0U || dimension > cir_polyphase_taps || problem.rows > cir_maximum_rows || taps.size() < dimension || prior.size() < dimension || lambda < 0.0F || epsilon <= 0.0F)
    {
        return {StatusCode::invalid_argument, "CIR polyphase problem is invalid"};
    }

    auto matrix = std::span<IQSample>{scratch.matrix.data(), dimension * dimension};
    auto rhs = std::span<IQSample>{scratch.vector.data(), dimension};

    for (std::size_t index = 0U; index < dimension * dimension; ++index)
    {
        matrix[index] = IQSample{};
    }

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

    for (std::size_t index = 0U; index < dimension; ++index)
    {
        taps[index] = rhs[index];
    }

    residual_energy = 0.0F;

    for (std::size_t row = 0U; row < problem.rows; ++row)
    {
        const auto& regressor = problem.regressors[row];
        const IQSample predicted = std::inner_product(regressor.begin(), regressor.begin() + static_cast<std::ptrdiff_t>(dimension), taps.begin(), IQSample{});

        residual_energy += problem.weights[row] * std::norm(problem.observations[row] - predicted);
    }

    return Status::success();
}

// -----------------------------------------------------------------------------
// cir_mmse_dfe_solve  (SOLVE the finite-length MMSE-DFE from the CIR)
// -----------------------------------------------------------------------------
// 50K view: Given the measured T/2 CIR, compute the optimal feed-forward + feedback
//   taps in one solve (with a regularization retry ladder).
// Detailed view: R = sum_{lag not in [1..fb]} h_lag h_lag^H + N0*Gamma + jitter*I,
//   where h_lag[t] = c[t - cursor + 2*lag + precursor], Gamma is the RRC noise
//   autocorrelation. Cursor lag 0 kept (MMSE, not zero-forcing); lags 1..fb dropped
//   (feedback cancels them); lags beyond fb stay as interference. Solve R*conj(w)=h0
//   -> feedforward; feedback[j] = sum_t w[t]*h_{j+1}[t]. The 2% diagonal jitter is
//   LOAD-BEARING (signal + band-limited noise share the out-of-band nullspace);
//   retry up to 4x with 10x jitter on a pivot collapse.
// Noise term: noise_variance_effective is the caller's N0 scalar; it sets how much
//   the solve trusts the channel vs. the noise (the MMSE/zero-forcing balance).
// 5th-grade view: From the canyon map, calculate the single best ear-setup and
//   echo-memory in one shot, with a stabilizer so the math never divides by ~zero.
// -----------------------------------------------------------------------------
Status cir_mmse_dfe_solve(std::span<const IQSample> cir, const CirMmseGeometry& geometry, float noise_variance_effective, CirSolveScratch& scratch, std::span<IQSample> feedforward,
                          std::span<IQSample> feedback) noexcept
{
    const auto taps = geometry.feedforward_taps;
    const auto support = geometry.precursor_taps + geometry.postcursor_taps + 1U;

    if (taps == 0U || taps > cir_maximum_taps || geometry.cursor >= taps || support > cir_maximum_taps || cir.size() < support || feedforward.size() < taps ||
        feedback.size() < geometry.feedback_taps || noise_variance_effective <= 0.0F)
    {
        return {StatusCode::invalid_argument, "CIR MMSE geometry is invalid"};
    }

    // Channel column at symbol lag j on the window: h_j[t] = c[t - cursor + 2j].
    const auto channel_column = [&](std::ptrdiff_t lag, std::size_t tap) noexcept -> IQSample
    {
        const auto q = static_cast<std::ptrdiff_t>(tap) - static_cast<std::ptrdiff_t>(geometry.cursor) + 2 * lag + static_cast<std::ptrdiff_t>(geometry.precursor_taps);

        if (q < 0 || q >= static_cast<std::ptrdiff_t>(support))
        {
            return {};
        }

        return cir[static_cast<std::size_t>(q)];
    };

    const auto lag_minimum = -((static_cast<std::ptrdiff_t>(geometry.precursor_taps) + static_cast<std::ptrdiff_t>(taps - 1U - geometry.cursor) + 1) / 2);
    const auto lag_maximum = (static_cast<std::ptrdiff_t>(geometry.postcursor_taps) + static_cast<std::ptrdiff_t>(geometry.cursor)) / 2;

    auto matrix = std::span<IQSample>{scratch.matrix.data(), taps * taps};
    auto rhs = std::span<IQSample>{scratch.vector.data(), taps};
    // The white-noise floor is load-bearing, not defensive: both the signal
    // term and the band-limited noise model share the same out-of-band
    // nullspace, so without it the feedforward solution's nullspace
    // component is set by estimation noise divided by an arbitrarily small
    // eigenvalue - measured as consecutive anchors' taps differing by 100-200%
    // over a nearly static channel, which destroys the tap interpolation the
    // data symbols rely on. Two percent of the mean diagonal pins that
    // component consistently near zero at a ~0.1 dB in-band MSE cost. The
    // retry ladder still rebuilds with ten times the floor on a pivot
    // collapse, bounded attempts.
    float jitter_scale = 2.0e-2F;

    for (std::size_t attempt = 0U; attempt < 4U; ++attempt)
    {
        for (std::size_t index = 0U; index < taps * taps; ++index)
        {
            matrix[index] = IQSample{};
        }

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
            // The code convention computes z = sum w[t]*window[t] with plain
            // products; writing u = conj(w) makes that u^H window, so the
            // Wiener solution u = R^{-1} h_0 is stored conjugated.
            for (std::size_t tap = 0U; tap < taps; ++tap)
            {
                feedforward[tap] = std::conj(rhs[tap]);
            }

            for (std::size_t index = 0U; index < geometry.feedback_taps; ++index)
            {
                IQSample value{};

                for (std::size_t tap = 0U; tap < taps; ++tap)
                {
                    value += feedforward[tap] * channel_column(static_cast<std::ptrdiff_t>(index) + 1, tap);
                }

                // The joint output subtracts the feedback sum, so the tap is
                // stored as the postcursor response itself, no sign flip.
                feedback[index] = value;
            }

            return Status::success();
        }

        jitter_scale *= 10.0F;
    }

    return {StatusCode::internal_error, "CIR MMSE system lost positive definiteness"};
}

} // namespace modem::common
