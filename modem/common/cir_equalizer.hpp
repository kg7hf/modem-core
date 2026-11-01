// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/convert.hpp"
#include "modem/common/status.hpp"
#include "modem/common/types.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <numeric>

// =============================================================================
// cir_equalizer.hpp - model-based equalizer (estimate the CIR, SOLVE the MMSE-DFE)
// =============================================================================
// Measure the T/2 channel impulse response from probes by regularized least squares
// (two polyphase problems), then SOLVE the finite-length MMSE-DFE from it - the
// data-symbol filter is computed, not adapted: a short probe can pin down the
// channel's few taps where it cannot train the equalizer's many.
//
// Build gating: the solvers in this file compile in EVERY target, but the engine
// that calls them - the receiver's runtime-selectable cir_mmse (model-based) body
// equalizer - is host-only, gated by the
// M110_BODY_RECEIVER_CIR_ENGINE build option (default ON for the native/host
// target, OFF for embedded targets, which omit the ~14 KiB workspace). See
// receiver.hpp. On an embedded build these helpers are therefore reachable only
// through code that is itself compiled out.
//
// Teaching walkthrough: modem/m110a/receiver-core-equalizer-explainer.md
// The detailed design rationale continues in the block comment below.
// =============================================================================

namespace modem::common
{

// Channel-estimate equalization. A 20-symbol probe is short for training a
// 46-tap equalizer, but long enough, with the frame's decisions and the
// previous estimate to help, for the channel's 10 to 16 taps per half-symbol
// phase. These helpers estimate the
// composite T/2 channel impulse response from known symbols by regularized
// least squares (split into two independent T-spaced polyphase problems)
// and compute the finite-length MMSE decision-feedback equalizer from the
// estimate, so the data-symbol filter is solved, not adapted.

// Composite T/2 CIR support: taps c[q], q in [-Qpre, Qpost], relative to
// each symbol's cursor sample. Qpre covers the raised-cosine precursor
// tails; Qpost covers the delayed path plus its tails and is bounded by the
// feedforward geometry.
inline constexpr std::size_t cir_precursor_taps = 4U;
inline constexpr std::size_t cir_maximum_taps = 32U;
inline constexpr std::size_t cir_polyphase_taps = cir_maximum_taps / 2U;
// Rows per polyphase: the preamble seed span (48 symbols) bounds it.
inline constexpr std::size_t cir_maximum_rows = 48U;
inline constexpr std::size_t cir_preamble_seed_symbols = 48U;

// One polyphase weighted least-squares problem: observation x_s against a
// row of known (or decided) unit-modulus symbols, weighted by decision
// reliability. The caller owns row construction because it owns the
// framing, the scrambler walk, and the decision history.
template <std::size_t TapsPerPolyphase, std::size_t Rows> struct CirPolyphaseProblemT
{
    static constexpr std::size_t taps_capacity = TapsPerPolyphase;
    static constexpr std::size_t rows_capacity = Rows;
    std::array<IQSample, Rows> observations{};
    std::array<std::array<IQSample, TapsPerPolyphase>, Rows> regressors{};
    std::array<float, Rows> weights{};
    std::size_t rows{};
    std::size_t taps{};
};

// The geometry the receiver solves.
using CirPolyphaseProblem = CirPolyphaseProblemT<cir_polyphase_taps, cir_maximum_rows>;

// Scratch for the Hermitian solves (the largest is Nf x Nf complex).
struct CirSolveScratch
{
    std::array<IQSample, cir_maximum_taps * cir_maximum_taps> matrix{};
    std::array<IQSample, cir_maximum_taps> vector{};
};

// Polyphase lag layout for a feedforward geometry: CIR support
// [-precursor, postcursor] around the cursor sample, split into the two
// T-spaced sub-channels. Shared by the receiver's preamble channel-fit
// search and the cir_mmse block walk, so both index one T/2 CIR the same way
// (integer arithmetic only).
struct CirLayout
{
    std::ptrdiff_t even_first_lag{};
    std::ptrdiff_t odd_first_lag{};
    std::size_t even_taps{};
    std::size_t odd_taps{};
    std::size_t precursor{};
    std::size_t postcursor{};
    std::size_t support{};

    // The even sub-channel's advance in T/2 taps: half the precursor.
    [[nodiscard]] constexpr std::size_t even_advance() const noexcept { return precursor / 2U; }
};

[[nodiscard]] CirLayout make_cir_layout(std::size_t feedforward_taps, std::size_t precursor) noexcept;

// Shared CIR-fit scalars, so every site that fits a CIR uses one derivation
// instead of repeating the formula with a hand cast of the support.
//
// Residual degrees of freedom: rows minus the (exact) support count, floored at 6 so the
// noise estimate never divides by a tiny or negative number. retention defaults to 1 (the
// shipped sites); a caller that retains only part of the support scales it. Byte-exact with
// the shipped `std::max(rows - static_cast<float>(support), 6.0F)` at retention 1.
[[nodiscard]] constexpr float cir_residual_dof(float rows, std::size_t support, float retention = 1.0F) noexcept
{
    return std::max(rows - as_real(support) * retention, 6.0F);
}

// Noise inflation from finite support: 1 + support / denominator. The denominator is the
// caller's (e.g. 2 * mean_rows), so the operation order matches the shipped expression
// `1.0F + static_cast<float>(support) / denominator` exactly.
[[nodiscard]] constexpr float cir_noise_inflation(std::size_t support, float denominator) noexcept
{
    return 1.0F + as_real(support) / denominator;
}

// Scatter/gather between one polyphase tap vector (indexed by lag order)
// and the assembled T/2 CIR (indexed q + precursor).
[[nodiscard]] std::size_t cir_tap_index(const CirLayout& layout, bool even, std::size_t lag_index) noexcept;

// In-place lower Cholesky factorization and solve of the Hermitian system
// held row-major in matrix (dimension x dimension active); the solution
// overwrites rhs. Returns a not-positive-definite Status when a pivot collapses
// (not above pivot_floor), so the caller can raise its diagonal jitter and rebuild;
// Status, not a bare bool, so a failed solve rides the same error channel as the
// rest of the library. The polyphase and MMSE-DFE solvers below keep their own
// private call path.
[[nodiscard]] Status cir_cholesky_solve(std::span<IQSample> matrix, std::span<IQSample> rhs, std::size_t dimension, float pivot_floor) noexcept;

// Weighted ridge solve of one polyphase problem:
//   minimize sum_s w_s |x_s - r_s^T g|^2 + lambda |g - prior|^2 + epsilon |g|^2
// Writes the solved taps and the post-solve weighted residual energy.
[[nodiscard]] Status cir_polyphase_solve(const CirPolyphaseProblem& problem, float lambda, float epsilon, std::span<const IQSample> prior, CirSolveScratch& scratch, std::span<IQSample> taps,
                                         float& residual_energy) noexcept;

// Equalizer geometry plus channel support for the MMSE computation.
struct CirMmseGeometry
{
    std::size_t feedforward_taps{};
    std::size_t cursor{};
    std::size_t feedback_taps{};
    std::size_t precursor_taps{};
    std::size_t postcursor_taps{};
};

// Finite-length MMSE-DFE from the T/2 CIR estimate (c indexed q+precursor,
// length precursor+postcursor+1). Conventions match ProbeDfe::output
// exactly: z = sum_t w[t]*window[t] - sum_j b[j]*d[k-j] with plain complex
// products, window cursor-centered and derotated by the carrier ramp. The
// interference term keeps the cursor lag (MMSE, not zero-forcing) and every
// lag the feedback register cannot cancel; the noise term uses the banded
// raised-cosine autocorrelation of the matched-filtered noise, not white
// noise, because the T/2 stream is band-limited. Mandatory diagonal jitter:
// with 25% excess bandwidth roughly a third of the T/2 band carries no
// signal or modeled noise, so the matrix is structurally near-singular.
// noise_variance_effective is supplied by the caller (its N0 term scales the
// coloured-noise covariance and therefore the MMSE/zero-forcing balance).
[[nodiscard]] Status cir_mmse_dfe_solve(std::span<const IQSample> cir, const CirMmseGeometry& geometry, float noise_variance_effective, CirSolveScratch& scratch, std::span<IQSample> feedforward,
                                        std::span<IQSample> feedback) noexcept;

// Raised-cosine autocorrelation (RRC * RRC, rolloff 0.25) at a half-symbol
// lag; zero at every nonzero even lag (Nyquist), which is why each
// polyphase least-squares problem legitimately sees white noise.
[[nodiscard]] float raised_cosine_autocorrelation_half_lag(std::size_t half_symbol_lag) noexcept;

} // namespace modem::common
