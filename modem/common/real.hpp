// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

// =============================================================================
// real.hpp - the one scalar type, and precision-parametric scalar math.
// =============================================================================
// Factored out of types.hpp so the unit types (decibel/frequency/angle) can depend
// on Real and rmath without pulling in the full types.hpp. Putting a Decibel in a
// types.hpp struct otherwise creates an include cycle (decibel <- types <- decibel);
// this header is the shared root that breaks it.
// =============================================================================

#include <cmath>

namespace modem::common
{

// Real is the one scalar type the whole modem is parameterized on. It is single
// precision: float carries all the precision the sampled HF signal has, the
// M33/U545 has no hardware double, and double is a needless slowdown on the M7s.
// There is intentionally no double build. (A double "numerical oracle" was a goal
// inherited from another project; it was dropped because verification here is the
// goldens + reference decoder + one-type discipline, not a parallel precision.)
using Real = float;

// Precision-parametric scalar real math, kept as a template dispatch layer for one
// reason that outlives the float/double question: it is the SEAM for a future
// fixed-point Real. See the note below before deleting it.
//
// For float, the standard <cmath> overloads select float arguments and results,
// so each wrapper calls the ordinary overloaded spelling (std::sqrt, std::sin, ...).
// The suffixed C names are not guaranteed in namespace std. The -Wdouble-promotion
// build warning remains the backstop against accidentally selecting a double overload.
// (The complex helpers - std::polar / abs / arg /
// conj / norm - are already templates on the value type and parameterize for free,
// so only the scalar-real transcendentals live here.)
//
// The real payoff is the seam. rmath is the ONLY place the modem names sqrt/sin/...
// on a scalar, so a future fixed-point Real can be introduced without touching any
// of the ~20 call sites:
//
//   1. Make Real the fixed-point type, e.g.  using Real = Q15;  (a 16-bit or 32-bit
//      integer wrapper; Q15 = 1 sign + 15 fraction bits, Q31 similarly on int32).
//   2. Add a specialization  template <> struct real_ops<Q15> { ... };  whose sqrt/
//      sin/cos/atan2 call the fixed-point kernels instead of libm. There is no
//      std::sqrt for a fixed-point type, so this dispatch is not optional there -
//      it is the whole point.
//   3. On the M33/U545 those kernels are one CORDIC iteration each (the U545's
//      CORDIC accelerator): sin/cos/atan2/hypot map directly onto CORDIC rotation/vector
//      modes, and sqrt onto the CORDIC hyperbolic mode - single-cycle-class ops
//      with no libm and no FPU. exp/log/pow would use the hyperbolic mode or a
//      small polynomial; fabs/floor/ceil/round/fmin/fmax/fmod become integer ops.
//   4. rmath = real_ops<Real> then re-points the entire modem at the fixed-point
//      backend with no other edits, and the goldens re-run to quantify the accuracy
//      trade against the float build.
//
// Until that work is scheduled, only the float specialization exists.
template <class T> struct real_ops;

template <> struct real_ops<float>
{
    [[nodiscard]] static float sqrt(float x) noexcept { return std::sqrt(x); }
    [[nodiscard]] static float sin(float x) noexcept { return std::sin(x); }
    [[nodiscard]] static float cos(float x) noexcept { return std::cos(x); }
    [[nodiscard]] static float atan2(float y, float x) noexcept { return std::atan2(y, x); }
    [[nodiscard]] static float fabs(float x) noexcept { return std::fabs(x); }
    [[nodiscard]] static float exp(float x) noexcept { return std::exp(x); }
    [[nodiscard]] static float log(float x) noexcept { return std::log(x); }
    [[nodiscard]] static float log10(float x) noexcept { return std::log10(x); }
    [[nodiscard]] static float pow(float base, float exponent) noexcept { return std::pow(base, exponent); }
    [[nodiscard]] static float hypot(float x, float y) noexcept { return std::hypot(x, y); }
    [[nodiscard]] static float floor(float x) noexcept { return std::floor(x); }
    [[nodiscard]] static float ceil(float x) noexcept { return std::ceil(x); }
    [[nodiscard]] static float round(float x) noexcept { return std::round(x); }
    [[nodiscard]] static float fmin(float a, float b) noexcept { return std::fmin(a, b); }
    [[nodiscard]] static float fmax(float a, float b) noexcept { return std::fmax(a, b); }
    [[nodiscard]] static float fmod(float a, float b) noexcept { return std::fmod(a, b); }
};

// The modem's scalar-real math entry point: rmath::sqrt(x), rmath::sin(x), ...
using rmath = real_ops<Real>;

} // namespace modem::common
