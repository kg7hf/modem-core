// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

// =============================================================================
// constants.hpp - the shared scalar constants (pi / two_pi), once.
// =============================================================================
// The DSP and family files each carry a local `constexpr float pi` / `two_pi`
// (~24 copies, several spellings). This is the single definition they collapse to.
// It is deliberately its OWN header rather than part of real.hpp: real.hpp is
// included everywhere, and modem::common::pi / two_pi would then redefine every
// existing file-scope copy in the same namespace. A file adopts this constant when
// it drops its local copy (byte-exact: every spelling in the tree rounds to the
// same float), not merely by including real.hpp.
// =============================================================================

#include "modem/common/real.hpp"

#include <bit>
#include <cstdint>
#include <numbers>

namespace modem::common
{

inline constexpr Real pi = std::numbers::pi_v<Real>;
inline constexpr Real two_pi = Real{2} * pi;

// Bit-identical to every spelling used in the tree: 2.0F * pi, 6.2831853071795864769F and
// 6.28318530717958647692F all round to this float, so a site swapping its local copy for
// this constant is byte-exact.
static_assert(std::bit_cast<std::uint32_t>(pi) == 0x40490FDBU);     // the float nearest pi
static_assert(std::bit_cast<std::uint32_t>(two_pi) == 0x40C90FDBU); // exact exponent bump of pi

} // namespace modem::common
