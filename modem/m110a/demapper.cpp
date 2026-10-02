// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

/*
================================================================================
demapper.cpp - 8-PSK symbol mapping and soft/hard demapping
================================================================================

WHAT THIS FILE IS
-----------------
The bridge from equalized complex SYMBOLS to BIT EVIDENCE. It maps tribits to the
8-PSK constellation (transmit) and demaps a received symbol back into soft bits
(receive). Soft = a log-likelihood ratio per bit (+ favors 0, - favors 1, magnitude
= confidence), which the FEC decoder needs instead of hard 0/1 guesses.

  INPUT  : one equalized symbol z (+ gain + noise variance for the a-priori demapper).
  OUTPUT : per-bit soft LLRs in source-bit order.
  DEPENDS: scrambler.hpp (modified_gray_decode - the body bit->tribit sub-mapping).

WHAT LIVES HERE
---------------
  psk8_symbol()                tribit -> constellation point e^{j*tribit*pi/4} (table).
  psk8_hard_demapper()         nearest point (hard decision, e.g. for feedback).
  psk8_soft_demapper()         max-log LLRs, no prior (min-distance form).
  psk8_soft_demapper_apriori() the TURBO demapper: folds in decoder a-priori LLRs and
                               returns EXTRINSIC LLRs, EXCLUDING each bit's own prior.
  In this repository psk8_symbol is implemented; the three demappers are declared in
  the header, and their bodies arrive with the receiver series.

SIGN CONVENTION (load-bearing)
------------------------------
  Positive soft value favors bit 0, negative favors bit 1. Both demappers agree by
  construction (one_cost - zero_cost / zero_best - one_best). The decoder assumes it.

DOWNSTREAM
----------
  These soft LLRs go to the deinterleaver (encoder order) and then the FEC decoder.

WORKED EXAMPLE / DEEP DIVE
--------------------------
    modem/m110a/demapper-and-deinterleaver-explainer.md
================================================================================
*/

#include "modem/m110a/demapper.hpp"

#include "modem/common/bits.hpp"
#include "modem/common/constants.hpp"
#include "modem/common/convert.hpp"
#include "modem/m110a/scrambler.hpp"

#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>

#include "modem/common/status.hpp"
namespace modem::m110a
{

using ::modem::common::as_real;
using ::modem::common::IQSample;
using ::modem::common::low_bits;
using ::modem::common::Status;
using ::modem::common::StatusCode;
using ::modem::common::two_pi;

constexpr float pi_over_four = 0.78539816339744830962F;

IQSample psk8_symbol(std::uint8_t tribit) noexcept
{
    // The 8-PSK constellation has only 8 possible outputs. Precompute them once
    // so the per-symbol hot path (demap derotation, reference generation) is a
    // table read, not a software sinf/cosf on cores without hardware trig (the
    // Cortex-M33). The table is filled with the same std::polar expression, so
    // every value is bit-identical to the previous per-call computation - the
    // decode is unchanged; only the M33 cost drops.
    static const std::array<IQSample, 8U> constellation = []() noexcept
    {
        std::array<IQSample, 8U> table{};

        for (std::uint8_t index = 0U; index < 8U; ++index)
        {
            table[index] = std::polar(1.0F, as_real(index) * pi_over_four);
        }

        return table;
    }();

    return constellation[tribit & 7U];
}

} // namespace modem::m110a
