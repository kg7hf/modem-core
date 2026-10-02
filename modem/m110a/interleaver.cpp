// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

/*
================================================================================
interleaver.cpp - MIL-STD-188-110B body matrix interleaver
================================================================================

WHAT THIS FILE IS
-----------------
Reorders coded bits so a channel FADE (which corrupts a contiguous run of received
symbols) becomes SCATTERED coded-bit errors after unscrambling - which convolutional
codes tolerate, unlike long bursts. Transmit INTERLEAVES; receive DEINTERLEAVES. The
soft variants move LLRs in both directions (to the decoder in encoder order, and back
in channel order); they only reorder the LLRs and never rescale them.
In this repository body_interleave is implemented; the deinterleavers and the soft
variants are declared in the header, and their bodies arrive with the receiver series.

  INPUT  : a bit or soft-LLR vector + an InterleaverSpec (matrix dims / increments).
  OUTPUT : the permuted vector (interleave) or its inverse (deinterleave).
  DEPENDS: waveform.hpp (InterleaverSpec). Caller owns the scratch matrix; no heap.

THE BODY MATRIX INTERLEAVER
---------------------------
A rows x columns matrix. LOAD (transmit scatter) writes down columns stepping rows by
load_row_increment; FETCH (transmit gather) reads across rows offsetting columns by
fetch_column_decrement. The staggered read is what spreads a burst.
  interleave   = scatter by load,  gather by fetch
  deinterleave = scatter by fetch, gather by load   (EXACT inverse; soft path too)

WORKED EXAMPLE / DEEP DIVE
--------------------------
    modem/m110a/demapper-and-deinterleaver-explainer.md
================================================================================
*/

#include "modem/m110a/interleaver.hpp"

#include <cstddef>
#include <cstdint>
#include <numeric>
#include <span>

#include "modem/common/status.hpp"
namespace modem::m110a
{

using ::modem::common::BitSpan;
using ::modem::common::InterleaverSpec;
using ::modem::common::MutableBitSpan;
using ::modem::common::Result;
using ::modem::common::Status;
using ::modem::common::StatusCode;

// -----------------------------------------------------------------------------
// body_load_address / body_fetch_address  (the MIL-STD-188-110B matrix permutation)
// -----------------------------------------------------------------------------
// 50K view: The two halves of the body interleaver over a rows x columns matrix.
// Detailed view: LOAD (transmit scatter) writes down columns stepping the row by
//   load_row_increment: row = (i%rows * load_row_increment)%rows, col = i/rows.
//   FETCH (transmit gather) reads across rows offsetting the column by
//   fetch_column_decrement: row = i%rows, col = (i/rows - row*fetch_column_decrement)
//   mod columns. address = row*columns + col. interleave = scatter load / gather
//   fetch; deinterleave swaps them (exact inverse). bypass returns the index.
// 5th-grade view: Fill the grid down the columns in one hop pattern, read it back
//   across the rows in a different hop pattern; the mismatch shuffles a burst apart.
// -----------------------------------------------------------------------------
Result<std::size_t> body_load_address(std::size_t input_index, const InterleaverSpec& spec) noexcept
{
    if (spec.bypass)
    {
        return input_index;
    }

    if (spec.rows == 0U || spec.columns == 0U || input_index >= spec.size_bits)
    {
        return Status{StatusCode::invalid_argument, "invalid body load address request"};
    }

    const auto within_column = input_index % spec.rows;
    const auto column = input_index / spec.rows;
    const auto row = (within_column * spec.load_row_increment) % spec.rows;
    return row * spec.columns + column;
}

// body_fetch_address: the FETCH half of the pair above (transmit gather / receive
// scatter). See the banner on body_load_address for the full matrix permutation.
Result<std::size_t> body_fetch_address(std::size_t output_index, const InterleaverSpec& spec) noexcept
{
    if (spec.bypass)
    {
        return output_index;
    }

    if (spec.rows == 0U || spec.columns == 0U || output_index >= spec.size_bits)
    {
        return Status{StatusCode::invalid_argument, "invalid body fetch address request"};
    }

    const auto row = output_index % spec.rows;
    const auto row_cycle = output_index / spec.rows;
    const auto column = (row_cycle + spec.columns - ((row * spec.fetch_column_decrement) % spec.columns)) % spec.columns;
    return row * spec.columns + column;
}

Status body_interleave(BitSpan input, MutableBitSpan scratch, MutableBitSpan output, const InterleaverSpec& spec) noexcept
{
    if (spec.bypass)
    {
        if (output.size() < input.size())
        {
            return {StatusCode::buffer_too_small, "body output too small"};
        }

        for (std::size_t index = 0U; index < input.size(); ++index)
        {
            output[index] = input[index];
        }

        return Status::success();
    }

    if (input.size() != spec.size_bits || scratch.size() < spec.size_bits || output.size() < spec.size_bits)
    {
        return {StatusCode::invalid_argument, "body interleaver buffers do not match matrix"};
    }

    for (std::size_t index = 0U; index < input.size(); ++index)
    {
        const auto address = body_load_address(index, spec);

        if (!address)
        {
            return address.error();
        }

        scratch[address.value()] = input[index];
    }

    for (std::size_t index = 0U; index < input.size(); ++index)
    {
        const auto address = body_fetch_address(index, spec);

        if (!address)
        {
            return address.error();
        }

        output[index] = scratch[address.value()];
    }

    return Status::success();
}

} // namespace modem::m110a
