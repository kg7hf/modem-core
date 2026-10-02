// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

/*
================================================================================
convolutional_k7.cpp - forward error correction: K=7 convolutional encoder + Viterbi decoders
================================================================================

WHAT THIS FILE IS
-----------------
The error-correction rulebook. The transmitter runs data through a K=7 rate-1/2
convolutional encoder (generators 133/171 octal), so the coded bits are correlated
in a known way; the receiver's Viterbi decoders exploit that structure to recover
the exact transmitted bits from a corrupted soft-bit stream. The decoders give
HARD output.

  INPUT  : (encode) data bits; (decode) soft LLRs, T1 before T2, + favors bit 0.
  OUTPUT : (encode) coded bits; (decode) recovered information bits.
  DEPENDS: types.hpp / status.hpp only.

THE CODE
--------
  K=7 => 64-state trellis. ConvolutionalEncoderK7::push shifts one input bit in and
  emits EncodedPair {t1,t2}. Generators 0x6D/0x4F are octal 133/171 with the newest
  input bit in bit 0.

WHAT LIVES HERE
---------------
  encode_rate_half / encode_repeated_pairs / encode_tail_biting_punctured_3_4
      the three transmit encodings (plain, repeat xN for low rates, punctured 3/4).
  viterbi_decode_rate_half (decode_with_end_state)
      whole-buffer maximum-likelihood decode: add-compare-select + traceback.
  StreamingViterbiK7
      continuous decode with a FIXED traceback window (bits emerge delayed by the
      window depth) - the block-by-block streaming decode mechanism.
  viterbi_decode_tail_biting_3_4
      depuncture, then try all 64 tail-biting start states, keep the best.
  In this repository the encoders are implemented; the three decoders are declared
  in the header, and their bodies arrive with the receiver series.

CONVENTIONS (shared with the demapper and interleaver)
------------------------------------------------------
  T1 before T2; positive soft metric favors bit 0; newest input bit in state bit 0;
  additive minimized cost. StreamingViterbiK7 keeps every one of these so it is
  bit-identical to the batch decoder on a merged trellis.

WORKED EXAMPLE
--------------
    modem/m110a/body-waveform-and-scrambler-explainer.md (the encoder inside one
    600 bps block)
================================================================================
*/

#include "modem/common/fec/convolutional_k7.hpp"

#include "modem/common/bits.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

namespace modem::common
{
// MIL-STD-188-110B K=7 generators, T1 first. The standard's octal
// polynomials 133 and 171 become 0x6D and 0x4F when the newest input bit is
// stored in bit 0, as ConvolutionalEncoderK7 does below.
constexpr std::uint8_t polynomial_t1 = 0x6DU;
constexpr std::uint8_t polynomial_t2 = 0x4FU;
// MIL-STD-188-110B Appendix C (C.5.3.2.3) 3/4-rate puncturing mask, applied
// to the serialized T1,T2 stream in that order.
constexpr std::array<std::uint8_t, 6> puncture_mask{1U, 1U, 1U, 0U, 0U, 1U};

static constexpr std::uint8_t parity(std::uint8_t value) noexcept
{
    return static_cast<std::uint8_t>(std::popcount(value) & 1);
}

// -----------------------------------------------------------------------------
// ConvolutionalEncoderK7::push  (the code core - one input bit -> two coded bits)
// -----------------------------------------------------------------------------
// 50K view: Shift one input bit into the 6-bit register and emit the rate-1/2 coded
//   pair (t1, t2).
// Detailed view: register = (state<<1 | bit) & 0x7F; state' = register & 0x3F;
//   t1 = parity(register & 0x6D), t2 = parity(register & 0x4F) - generators 133/171
//   octal (bit-reversed, newest input in bit 0). Each output is the XOR of the
//   register bits its polynomial selects.
// 5th-grade view: Slide the new digit into a little window and announce two check
//   sounds computed from the window.
// -----------------------------------------------------------------------------
EncodedPair ConvolutionalEncoderK7::push(std::uint8_t bit) noexcept
{
    const auto register_value = low_bits<7U>(shift_in_bit(mState, (bit & 1U) != 0U));
    mState = static_cast<std::uint8_t>(register_value & 0x3FU);
    return {parity(static_cast<std::uint8_t>(register_value & polynomial_t1)), parity(static_cast<std::uint8_t>(register_value & polynomial_t2))};
}

Result<std::size_t> encode_rate_half(BitSpan input, MutableBitSpan output, std::uint8_t initial_state) noexcept
{
    if (output.size() < input.size() * 2U)
    {
        return Status{StatusCode::buffer_too_small, "FEC output too small"};
    }

    ConvolutionalEncoderK7 encoder(initial_state);
    std::size_t written = 0U;

    for (const auto bit : input)
    {
        const auto pair = encoder.push(bit);
        output[written++] = pair.t1;
        output[written++] = pair.t2;
    }

    return written;
}

Result<std::size_t> encode_repeated_pairs(BitSpan input, MutableBitSpan output, std::uint8_t pair_repetitions, std::uint8_t initial_state) noexcept
{
    if (pair_repetitions == 0U || output.size() < input.size() * 2U * pair_repetitions)
    {
        return Status{StatusCode::buffer_too_small, "repeated FEC output too small"};
    }

    ConvolutionalEncoderK7 encoder(initial_state);
    std::size_t written = 0U;

    for (const auto bit : input)
    {
        const auto pair = encoder.push(bit);

        for (std::uint8_t repeat = 0U; repeat < pair_repetitions; ++repeat)
        {
            output[written++] = pair.t1;
            output[written++] = pair.t2;
        }
    }

    return written;
}

// -----------------------------------------------------------------------------
// encode_tail_biting_punctured_3_4
// -----------------------------------------------------------------------------
// 50K view: The high-rate transmit encoding: tail-biting (no flush tail) + puncture
//   to rate 3/4.
// Detailed view: Warm the register with input[0..5] (no output), encode input
//   [6..N-1], then wrap to encode input[0..5] - so the end state equals the start
//   state (tail-biting). Keep only 4 of every 6 serialized T1,T2 bits via
//   puncture_mask {1,1,1,0,0,1} -> 3 in / 4 out.
// 5th-grade view: Encode in a loop so the start and end line up, and skip some
//   check-sounds to talk faster.
// -----------------------------------------------------------------------------
Result<std::size_t> encode_tail_biting_punctured_3_4(BitSpan input, MutableBitSpan output) noexcept
{
    if (input.size() < 7U || (input.size() % 3U) != 0U)
    {
        return Status{StatusCode::invalid_argument, "tail-biting input must be >=7 bits and divisible by 3"};
    }

    const auto required = (input.size() * 4U) / 3U;

    if (output.size() < required)
    {
        return Status{StatusCode::buffer_too_small, "punctured FEC output too small"};
    }

    ConvolutionalEncoderK7 encoder;

    for (std::size_t index = 0U; index < 6U; ++index)
    {
        static_cast<void>(encoder.push(input[index]));
    }

    std::size_t unpunctured_index = 0U;
    std::size_t written = 0U;
    auto emit_pair = [&](EncodedPair pair) noexcept
    {
        const std::array<std::uint8_t, 2> bits{pair.t1, pair.t2};

        for (const auto bit : bits)
        {
            if (puncture_mask[unpunctured_index % puncture_mask.size()] != 0U)
            {
                output[written++] = bit;
            }

            ++unpunctured_index;
        }
    };

    for (std::size_t index = 6U; index < input.size(); ++index)
    {
        emit_pair(encoder.push(input[index]));
    }

    for (std::size_t index = 0U; index < 6U; ++index)
    {
        emit_pair(encoder.push(input[index]));
    }

    return written;
}

} // namespace modem::common
