// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

// =============================================================================
// bits.hpp - named bit-field extraction and single-bit ops.
// =============================================================================
// The modem serialises and deserialises tribits, dibits, octets and packed words
// constantly. Done by hand these are shifts and masks with a static_cast to fit,
// and each cast is a place a width can drift. Here the WIDTH is a template
// parameter and the MASK is the range proof, so the extraction is unchecked yet
// cannot overflow the target: bits<3>(word) is a 3-bit value in a uint8 by
// construction. This header (with convert.hpp) is one of the few files allowed to
// spell a numeric static_cast; the cast ratchet enforces that.
// =============================================================================

#include "modem/common/convert.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace modem::common
{

// Extract Width bits at `shift`, into To. The mask is the proof: the result cannot exceed
// the target, so no run-time check is needed. Width must fit both the source and the target.
template <unsigned Width, std::unsigned_integral To = std::uint32_t, Integer From>
    requires(Width >= 1U && Width <= std::numeric_limits<To>::digits && Width <= std::numeric_limits<std::make_unsigned_t<From>>::digits)
[[nodiscard]] constexpr To bits(From word, unsigned shift = 0U) noexcept
{
    using U = std::make_unsigned_t<From>;
    constexpr U mask = Width == std::numeric_limits<U>::digits ? static_cast<U>(~U{0}) : static_cast<U>((U{1} << Width) - 1U);
    return static_cast<To>((static_cast<U>(word) >> shift) & mask);
}

template <unsigned Width, std::unsigned_integral To = std::uint8_t, Integer From> [[nodiscard]] constexpr To low_bits(From word) noexcept
{
    return bits<Width, To>(word);
}

template <Integer Word> [[nodiscard]] constexpr std::uint8_t bit_at(Word w, unsigned position) noexcept
{
    return bits<1U, std::uint8_t>(w, position);
}

// A single set bit, with a checked position (traps at compile time in constant evaluation,
// at run time through the handler).
template <std::unsigned_integral U> [[nodiscard]] constexpr U bit(unsigned position) noexcept
{
    if (position >= std::numeric_limits<U>::digits)
    {
        convert_detail::violated("bit: position out of range");
    }
    return static_cast<U>(U{1} << position);
}

// Set or clear one bit of an octet (index 0..7, LSB first).
[[nodiscard]] constexpr std::uint8_t with_bit(std::uint8_t octet, std::size_t index, bool set) noexcept
{
    const auto mask = static_cast<std::uint8_t>(1U << index);
    return set ? static_cast<std::uint8_t>(octet | mask) : static_cast<std::uint8_t>(octet & static_cast<std::uint8_t>(~mask));
}

// Shift a new LSB into an accumulator (MSB-first bit assembly).
[[nodiscard]] constexpr std::uint8_t shift_in_bit(std::uint8_t acc, bool bit) noexcept
{
    return static_cast<std::uint8_t>(static_cast<unsigned>(acc) << 1U | (bit ? 1U : 0U));
}

template <std::unsigned_integral To = std::uint32_t, std::unsigned_integral T> [[nodiscard]] constexpr To popcount_as(T x) noexcept
{
    return static_cast<To>(std::popcount(x)); // 0..digits(T), fits any To with digits >= log2(digits(T))+1
}

template <std::unsigned_integral To = std::uint32_t, std::unsigned_integral T> [[nodiscard]] constexpr To countr_zero_as(T x) noexcept
{
    return static_cast<To>(std::countr_zero(x)); // 0..digits(T)
}

} // namespace modem::common
