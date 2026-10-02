// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// =============================================================================
// scrambler.hpp - body whitening, sync sequence, Gray map
// =============================================================================
// BodyDataRandomizer (whitening LFSR, data and probes), SyncRandomizer (fixed
// preamble sequence) and modified_gray_decode (the
// constellation bit->tribit mapping shared with the demapper).
// Teaching walkthrough: modem/m110a/body-waveform-and-scrambler-explainer.md
// =============================================================================

#include "modem/common/bits.hpp"
#include "modem/common/status.hpp"
namespace modem::m110a
{

class BodyDataRandomizer
{
public:
    BodyDataRandomizer() noexcept { reset(); }

    void reset() noexcept;
    [[nodiscard]] std::uint8_t next_tribit() noexcept;
    // Steps past one tribit, exactly as next_tribit() does, without reading it.
    void advance() noexcept;
    [[nodiscard]] std::uint16_t state() const noexcept;

    [[nodiscard]] std::uint16_t symbols_generated() const noexcept { return mSymbolsGenerated; }

private:
    void clock() noexcept;

    std::uint16_t mState{};
    std::uint16_t mSymbolsGenerated{};
};

class SyncRandomizer
{
public:
    [[nodiscard]] std::uint8_t next_tribit() noexcept;
    // Steps past one tribit, exactly as next_tribit() does, without reading it.
    void advance() noexcept;

    void reset() noexcept { mIndex = 0U; }

private:
    std::size_t mIndex{};
};

[[nodiscard]] constexpr std::uint8_t modified_gray_decode(std::uint8_t value, std::uint8_t width) noexcept
{
    // MIL-STD-188-110B body-waveform modified-Gray mapping. This is not the
    // Appendix C tribit-to-8PSK transcoding table.
    if (width == 1U)
    {
        constexpr std::array<std::uint8_t, 2> map{0U, 4U};
        return map[value & 1U];
    }

    if (width == 2U)
    {
        constexpr std::array<std::uint8_t, 4> map{0U, 1U, 3U, 2U};
        return map[value & 3U];
    }

    constexpr std::array<std::uint8_t, 8> map{0U, 1U, 3U, 2U, 7U, 6U, 4U, 5U};
    return map[value & 7U];
}

// The body mapping of a width-bit source value to its tribit, before the
// scrambler: 1 bit -> 0/4, 2 bits -> Gray << 1, 3 bits -> Gray. The transmit
// mapper and both demappers use it.
[[nodiscard]] constexpr std::uint8_t mapped_tribit(std::uint8_t value, std::uint8_t width) noexcept
{
    if (width == 1U)
    {
        return modified_gray_decode(value, width);
    }

    if (width == 2U)
    {
        // The dibit's Gray code shifted onto the even tribits, from the one Gray table above.
        return ::modem::common::low_bits<3>(modified_gray_decode(value, width) << 1U);
    }

    return modified_gray_decode(value, width);
}

// The scrambler combine on the 8-PSK tribit ring: (tribit + randomizer) mod 8. Every
// transmit mapper, probe generator and receive reference adds the randomizer this way.
[[nodiscard]] constexpr std::uint8_t tribit_add(unsigned tribit, unsigned randomizer) noexcept
{
    return ::modem::common::low_bits<3>(tribit + randomizer);
}

// The width-2 mapping is derived, not tabulated: pin all four entries to the former table {0, 2, 6, 4}.
static_assert(mapped_tribit(0U, 2U) == 0U && mapped_tribit(1U, 2U) == 2U && mapped_tribit(2U, 2U) == 6U && mapped_tribit(3U, 2U) == 4U);
static_assert(tribit_add(7U, 5U) == 4U);

} // namespace modem::m110a
