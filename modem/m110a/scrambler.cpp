// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

/*
================================================================================
scrambler.cpp - the three scramblers + the modified-Gray constellation map
================================================================================
Three DIFFERENT mechanisms for three purposes:
  BodyDataRandomizer  WHITENING: a 12-bit LFSR (seed 0xBAD, 110B Fig 6, 8 clocks/
                      tribit, reset @160 sym). Its tribit is ADDED mod 8 to every
                      transmitted body symbol (subtracted on receive) so the spectrum
                      stays even regardless of the data.
  SyncRandomizer      PREAMBLE: a fixed 32-entry sequence cycled by index; the sync
                      scramble added to every preamble symbol. (Probe symbols are
                      whitened by BodyDataRandomizer, like the data.)
  modified_gray_decode  the constellation bit->tribit map shared by the demapper and
                      the transmit mapper (adjacent points differ in one bit).
Teaching walkthrough: modem/m110a/body-waveform-and-scrambler-explainer.md
================================================================================
*/

#include "modem/m110a/scrambler.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

#include "modem/common/status.hpp"
namespace modem::m110a
{

using ::modem::common::low_bits;

constexpr std::array<std::uint8_t, 32> sync_sequence{7U, 4U, 3U, 0U, 5U, 1U, 5U, 0U, 2U, 2U, 1U, 1U, 5U, 7U, 4U, 3U, 5U, 0U, 2U, 6U, 2U, 1U, 6U, 2U, 0U, 0U, 5U, 0U, 5U, 2U, 6U, 6U};

void BodyDataRandomizer::reset() noexcept
{
    // MIL-STD-188-110B Figure 6, printed page 46. Bit 11 is the leftmost
    // stage and bits 2..0 are the three output stages. An independent
    // reference implementation confirms all 160 output symbols for this
    // convention.
    mState = 0xBADU;
    mSymbolsGenerated = 0U;
}

// -----------------------------------------------------------------------------
// BodyDataRandomizer::clock / next_tribit  (the whitening LFSR)
// -----------------------------------------------------------------------------
// 50K view: Generate the whitening tribit added to every transmitted body symbol.
// Detailed view: A 12-bit LFSR (seed 0xBAD, 110B Figure 6); feedback from bit 11
//   XORs taps {6,4,1}. next_tribit clocks it 8 times and returns the low 3 bits,
//   resetting after 160 symbols. The output tribit is added mod 8 on transmit and
//   subtracted (via conj derotation) on receive.
// 5th-grade view: A shared pseudo-random dial that both sides turn in lock-step to
//   scramble and unscramble the voice.
// -----------------------------------------------------------------------------
void BodyDataRandomizer::clock() noexcept
{
    constexpr std::uint16_t feedback_taps = (1U << 6U) | (1U << 4U) | (1U << 1U);
    const bool feedback = ((mState >> 11U) & 1U) != 0U;
    mState = low_bits<12, std::uint16_t>(mState << 1U); // the 12-bit register

    if (feedback)
    {
        mState ^= feedback_taps;
        mState |= 1U;
    }
}

std::uint8_t BodyDataRandomizer::next_tribit() noexcept
{
    advance();
    return low_bits<3>(mState);
}

void BodyDataRandomizer::advance() noexcept
{
    if (mSymbolsGenerated == 160U)
    {
        reset();
    }

    for (std::uint8_t count = 0U; count < 8U; ++count)
    {
        clock();
    }

    ++mSymbolsGenerated;
}

std::uint16_t BodyDataRandomizer::state() const noexcept
{
    return mState;
}

std::uint8_t SyncRandomizer::next_tribit() noexcept
{
    const auto value = sync_sequence[mIndex];
    advance();
    return value;
}

void SyncRandomizer::advance() noexcept
{
    mIndex = (mIndex + 1U) % sync_sequence.size();
}

} // namespace modem::m110a
