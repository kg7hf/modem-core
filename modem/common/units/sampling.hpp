// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

// =============================================================================
// Sampling quantities - the integral rates and counts of the sample stream.
// =============================================================================
// These are counts, not measured reals, so they are backed by fixed-width
// integers, not Real: a sample rate (Hz), a symbol rate (baud), a user bit rate
// (bps), and the running sample position/length of the stream (the 64-bit counter
// from the overview). Making them distinct types stops the classic transposition
// bug - passing a baud where a bit rate is wanted, or a count where an index is -
// because the compiler will not convert one to another.
// =============================================================================

#include "modem/common/convert.hpp"

#include <compare>
#include <cstdint>

namespace modem::common
{

// Samples per second (e.g. the 48 kHz audio front end, or a 9600 Hz baseband).
class SampleRate
{
public:
    constexpr SampleRate() noexcept = default;

    constexpr explicit SampleRate(std::uint32_t hz) noexcept : mHertz{hz} {}

    [[nodiscard]] constexpr std::uint32_t hertz() const noexcept { return mHertz; }

    // The sample rate on the Real signal path; exact for the sub-2^24 rates the modem uses
    // (checked by as_real), mirroring Frequency::hertz_real().
    [[nodiscard]] constexpr Real hertz_real() const noexcept { return as_real(mHertz); }

    [[nodiscard]] constexpr auto operator<=>(const SampleRate&) const noexcept = default;

private:
    std::uint32_t mHertz{};
};

// Channel symbols per second (the 2400 baud serial-tone symbol rate).
class Baud
{
public:
    constexpr Baud() noexcept = default;

    constexpr explicit Baud(std::uint32_t symbols_per_second) noexcept : mRate{symbols_per_second} {}

    [[nodiscard]] constexpr std::uint32_t value() const noexcept { return mRate; }

    // The symbol rate on the Real signal path; exact for the modem's rates (checked by
    // as_real). Used where a rad/symbol or per-symbol scaling needs the baud as a Real.
    [[nodiscard]] constexpr Real per_second_real() const noexcept { return as_real(mRate); }

    [[nodiscard]] constexpr auto operator<=>(const Baud&) const noexcept = default;

private:
    std::uint32_t mRate{};
};

// User data rate in bits per second (the 75..4800 bps ladder).
class BitRate
{
public:
    constexpr BitRate() noexcept = default;

    constexpr explicit BitRate(std::uint32_t bits_per_second) noexcept : mRate{bits_per_second} {}

    [[nodiscard]] constexpr std::uint32_t value() const noexcept { return mRate; }

    [[nodiscard]] constexpr auto operator<=>(const BitRate&) const noexcept = default;

private:
    std::uint32_t mRate{};
};

// A number of samples (a duration/length of the stream).
class SampleCount
{
public:
    constexpr SampleCount() noexcept = default;

    constexpr explicit SampleCount(std::uint64_t samples) noexcept : mSamples{samples} {}

    [[nodiscard]] constexpr std::uint64_t value() const noexcept { return mSamples; }

    [[nodiscard]] constexpr auto operator<=>(const SampleCount&) const noexcept = default;

    [[nodiscard]] constexpr SampleCount operator+(SampleCount other) const noexcept { return SampleCount{mSamples + other.mSamples}; }

    [[nodiscard]] constexpr SampleCount operator-(SampleCount other) const noexcept { return SampleCount{mSamples - other.mSamples}; }

    [[nodiscard]] constexpr SampleCount operator*(std::uint64_t factor) const noexcept { return SampleCount{mSamples * factor}; }

    [[nodiscard]] constexpr SampleCount operator/(std::uint64_t divisor) const noexcept { return SampleCount{mSamples / divisor}; }

private:
    std::uint64_t mSamples{};
};

// An absolute position in the sample stream (the running 64-bit counter). The
// difference of two indices is a SampleCount, and advancing an index by a count
// gives another index; index + index is meaningless and is not defined.
class SampleIndex
{
public:
    constexpr SampleIndex() noexcept = default;

    constexpr explicit SampleIndex(std::uint64_t index) noexcept : mIndex{index} {}

    [[nodiscard]] constexpr std::uint64_t value() const noexcept { return mIndex; }

    [[nodiscard]] constexpr auto operator<=>(const SampleIndex&) const noexcept = default;

    [[nodiscard]] constexpr SampleIndex operator+(SampleCount count) const noexcept { return SampleIndex{mIndex + count.value()}; }

    [[nodiscard]] constexpr SampleIndex operator-(SampleCount count) const noexcept { return SampleIndex{mIndex - count.value()}; }

    [[nodiscard]] constexpr SampleCount operator-(SampleIndex earlier) const noexcept { return SampleCount{mIndex - earlier.mIndex}; }

    constexpr SampleIndex& operator+=(SampleCount count) noexcept
    {
        mIndex += count.value();
        return *this;
    }

    constexpr SampleIndex& operator++() noexcept
    {
        ++mIndex;
        return *this;
    }

private:
    std::uint64_t mIndex{};
};

// Can an index advance by a count without passing a limit (and without wrapping)? The
// subtraction limit - from cannot underflow because of the from <= limit guard.
[[nodiscard]] constexpr bool can_advance(SampleIndex from, SampleCount by, SampleIndex limit) noexcept
{
    return from <= limit && by.value() <= (limit - from).value();
}

} // namespace modem::common
