// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

// =============================================================================
// Frequency - a physical quantity that carries its unit AND its algebra in the type.
// =============================================================================
// Canonical storage is Hz in DOUBLE, deliberately not the modem's Real. A
// frequency is configuration/metadata (a carrier, a tone spacing, an on-air dial
// setting), never a per-sample value in the MAC loop, so the extra bits are free
// where it lives; and a frequency can be an RF value where float would lose single
// hertz (float is exact only to 2^24, about 16.7 MHz). The signal path stays Real;
// convert once, on purpose, at the boundary with hertz_real(). Build with a unit,
// read with a unit, so a hertz cannot be mistaken for a kilohertz and a Frequency
// cannot be passed where a sample count belongs.
//
// Two KINDS, split by algebra rather than by role. An ABSOLUTE frequency is a point
// on the dial (a carrier, a subcarrier, an on-air setting). A DIFFERENCE is a signed
// gap between two of them - and every "kind of gap" is the same algebra, so offset,
// deviation, bandwidth, tone spacing and Doppler are all FrequencyDelta; the variable
// name carries the role. This is an affine space, and the operators encode it:
//   Absolute - Absolute -> Difference   (how far apart two dial points are)
//   Absolute +/- Difference -> Absolute (retune by an offset)
//   Difference +/- Difference -> Difference
//   Difference * / scalar -> Difference,  Difference / Difference -> ratio
// Absolute + Absolute is meaningless (two carriers do not add), so it does not compile;
// neither does scaling or negating an absolute point, nor comparing across kinds. Same
// pattern as Decibel<Tag> (Power vs Gain/Ratio) and SampleIndex vs SampleCount.
//
// Byte-exact: both kinds store the same double and use the same double +/-/*/ as the
// pre-split Frequency, so no numeric result changes; only the type of a difference does.
// =============================================================================

#include "modem/common/real.hpp"

#include <compare>
#include <cstdint>
#include <utility>

namespace modem::common
{

enum class FreqUnit : std::int64_t
{
    Hz = 1,
    kHz = 1'000,
    MHz = 1'000'000
};

inline constexpr auto Hz = FreqUnit::Hz;
inline constexpr auto kHz = FreqUnit::kHz;
inline constexpr auto MHz = FreqUnit::MHz;

// The two algebraic kinds. Tags only; the class below carries the storage.
struct AbsoluteFrequencyTag
{
};

struct FrequencyDifferenceTag
{
};

template <typename Kind> class BasicFrequency
{
public:
    constexpr BasicFrequency() noexcept = default;

    constexpr explicit BasicFrequency(double value, FreqUnit unit) noexcept : mHertz{value * static_cast<double>(std::to_underlying(unit))} {}

    // Construct straight from canonical Hz, skipping the unit multiply. This is how the
    // algebra operators below build their results, and how a caller states "these hertz
    // are already canonical".
    [[nodiscard]] static constexpr BasicFrequency from_hz(double hz) noexcept { return BasicFrequency{hz, RawTag{}}; }

    [[nodiscard]] constexpr double in(FreqUnit unit) const noexcept { return mHertz / static_cast<double>(std::to_underlying(unit)); }

    [[nodiscard]] constexpr double hertz() const noexcept { return mHertz; }

    // The one sanctioned narrowing: hand the frequency to the Real signal path.
    [[nodiscard]] constexpr Real hertz_real() const noexcept { return static_cast<Real>(mHertz); }

    [[nodiscard]] constexpr auto operator<=>(const BasicFrequency&) const noexcept = default;

    // Scale and negate exist only for a DIFFERENCE: an offset has a true zero to scale
    // about, an absolute dial point does not. (Difference / Difference is a free operator
    // below, since it crosses to a plain ratio.)
    [[nodiscard]] constexpr BasicFrequency operator-() const noexcept
        requires std::same_as<Kind, FrequencyDifferenceTag>
    {
        return from_hz(-mHertz);
    }

    [[nodiscard]] constexpr BasicFrequency operator*(double scale) const noexcept
        requires std::same_as<Kind, FrequencyDifferenceTag>
    {
        return from_hz(mHertz * scale);
    }

    [[nodiscard]] constexpr BasicFrequency operator/(double divisor) const noexcept
        requires std::same_as<Kind, FrequencyDifferenceTag>
    {
        return from_hz(mHertz / divisor);
    }

private:
    struct RawTag
    {
    };

    constexpr BasicFrequency(double hz, RawTag) noexcept : mHertz{hz} {}

    double mHertz{};
};

// An absolute point on the dial; the difference alias names a signed gap of any role.
using Frequency = BasicFrequency<AbsoluteFrequencyTag>;
using FrequencyDelta = BasicFrequency<FrequencyDifferenceTag>;

// --- cross-kind affine algebra ---

// Absolute - Absolute -> Difference: how far apart, and in which direction.
[[nodiscard]] inline constexpr FrequencyDelta operator-(Frequency a, Frequency b) noexcept
{
    return FrequencyDelta::from_hz(a.hertz() - b.hertz());
}

// Absolute +/- Difference -> Absolute: retune by an offset.
[[nodiscard]] inline constexpr Frequency operator+(Frequency base, FrequencyDelta delta) noexcept
{
    return Frequency::from_hz(base.hertz() + delta.hertz());
}

[[nodiscard]] inline constexpr Frequency operator+(FrequencyDelta delta, Frequency base) noexcept
{
    return Frequency::from_hz(base.hertz() + delta.hertz());
}

[[nodiscard]] inline constexpr Frequency operator-(Frequency base, FrequencyDelta delta) noexcept
{
    return Frequency::from_hz(base.hertz() - delta.hertz());
}

// Difference +/- Difference -> Difference.
[[nodiscard]] inline constexpr FrequencyDelta operator+(FrequencyDelta a, FrequencyDelta b) noexcept
{
    return FrequencyDelta::from_hz(a.hertz() + b.hertz());
}

[[nodiscard]] inline constexpr FrequencyDelta operator-(FrequencyDelta a, FrequencyDelta b) noexcept
{
    return FrequencyDelta::from_hz(a.hertz() - b.hertz());
}

// Difference / Difference -> a pure ratio (e.g. an offset as a fraction of a bandwidth).
[[nodiscard]] inline constexpr double operator/(FrequencyDelta a, FrequencyDelta b) noexcept
{
    return a.hertz() / b.hertz();
}

// scalar * Difference (the member gives Difference * scalar).
[[nodiscard]] inline constexpr FrequencyDelta operator*(double scale, FrequencyDelta delta) noexcept
{
    return FrequencyDelta::from_hz(delta.hertz() * scale);
}

// Magnitude of an offset.
[[nodiscard]] inline constexpr FrequencyDelta abs(FrequencyDelta delta) noexcept
{
    return FrequencyDelta::from_hz(delta.hertz() < 0.0 ? -delta.hertz() : delta.hertz());
}

} // namespace modem::common
