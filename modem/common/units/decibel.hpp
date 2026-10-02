// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

// =============================================================================
// Decibel<Tag> - distinct dB types that know their own dimensional algebra.
// =============================================================================
// A number of decibels is meaningless without saying "decibels of what." Power
// (dBW/dBm), antenna Gain (dBi/dBd), and a dimensionless Ratio (dB, e.g. an SNR or
// Eb/N0) are made three different types by their Tag, so they cannot be added by
// accident. The cross-type operators encode the physics instead: Power + Gain is a
// Power (amplification); Power - Power is a Ratio (how many dB apart two levels are).
// Adding two absolute Power levels is nonsense, so it does not compile at all.
//
// The stored value is always canonical dB in the one build-time Real. The dB-domain
// operators are constexpr and fold away; only the linear<->dB conversions call the
// runtime log/pow, and they go through the modem's rmath so a float build never
// promotes to double.
// =============================================================================

#include "modem/common/real.hpp"

#include <compare>
#include <concepts>

namespace modem::common
{

namespace decibel_detail
{

[[nodiscard]] inline Real to_db_power(Real linear) noexcept
{
    return static_cast<Real>(10) * (rmath::log(linear) / rmath::log(static_cast<Real>(10)));
}

[[nodiscard]] inline Real from_db_power(Real db) noexcept
{
    return rmath::pow(static_cast<Real>(10), db / static_cast<Real>(10));
}

} // namespace decibel_detail

struct PowerTag
{
    enum class Unit
    {
        dBW,
        dBm,
        Watts
    };

    static constexpr Unit canonical = Unit::dBW;

    // An absolute level (a power), so it does not add to or negate itself in the dB domain.
    static constexpr bool is_absolute = true;

    // constexpr: the dBW/dBm offsets fold at compile time. The Watts branch calls
    // the runtime log/pow, so a Watts value used in a constant expression is
    // (correctly) rejected; dB-domain construction is not.
    [[nodiscard]] static constexpr Real to_db(Real value, Unit unit) noexcept
    {
        switch (unit)
        {
        case Unit::dBW:
            return value;
        case Unit::dBm:
            return value - static_cast<Real>(30);
        case Unit::Watts:
            return decibel_detail::to_db_power(value);
        }

        return static_cast<Real>(0);
    }

    [[nodiscard]] static constexpr Real from_db(Real db, Unit unit) noexcept
    {
        switch (unit)
        {
        case Unit::dBW:
            return db;
        case Unit::dBm:
            return db + static_cast<Real>(30);
        case Unit::Watts:
            return decibel_detail::from_db_power(db);
        }

        return static_cast<Real>(0);
    }
};

struct GainTag
{
    enum class Unit
    {
        dBi,
        dBd
    };

    static constexpr Unit canonical = Unit::dBi;

    // A relative quantity (an antenna gain), so it adds and negates in the dB domain.
    static constexpr bool is_absolute = false;

    [[nodiscard]] static constexpr Real to_db(Real value, Unit unit) noexcept { return unit == Unit::dBd ? value + static_cast<Real>(2.15) : value; }

    [[nodiscard]] static constexpr Real from_db(Real db, Unit unit) noexcept { return unit == Unit::dBd ? db - static_cast<Real>(2.15) : db; }
};

struct RatioTag
{
    enum class Unit
    {
        dB,
        Linear
    };

    static constexpr Unit canonical = Unit::dB;

    // A dimensionless ratio (SNR, Eb/N0), so it adds and negates in the dB domain.
    static constexpr bool is_absolute = false;

    // constexpr for the dB passthrough; the Linear branch is runtime-only (log/pow).
    [[nodiscard]] static constexpr Real to_db(Real value, Unit unit) noexcept { return unit == Unit::Linear ? decibel_detail::to_db_power(value) : value; }

    [[nodiscard]] static constexpr Real from_db(Real db, Unit unit) noexcept { return unit == Unit::Linear ? decibel_detail::from_db_power(db) : db; }
};

inline constexpr auto dBW = PowerTag::Unit::dBW;
inline constexpr auto dBm = PowerTag::Unit::dBm;
inline constexpr auto Watts = PowerTag::Unit::Watts;

inline constexpr auto dBi = GainTag::Unit::dBi;
inline constexpr auto dBd = GainTag::Unit::dBd;

inline constexpr auto dB = RatioTag::Unit::dB;
inline constexpr auto Linear = RatioTag::Unit::Linear;

template <typename Tag> class Decibel
{
public:
    using Unit = typename Tag::Unit;

    constexpr Decibel() noexcept = default;

    constexpr Decibel(Real value, Unit unit) noexcept : mDb{Tag::to_db(value, unit)} {}

    [[nodiscard]] static constexpr Decibel from_db(Real db) noexcept { return Decibel{db, RawTag{}}; }

    [[nodiscard]] constexpr Real in(Unit unit) const noexcept { return Tag::from_db(mDb, unit); }

    [[nodiscard]] constexpr Real db() const noexcept { return mDb; }

    [[nodiscard]] constexpr auto operator<=>(const Decibel&) const noexcept = default;

    // A relative quantity (Gain, Ratio) adds and negates in the dB domain; an
    // absolute level (Power) does not, so these are removed for an absolute Tag.
    // The Tag::is_absolute trait carries that, so a future absolute dB type opts in
    // without editing these operators.
    [[nodiscard]] constexpr Decibel operator+(Decibel other) const noexcept
        requires(!Tag::is_absolute)
    {
        return from_db(mDb + other.mDb);
    }

    [[nodiscard]] constexpr Decibel operator-(Decibel other) const noexcept
        requires(!Tag::is_absolute)
    {
        return from_db(mDb - other.mDb);
    }

    [[nodiscard]] constexpr Decibel operator-() const noexcept
        requires(!Tag::is_absolute)
    {
        return from_db(-mDb);
    }

private:
    struct RawTag
    {
    };

    constexpr Decibel(Real db, RawTag) noexcept : mDb{db} {}

    Real mDb{};
};

using Power = Decibel<PowerTag>;
using Gain = Decibel<GainTag>;
using Db = Decibel<RatioTag>;

// --- cross-type dimensional algebra ---

[[nodiscard]] inline constexpr Power operator+(Power level, Gain gain) noexcept
{
    return Power::from_db(level.db() + gain.db());
}

[[nodiscard]] inline constexpr Power operator+(Gain gain, Power level) noexcept
{
    return Power::from_db(level.db() + gain.db());
}

[[nodiscard]] inline constexpr Power operator-(Power level, Gain gain) noexcept
{
    return Power::from_db(level.db() - gain.db());
}

[[nodiscard]] inline constexpr Db operator-(Power a, Power b) noexcept
{
    return Db::from_db(a.db() - b.db());
}

[[nodiscard]] inline constexpr Power operator+(Power level, Db ratio) noexcept
{
    return Power::from_db(level.db() + ratio.db());
}

[[nodiscard]] inline constexpr Power operator+(Db ratio, Power level) noexcept
{
    return Power::from_db(level.db() + ratio.db());
}

[[nodiscard]] inline constexpr Power operator-(Power level, Db ratio) noexcept
{
    return Power::from_db(level.db() - ratio.db());
}

[[nodiscard]] inline constexpr Gain operator+(Gain gain, Db ratio) noexcept
{
    return Gain::from_db(gain.db() + ratio.db());
}

[[nodiscard]] inline constexpr Gain operator+(Db ratio, Gain gain) noexcept
{
    return Gain::from_db(gain.db() + ratio.db());
}

[[nodiscard]] inline constexpr Gain operator-(Gain gain, Db ratio) noexcept
{
    return Gain::from_db(gain.db() - ratio.db());
}

// --- amplitude (field) dB: a helper pair, not a type ---
// An amplitude/voltage ratio in dB is 20*log10, not the 10*log10 of a power ratio; mixing
// the two is a classic 2x error. These name the field-quantity form. Byte-exact with the
// shipped `20.0F * std::log10(ratio)` (std::log10 on a float is log10f), via rmath.
[[nodiscard]] inline Real db_from_amplitude_ratio(Real ratio) noexcept
{
    return static_cast<Real>(20) * rmath::log10(ratio);
}

[[nodiscard]] inline Real amplitude_ratio(Real db) noexcept
{
    return rmath::pow(static_cast<Real>(10), db / static_cast<Real>(20));
}

} // namespace modem::common
