// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#include "modem/common/units/decibel.hpp"
#include "modem/common/units/frequency.hpp"
#include "modem/common/units/rates.hpp"
#include "modem/common/units/sampling.hpp"

#include <catch2/catch_test_macros.hpp>

#include <bit>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <optional>
#include <type_traits>

// =============================================================================
// units_tests - the strong quantity types (modem/common/units/).
// =============================================================================
// It proves the conversions and the dimensional algebra at run time, with Catch2,
// and proves the ILLEGAL operations at compile time with static_assert: a quantity
// you cannot misuse is the whole point. (Ported from the legacy harness to Catch2.)
// =============================================================================

using namespace modem::common;

namespace
{

[[nodiscard]] bool close(float lhs, float rhs, float tolerance = 1.0e-5F) noexcept
{
    return std::fabs(lhs - rhs) <= tolerance;
}

template <typename A, typename B>
concept Addable = requires(A a, B b) { a + b; };

template <typename A, typename B>
concept Subtractable = requires(A a, B b) { a - b; };

template <typename A, typename B>
concept EqualityComparable = requires(A a, B b) { a == b; };

template <typename A>
concept Scalable = requires(A a) { a * 2.0; };

template <typename A>
concept Negatable = requires(A a) { -a; };

// Positive controls for the helpers above, so that no negative use below can pass vacuously.
static_assert(Addable<SampleIndex, SampleCount> && Subtractable<Frequency, Frequency> && EqualityComparable<SampleIndex, SampleIndex>);

// --- compile-time proofs: the type system refuses the nonsense ---

// A bare Real cannot masquerade as a Frequency, and a Baud is not a BitRate.
static_assert(!std::is_convertible_v<Frequency, Real>);
static_assert(!std::is_convertible_v<Real, Frequency>);
static_assert(!std::is_convertible_v<Baud, BitRate>);
static_assert(!std::is_constructible_v<BitRate, Baud>);
static_assert(!std::is_convertible_v<SampleCount, SampleIndex>);

// Relative dB quantities add; two absolute Power levels do not.
static_assert(Addable<Gain, Gain>);
static_assert(Addable<Db, Db>);
static_assert(Addable<Power, Gain>);
static_assert(Addable<Power, Db>);
static_assert(!Addable<Power, Power>); // two absolute power levels do not add
static_assert(Addable<Gain, Power>);   // commutative: Gain + Power is a Power
static_assert(!Addable<Baud, BitRate>);

// Frequency is an affine space: a dial point (Frequency) and a signed gap between two
// of them (FrequencyDelta: offset, bandwidth, spacing, Doppler).
static_assert(std::same_as<decltype(Frequency{} - Frequency{}), FrequencyDelta>);
static_assert(std::same_as<decltype(Frequency{} + FrequencyDelta{}), Frequency>);
static_assert(std::same_as<decltype(FrequencyDelta{} + Frequency{}), Frequency>);
static_assert(std::same_as<decltype(Frequency{} - FrequencyDelta{}), Frequency>);
static_assert(std::same_as<decltype(FrequencyDelta{} + FrequencyDelta{}), FrequencyDelta>);
static_assert(std::same_as<decltype(FrequencyDelta{} / FrequencyDelta{}), double>);
static_assert(!Addable<Frequency, Frequency>);           // two carriers do not add
static_assert(!Subtractable<FrequencyDelta, Frequency>); // a gap minus a dial point means nothing
static_assert(!EqualityComparable<Frequency, FrequencyDelta>);
static_assert(Scalable<FrequencyDelta> && !Scalable<Frequency>);
static_assert(Negatable<FrequencyDelta> && !Negatable<Frequency>);
static_assert(!std::is_convertible_v<FrequencyDelta, Frequency>);
static_assert(!std::is_convertible_v<Frequency, FrequencyDelta>);
// A dial point plus half a bandwidth (300 Hz + BW/2) is exact: 3000 / 2.0 and 300 + 1500 are whole doubles.
static_assert((Frequency{300.0, Hz} + FrequencyDelta{3000.0, Hz} / 2.0).hertz() == 1800.0);
// --- rate and sample conversions ---
static_assert(SampleRate{48000U}.hertz_real() == 48000.0F);
static_assert(Baud{2400U}.per_second_real() == 2400.0F);
// The carrier step: bit-identical to the body carrier at 1800 Hz / 48 kHz.
static_assert(std::bit_cast<std::uint32_t>(radians_per_sample(Frequency{1800.0, Hz}, SampleRate{48000U})) == 0x3E714639U);
static_assert(samples_per_symbol(SampleRate{48000U}, Baud{2400U}) == 20U);
static_assert(try_samples_per_symbol(SampleRate{48000U}, Baud{2400U}) == std::optional<std::size_t>{20U});
static_assert(try_samples_per_symbol(SampleRate{44100U}, Baud{2400U}) == std::nullopt); // inexact: no truncation
// rad/symbol and its inverse round-trip a representable offset (75 Hz at 2400 baud).
static_assert(hertz_from_radians_per_symbol(radians_per_symbol(Frequency{75.0, Hz}, Baud{2400U}), Baud{2400U}) == 75.0F);

// --- the absolute-level trait and the sample-position operators ---
static_assert(PowerTag::is_absolute);
static_assert(!GainTag::is_absolute && !RatioTag::is_absolute);
static_assert((SampleCount{10U} - SampleCount{3U}) == SampleCount{7U});
static_assert((SampleCount{5U} * 4U) == SampleCount{20U});
static_assert((SampleCount{20U} / 4U) == SampleCount{5U});
static_assert((SampleIndex{100U} - SampleCount{30U}) == SampleIndex{70U});
static_assert(can_advance(SampleIndex{10U}, SampleCount{5U}, SampleIndex{20U}));
static_assert(!can_advance(SampleIndex{10U}, SampleCount{15U}, SampleIndex{20U}));
static_assert(
    []
    {
        SampleIndex i{10U};
        i += SampleCount{5U};
        ++i;
        return i;
    }() == SampleIndex{16U});

TEST_CASE("Frequency: the kinds convert exactly; a delta moves a dial point", "[units]")
{
    constexpr Frequency carrier{1800, Hz};
    CHECK(close(carrier.hertz(), 1800.0F));
    // in(kHz) is the double canonical value; narrow once at the double->Real boundary
    // (as hertz_real() does) for the float compare - 1.8 is not float-exact, so the
    // implicit narrowing would trip -Wfloat-conversion.
    CHECK(close(static_cast<Real>(carrier.in(kHz)), 1.8F));

    CHECK(close(Frequency{14, kHz}.in(Hz), 14000.0F));

    // A +75 Hz Doppler offset is a FrequencyDelta: a dial point plus a gap is a dial point.
    constexpr FrequencyDelta doppler{75, Hz};
    CHECK(close((carrier + doppler).hertz(), 1875.0F));
    CHECK(carrier < (carrier + doppler));
}

TEST_CASE("Decibels: ratios and gains and levels convert; the dimensional algebra holds", "[units]")
{
    // Ratio: 20 dB is a linear factor of 100; 3.0103 dB is ~2x power.
    const Db twenty{20, dB};
    CHECK(close(twenty.in(Linear), 100.0F, 1.0e-2F));
    const Db doubling{2, Linear};
    CHECK(close(doubling.db(), 3.0103F, 1.0e-3F));

    // Gain: 0 dBd is 2.15 dBi.
    const Gain dipole{0, dBd};
    CHECK(close(dipole.in(dBi), 2.15F));
    CHECK(close(dipole.in(dBd), 0.0F));

    // Dimensional algebra: amplify a power level by a gain; subtract two levels
    // to get the ratio between them.
    constexpr Power transmit{10, dBW};
    constexpr Gain amplifier{3, dBi};
    CHECK(close((transmit + amplifier).in(dBW), 13.0F));
    CHECK(close((amplifier + transmit).in(dBW), 13.0F));

    constexpr Power received{-120, dBW};
    const Db path_loss = transmit - received;
    CHECK(close(path_loss.db(), 130.0F));

    // An SNR margin adjusts a level.
    CHECK(close((transmit - Db{6.0F, dB}).in(dBW), 4.0F));

    // Amplitude (field) dB is 20*log10: a ratio of 10 is 20 dB, and it round-trips.
    CHECK(close(db_from_amplitude_ratio(10.0F), 20.0F, 1.0e-4F));
    CHECK(close(amplitude_ratio(20.0F), 10.0F, 1.0e-3F));
}

TEST_CASE("Sampling: the rates and the index and count algebra", "[units]")
{
    constexpr SampleRate audio{48000U};
    CHECK(audio.hertz() == 48000U);

    constexpr Baud symbols{2400U};
    CHECK(symbols.value() == 2400U);
    constexpr BitRate payload{4800U};
    CHECK(payload.value() == 4800U);

    // Index/count algebra: advance an index by a count; difference of two
    // indices is a count.
    constexpr SampleIndex start{1000U};
    constexpr SampleCount block{256U};
    constexpr SampleIndex next = start + block;
    CHECK(next.value() == 1256U);
    CHECK((next - start).value() == 256U);
}

} // namespace
