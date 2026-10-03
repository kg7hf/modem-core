// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

// =============================================================================
// rates.hpp - the carrier-step and per-symbol phase-rate conversions, once.
// =============================================================================
// A carrier phase step (rad/sample) and a per-symbol phase rate (rad/symbol) are easy
// to spell by hand at every site, each a two_pi * f / rate with its own local 2pi and
// its own cast of the rate. Here they are one function each, in Real and in the
// shipped operation order, so every site is bit-identical and the order that
// produced the on-air waveform (and the goldens) cannot drift. radians_per_sample
// carries the 0x3E714639 pin at 1800 Hz / 48 kHz.
//
// The frequency-taking helpers are kind-agnostic: they READ a frequency through
// .hertz() (double), so they compile against both the single-kind Frequency and the
// two-kind BasicFrequency<Kind> without editing frequency.hpp. A carrier (absolute)
// feeds radians_per_sample; a frequency offset (a difference) feeds radians_per_symbol.
// These return Real, and the rad/symbol inverse returns plain Hz that a caller
// re-wraps with FrequencyDelta::from_hz - never a cast.
// =============================================================================

#include "modem/common/constants.hpp"
#include "modem/common/convert.hpp"
#include "modem/common/real.hpp"
#include "modem/common/units/frequency.hpp"
#include "modem/common/units/sampling.hpp"

#include <bit>
#include <concepts>
#include <cstdint>
#include <optional>

namespace modem::common
{

// Anything that reports canonical hertz as a double: Frequency, FrequencyDelta, and any
// future kind. Duck-typed on purpose, so this header does not depend on which kinds exist.
template <class F>
concept HertzSource = requires(const F f) {
    { f.hertz() } -> std::same_as<double>;
};

// Carrier phase advance per sample: (two_pi * f) / fs, in Real, in the shipped order.
template <HertzSource F> [[nodiscard]] constexpr Real radians_per_sample(F carrier, SampleRate sample_rate) noexcept
{
    return two_pi * static_cast<Real>(carrier.hertz()) / sample_rate.hertz_real();
}

// Phase advance per symbol for a frequency (offset) at a symbol rate: (two_pi * f) / baud.
template <HertzSource F> [[nodiscard]] constexpr Real radians_per_symbol(F frequency, Baud baud) noexcept
{
    return two_pi * static_cast<Real>(frequency.hertz()) / baud.per_second_real();
}

// The inverse used by the acquisition frequency-error estimate: (w * baud) / two_pi, in Hz.
// Returns plain Real Hz; the caller wraps it with FrequencyDelta::from_hz (not a cast).
[[nodiscard]] constexpr Real hertz_from_radians_per_symbol(Real radians_per_symbol_value, Baud baud) noexcept
{
    return radians_per_symbol_value * baud.per_second_real() / two_pi;
}

// Samples per symbol, compile-time: a hard error if the division is inexact (so 44100/2400
// cannot silently truncate). violated() is not constexpr, so the bad branch fails consteval.
[[nodiscard]] consteval std::size_t samples_per_symbol(SampleRate sample_rate, Baud baud)
{
    if (baud.value() == 0U || sample_rate.hertz() % baud.value() != 0U)
    {
        convert_detail::violated("samples_per_symbol: rate not an integer multiple of baud");
    }
    return widen<std::size_t>(sample_rate.hertz() / baud.value());
}

// The runtime twin: returns nullopt instead of failing, for the rates that are not exact
// (e.g. 44100 Hz, which is not a whole number of samples per 2400 Bd symbol).
[[nodiscard]] constexpr std::optional<std::size_t> try_samples_per_symbol(SampleRate sample_rate, Baud baud) noexcept
{
    if (baud.value() == 0U || sample_rate.hertz() % baud.value() != 0U)
    {
        return std::nullopt;
    }
    return widen<std::size_t>(sample_rate.hertz() / baud.value());
}

// The carrier-step pin: the exact float the shipped code produced for the body carrier.
static_assert(std::bit_cast<std::uint32_t>(radians_per_sample(Frequency{1800.0, Hz}, SampleRate{48000U})) == 0x3E714639U);
static_assert(samples_per_symbol(SampleRate{48000U}, Baud{2400U}) == 20U);

} // namespace modem::common
