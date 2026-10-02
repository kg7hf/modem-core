// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

// =============================================================================
// convert.hpp - the one place the modem spells a numeric conversion.
// =============================================================================
// The casting rule (part 1 of the white paper): inside the library a value never
// narrows or changes sign silently, and a conversion that is really needed is
// spelled out ONCE, here, behind a named helper that says what it means. Every
// other translation unit then reads intent - narrow(), widen(), as_real(),
// to_subscript(), ring_slot() - instead of a bare static_cast whose safety the
// reader has to re-derive. This header (with bits.hpp) is one of the few files
// allowed to contain a numeric static_cast; the cast ratchet enforces that.
//
// Failure policy under -fno-exceptions:
//   * proven at compile time  -> the concept or the mask IS the proof (no check):
//     widen, as_real from <=24-bit types, ring_slot<pow2>, the bits helpers.
//   * proven by a local compare -> return an optional or a fallback:
//     index_in, element_or, offset_back, clamp_to_size, enum_from.
//   * checked -> traps via critical_error (status.hpp): narrow, to_offset,
//     to_subscript, to_unsigned, at, ring_slot(pos,period), round_to, floor_to.
//     A violation during constant evaluation is a COMPILE error (critical_error is
//     not constexpr); at run time it traps through the installable handler.
//   * documented lossy -> approx_real, for absolute sample indices past 2^24.
// =============================================================================

#include "modem/common/real.hpp"
#include "modem/common/status.hpp"

#include <algorithm>
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <optional>
#include <ranges>
#include <span>
#include <type_traits>
#include <utility>

namespace modem::common
{

namespace convert_detail
{

template <class T>
inline constexpr bool is_character_v = std::same_as<std::remove_cv_t<T>, char> || std::same_as<std::remove_cv_t<T>, char8_t> || std::same_as<std::remove_cv_t<T>, char16_t> ||
                                       std::same_as<std::remove_cv_t<T>, char32_t> || std::same_as<std::remove_cv_t<T>, wchar_t>; // std::in_range rejects these (and bool)

// Fail-fast contract hook: critical_error (status.hpp) is [[noreturn]] noexcept and not
// constexpr, so a violated contract during constant evaluation is a compile error and at
// run time it traps through the installable handler. No exceptions (-fno-exceptions).
[[noreturn]] inline void violated(const char* what) noexcept
{
    critical_error(Status{StatusCode::internal_error, what});
}

// A value whose range the caller has already proven (mask, modulo, compare). Spelled once so
// the host (size_t == uint64_t: no conversion, and -Wuseless-cast stays quiet) and the M33
// (32-bit size_t: a real narrowing) share one source line.
template <class To, class From> [[nodiscard]] constexpr To proven(From v) noexcept
{
    if constexpr (std::same_as<To, From>)
    {
        return v;
    }
    else
    {
        return static_cast<To>(v);
    }
}

} // namespace convert_detail

// Integers that std::in_range / std::cmp_* accept (no bool, no character types).
template <class T>
concept Integer = std::integral<T> && !std::same_as<std::remove_cv_t<T>, bool> && !convert_detail::is_character_v<T>;

// ------------------------------------------------------------- integer <-> integer

// gsl::narrow semantics: fail-fast if the value does not fit the target.
template <Integer To, Integer From> [[nodiscard]] constexpr To narrow(From v) noexcept
{
    if (!std::in_range<To>(v))
    {
        convert_detail::violated("narrow: value out of range");
    }
    return static_cast<To>(v);
}

// Lossless by construction: the range fits, so there is no cast and no check.
template <Integer To, Integer From>
    requires(std::in_range<To>(std::numeric_limits<From>::min()) && std::in_range<To>(std::numeric_limits<From>::max()))
[[nodiscard]] constexpr To widen(From v) noexcept
{
    return v;
}

// The float -> double island (e.g. the sequence-detector FFT). Explicit, so it does not
// count as a silent -Wdouble-promotion.
template <std::floating_point To, std::floating_point From>
    requires(std::numeric_limits<To>::digits >= std::numeric_limits<From>::digits)
[[nodiscard]] constexpr To widen(From v) noexcept
{
    return static_cast<To>(v);
}

// C++26 std::saturate_cast shim (clamps rather than traps).
template <Integer To, Integer From> [[nodiscard]] constexpr To saturate_cast(From v) noexcept
{
    if (std::cmp_less(v, std::numeric_limits<To>::min()))
    {
        return std::numeric_limits<To>::min();
    }
    if (std::cmp_greater(v, std::numeric_limits<To>::max()))
    {
        return std::numeric_limits<To>::max();
    }
    return static_cast<To>(v);
}

// C++26 std::add_sat shim.
template <std::unsigned_integral T> [[nodiscard]] constexpr T add_sat(T a, T b) noexcept
{
    const T limit = std::numeric_limits<T>::max();
    return b > limit - a ? limit : static_cast<T>(a + b);
}

template <std::signed_integral S> [[nodiscard]] constexpr std::make_unsigned_t<S> to_unsigned(S v) noexcept
{
    return narrow<std::make_unsigned_t<S>>(v);
}

// ------------------------------------------------------------- positions vs containers

using Index = std::ptrdiff_t; // ES.107 (gsl::index); the fallback when the symbol-position types are not adopted

template <Integer From> [[nodiscard]] constexpr Index to_offset(From n) noexcept
{
    return narrow<Index>(n);
}

// Traps on a negative or too-large index.
template <Integer From> [[nodiscard]] constexpr std::size_t to_subscript(From i) noexcept
{
    return narrow<std::size_t>(i);
}

[[nodiscard]] constexpr std::optional<std::size_t> index_in(Index i, std::size_t extent) noexcept
{
    if (i < 0 || std::cmp_greater_equal(i, extent))
    {
        return std::nullopt;
    }
    return convert_detail::proven<std::size_t>(i); // range proven by the compare above
}

template <class T> [[nodiscard]] constexpr T element_or(std::span<const T> s, Index i, T fallback = T{}) noexcept
{
    const auto k = index_in(i, s.size());
    return k ? s[*k] : fallback;
}

// gsl::at: a checked subscript over any sized random-access range.
template <std::ranges::random_access_range R>
    requires std::ranges::sized_range<R>
[[nodiscard]] constexpr decltype(auto) at(R&& r, Index i) noexcept
{
    const auto k = index_in(i, std::ranges::size(r));
    if (!k)
    {
        convert_detail::violated("at: index out of range");
    }
    return std::ranges::begin(r)[narrow<std::ranges::range_difference_t<R>>(*k)];
}

// A cast-free signed-reach guard: base - back, only when it stays in [0, limit).
[[nodiscard]] constexpr std::optional<std::size_t> offset_back(std::size_t base, std::size_t back, std::size_t limit) noexcept
{
    if (back > base || base - back >= limit)
    {
        return std::nullopt;
    }
    return base - back;
}

// Power-of-two ring: the mask proves the range.
template <std::size_t Capacity>
    requires(std::has_single_bit(Capacity))
[[nodiscard]] constexpr std::size_t ring_slot(std::uint64_t position) noexcept
{
    return convert_detail::proven<std::size_t>(position & (Capacity - 1U));
}

// Arbitrary period: position % period is < period <= SIZE_MAX; a zero period traps.
[[nodiscard]] constexpr std::size_t ring_slot(std::uint64_t position, std::size_t period) noexcept
{
    if (period == 0U)
    {
        convert_detail::violated("ring_slot: zero period");
    }
    return convert_detail::proven<std::size_t>(position % period);
}

template <std::unsigned_integral Wide> [[nodiscard]] constexpr std::size_t clamp_to_size(Wide count, std::size_t limit) noexcept
{
    return std::cmp_less(count, limit) ? convert_detail::proven<std::size_t>(count) : limit; // compare proves range
}

template <std::random_access_iterator It> [[nodiscard]] constexpr It advance_by(It it, std::size_t n) noexcept
{
    return it + narrow<std::iter_difference_t<It>>(n);
}

template <std::unsigned_integral T> [[nodiscard]] constexpr T ceil_div(T n, T d) noexcept
{
    return static_cast<T>(n / d + (n % d != 0U ? T{1} : T{0}));
}

// ------------------------------------------------------------- integer <-> Real

// Exact: only checked when the source type can exceed 2^digits(Real) (e.g. a 64-bit
// sample index). Small counts fold with no check.
template <Integer I> [[nodiscard]] constexpr Real as_real(I n) noexcept
{
    if constexpr (std::numeric_limits<I>::digits > std::numeric_limits<Real>::digits)
    {
        constexpr std::uintmax_t limit = std::uintmax_t{1} << std::numeric_limits<Real>::digits;
        if (std::cmp_greater(n, limit) || (std::is_signed_v<I> && std::cmp_less(n, -static_cast<std::intmax_t>(limit))))
        {
            convert_detail::violated("as_real: not exactly representable");
        }
    }
    return static_cast<Real>(n);
}

// Documented lossy: an absolute sample index past 2^24 rounds. Use only where the loss is
// intended and understood.
template <Integer I> [[nodiscard]] constexpr Real approx_real(I n) noexcept
{
    return static_cast<Real>(n);
}

template <std::floating_point R, std::unsigned_integral N> [[nodiscard]] constexpr R mean(R sum, N count, R if_empty = R{}) noexcept
{
    return count == 0U ? if_empty : sum / as_real(count);
}

// std::llround (not lround: lround is 32-bit long on Win64 and the M33).
template <Integer To, std::floating_point F> [[nodiscard]] inline To round_to(F x) noexcept
{
    return narrow<To>(std::llround(x));
}

template <Integer To, std::floating_point F> [[nodiscard]] inline To floor_to(F x) noexcept
{
    const F f = std::floor(x);
    if (!(f >= static_cast<F>(std::numeric_limits<To>::min()) && f < static_cast<F>(std::numeric_limits<To>::max())))
    {
        convert_detail::violated("floor_to: out of range");
    }
    return static_cast<To>(f);
}

// ------------------------------------------------------------- enums

template <class E>
concept ScopedEnum = std::is_scoped_enum_v<E>;

// Specialise next to the enum: `template <> struct enum_traits<E> { static constexpr std::array values{...}; };`
template <ScopedEnum E> struct enum_traits;

template <ScopedEnum E>
    requires std::unsigned_integral<std::underlying_type_t<E>>
[[nodiscard]] constexpr std::size_t enum_index(E e) noexcept
{
    return widen<std::size_t>(std::to_underlying(e));
}

template <ScopedEnum E> [[nodiscard]] constexpr bool is_enumerator(E e) noexcept
{
    return std::ranges::find(enum_traits<E>::values, e) != std::ranges::end(enum_traits<E>::values);
}

// No from-underlying cast: the value is found in the declared set, not manufactured.
template <ScopedEnum E> [[nodiscard]] constexpr std::optional<E> enum_from(std::underlying_type_t<E> v) noexcept
{
    for (const E e : enum_traits<E>::values)
    {
        if (std::to_underlying(e) == v)
        {
            return e;
        }
    }
    return std::nullopt;
}

// A fixed table indexed by a scoped enum, without a cast at the call site.
template <ScopedEnum E, class T> struct EnumArray
{
    std::array<T, enum_traits<E>::values.size()> data{};

    [[nodiscard]] constexpr T& operator[](E e) noexcept { return data[enum_index(e)]; }

    [[nodiscard]] constexpr const T& operator[](E e) const noexcept { return data[enum_index(e)]; }
};

// Opt an enum into the bitmask operators with `template <> inline constexpr bool
// enable_bitmask<E> = true;`. ADL finds these only for enums in modem::common; a family
// enum brings them in with `using ::modem::common::operator|;` etc. in its namespace.
template <class E> inline constexpr bool enable_bitmask = false;

template <class E>
concept BitmaskEnum = ScopedEnum<E> && enable_bitmask<E> && std::unsigned_integral<std::underlying_type_t<E>>;

template <BitmaskEnum E> [[nodiscard]] constexpr E operator|(E a, E b) noexcept
{
    return static_cast<E>(std::to_underlying(a) | std::to_underlying(b));
}

template <BitmaskEnum E> [[nodiscard]] constexpr E operator&(E a, E b) noexcept
{
    return static_cast<E>(std::to_underlying(a) & std::to_underlying(b));
}

template <BitmaskEnum E> [[nodiscard]] constexpr E operator~(E a) noexcept
{
    return static_cast<E>(~std::to_underlying(a));
}

} // namespace modem::common
