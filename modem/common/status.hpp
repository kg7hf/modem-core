// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include <cstdint>
#include <expected>
#include <utility>

namespace modem::common
{

enum class [[nodiscard("check the StatusCode; a dropped error is a silent failure")]] StatusCode : std::uint8_t
{
    ok = 0,
    invalid_argument,
    invalid_configuration,
    buffer_too_small,
    unavailable,
    io_error,
    timeout,
    busy,
    internal_error
};

struct [[nodiscard]] Status
{
    StatusCode code{StatusCode::ok};
    const char* message{"ok"};

    [[nodiscard]] constexpr bool is_ok() const noexcept { return code == StatusCode::ok; }

    [[nodiscard]] static constexpr Status success() noexcept { return {}; }
};

using CriticalErrorHandler = void (*)(Status status) noexcept;

// Install during single-threaded startup, before any Result is consumed.
// A handler may record diagnostics, but it must return; critical_error() then
// traps so a failed Result can never be observed as a default-constructed value.
void set_critical_error_handler(CriticalErrorHandler handler) noexcept;
[[noreturn]] void critical_error(Status status) noexcept;

// A fallible T: a value, or the Status explaining its absence. This is
// std::expected<T, Status> underneath (standard, constexpr, trivially copyable when
// T is), wrapped for ONE reason the standard type does not give us: value() on an
// error routes through the installable critical_error() handler. std::expected::value()
// under -fno-exceptions just aborts with no diagnostic and no Status payload; ours lets
// a handler record the failure before it traps. Same value-or-error contract, plus the
// safety net a mission-critical build wants.
//   return value;          -> a value
//   return Status{code,..} -> the reason there isn't one (wrapped in std::unexpected)
template <typename T> class [[nodiscard]] Result
{
public:
    using value_type = T;
    using error_type = Status;

    constexpr Result(T value) noexcept : mResult{std::move(value)} {}

    constexpr Result(Status error) noexcept : mResult{std::unexpected(std::move(error))} {}

    [[nodiscard]] constexpr bool has_value() const noexcept { return mResult.has_value(); }

    [[nodiscard]] constexpr explicit operator bool() const noexcept { return mResult.has_value(); }

    [[nodiscard]] constexpr T& value() & noexcept
    {
        if (!mResult)
        {
            critical_error(mResult.error());
        }

        return *mResult;
    }

    [[nodiscard]] constexpr const T& value() const& noexcept
    {
        if (!mResult)
        {
            critical_error(mResult.error());
        }

        return *mResult;
    }

    [[nodiscard]] constexpr T&& value() && noexcept
    {
        if (!mResult)
        {
            critical_error(mResult.error());
        }

        return *std::move(mResult);
    }

    // Safe on success (returns an ok Status), unlike std::expected::error() whose
    // precondition is !has_value(). Status is a trivial 16-byte value, so by-value is
    // free, and this matches the contract every .error() call site already relies on.
    [[nodiscard]] constexpr Status error() const noexcept { return mResult.has_value() ? Status::success() : mResult.error(); }

private:
    std::expected<T, Status> mResult;
};

} // namespace modem::common
