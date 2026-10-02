// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#include "modem/common/status.hpp"

namespace modem::common
{

static CriticalErrorHandler critical_error_handler{};

void set_critical_error_handler(CriticalErrorHandler handler) noexcept
{
    critical_error_handler = handler;
}

[[noreturn]] void critical_error(Status status) noexcept
{
    if (critical_error_handler != nullptr)
    {
        critical_error_handler(status);
    }

#if defined(__GNUC__) || defined(__clang__)
    __builtin_trap();
#elif defined(_MSC_VER)
    __debugbreak();
#endif

    for (;;)
    {
    }
}

} // namespace modem::common
