// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/real.hpp"
#include "modem/common/status.hpp"
#include "modem/common/units/decibel.hpp"
#include "modem/common/units/sampling.hpp"

#include <complex>
#include <cstddef>
#include <cstdint>
#include <span>

namespace modem::common
{

using Sample = Real;
using IQSample = std::complex<Real>;

using SampleSpan = std::span<const Sample>;
using MutableSampleSpan = std::span<Sample>;
using IQSampleSpan = std::span<const IQSample>;
using MutableIQSampleSpan = std::span<IQSample>;
using BitSpan = std::span<const std::uint8_t>;
using MutableBitSpan = std::span<std::uint8_t>;

enum class BlockFlag : std::uint32_t
{
    none = 0,
    discontinuity = 1U << 0U,
    clipping = 1U << 1U,
    gain_transition = 1U << 2U,
    end_of_stream = 1U << 3U
};

[[nodiscard]] constexpr BlockFlag operator|(BlockFlag lhs, BlockFlag rhs) noexcept
{
    return static_cast<BlockFlag>(static_cast<std::uint32_t>(lhs) | static_cast<std::uint32_t>(rhs));
}

[[nodiscard]] constexpr BlockFlag operator&(BlockFlag lhs, BlockFlag rhs) noexcept
{
    return static_cast<BlockFlag>(static_cast<std::uint32_t>(lhs) & static_cast<std::uint32_t>(rhs));
}

[[nodiscard]] constexpr bool has_flag(BlockFlag flags, BlockFlag flag) noexcept
{
    return (flags & flag) != BlockFlag::none;
}

struct BasebandBlockView
{
    IQSampleSpan samples{};
    SampleRate sample_rate_hz{};
    std::uint64_t first_sample_index{};
    std::uint32_t stream_id{};
    std::uint32_t coherence_group{};
    Db gain_db{};
    BlockFlag flags{BlockFlag::none};
};

struct MutableBasebandBlockView
{
    MutableIQSampleSpan samples{};
    SampleRate sample_rate_hz{};
    std::uint64_t first_sample_index{};
    std::uint32_t stream_id{};
    std::uint32_t coherence_group{};
    Db gain_db{};
    BlockFlag flags{BlockFlag::none};

    [[nodiscard]] BasebandBlockView as_const() const noexcept { return {samples, sample_rate_hz, first_sample_index, stream_id, coherence_group, gain_db, flags}; }
};

struct StreamContinuity
{
    std::uint32_t stream_id{};
    std::uint64_t expected_first_sample{};
    bool initialized{};
};

[[nodiscard]] Status validate_block(const BasebandBlockView& block) noexcept;
[[nodiscard]] Status validate_and_advance(const BasebandBlockView& block, StreamContinuity& continuity) noexcept;

} // namespace modem::common
