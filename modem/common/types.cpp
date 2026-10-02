// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#include "modem/common/types.hpp"

#include <limits>

namespace modem::common
{

Status validate_block(const BasebandBlockView& block) noexcept
{
    if (block.sample_rate_hz == SampleRate{})
    {
        return {StatusCode::invalid_argument, "sample rate must be non-zero"};
    }

    if (block.samples.size() > (std::numeric_limits<std::uint64_t>::max() - block.first_sample_index))
    {
        return {StatusCode::invalid_argument, "sample index overflow"};
    }

    return Status::success();
}

Status validate_and_advance(const BasebandBlockView& block, StreamContinuity& continuity) noexcept
{
    const auto status = validate_block(block);

    if (!status.is_ok())
    {
        return status;
    }

    if (continuity.initialized && continuity.stream_id == block.stream_id && continuity.expected_first_sample != block.first_sample_index && !has_flag(block.flags, BlockFlag::discontinuity))
    {
        return {StatusCode::invalid_argument, "non-contiguous block lacks discontinuity flag"};
    }

    continuity.stream_id = block.stream_id;
    continuity.expected_first_sample = block.first_sample_index + block.samples.size();
    continuity.initialized = true;
    return Status::success();
}

} // namespace modem::common
