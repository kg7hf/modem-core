// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "modem/common/types.hpp"
#include "modem/common/waveform/waveform.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

// =============================================================================
// interleaver.hpp - body matrix interleaver (hard + soft)
// =============================================================================
// Reorders coded bits so a channel fade becomes scattered (not bursty) coded-bit
// errors. This is the MIL-STD-188-110B body's rows x columns matrix: load down
// columns, fetch across rows. Deinterleave is the exact inverse of interleave;
// the soft variants carry the turbo loop's LLRs both directions
// (interleave_soft(deinterleave_soft(x)) == x): deinterleave_soft hands the
// demapper's soft LLRs to the decoder in encoder order, and interleave_soft
// returns the decoder's a-priori to the demapper in channel order. This stage only
// REORDERS those LLRs; it never rescales them. body_interleave is implemented here;
// the deinterleavers and the soft variants are declared, and their bodies arrive
// with the receiver series. Teaching walkthrough:
//   modem/m110a/demapper-and-deinterleaver-explainer.md
// =============================================================================

namespace modem::m110a
{

using ::modem::common::BitSpan;
using ::modem::common::InterleaverSpec;
using ::modem::common::MutableBitSpan;
using ::modem::common::Result;
using ::modem::common::Status;

[[nodiscard]] Status body_interleave(BitSpan input, MutableBitSpan scratch, MutableBitSpan output, const InterleaverSpec& spec) noexcept;
[[nodiscard]] Status body_deinterleave(BitSpan input, MutableBitSpan scratch, MutableBitSpan output, const InterleaverSpec& spec) noexcept;
[[nodiscard]] Status body_deinterleave_soft(std::span<const float> input, std::span<float> scratch, std::span<float> output, const InterleaverSpec& spec) noexcept;
// Exact inverse of body_deinterleave_soft (the transmit load/fetch walk on
// soft values): a turbo equalizer re-interleaves the decoder's a-priori
// back into channel order. interleave_soft(deinterleave_soft(x)) == x.
[[nodiscard]] Status body_interleave_soft(std::span<const float> input, std::span<float> scratch, std::span<float> output, const InterleaverSpec& spec) noexcept;

[[nodiscard]] Result<std::size_t> body_load_address(std::size_t input_index, const InterleaverSpec& spec) noexcept;
[[nodiscard]] Result<std::size_t> body_fetch_address(std::size_t output_index, const InterleaverSpec& spec) noexcept;

} // namespace modem::m110a
