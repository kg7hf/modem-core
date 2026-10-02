// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/m110a/body_waveform.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

// =============================================================================
// transmitter.hpp - transmit-side orchestrator (plan + render a transmission)
// =============================================================================
// body_transmission_plan() sizes a whole DTE-octet transmission (preamble, EOM,
// flush, padding); generate_body_transmission_audio() renders it to 48 kHz audio,
// allocation-free. The mirror of the receive chain, over body_waveform.
// Teaching walkthrough: modem/m110a/transmitter-and-dsp-explainer.md
// =============================================================================

#include "modem/common/status.hpp"
namespace modem::m110a
{

using ::modem::common::MutableBitSpan;
using ::modem::common::MutableSampleSpan;
using ::modem::common::Result;
using ::modem::common::Status;

struct BodyTransmissionPlan
{
    BodyBlockPlan block{};
    std::size_t payload_bits{};
    std::size_t framed_bits{};
    std::size_t body_blocks{};
    std::size_t preamble_symbols{};
    std::size_t transmitted_symbols{};
    std::size_t audio_samples{};
};

struct BodyTransmissionScratch
{
    MutableBitSpan framed_bits{};
    MutableBitSpan transmitted_tribits{};
    MutableBitSpan coded_bits{};
    MutableBitSpan interleaver_matrix{};
    MutableBitSpan interleaved_bits{};
};

// Plans one complete DTE-octet transmission, including the body preamble,
// EOM, coder flush, and final interleaver-matrix padding. User octets enter
// the modem least-significant bit first.
[[nodiscard]] Result<BodyTransmissionPlan> body_transmission_plan(BodyMode mode, std::size_t payload_bytes) noexcept;

// Generates a complete mono 48 kHz waveform without allocating memory. This
// is the common path for host reference audio and embedded DAC/codec backends.
[[nodiscard]] Status generate_body_transmission_audio(const BodyTransmissionPlan& plan, std::span<const std::uint8_t> payload, BodyTransmissionScratch scratch, MutableSampleSpan audio) noexcept;

} // namespace modem::m110a
