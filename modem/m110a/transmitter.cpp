// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

/*
================================================================================
transmitter.cpp - transmit-side orchestrator (octets -> 48 kHz audio)
================================================================================
The top of the TX stack, and the exact mirror of the receive chain. Given a mode and
DTE octets it plans the whole transmission, frames the payload (EOM + flush + block
padding), prepends the preamble, encodes each body block (one continuous BodyEncode
State), and renders the symbol stream to real 48 kHz audio - all into caller-owned
buffers, no allocation. A thin orchestrator over body_waveform's building blocks.
  INPUT  : BodyMode + payload octets (LSB-first) + caller scratch.
  OUTPUT : mono 48 kHz waveform.
  DEPENDS: body_waveform (plan, preamble, encode_body_block, body_tribits_to_audio).
Teaching walkthrough: modem/m110a/transmitter-and-dsp-explainer.md
================================================================================
*/

#include "modem/m110a/transmitter.hpp"

#include "modem/common/bits.hpp"
#include "modem/common/convert.hpp"

#include <limits>

#include "modem/common/status.hpp"
namespace modem::m110a
{

using ::modem::common::bit_at;
using ::modem::common::ceil_div;
using ::modem::common::MutableSampleSpan;
using ::modem::common::Result;
using ::modem::common::Status;
using ::modem::common::StatusCode;

static bool multiply_would_overflow(std::size_t lhs, std::size_t rhs) noexcept
{
    return rhs != 0U && lhs > std::numeric_limits<std::size_t>::max() / rhs;
}

// -----------------------------------------------------------------------------
// body_transmission_plan  (size the whole transmission before touching a buffer)
// -----------------------------------------------------------------------------
// 50K view: Compute every count the render needs: framed bits, body blocks, preamble/
//   transmitted symbols, audio samples.
// Detailed view: From the BodyBlockPlan, payload+EOM+flush rounded up to whole blocks
//   -> framed_bits/body_blocks; + preamble -> transmitted_symbols; x20 -> audio_samples.
//   Every multiply is overflow-guarded, so a too-large payload fails cleanly.
// 5th-grade view: Measure the paper, ink, and tape before you start writing.
// -----------------------------------------------------------------------------
Result<BodyTransmissionPlan> body_transmission_plan(BodyMode mode, std::size_t payload_bytes) noexcept
{
    const auto block_result = body_block_plan(mode);

    if (!block_result)
    {
        return block_result.error();
    }

    constexpr std::size_t bits_per_octet = 8U;
    constexpr std::size_t samples_per_symbol = body_audio_sample_rate_hz / body_symbol_rate_baud;

    if (multiply_would_overflow(payload_bytes, bits_per_octet))
    {
        return Status{StatusCode::invalid_argument, "body transmission payload size overflows"};
    }

    BodyTransmissionPlan plan{};
    plan.block = block_result.value();
    plan.payload_bits = payload_bytes * bits_per_octet;

    if (plan.payload_bits > std::numeric_limits<std::size_t>::max() - body_eom_bits - body_flush_bits)
    {
        return Status{StatusCode::invalid_argument, "body transmission framing size overflows"};
    }

    const auto unpadded_bits = plan.payload_bits + body_eom_bits + body_flush_bits;
    plan.body_blocks = ceil_div(unpadded_bits, plan.block.information_bits);

    if (multiply_would_overflow(plan.body_blocks, plan.block.information_bits))
    {
        return Status{StatusCode::invalid_argument, "body transmission framed size overflows"};
    }

    plan.framed_bits = plan.body_blocks * plan.block.information_bits;
    plan.preamble_symbols = body_preamble_symbols(mode.interleave);

    if (multiply_would_overflow(plan.body_blocks, plan.block.transmitted_symbols))
    {
        return Status{StatusCode::invalid_argument, "body transmission symbol count overflows"};
    }

    const auto body_symbols = plan.body_blocks * plan.block.transmitted_symbols;

    if (body_symbols > std::numeric_limits<std::size_t>::max() - plan.preamble_symbols)
    {
        return Status{StatusCode::invalid_argument, "body transmission symbol count overflows"};
    }

    plan.transmitted_symbols = plan.preamble_symbols + body_symbols;

    if (multiply_would_overflow(plan.transmitted_symbols, samples_per_symbol))
    {
        return Status{StatusCode::invalid_argument, "body transmission audio size overflows"};
    }

    plan.audio_samples = plan.transmitted_symbols * samples_per_symbol;
    return plan;
}

// -----------------------------------------------------------------------------
// generate_body_transmission_audio  (render one transmission to audio)
// -----------------------------------------------------------------------------
// 50K view: Turn payload octets into a complete 48 kHz waveform, allocation-free.
// Detailed view: unpack octets LSB-first -> append_body_eom_and_flush -> generate_body
//   _preamble -> encode_body_block per block (one continuous BodyEncodeState) ->
//   body_tribits_to_audio (RRC + 1800 Hz). Buffers validated against the plan first.
// 5th-grade view: Wrap, encode, and play the note as sound, start to finish.
// -----------------------------------------------------------------------------
Status generate_body_transmission_audio(const BodyTransmissionPlan& plan, std::span<const std::uint8_t> payload, BodyTransmissionScratch scratch, MutableSampleSpan audio) noexcept
{
    constexpr std::size_t bits_per_octet = 8U;

    if (multiply_would_overflow(payload.size(), bits_per_octet) || payload.size() * bits_per_octet != plan.payload_bits)
    {
        return {StatusCode::invalid_argument, "body transmission payload does not match plan"};
    }

    if (scratch.framed_bits.size() < plan.framed_bits || scratch.transmitted_tribits.size() < plan.transmitted_symbols || scratch.coded_bits.size() < plan.block.coded_bits ||
        scratch.interleaver_matrix.size() < plan.block.coded_bits || scratch.interleaved_bits.size() < plan.block.coded_bits || audio.size() < plan.audio_samples)
    {
        return {StatusCode::buffer_too_small, "body transmission buffers do not match plan"};
    }

    std::size_t payload_bit{};

    for (const auto octet : payload)
    {
        for (std::uint8_t bit = 0U; bit < bits_per_octet; ++bit)
        {
            scratch.framed_bits[payload_bit++] = bit_at(octet, bit);
        }
    }

    const auto frame_result = append_body_eom_and_flush(scratch.framed_bits.first(plan.payload_bits), plan.block.information_bits, scratch.framed_bits.first(plan.framed_bits));

    if (!frame_result)
    {
        return frame_result.error();
    }

    if (frame_result.value() != plan.framed_bits)
    {
        return {StatusCode::internal_error, "body transmission framing count mismatch"};
    }

    auto status = generate_body_preamble(plan.block.mode, scratch.transmitted_tribits.first(plan.preamble_symbols));

    if (!status.is_ok())
    {
        return status;
    }

    BodyEncodeState encode_state{};

    for (std::size_t block = 0U; block < plan.body_blocks; ++block)
    {
        const auto information = scratch.framed_bits.subspan(block * plan.block.information_bits, plan.block.information_bits);
        const auto output = scratch.transmitted_tribits.subspan(plan.preamble_symbols + block * plan.block.transmitted_symbols, plan.block.transmitted_symbols);
        status = encode_body_block(information, plan.block, {scratch.coded_bits, scratch.interleaver_matrix, scratch.interleaved_bits}, output, encode_state);

        if (!status.is_ok())
        {
            return status;
        }
    }

    float carrier_phase{};
    return body_tribits_to_audio(scratch.transmitted_tribits.first(plan.transmitted_symbols), audio.first(plan.audio_samples), carrier_phase);
}

} // namespace modem::m110a
