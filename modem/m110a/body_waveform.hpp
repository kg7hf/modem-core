// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "modem/common/types.hpp"
#include "modem/common/units/rates.hpp"
#include "modem/common/waveform/waveform.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

// =============================================================================
// body_waveform.hpp - serial-tone body waveform: plan, preamble, encode/decode,
//                     framing, and audio rendering
// =============================================================================
// The structural backbone of a serial-tone body burst. body_block_plan() computes
// the per-mode geometry (info/coded/data/probe counts) that sizes every receiver
// stage; the encode/decode/soft-metric functions build and parse a block; the
// EOM/flush + pack_body_payload functions frame the payload; BodyAudioStreamModulator
// renders symbols to 48 kHz audio. Constants (symbol rate 2400, segment 480, EOM
// 0x4B65A5B2) are here. The decode-side functions are declared here; their bodies
// arrive with the receiver series. Teaching walkthrough:
//   modem/m110a/body-waveform-and-scrambler-explainer.md
// =============================================================================

namespace modem::m110a
{

using ::modem::common::BitSpan;
using ::modem::common::BodyInterleave;
using ::modem::common::DataRate;
using ::modem::common::IQSampleSpan;
using ::modem::common::MutableBitSpan;
using ::modem::common::MutableIQSampleSpan;
using ::modem::common::MutableSampleSpan;
using ::modem::common::Result;
using ::modem::common::Status;
using ::modem::common::StatusCode;

constexpr std::uint32_t body_symbol_rate_baud = 2400U;
constexpr std::uint32_t body_carrier_hz = 1800U;
constexpr std::uint32_t body_audio_sample_rate_hz = 48000U;
constexpr std::size_t body_preamble_acquisition_prefix_symbols = 9U * 32U;
constexpr std::size_t body_preamble_segment_symbols = 15U * 32U;
constexpr std::uint32_t body_eom_word = 0x4B65A5B2U;
constexpr std::size_t body_eom_bits = 32U;
constexpr std::size_t body_flush_bits = 144U;
// The same three rates as strong types, for the common rate helpers.
inline constexpr ::modem::common::Baud body_symbol_rate{body_symbol_rate_baud};
inline constexpr ::modem::common::SampleRate body_audio_sample_rate{body_audio_sample_rate_hz};
inline constexpr ::modem::common::Frequency body_carrier{body_carrier_hz, ::modem::common::Hz};
// The body carrier's phase advance per 48 kHz sample: two_pi * 1800 / 48000 in Real,
// the shipped operation order (0x3E714639, locked in rates.hpp).
inline constexpr float body_carrier_radians_per_sample = ::modem::common::radians_per_sample(body_carrier, body_audio_sample_rate);
constexpr std::size_t body_audio_samples_per_symbol = ::modem::common::samples_per_symbol(body_audio_sample_rate, body_symbol_rate);

// One carrier period: 1800 Hz at 48 kHz repeats exactly every 80 samples (3 cycles), so the
// transmit modulator and the receive mixer index this table by sample number mod 80 instead of
// calling sin/cos or running an oscillator recurrence. Entry n is exp(j * 2 * pi * 3n / 80), the
// carrier phasor at sample n (3/80 of a cycle per sample); each part is computed in double
// precision from the exactly reduced angle and rounded once to float, written as a hex float
// literal so every compiler and library reads back the same bits. (The parts that are exactly zero
// in real arithmetic at the quadrant entries 20, 40 and 60 are written as exact zeros.)
constexpr std::size_t body_carrier_period_samples = 80U;
static_assert(body_carrier_hz * body_carrier_period_samples == 3U * body_audio_sample_rate_hz, "the 1800 Hz carrier repeats every 80 samples at 48 kHz");
inline constexpr std::array<::modem::common::IQSample, body_carrier_period_samples> body_carrier_table{{
    {0x1.0p+0F, 0x0.0p+0F},
    {0x1.f1da78p-1F, 0x1.de189ap-3F},
    {0x1.c83202p-1F, 0x1.d0e2e2p-2F},
    {0x1.8553eep-1F, 0x1.4c8474p-1F},
    {0x1.2cf23p-1F, 0x1.9e377ap-1F},
    {0x1.87de2ap-2F, 0x1.d906bcp-1F},
    {0x1.4060b6p-3F, 0x1.f9b24ap-1F},
    {-0x1.415e54p-4F, 0x1.fe6bf2p-1F},
    {-0x1.3c6ef4p-2F, 0x1.e6f0e2p-1F},
    {-0x1.0b84eep-1F, 0x1.b48d4p-1F},
    {-0x1.6a09e6p-1F, 0x1.6a09e6p-1F},
    {-0x1.b48d4p-1F, 0x1.0b84eep-1F},
    {-0x1.e6f0e2p-1F, 0x1.3c6ef4p-2F},
    {-0x1.fe6bf2p-1F, 0x1.415e54p-4F},
    {-0x1.f9b24ap-1F, -0x1.4060b6p-3F},
    {-0x1.d906bcp-1F, -0x1.87de2ap-2F},
    {-0x1.9e377ap-1F, -0x1.2cf23p-1F},
    {-0x1.4c8474p-1F, -0x1.8553eep-1F},
    {-0x1.d0e2e2p-2F, -0x1.c83202p-1F},
    {-0x1.de189ap-3F, -0x1.f1da78p-1F},
    {0x0p+0F, -0x1.0p+0F},
    {0x1.de189ap-3F, -0x1.f1da78p-1F},
    {0x1.d0e2e2p-2F, -0x1.c83202p-1F},
    {0x1.4c8474p-1F, -0x1.8553eep-1F},
    {0x1.9e377ap-1F, -0x1.2cf23p-1F},
    {0x1.d906bcp-1F, -0x1.87de2ap-2F},
    {0x1.f9b24ap-1F, -0x1.4060b6p-3F},
    {0x1.fe6bf2p-1F, 0x1.415e54p-4F},
    {0x1.e6f0e2p-1F, 0x1.3c6ef4p-2F},
    {0x1.b48d4p-1F, 0x1.0b84eep-1F},
    {0x1.6a09e6p-1F, 0x1.6a09e6p-1F},
    {0x1.0b84eep-1F, 0x1.b48d4p-1F},
    {0x1.3c6ef4p-2F, 0x1.e6f0e2p-1F},
    {0x1.415e54p-4F, 0x1.fe6bf2p-1F},
    {-0x1.4060b6p-3F, 0x1.f9b24ap-1F},
    {-0x1.87de2ap-2F, 0x1.d906bcp-1F},
    {-0x1.2cf23p-1F, 0x1.9e377ap-1F},
    {-0x1.8553eep-1F, 0x1.4c8474p-1F},
    {-0x1.c83202p-1F, 0x1.d0e2e2p-2F},
    {-0x1.f1da78p-1F, 0x1.de189ap-3F},
    {-0x1.0p+0F, 0x0p+0F},
    {-0x1.f1da78p-1F, -0x1.de189ap-3F},
    {-0x1.c83202p-1F, -0x1.d0e2e2p-2F},
    {-0x1.8553eep-1F, -0x1.4c8474p-1F},
    {-0x1.2cf23p-1F, -0x1.9e377ap-1F},
    {-0x1.87de2ap-2F, -0x1.d906bcp-1F},
    {-0x1.4060b6p-3F, -0x1.f9b24ap-1F},
    {0x1.415e54p-4F, -0x1.fe6bf2p-1F},
    {0x1.3c6ef4p-2F, -0x1.e6f0e2p-1F},
    {0x1.0b84eep-1F, -0x1.b48d4p-1F},
    {0x1.6a09e6p-1F, -0x1.6a09e6p-1F},
    {0x1.b48d4p-1F, -0x1.0b84eep-1F},
    {0x1.e6f0e2p-1F, -0x1.3c6ef4p-2F},
    {0x1.fe6bf2p-1F, -0x1.415e54p-4F},
    {0x1.f9b24ap-1F, 0x1.4060b6p-3F},
    {0x1.d906bcp-1F, 0x1.87de2ap-2F},
    {0x1.9e377ap-1F, 0x1.2cf23p-1F},
    {0x1.4c8474p-1F, 0x1.8553eep-1F},
    {0x1.d0e2e2p-2F, 0x1.c83202p-1F},
    {0x1.de189ap-3F, 0x1.f1da78p-1F},
    {0x0p+0F, 0x1.0p+0F},
    {-0x1.de189ap-3F, 0x1.f1da78p-1F},
    {-0x1.d0e2e2p-2F, 0x1.c83202p-1F},
    {-0x1.4c8474p-1F, 0x1.8553eep-1F},
    {-0x1.9e377ap-1F, 0x1.2cf23p-1F},
    {-0x1.d906bcp-1F, 0x1.87de2ap-2F},
    {-0x1.f9b24ap-1F, 0x1.4060b6p-3F},
    {-0x1.fe6bf2p-1F, -0x1.415e54p-4F},
    {-0x1.e6f0e2p-1F, -0x1.3c6ef4p-2F},
    {-0x1.b48d4p-1F, -0x1.0b84eep-1F},
    {-0x1.6a09e6p-1F, -0x1.6a09e6p-1F},
    {-0x1.0b84eep-1F, -0x1.b48d4p-1F},
    {-0x1.3c6ef4p-2F, -0x1.e6f0e2p-1F},
    {-0x1.415e54p-4F, -0x1.fe6bf2p-1F},
    {0x1.4060b6p-3F, -0x1.f9b24ap-1F},
    {0x1.87de2ap-2F, -0x1.d906bcp-1F},
    {0x1.2cf23p-1F, -0x1.9e377ap-1F},
    {0x1.8553eep-1F, -0x1.4c8474p-1F},
    {0x1.c83202p-1F, -0x1.d0e2e2p-2F},
    {0x1.f1da78p-1F, -0x1.de189ap-3F},
}};
static_assert(body_carrier_table[0] == ::modem::common::IQSample{1.0F, 0.0F});
static_assert(body_carrier_table[1] == ::modem::common::IQSample{0x1.f1da78p-1F, 0x1.de189ap-3F});
static_assert(body_carrier_table[27] == ::modem::common::IQSample{0x1.fe6bf2p-1F, 0x1.415e54p-4F});
static_assert(body_carrier_table[79] == ::modem::common::IQSample{0x1.f1da78p-1F, -0x1.de189ap-3F});
static_assert(body_carrier_table[20].imag() == -1.0F && body_carrier_table[40].real() == -1.0F && body_carrier_table[60].imag() == 1.0F);
static_assert(body_carrier_table[20].real() == 0.0F && body_carrier_table[40].imag() == 0.0F && body_carrier_table[60].real() == 0.0F, "the quadrant entries are exact");
// The carrier phase in radians, wrapped to [0, 2 pi), for a table index and back (the modulator's
// start phase and the phase it hands to the next call).
inline constexpr float body_carrier_table_radians = ::modem::common::two_pi / 80.0F;
constexpr std::size_t body_audio_shaping_span_symbols = 16U;
constexpr std::size_t body_audio_shaping_taps = 2U * body_audio_shaping_span_symbols * body_audio_samples_per_symbol + 1U;

struct BodyMode
{
    DataRate data_rate{};
    BodyInterleave interleave{};
};

struct BodyDesignators
{
    std::uint8_t d1{};
    std::uint8_t d2{};
};

enum class BodyDesignatorState : std::uint8_t
{
    supported,
    recognized_unsupported,
    invalid
};

struct BodyModeRecognition
{
    BodyDesignatorState state{BodyDesignatorState::invalid};
    BodyMode mode{};
};

struct BodyBlockPlan
{
    BodyMode mode{};
    BodyDesignators designators{};
    std::size_t information_bits{};
    std::size_t coded_bits{};
    std::size_t data_channel_symbols{};
    std::size_t transmitted_symbols{};
    std::size_t unknown_symbols_per_probe{};
    std::size_t known_symbols_per_probe{};
    std::uint8_t information_bits_per_channel_symbol{};
    // Coded-pair copies emitted per information bit: 1 for 75/600/1200/2400, 2 for
    // the 300 repetition mode, 4 for the 150 repetition mode. 4800 is uncoded and
    // leaves this 0. body_block_soft_metrics sums the repeated copies on receive.
    std::size_t fec_pair_repetitions{};

    // One data frame: the unknown data symbols and the probe that follows them.
    [[nodiscard]] constexpr std::size_t frame_symbols() const noexcept { return unknown_symbols_per_probe + known_symbols_per_probe; }
};

struct BodyEncodeScratch
{
    MutableBitSpan coded{};
    MutableBitSpan interleaver_matrix{};
    MutableBitSpan interleaved{};
};

struct BodyEncodeState
{
    // MIL-STD-188-110B routes UNKNOWN DATA, EOM, and FLUSH through one
    // continuous K=7 encoder. Zero is our transmission-start convention;
    // the standard does not specify an initial encoder register value.
    std::uint8_t fec_state{};
};

struct BodyDecodeScratch
{
    std::span<float> interleaved_soft{};
    std::span<float> interleaver_matrix{};
    std::span<float> coded_soft{};
    std::span<float> rate_half_soft{};
    MutableBitSpan survivors{};
};

struct BodyDecodeState
{
    std::uint8_t fec_state{};
};

// The designator table lives in the header (constexpr) so body_block_plan can be
// folded at compile time: the geometry table below is also the high-water mark for
// every per-block receive buffer (see body_scratch_max), proved with static_assert.
struct DesignatorRow
{
    DataRate rate;
    BodyDesignators short_code;
    BodyDesignators long_code;
};

inline constexpr std::array<DesignatorRow, 7> designator_rows{{
    {DataRate::bps75, {7U, 5U}, {5U, 5U}},
    {DataRate::bps150, {7U, 4U}, {5U, 4U}},
    {DataRate::bps300, {6U, 7U}, {4U, 7U}},
    {DataRate::bps600, {6U, 6U}, {4U, 6U}},
    {DataRate::bps1200, {6U, 5U}, {4U, 5U}},
    {DataRate::bps2400, {6U, 4U}, {4U, 4U}},
    {DataRate::bps4800, {7U, 6U}, {7U, 6U}},
}};

[[nodiscard]] constexpr const DesignatorRow* find_designator_row(DataRate rate) noexcept
{
    for (const auto& row : designator_rows)
    {
        if (row.rate == rate)
        {
            return &row;
        }
    }

    return nullptr;
}

[[nodiscard]] constexpr Result<BodyDesignators> body_designators(BodyMode mode) noexcept
{
    const auto* row = find_designator_row(mode.data_rate);

    if (row == nullptr || (mode.data_rate == DataRate::bps4800 && mode.interleave != BodyInterleave::zero))
    {
        return Status{StatusCode::invalid_configuration, "unsupported body mode"};
    }

    return mode.interleave == BodyInterleave::long_block ? row->long_code : row->short_code;
}

// body_block_plan  (THE per-mode geometry - read this first)
// -----------------------------------------------------------------------------
// 50K view: Fill the BodyBlockPlan that sizes EVERY receiver stage: information/
//   coded bits, data/probe symbols per frame, bits per symbol, FEC repetitions,
//   transmitted symbols per block.
// Detailed view: Short block = 1440 transmitted symbols (0.6 s); long interleave is
//   x8. Frames are 20 data / 20 probe for 150..1200, 32 data / 16 probe for
//   2400/4800; 75 bps is Walsh-spread dibits. FEC: rate 1/2, x2 (300), x4 (150),
//   uncoded (4800). constexpr so the whole table folds at compile time.
// 5th-grade view: Fill in the recipe card for this mode - how many of each kind of
//   symbol - so every other part knows what to expect.
// -----------------------------------------------------------------------------
[[nodiscard]] constexpr Result<BodyBlockPlan> body_block_plan(BodyMode mode) noexcept
{
    const auto designators = body_designators(mode);

    if (!designators)
    {
        return designators.error();
    }

    const bool is_long = mode.interleave == BodyInterleave::long_block;
    const auto multiplier = is_long ? 8U : 1U;
    BodyBlockPlan plan{mode, designators.value()};
    plan.transmitted_symbols = 1440U * multiplier;

    switch (mode.data_rate)
    {
    case DataRate::bps75:
        plan.information_bits = 45U * multiplier;
        plan.coded_bits = 90U * multiplier;
        plan.data_channel_symbols = 45U * multiplier;
        plan.information_bits_per_channel_symbol = 2U;
        plan.fec_pair_repetitions = 1U;
        break;

    case DataRate::bps150:
        plan.information_bits = 90U * multiplier;
        plan.coded_bits = 720U * multiplier;
        plan.data_channel_symbols = plan.coded_bits;
        plan.unknown_symbols_per_probe = 20U;
        plan.known_symbols_per_probe = 20U;
        plan.information_bits_per_channel_symbol = 1U;
        plan.fec_pair_repetitions = 4U;
        break;

    case DataRate::bps300:
        plan.information_bits = 180U * multiplier;
        plan.coded_bits = 720U * multiplier;
        plan.data_channel_symbols = plan.coded_bits;
        plan.unknown_symbols_per_probe = 20U;
        plan.known_symbols_per_probe = 20U;
        plan.information_bits_per_channel_symbol = 1U;
        plan.fec_pair_repetitions = 2U;
        break;

    case DataRate::bps600:
        plan.information_bits = 360U * multiplier;
        plan.coded_bits = 720U * multiplier;
        plan.data_channel_symbols = plan.coded_bits;
        plan.unknown_symbols_per_probe = 20U;
        plan.known_symbols_per_probe = 20U;
        plan.information_bits_per_channel_symbol = 1U;
        plan.fec_pair_repetitions = 1U;
        break;

    case DataRate::bps1200:
        plan.information_bits = 720U * multiplier;
        plan.coded_bits = 1440U * multiplier;
        plan.data_channel_symbols = plan.coded_bits / 2U;
        plan.unknown_symbols_per_probe = 20U;
        plan.known_symbols_per_probe = 20U;
        plan.information_bits_per_channel_symbol = 2U;
        plan.fec_pair_repetitions = 1U;
        break;

    case DataRate::bps2400:
        plan.information_bits = 1440U * multiplier;
        plan.coded_bits = 2880U * multiplier;
        plan.data_channel_symbols = plan.coded_bits / 3U;
        plan.unknown_symbols_per_probe = 32U;
        plan.known_symbols_per_probe = 16U;
        plan.information_bits_per_channel_symbol = 3U;
        plan.fec_pair_repetitions = 1U;
        break;

    case DataRate::bps4800:
        plan.information_bits = 2880U;
        plan.coded_bits = 2880U;
        plan.data_channel_symbols = plan.coded_bits / 3U;
        plan.unknown_symbols_per_probe = 32U;
        plan.known_symbols_per_probe = 16U;
        plan.information_bits_per_channel_symbol = 3U;
        break;

    default:
        return Status{StatusCode::invalid_configuration, "rate is not a serial-tone body mode"};
    }

    return plan;
}

[[nodiscard]] BodyModeRecognition recognize_body_designators(std::uint8_t d1, std::uint8_t d2, bool long_preamble) noexcept;

[[nodiscard]] constexpr std::size_t body_preamble_segments(BodyInterleave interleave) noexcept
{
    return interleave == BodyInterleave::long_block ? 24U : 3U;
}

[[nodiscard]] constexpr std::size_t body_preamble_symbols(BodyInterleave interleave) noexcept
{
    return body_preamble_segments(interleave) * 15U * 32U;
}

// -----------------------------------------------------------------------------
// body_scratch_max  (compile-time high-water marks for per-block receive scratch)
// -----------------------------------------------------------------------------
// Every serial-tone body mode, folded at compile time off body_block_plan, so a
// receiver's per-block scratch is sized to the worst reachable configuration and
// nothing bigger - and the bound is PROVED (static_assert below) rather than
// eyeballed. Add or edit a mode and the numbers move here, at compile time, with
// the build failing the instant a buffer sized to these would overflow.
struct BodyScratchBounds
{
    std::size_t transmitted_symbols{}; // equalized IQ symbols per block
    std::size_t coded_bits{};          // interleaved/coded soft floats per block
    std::size_t information_bits{};    // decoded bits per block
    std::size_t rate_half_soft{};      // rate-1/2 soft floats = information_bits * 2
    std::size_t survivors{};           // Viterbi survivor bytes = information_bits * 64 (coded modes)
};

// Every valid (data_rate, interleave) pairing; the fold below skips any that
// body_designators rejects, so this list is allowed to be permissive.
inline constexpr std::array<BodyMode, 13> body_modes_all{{
    {DataRate::bps75, BodyInterleave::short_block},
    {DataRate::bps75, BodyInterleave::long_block},
    {DataRate::bps150, BodyInterleave::short_block},
    {DataRate::bps150, BodyInterleave::long_block},
    {DataRate::bps300, BodyInterleave::short_block},
    {DataRate::bps300, BodyInterleave::long_block},
    {DataRate::bps600, BodyInterleave::short_block},
    {DataRate::bps600, BodyInterleave::long_block},
    {DataRate::bps1200, BodyInterleave::short_block},
    {DataRate::bps1200, BodyInterleave::long_block},
    {DataRate::bps2400, BodyInterleave::short_block},
    {DataRate::bps2400, BodyInterleave::long_block},
    {DataRate::bps4800, BodyInterleave::zero},
}};

[[nodiscard]] consteval BodyScratchBounds body_scratch_bounds() noexcept
{
    BodyScratchBounds bounds{};

    for (const auto mode : body_modes_all)
    {
        const auto plan_result = body_block_plan(mode);

        if (!plan_result)
        {
            continue;
        }

        const auto& plan = plan_result.value();
        bounds.transmitted_symbols = std::max(bounds.transmitted_symbols, plan.transmitted_symbols);
        bounds.coded_bits = std::max(bounds.coded_bits, plan.coded_bits);
        bounds.information_bits = std::max(bounds.information_bits, plan.information_bits);
        bounds.rate_half_soft = std::max(bounds.rate_half_soft, plan.information_bits * 2U);

        if (plan.mode.data_rate != DataRate::bps4800)
        {
            bounds.survivors = std::max(bounds.survivors, plan.information_bits * 64U);
        }
    }

    return bounds;
}

inline constexpr BodyScratchBounds body_scratch_max = body_scratch_bounds();

// Assert the folded maxima so the numbers are legible here and any drift fails the
// build (long 2400 dominates every field except the uncoded 4800 information run).
static_assert(body_scratch_max.transmitted_symbols == 11520U);
static_assert(body_scratch_max.coded_bits == 23040U);
static_assert(body_scratch_max.information_bits == 11520U);
static_assert(body_scratch_max.rate_half_soft == 23040U);
static_assert(body_scratch_max.survivors == 737280U);
[[nodiscard]] std::uint8_t body_channel_symbol_tribit(unsigned channel_symbol, std::size_t spread_index) noexcept;
[[nodiscard]] std::uint8_t body_75_spread_tribit(std::uint8_t information_dibit, bool exceptional_set, std::size_t spread_index) noexcept;
[[nodiscard]] Status generate_body_preamble(BodyMode mode, MutableBitSpan output) noexcept;
// The last tail_segments segments of the preamble, exactly as
// generate_body_preamble would emit them. A streaming receiver trains on a
// bounded segment-aligned suffix of a long preamble without a buffer for the
// 11,520-symbol whole.
[[nodiscard]] Status generate_body_preamble_tail(BodyMode mode, std::size_t tail_segments, MutableBitSpan output) noexcept;

[[nodiscard]] Status encode_body_block(BitSpan information_bits, const BodyBlockPlan& plan, BodyEncodeScratch scratch, MutableBitSpan transmitted_tribits) noexcept;
[[nodiscard]] Status encode_body_block(BitSpan information_bits, const BodyBlockPlan& plan, BodyEncodeScratch scratch, MutableBitSpan transmitted_tribits, BodyEncodeState& state) noexcept;
[[nodiscard]] Status decode_body_block(IQSampleSpan received_symbols, const BodyBlockPlan& plan, BodyDecodeScratch scratch, MutableBitSpan information_bits) noexcept;
[[nodiscard]] Status decode_body_block(IQSampleSpan received_symbols, const BodyBlockPlan& plan, BodyDecodeScratch scratch, MutableBitSpan information_bits, BodyDecodeState& state) noexcept;
// One block's combined rate-1/2 soft metrics (extract, deinterleave,
// repetition-combine) without the Viterbi pass. The transmit FEC is one
// continuous stream across blocks with flush at the transmission end, so a
// receiver can concatenate these per-block metrics and run a single
// continuous Viterbi over the whole transmission: per-block decoding leaves
// each boundary's trailing bits unterminated and ties the next block to a
// single possibly-wrong state, which measurably concentrates rare-tail
// errors in the ~30 bits straddling every block boundary. Coded rates only.
[[nodiscard]] Status body_block_soft_metrics(IQSampleSpan received_symbols, const BodyBlockPlan& plan, BodyDecodeScratch scratch, std::span<float> rate_half_soft) noexcept;

[[nodiscard]] Result<std::size_t> append_body_eom_and_flush(BitSpan payload, std::size_t block_information_bits, MutableBitSpan framed_bits) noexcept;

// The EOM search on a stream of decoded information bits, for a receiver
// that holds a burst's decisions (the host-side whole-burst decoders; the
// streaming receiver's holdback matcher is its own exact-match path): the
// position of the best match of body_eom_word (MSB-first, as
// append_body_eom_and_flush frames it) and its Hamming distance. The scan
// stops at the first exact match and otherwise keeps the earliest position
// of the smallest distance; a stream too short for one 32-bit window
// reports first_bit = bits.size() and body_eom_bits + 1 errors.
struct BodyEomMatch
{
    std::size_t first_bit{};
    std::size_t bit_errors{body_eom_bits + 1U};
};

[[nodiscard]] std::size_t body_eom_errors_at(BitSpan bits, std::size_t first) noexcept;
[[nodiscard]] BodyEomMatch find_body_eom(BitSpan bits) noexcept;

// The payload's extent under an EOM tolerance: the bits before an accepted
// EOM (within maximum_bit_errors), or every decoded bit when the stream
// carries none - the EOM and flush are never payload.
[[nodiscard]] inline std::size_t body_payload_bits(BitSpan bits, const BodyEomMatch& eom, std::size_t maximum_bit_errors) noexcept
{
    return eom.bit_errors <= maximum_bit_errors ? eom.first_bit : bits.size();
}

// Packs octets[i] from bits[(first_octet + i) * 8 ...] LSB-first: the first
// decoded bit is bit 0 of the octet, the DTE byte order. buffer_too_small when
// the bits do not cover the requested octets.
[[nodiscard]] Status pack_body_payload(BitSpan bits, std::size_t first_octet, MutableBitSpan octets) noexcept;
[[nodiscard]] Status body_tribits_to_iq(BitSpan tribits, MutableIQSampleSpan output) noexcept;
[[nodiscard]] Status body_tribits_to_audio(BitSpan tribits, MutableSampleSpan output, float& carrier_phase_radians) noexcept;

// Stateful, allocation-free pulse shaper for a long transmission rendered in
// codec-sized windows. The caller supplies the output symbols plus up to
// body_audio_shaping_span_symbols of real context on each side. Symbols outside
// that window are treated as zero. Carrier phase is retained across calls.
class BodyAudioStreamModulator
{
public:
    [[nodiscard]] Status initialize(float carrier_phase_radians = 0.0F) noexcept;
    void reset() noexcept;

    // The carrier position as a phase in [0, 2 pi): the table index is the sample number mod 80, three table steps per sample.
    [[nodiscard]] float carrier_phase_radians() const noexcept { return ::modem::common::as_real((3U * mCarrierIndex) % body_carrier_period_samples) * body_carrier_table_radians; }

    [[nodiscard]] Status render_window(BitSpan symbols, std::size_t first_output_symbol, std::size_t output_symbols, MutableSampleSpan output) noexcept;

private:
    std::array<float, body_audio_shaping_taps> mTaps{};
    float mAmplitudeScale{};
    std::size_t mCarrierIndex{}; // sample number mod body_carrier_period_samples
    bool mInitialized{};
};

} // namespace modem::m110a
