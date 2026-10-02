// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"

#include <cstdint>
#include <variant>

// =============================================================================
// waveform.hpp - waveform mode enums + the descriptor/interleaver table API
// =============================================================================
// The mode vocabulary (DataRate, WaveformFamily, Modulation, FecProfile,
// interleave depths) and the three lookups that resolve a WaveformConfig into a
// WaveformDescriptor and InterleaverSpec (validate / descriptor_for /
// interleaver_for). The authoritative table lives in waveform.cpp.
// Teaching walkthrough: modem/m110a/body-waveform-and-scrambler-explainer.md
// =============================================================================

namespace modem::common
{

// Bit-rate vocabulary shared by all waveform families. The family + FEC a given
// rate uses is decided by the WaveformDescriptor table (waveform.cpp), not by this
// enum: the serial-tone body spans 75-4800, Appendix C/F reuse 3200-19200, and 4800
// exists in both families. Which body rates are repetition vs. Walsh: see FecProfile
// below (75 = Walsh, 150/300 = repetition, 600/1200/2400 = rate 1/2, 4800 = uncoded).
enum class DataRate : std::uint32_t
{
    bps75 = 75,
    bps150 = 150,
    bps300 = 300,
    bps600 = 600,
    bps1200 = 1200,
    bps2400 = 2400,
    bps3200 = 3200,
    bps4800 = 4800,
    bps6400 = 6400,
    bps8000 = 8000,
    bps9600 = 9600,
    bps12800 = 12800,
    bps16000 = 16000,
    bps19200 = 19200
};

enum class WaveformFamily : std::uint8_t
{
    serial_tone,
    appendix_c,
    appendix_f
};
enum class BodyInterleave : std::uint8_t
{
    zero,
    short_block,
    long_block
};
enum class HighRateInterleave : std::uint8_t
{
    ultra_short,
    very_short,
    short_block,
    medium,
    long_block,
    very_long
};
enum class ChannelSidebandPolicy : std::uint8_t
{
    channel0_upper,
    channel0_lower,
    detect
};
enum class Modulation : std::uint8_t
{
    bpsk,
    qpsk,
    psk8,
    qam16,
    qam32,
    qam64
};
enum class FecProfile : std::uint8_t
{
    uncoded,                     // 4800-bps body (hard slice; interleaver bypassed)
    k7_rate_half,                // 600/1200/2400 body; also 75, which then Walsh-spreads
    k7_rate_half_repeat2,        // 300 body: rate 1/2 then x2 coded copies (a REPETITION mode)
    k7_rate_half_repeat4,        // 150 body: rate 1/2 then x4 coded copies (a REPETITION mode)
    k7_tail_biting_punctured_3_4 // Appendix C/F high-rate
};

using InterleaveSelection = std::variant<BodyInterleave, HighRateInterleave>;

struct WaveformConfig
{
    WaveformFamily family{};
    DataRate data_rate{};
    InterleaveSelection interleaver{};
    ChannelSidebandPolicy channel0_policy{ChannelSidebandPolicy::detect};
};

struct WaveformDescriptor
{
    WaveformFamily family{};
    DataRate data_rate{};
    Modulation modulation{};
    FecProfile fec{};
    std::uint32_t symbol_rate_baud{2400U};
    std::uint8_t bits_per_symbol{};
    std::uint8_t information_bits_per_channel_symbol{};
    std::uint8_t orthogonal_spreading_factor{1U};
    std::uint8_t sidebands{1U};
    std::uint16_t data_symbols_per_frame{};
    std::uint16_t probe_symbols_per_frame{};
};

struct InterleaverSpec
{
    std::uint32_t input_bits{};
    std::uint32_t size_bits{};
    std::uint32_t increment{};
    std::uint16_t rows{};
    std::uint16_t columns{};
    std::uint8_t load_row_increment{};
    std::uint8_t fetch_column_decrement{};
    bool bypass{};
};

[[nodiscard]] Status validate(const WaveformConfig& config) noexcept;
[[nodiscard]] Result<WaveformDescriptor> descriptor_for(const WaveformConfig& config) noexcept;
[[nodiscard]] Result<InterleaverSpec> interleaver_for(const WaveformConfig& config) noexcept;

} // namespace modem::common
