// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

/*
================================================================================
waveform.cpp - the waveform descriptor tables (mode -> geometry authority)
================================================================================
The single source of truth mapping a WaveformConfig (family + rate + interleave) to
a WaveformDescriptor (modulation, FEC profile, bits/symbol, spreading, frame
geometry) and an InterleaverSpec. 16 rows: 7 serial-tone body modes (75..4800) plus
Appendix C/F high-rate modes. Do NOT duplicate rate/modulation decisions elsewhere -
descriptor_for()/interleaver_for()/validate() are the authority. consteval self-
checks make a malformed table or interleaver a COMPILE error.
  Body interleaver: 40xN matrix (load 9 / fetch 17; 75 bps is 20x36 / 10x9, step 7).
  High-rate interleaver: multiplicative permutation with a depth scale.
Teaching walkthrough: modem/m110a/body-waveform-and-scrambler-explainer.md
================================================================================
*/

#include "modem/common/waveform/waveform.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <variant>

namespace modem::common
{
// MIL-STD-188-110B 5.3.2 serial-tone modes and Appendices C/F. These
// descriptors are configuration authority for later TX/RX stages; do not
// duplicate rate/modulation decisions in platform code.
// Serial-tone FEC profiles below: 75/600/1200/2400 = rate 1/2 (75 then Walsh-
// spreads, orthogonal_spreading_factor 32); 150 = repeat4 and 300 = repeat2 (the
// two REPETITION modes, summed on receive); 4800 = uncoded.
constexpr std::array<WaveformDescriptor, 16> descriptors{{
    {WaveformFamily::serial_tone, DataRate::bps75, Modulation::psk8, FecProfile::k7_rate_half, 2400U, 3U, 2U, 32U, 1U, 0U, 0U},
    {WaveformFamily::serial_tone, DataRate::bps150, Modulation::psk8, FecProfile::k7_rate_half_repeat4, 2400U, 3U, 1U, 1U, 1U, 0U, 0U},
    {WaveformFamily::serial_tone, DataRate::bps300, Modulation::psk8, FecProfile::k7_rate_half_repeat2, 2400U, 3U, 1U, 1U, 1U, 0U, 0U},
    {WaveformFamily::serial_tone, DataRate::bps600, Modulation::psk8, FecProfile::k7_rate_half, 2400U, 3U, 1U, 1U, 1U, 0U, 0U},
    {WaveformFamily::serial_tone, DataRate::bps1200, Modulation::psk8, FecProfile::k7_rate_half, 2400U, 3U, 2U, 1U, 1U, 0U, 0U},
    {WaveformFamily::serial_tone, DataRate::bps2400, Modulation::psk8, FecProfile::k7_rate_half, 2400U, 3U, 3U, 1U, 1U, 0U, 0U},
    {WaveformFamily::serial_tone, DataRate::bps4800, Modulation::psk8, FecProfile::uncoded, 2400U, 3U, 3U, 1U, 1U, 0U, 0U},
    {WaveformFamily::appendix_c, DataRate::bps3200, Modulation::qpsk, FecProfile::k7_tail_biting_punctured_3_4, 2400U, 2U, 2U, 1U, 1U, 256U, 31U},
    {WaveformFamily::appendix_c, DataRate::bps4800, Modulation::psk8, FecProfile::k7_tail_biting_punctured_3_4, 2400U, 3U, 3U, 1U, 1U, 256U, 31U},
    {WaveformFamily::appendix_c, DataRate::bps6400, Modulation::qam16, FecProfile::k7_tail_biting_punctured_3_4, 2400U, 4U, 4U, 1U, 1U, 256U, 31U},
    {WaveformFamily::appendix_c, DataRate::bps8000, Modulation::qam32, FecProfile::k7_tail_biting_punctured_3_4, 2400U, 5U, 5U, 1U, 1U, 256U, 31U},
    {WaveformFamily::appendix_c, DataRate::bps9600, Modulation::qam64, FecProfile::k7_tail_biting_punctured_3_4, 2400U, 6U, 6U, 1U, 1U, 256U, 31U},
    {WaveformFamily::appendix_f, DataRate::bps9600, Modulation::psk8, FecProfile::k7_tail_biting_punctured_3_4, 2400U, 3U, 3U, 1U, 2U, 256U, 31U},
    {WaveformFamily::appendix_f, DataRate::bps12800, Modulation::qam16, FecProfile::k7_tail_biting_punctured_3_4, 2400U, 4U, 4U, 1U, 2U, 256U, 31U},
    {WaveformFamily::appendix_f, DataRate::bps16000, Modulation::qam32, FecProfile::k7_tail_biting_punctured_3_4, 2400U, 5U, 5U, 1U, 2U, 256U, 31U},
    {WaveformFamily::appendix_f, DataRate::bps19200, Modulation::qam64, FecProfile::k7_tail_biting_punctured_3_4, 2400U, 6U, 6U, 1U, 2U, 256U, 31U},
}};

static_assert(descriptors.size() == 16U);
static_assert(descriptors.front().data_rate == DataRate::bps75);
static_assert(descriptors.front().information_bits_per_channel_symbol == 2U);
static_assert(descriptors.front().orthogonal_spreading_factor == 32U);
static_assert(descriptors.back().family == WaveformFamily::appendix_f && descriptors.back().data_rate == DataRate::bps19200);

constexpr std::array<std::uint32_t, 6> depth_scale{1U, 3U, 9U, 18U, 36U, 72U};

struct HighRateInterleaverRow
{
    DataRate rate;
    std::uint32_t input_base;
    std::uint32_t encoded_base;
    std::array<std::uint32_t, 6> increments;
};

constexpr std::array<HighRateInterleaverRow, 5> appendix_c_interleavers{{
    {DataRate::bps3200, 384U, 512U, {97U, 229U, 805U, 1393U, 3281U, 6985U}},
    {DataRate::bps4800, 576U, 768U, {145U, 361U, 1045U, 2089U, 5137U, 10273U}},
    {DataRate::bps6400, 768U, 1024U, {189U, 481U, 1393U, 3281U, 6985U, 11141U}},
    {DataRate::bps8000, 960U, 1280U, {201U, 601U, 1741U, 3481U, 8561U, 14441U}},
    {DataRate::bps9600, 1152U, 1536U, {229U, 805U, 2089U, 5137U, 10273U, 17329U}},
}};

// The printed Appendix F tables have a known vertical-row displacement. These
// reconstructed rows were checked against Table F-IV and are mechanically
// constrained below to the standard's two-ISB relationship with Appendix C.
constexpr std::array<HighRateInterleaverRow, 4> appendix_f_interleavers{{
    {DataRate::bps9600, 1152U, 1536U, {229U, 805U, 2089U, 5137U, 10273U, 17329U}},
    {DataRate::bps12800, 1536U, 2048U, {363U, 1303U, 3281U, 6985U, 11141U, 28007U}},
    {DataRate::bps16000, 1920U, 2560U, {453U, 1343U, 3481U, 8561U, 14441U, 34907U}},
    {DataRate::bps19200, 2304U, 3072U, {481U, 1393U, 5137U, 10273U, 17329U, 47069U}},
}};

static constexpr std::size_t depth_index(HighRateInterleave depth) noexcept
{
    return static_cast<std::size_t>(depth);
}

constexpr const WaveformDescriptor* find_descriptor(WaveformFamily family, DataRate rate) noexcept
{
    for (const auto& descriptor : descriptors)
    {
        if (descriptor.family == family && descriptor.data_rate == rate)
        {
            return &descriptor;
        }
    }

    return nullptr;
}

template <std::size_t Size> constexpr const HighRateInterleaverRow* find_interleaver_row(const std::array<HighRateInterleaverRow, Size>& rows, DataRate rate) noexcept
{
    for (const auto& row : rows)
    {
        if (row.rate == rate)
        {
            return &row;
        }
    }

    return nullptr;
}

consteval bool descriptors_are_unique_and_well_formed()
{
    for (std::size_t left = 0U; left < descriptors.size(); ++left)
    {
        const auto& descriptor = descriptors[left];
        const std::uint8_t expected_bits = descriptor.modulation == Modulation::qpsk    ? 2U
                                           : descriptor.modulation == Modulation::psk8  ? 3U
                                           : descriptor.modulation == Modulation::qam16 ? 4U
                                           : descriptor.modulation == Modulation::qam32 ? 5U
                                           : descriptor.modulation == Modulation::qam64 ? 6U
                                                                                        : 1U;

        if (descriptor.bits_per_symbol != expected_bits)
        {
            return false;
        }

        for (std::size_t right = left + 1U; right < descriptors.size(); ++right)
        {
            if (descriptor.family == descriptors[right].family && descriptor.data_rate == descriptors[right].data_rate)
            {
                return false;
            }
        }
    }

    return true;
}

template <std::size_t Size> consteval bool interleaver_rows_are_valid(WaveformFamily family, const std::array<HighRateInterleaverRow, Size>& rows)
{
    for (const auto& row : rows)
    {
        if (find_descriptor(family, row.rate) == nullptr || row.encoded_base != (row.input_base * 4U) / 3U)
        {
            return false;
        }

        for (std::size_t depth = 0U; depth < depth_scale.size(); ++depth)
        {
            if (std::gcd(row.increments[depth], row.encoded_base * depth_scale[depth]) != 1U)
            {
                return false;
            }
        }
    }

    for (const auto& descriptor : descriptors)
    {
        if (descriptor.family == family && find_interleaver_row(rows, descriptor.data_rate) == nullptr)
        {
            return false;
        }
    }

    return true;
}

static_assert(depth_scale.size() == static_cast<std::size_t>(HighRateInterleave::very_long) + 1U);
static_assert(descriptors_are_unique_and_well_formed());
static_assert(interleaver_rows_are_valid(WaveformFamily::appendix_c, appendix_c_interleavers));
static_assert(interleaver_rows_are_valid(WaveformFamily::appendix_f, appendix_f_interleavers));
static_assert(appendix_f_interleavers.front().input_base == appendix_c_interleavers.back().input_base);
static_assert(appendix_f_interleavers.front().encoded_base == appendix_c_interleavers.back().encoded_base);

Status validate_structure(const WaveformConfig& config) noexcept
{
    const bool body_selection = std::holds_alternative<BodyInterleave>(config.interleaver);

    if ((config.family == WaveformFamily::serial_tone) != body_selection)
    {
        return {StatusCode::invalid_configuration, "interleaver type does not match waveform family"};
    }

    if (config.family != WaveformFamily::appendix_f && config.channel0_policy != ChannelSidebandPolicy::detect)
    {
        return {StatusCode::invalid_configuration, "sideband policy is only valid for Appendix F"};
    }

    if (config.family == WaveformFamily::serial_tone && config.data_rate == DataRate::bps4800 && std::get<BodyInterleave>(config.interleaver) != BodyInterleave::zero)
    {
        return {StatusCode::invalid_configuration, "4800-bps body waveform requires interleaver bypass"};
    }

    return Status::success();
}

Status validate(const WaveformConfig& config) noexcept
{
    const auto structure_status = validate_structure(config);

    if (!structure_status.is_ok())
    {
        return structure_status;
    }

    return find_descriptor(config.family, config.data_rate) != nullptr ? Status::success() : Status{StatusCode::invalid_configuration, "rate is not legal for waveform family"};
}

Result<WaveformDescriptor> descriptor_for(const WaveformConfig& config) noexcept
{
    const auto status = validate_structure(config);

    if (!status.is_ok())
    {
        return status;
    }

    const auto* descriptor = find_descriptor(config.family, config.data_rate);

    if (descriptor != nullptr)
    {
        return *descriptor;
    }

    return Status{StatusCode::invalid_configuration, "rate is not legal for waveform family"};
}

// -----------------------------------------------------------------------------
// interleaver_for / descriptor_for  (resolve a config to its geometry)
// -----------------------------------------------------------------------------
// 50K view: Turn a WaveformConfig into the InterleaverSpec (or WaveformDescriptor)
//   the encoder/decoder use.
// Detailed view: Body modes -> a 40xN matrix (load 9 / fetch 17; 75 bps 20x36 / 10x9
//   step 7; 4800 bypass). High-rate modes -> a multiplicative permutation whose
//   size/increment scale with the interleave depth. validate() enforces the family/
//   interleaver-type and 4800-bypass rules first.
// 5th-grade view: Look up this mode's shuffling grid (or skip-count) from the table.
// -----------------------------------------------------------------------------
Result<InterleaverSpec> interleaver_for(const WaveformConfig& config) noexcept
{
    const auto status = validate(config);

    if (!status.is_ok())
    {
        return status;
    }

    if (config.family == WaveformFamily::serial_tone)
    {
        const auto depth = std::get<BodyInterleave>(config.interleaver);

        if (depth == BodyInterleave::zero)
        {
            return InterleaverSpec{0U, 0U, 0U, 0U, 0U, 0U, 0U, true};
        }

        const bool is_long = depth == BodyInterleave::long_block;
        std::uint16_t rows = 40U;
        std::uint16_t columns = 0U;
        std::uint8_t load_step = 9U;
        std::uint8_t fetch_step = 17U;

        switch (config.data_rate)
        {
        case DataRate::bps2400:
            columns = is_long ? 576U : 72U;
            break;

        case DataRate::bps1200:
            columns = is_long ? 288U : 36U;
            break;

        case DataRate::bps600:
        case DataRate::bps300:
        case DataRate::bps150:
            columns = is_long ? 144U : 18U;
            break;

        case DataRate::bps75:
            rows = is_long ? 20U : 10U;
            columns = is_long ? 36U : 9U;
            load_step = 7U;
            fetch_step = 7U;
            break;

        default:
            return Status{StatusCode::invalid_configuration, "body interleaver unavailable for rate"};
        }

        const auto size = static_cast<std::uint32_t>(rows) * columns;
        return InterleaverSpec{size, size, 0U, rows, columns, load_step, fetch_step, false};
    }

    const auto depth = std::get<HighRateInterleave>(config.interleaver);
    const auto column = depth_index(depth);

    if (column >= depth_scale.size())
    {
        return Status{StatusCode::invalid_configuration, "invalid high-rate interleave depth"};
    }

    if (config.family == WaveformFamily::appendix_c)
    {
        const auto* row = find_interleaver_row(appendix_c_interleavers, config.data_rate);

        if (row == nullptr)
        {
            return Status{StatusCode::invalid_configuration, "Appendix C interleaver rate missing"};
        }

        return InterleaverSpec{row->input_base * depth_scale[column], row->encoded_base * depth_scale[column], row->increments[column], 0U, 0U, 0U, 0U, false};
    }

    const auto* row = find_interleaver_row(appendix_f_interleavers, config.data_rate);

    if (row == nullptr)
    {
        return Status{StatusCode::invalid_configuration, "Appendix F interleaver rate missing"};
    }

    const auto input_bits = row->input_base * depth_scale[column];
    return InterleaverSpec{input_bits, row->encoded_base * depth_scale[column], row->increments[column], 0U, 0U, 0U, 0U, false};
}

} // namespace modem::common
