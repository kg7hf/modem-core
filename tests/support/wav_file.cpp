// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#include "tests/support/wav_file.hpp"

#include "modem/common/bits.hpp"
#include "modem/common/convert.hpp"

#include <algorithm>
#include <bit>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <string_view>

namespace modem_test
{

using ::modem::common::as_real;
using ::modem::common::bits;
using ::modem::common::low_bits;
using ::modem::common::narrow;
using ::modem::common::Result;
using ::modem::common::round_to;
using ::modem::common::Status;
using ::modem::common::StatusCode;
using ::modem::common::widen;

namespace
{

constexpr std::uint16_t pcm_format = 1U;
constexpr std::uint16_t float_format = 3U;
constexpr std::uint16_t extensible_format = 0xFFFEU; // the real format is the first two bytes of its subformat
constexpr std::size_t riff_header_bytes = 12U;       // "RIFF", the size, "WAVE"
constexpr std::size_t chunk_header_bytes = 8U;       // the tag and the size
constexpr std::size_t format_chunk_bytes = 16U;
constexpr std::size_t subformat_offset = 24U; // inside an extensible format chunk
constexpr std::uint16_t pcm_bits = 16U;
constexpr std::uint16_t float_bits = 32U;
constexpr float pcm_read_scale = 32768.0F;  // -32768 reads as -1
constexpr float pcm_write_scale = 32767.0F; // 1 writes as 32767

std::uint16_t u16_at(std::span<const std::uint8_t> bytes, std::size_t at) noexcept
{
    return narrow<std::uint16_t>(widen<std::uint32_t>(bytes[at]) | (widen<std::uint32_t>(bytes[at + 1U]) << 8U));
}

std::uint32_t u32_at(std::span<const std::uint8_t> bytes, std::size_t at) noexcept
{
    return widen<std::uint32_t>(u16_at(bytes, at)) | (widen<std::uint32_t>(u16_at(bytes, at + 2U)) << 16U);
}

bool tag_at(std::span<const std::uint8_t> bytes, std::size_t at, std::string_view tag) noexcept
{
    return at + tag.size() <= bytes.size() && std::ranges::equal(bytes.subspan(at, tag.size()), tag, [](std::uint8_t byte, char letter) { return byte == std::bit_cast<std::uint8_t>(letter); });
}

void put_u16(std::string& out, std::uint16_t value)
{
    out.push_back(std::bit_cast<char>(bits<8U, std::uint8_t>(value, 0U)));
    out.push_back(std::bit_cast<char>(bits<8U, std::uint8_t>(value, 8U)));
}

void put_u32(std::string& out, std::uint32_t value)
{
    put_u16(out, bits<16U, std::uint16_t>(value, 0U));
    put_u16(out, bits<16U, std::uint16_t>(value, 16U));
}

// The chunks the reader needs, found by walking the file chunk by chunk.
struct Chunks
{
    std::span<const std::uint8_t> format{};
    std::span<const std::uint8_t> data{};
};

Chunks find_chunks(std::span<const std::uint8_t> bytes) noexcept
{
    Chunks chunks;
    std::size_t at = riff_header_bytes;
    while (at + chunk_header_bytes <= bytes.size())
    {
        // A chunk cut short at the end of the file keeps what is there.
        const auto size = std::min<std::size_t>(u32_at(bytes, at + 4U), bytes.size() - at - chunk_header_bytes);
        const auto body = bytes.subspan(at + chunk_header_bytes, size);
        if (tag_at(bytes, at, "fmt "))
        {
            chunks.format = body;
        }
        else if (tag_at(bytes, at, "data"))
        {
            chunks.data = body;
        }
        at += chunk_header_bytes + size + size % 2U; // chunks are padded to an even size
    }
    return chunks;
}

} // namespace

Result<WavAudio> read_wav(const std::string& path)
{
    std::ifstream in(path, std::ios::binary);
    if (!in)
    {
        return Status{StatusCode::io_error, "cannot open the WAV file"};
    }
    const std::vector<char> raw{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    std::vector<std::uint8_t> file(raw.size());
    std::ranges::transform(raw, file.begin(), [](char c) { return std::bit_cast<std::uint8_t>(c); });
    const std::span<const std::uint8_t> bytes{file};

    if (!tag_at(bytes, 0U, "RIFF") || !tag_at(bytes, 8U, "WAVE"))
    {
        return Status{StatusCode::invalid_argument, "not a RIFF WAVE file"};
    }
    const auto chunks = find_chunks(bytes);
    if (chunks.format.size() < format_chunk_bytes || chunks.data.empty())
    {
        return Status{StatusCode::invalid_argument, "the WAV file has no format or no audio"};
    }

    WavAudio audio;
    auto format = u16_at(chunks.format, 0U);
    audio.channels = u16_at(chunks.format, 2U);
    audio.sample_rate_hz = u32_at(chunks.format, 4U);
    const auto block_bytes = u16_at(chunks.format, 12U);
    const auto sample_bits = u16_at(chunks.format, 14U);
    if (format == extensible_format && chunks.format.size() >= subformat_offset + 2U)
    {
        format = u16_at(chunks.format, subformat_offset);
    }
    const bool pcm16 = format == pcm_format && sample_bits == pcm_bits;
    const bool float32 = format == float_format && sample_bits == float_bits;
    if (!pcm16 && !float32)
    {
        return Status{StatusCode::invalid_argument, "only 16-bit PCM and 32-bit float WAV files are read"};
    }
    if (audio.channels == 0U || widen<std::uint32_t>(block_bytes) < widen<std::uint32_t>(audio.channels) * sample_bits / 8U)
    {
        return Status{StatusCode::invalid_argument, "the WAV file's format chunk does not add up"};
    }

    // Each frame holds one sample per channel; the first is kept.
    const std::size_t frames = chunks.data.size() / block_bytes;
    audio.samples.resize(frames);
    for (std::size_t frame = 0U; frame < frames; ++frame)
    {
        const auto at = frame * block_bytes;
        audio.samples[frame] = pcm16 ? as_real(std::bit_cast<std::int16_t>(u16_at(chunks.data, at))) / pcm_read_scale : std::bit_cast<float>(u32_at(chunks.data, at));
    }
    return audio;
}

Status write_wav(const std::string& path, std::span<const float> samples, std::uint32_t sample_rate_hz)
{
    constexpr std::uint16_t mono = 1U;
    constexpr std::uint16_t bytes_per_sample = pcm_bits / 8U;
    const auto data_bytes = narrow<std::uint32_t>(samples.size() * bytes_per_sample);

    std::string out;
    out.reserve(riff_header_bytes + 2U * chunk_header_bytes + format_chunk_bytes + data_bytes);
    out += "RIFF";
    put_u32(out, narrow<std::uint32_t>(riff_header_bytes - chunk_header_bytes + 2U * chunk_header_bytes + format_chunk_bytes) + data_bytes);
    out += "WAVEfmt ";
    put_u32(out, narrow<std::uint32_t>(format_chunk_bytes));
    put_u16(out, pcm_format);
    put_u16(out, mono);
    put_u32(out, sample_rate_hz);
    put_u32(out, sample_rate_hz * bytes_per_sample); // bytes a second
    put_u16(out, bytes_per_sample);                  // bytes a frame
    put_u16(out, pcm_bits);
    out += "data";
    put_u32(out, data_bytes);
    for (const float sample : samples)
    {
        put_u16(out, low_bits<16U, std::uint16_t>(round_to<std::int16_t>(std::clamp(sample, -1.0F, 1.0F) * pcm_write_scale)));
    }

    std::ofstream file(path, std::ios::binary);
    file.write(out.data(), narrow<std::streamsize>(out.size()));
    if (!file)
    {
        return Status{StatusCode::io_error, "cannot write the WAV file"};
    }
    return Status::success();
}

} // namespace modem_test
