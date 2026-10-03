// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// tx_demo - walk a short message through the MIL-STD-188-110A/B serial-tone
//           transmit chain and write the result to a 48 kHz WAV file.
// =============================================================================
// This is the companion program to the Signal Path article "The transmit chain:
// the easy half, done right." It works in the 600 bps, long-interleave mode
// ("600L") and runs in two parts:
//
//   Steps 1-5 trace the FIRST byte by hand through the library's building blocks
//   (bit order, K=7 encoder, BPSK map, randomizer, 8-PSK point), so you can see
//   the numbers the article walks through. A one-byte trace skips the stages it
//   cannot show: the preamble, the interleaver and the probes.
//
//   Step 6 is the real transmitter: generate_body_transmission_audio() runs the
//   whole chain for every byte. Step 7 hashes every symbol it produced; the
//   tx_golden_symbols test pins that hash.
//
//   Build:  cmake -B build && cmake --build build
//   Run:    ./build/tx_demo "Hi"      (writes tx_out.wav)
// =============================================================================

#include "modem/common/bits.hpp"
#include "modem/common/convert.hpp"
#include "modem/common/fec/convolutional_k7.hpp"
#include "modem/common/types.hpp"
#include "modem/m110a/body_waveform.hpp"
#include "modem/m110a/demapper.hpp"
#include "modem/m110a/scrambler.hpp"
#include "modem/m110a/transmitter.hpp"

#include <algorithm>
#include <array>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <vector>

using namespace modem::common;
using namespace modem::m110a;

namespace
{
// The message's characters as octets: std::string holds char, the transmitter takes
// bytes. This is the demo's one boundary cast on the way in.
std::span<const std::uint8_t> as_octets(std::string_view text)
{
    return {reinterpret_cast<const std::uint8_t*>(text.data()), text.size()};
}

// std::ostream writes char. This is the demo's one boundary cast on the way out.
void write_bytes(std::ofstream& out, std::span<const std::uint8_t> bytes)
{
    out.write(reinterpret_cast<const char*>(bytes.data()), narrow<std::streamsize>(bytes.size()));
}

// WAV is little-endian whatever the host, so each value is split into bytes explicitly.
void put_u16(std::ofstream& out, std::uint16_t value)
{
    const std::array<std::uint8_t, 2> bytes{bits<8U, std::uint8_t>(value, 0U), bits<8U, std::uint8_t>(value, 8U)};
    write_bytes(out, bytes);
}

void put_u32(std::ofstream& out, std::uint32_t value)
{
    const std::array<std::uint8_t, 4> bytes{bits<8U, std::uint8_t>(value, 0U), bits<8U, std::uint8_t>(value, 8U), bits<8U, std::uint8_t>(value, 16U), bits<8U, std::uint8_t>(value, 24U)};
    write_bytes(out, bytes);
}

void put_tag(std::ofstream& out, std::string_view tag)
{
    write_bytes(out, as_octets(tag));
}

// Minimal 16-bit PCM mono WAV writer.
void write_wav(const std::string& path, std::span<const float> samples, std::uint32_t rate)
{
    constexpr std::uint32_t header_bytes_after_riff_size = 36U; // the rest of the 44-byte header
    constexpr std::uint32_t fmt_chunk_bytes = 16U;
    constexpr std::uint16_t pcm_format = 1U;
    constexpr std::uint16_t mono = 1U;
    constexpr std::uint16_t bytes_per_sample = 2U;
    constexpr std::uint16_t bits_per_sample = 16U;
    constexpr float full_scale = 32767.0F;
    const auto data_bytes = narrow<std::uint32_t>(samples.size() * bytes_per_sample);

    std::ofstream out(path, std::ios::binary);
    put_tag(out, "RIFF");
    put_u32(out, header_bytes_after_riff_size + data_bytes);
    put_tag(out, "WAVE");
    put_tag(out, "fmt ");
    put_u32(out, fmt_chunk_bytes);
    put_u16(out, pcm_format);
    put_u16(out, mono);
    put_u32(out, rate);
    put_u32(out, rate * bytes_per_sample); // byte rate
    put_u16(out, bytes_per_sample);        // block align
    put_u16(out, bits_per_sample);
    put_tag(out, "data");
    put_u32(out, data_bytes);
    for (const float sample : samples)
    {
        const auto pcm = round_to<std::int16_t>(std::clamp(sample, -1.0F, 1.0F) * full_scale);
        put_u16(out, low_bits<bits_per_sample, std::uint16_t>(pcm));
    }
}

// FNV-1a over the transmitted tribits. They are integers, so the hash is the
// same on every platform, and it pins every bit decision in the chain: framing,
// coding, interleaving, mapping, probes, preamble and scrambling.
std::uint64_t fnv1a(std::span<const std::uint8_t> values)
{
    constexpr std::uint64_t fnv_offset_basis = 14695981039346656037ULL;
    constexpr std::uint64_t fnv_prime = 1099511628211ULL;
    std::uint64_t hash = fnv_offset_basis;
    for (const std::uint8_t value : values)
    {
        hash ^= value;
        hash *= fnv_prime;
    }
    return hash;
}
} // namespace

int main(int argc, char** argv)
{
    const std::span<char*> args(argv, narrow<std::size_t>(argc));
    const std::string message = args.size() > 1U ? args[1] : "Hi";
    if (message.empty())
    {
        std::println(stderr, "usage: tx_demo [message]   (default: \"Hi\")");
        return 1;
    }
    const std::span<const std::uint8_t> payload = as_octets(message);

    std::println("== Transmit chain walkthrough (600 bps, long interleave) ==");
    std::println("message: \"{}\"", message);
    std::println("steps 1-5 trace the first byte by hand; step 6 runs the real transmitter\n");

    // 1. By hand: user octets enter least-significant bit first. The transmitter does
    //    this for every byte inside generate_body_transmission_audio (step 6).
    const std::uint8_t first = payload.front();
    std::print("1. '{}' = 0x{:02X}, LSB-first bits: ", message.front(), first);
    constexpr std::size_t bits_per_octet = 8U;
    std::array<std::uint8_t, bits_per_octet> first_bits{};
    unsigned position = 0U;
    for (auto& bit : first_bits)
    {
        bit = bit_at(first, position++);
        std::print("{}", bit);
    }
    std::print("\n");

    // 2. By hand: the K=7 rate-1/2 convolutional code (generators 133/171 octal),
    //    starting from the all-zero state.
    ConvolutionalEncoderK7 encoder(0U);
    std::print("2. K=7 coded pairs (t1t2): ");
    for (const std::uint8_t bit : first_bits)
    {
        const auto pair = encoder.push(bit);
        std::print("{}{} ", pair.t1, pair.t2);
    }
    std::print("\n");

    // 3. By hand: 600L is BPSK, one coded bit per symbol -> tribit 0 (0 deg) or 4 (180 deg).
    std::println("3. BPSK map: coded 0 -> tribit {} (0 deg), coded 1 -> tribit {} (180 deg)", mapped_tribit(0U, 1U), mapped_tribit(1U, 1U));

    // 4. By hand: scramble on the 8-PSK ring, (tribit + randomizer) mod 8. The randomizer
    //    starts at the standard's load value; the example uses its second tribit.
    BodyDataRandomizer randomizer;
    constexpr std::size_t tribits_shown = 6U;
    std::array<std::uint8_t, tribits_shown> randomizer_tribits{};
    std::print("4. body randomizer tribits: ");
    for (auto& tribit : randomizer_tribits)
    {
        tribit = randomizer.next_tribit();
        std::print("{} ", tribit);
    }
    const std::uint8_t example = randomizer_tribits[1];
    const std::uint8_t tx_tribit = tribit_add(4U, example);
    std::println("\n   example: coded 1 -> tribit 4, + randomizer {} => transmitted {}", example, tx_tribit);

    // 5. By hand: a transmitted tribit is a point on the 8-PSK circle.
    const IQSample point = psk8_symbol(tx_tribit);
    constexpr unsigned degrees_per_tribit = 45U;
    std::println("5. psk8_symbol({}): I={: .3f} Q={: .3f} ({} x {} = {} deg)\n", tx_tribit, point.real(), point.imag(), tx_tribit, degrees_per_tribit, tx_tribit * degrees_per_tribit);

    // 6. The real transmitter, for every byte. Plan every size first, allocate the
    //    buffers here in the caller (the library never allocates), then render: framing
    //    (EOM, flush, padding), the countdown preamble, then per block the encoder,
    //    interleaver, mapping, probes and randomizer, and finally the pulse shaping onto
    //    the 1800 Hz carrier. The preamble goes out first, so the first samples printed
    //    are preamble, not 'H'.
    const BodyMode mode{DataRate::bps600, BodyInterleave::long_block};
    const auto plan_result = body_transmission_plan(mode, payload.size());
    if (!plan_result)
    {
        std::println(stderr, "plan failed");
        return 1;
    }
    const BodyTransmissionPlan& plan = plan_result.value();

    std::vector<std::uint8_t> framed_bits(plan.framed_bits);                 // payload + EOM + flush + padding
    std::vector<std::uint8_t> transmitted_tribits(plan.transmitted_symbols); // preamble + body: the symbol stream
    std::vector<std::uint8_t> coded_bits(plan.block.coded_bits);             // one block, reused for every block
    std::vector<std::uint8_t> interleaver_matrix(plan.block.coded_bits);     // the interleaver's work matrix
    std::vector<std::uint8_t> interleaved_bits(plan.block.coded_bits);       // one block in channel order
    std::vector<float> audio(plan.audio_samples);                            // 48 kHz mono samples

    const BodyTransmissionScratch scratch{
        .framed_bits = framed_bits,
        .transmitted_tribits = transmitted_tribits,
        .coded_bits = coded_bits,
        .interleaver_matrix = interleaver_matrix,
        .interleaved_bits = interleaved_bits,
    };
    const auto status = generate_body_transmission_audio(plan, payload, scratch, audio);
    if (!status.is_ok())
    {
        std::println(stderr, "render failed");
        return 1;
    }

    const Real seconds = approx_real(audio.size()) / as_real(body_audio_sample_rate_hz); // display only
    std::println("6. generate_body_transmission_audio rendered {} samples at 48 kHz ({:.2f} s); first: {:.5f} {:.5f} {:.5f} (preamble)", audio.size(), seconds, audio[0], audio[1], audio[2]);

    write_wav("tx_out.wav", audio, body_audio_sample_rate_hz);
    std::println("   wrote tx_out.wav");

    // 7. Every transmitted tribit, pinned exactly (the tx_golden_symbols test).
    std::println("7. {} transmitted tribits, FNV-1a {:016x}", transmitted_tribits.size(), fnv1a(transmitted_tribits));
    return 0;
}
