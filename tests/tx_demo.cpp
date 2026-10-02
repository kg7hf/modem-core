// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// tx_demo - walk a short message through the MIL-STD-188-110A/B serial-tone
//           transmit chain and write the result to a 48 kHz WAV file.
// =============================================================================
// This is the companion program to the Signal Path article "The transmit chain:
// the easy half, done right." It encodes a message in the 600 bps, long-
// interleave mode ("600L"), prints the intermediate values the article walks
// through, and renders the whole burst to audio you can listen to.
//
//   Build:  cmake -B build && cmake --build build
//   Run:    ./build/tx_demo "Hi"      (writes tx_out.wav)
// =============================================================================

#include "modem/common/fec/convolutional_k7.hpp"
#include "modem/common/types.hpp"
#include "modem/m110a/body_waveform.hpp"
#include "modem/m110a/demapper.hpp"
#include "modem/m110a/scrambler.hpp"
#include "modem/m110a/transmitter.hpp"

#include <cinttypes>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

using namespace modem::common;
using namespace modem::m110a;

namespace
{
// Minimal 16-bit PCM mono WAV writer.
void write_wav(const std::string& path, const std::vector<float>& samples, std::uint32_t rate)
{
    const std::uint32_t data_bytes = static_cast<std::uint32_t>(samples.size()) * 2U;
    const std::uint32_t byte_rate = rate * 2U;
    std::ofstream out(path, std::ios::binary);
    auto u32 = [&](std::uint32_t v) { out.write(reinterpret_cast<const char*>(&v), 4); };
    auto u16 = [&](std::uint16_t v) { out.write(reinterpret_cast<const char*>(&v), 2); };
    out.write("RIFF", 4);
    u32(36U + data_bytes);
    out.write("WAVE", 4);
    out.write("fmt ", 4);
    u32(16U);
    u16(1U);
    u16(1U);
    u32(rate);
    u32(byte_rate);
    u16(2U);
    u16(16U);
    out.write("data", 4);
    u32(data_bytes);
    for (float s : samples)
    {
        float c = s < -1.0F ? -1.0F : (s > 1.0F ? 1.0F : s);
        std::int16_t pcm = static_cast<std::int16_t>(c * 32767.0F);
        out.write(reinterpret_cast<const char*>(&pcm), 2);
    }
}

// FNV-1a over the transmitted tribits. They are integers, so the hash is the
// same on every platform, and it pins every bit decision in the chain: framing,
// coding, interleaving, mapping, probes, preamble and scrambling.
std::uint64_t fnv1a(const std::vector<std::uint8_t>& values)
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
    const std::string message = argc > 1 ? argv[1] : "Hi";

    std::printf("== Transmit chain walkthrough (600 bps, long interleave) ==\n");
    std::printf("message: \"%s\"\n\n", message.c_str());

    // 1. User octets enter least-significant bit first.
    const auto first = static_cast<std::uint8_t>(message[0]);
    std::printf("1. '%c' = 0x%02X, LSB-first bits: ", message[0], first);
    std::uint8_t bits[8];
    for (int i = 0; i < 8; ++i)
    {
        bits[i] = (first >> i) & 1U;
        std::printf("%u", bits[i]);
    }
    std::printf("\n");

    // 2. K=7 rate-1/2 convolutional code (generators 133/171 octal).
    ConvolutionalEncoderK7 encoder(0U);
    std::printf("2. K=7 coded pairs (t1t2): ");
    for (int i = 0; i < 8; ++i)
    {
        auto p = encoder.push(bits[i]);
        std::printf("%u%u ", p.t1, p.t2);
    }
    std::printf("\n");

    // 3. 600L is BPSK: one coded bit per symbol -> tribit 0 (0 deg) or 4 (180 deg).
    std::printf("3. BPSK map: coded 0 -> tribit %u (0 deg), coded 1 -> tribit %u (180 deg)\n", mapped_tribit(0U, 1U), mapped_tribit(1U, 1U));

    // 4. Scramble on the 8-PSK ring: (tribit + randomizer) mod 8.
    BodyDataRandomizer rnd;
    rnd.reset();
    std::uint8_t r[6];
    std::printf("4. body randomizer tribits: ");
    for (int i = 0; i < 6; ++i)
    {
        r[i] = rnd.next_tribit();
        std::printf("%u ", r[i]);
    }
    std::printf("\n   example: coded 1 -> tribit 4, + randomizer %u => transmitted %u\n", r[1], tribit_add(4U, r[1]));

    // 5. A transmitted tribit is a point on the 8-PSK circle.
    const std::uint8_t tx_tribit = tribit_add(4U, r[1]);
    const IQSample pt = psk8_symbol(tx_tribit);
    std::printf("5. psk8_symbol(%u): I=% .3f Q=% .3f (%u x 45 = %u deg)\n\n", tx_tribit, pt.real(), pt.imag(), tx_tribit, tx_tribit * 45U);

    // 6. Render the whole burst to 48 kHz audio.
    BodyMode mode{DataRate::bps600, BodyInterleave::long_block};
    auto plan = body_transmission_plan(mode, message.size());
    if (!plan)
    {
        std::printf("plan failed\n");
        return 1;
    }
    const auto& p = plan.value();

    std::vector<std::uint8_t> framed(p.framed_bits, 0U), tribits(p.transmitted_symbols, 0U), coded(p.block.coded_bits, 0U), matrix(p.block.coded_bits, 0U), interleaved(p.block.coded_bits, 0U);
    std::vector<float> audio(p.audio_samples, 0.0F);

    BodyTransmissionScratch scratch{
        MutableBitSpan(framed.data(), framed.size()), MutableBitSpan(tribits.data(), tribits.size()),         MutableBitSpan(coded.data(), coded.size()),
        MutableBitSpan(matrix.data(), matrix.size()), MutableBitSpan(interleaved.data(), interleaved.size()),
    };
    const auto status = generate_body_transmission_audio(p, std::span<const std::uint8_t>(reinterpret_cast<const std::uint8_t*>(message.data()), message.size()), scratch,
                                                         MutableSampleSpan(audio.data(), audio.size()));
    if (!status.is_ok())
    {
        std::printf("render failed\n");
        return 1;
    }

    std::printf("6. rendered %zu samples at 48 kHz (%.2f s); first: %.5f %.5f %.5f\n", audio.size(), static_cast<double>(audio.size()) / 48000.0, audio[0], audio[1], audio[2]);

    write_wav("tx_out.wav", audio, 48000U);
    std::printf("   wrote tx_out.wav\n");

    // 7. Every transmitted tribit, pinned exactly (the tx_golden_symbols test).
    std::printf("7. %zu transmitted tribits, FNV-1a %016" PRIx64 "\n", tribits.size(), fnv1a(tribits));
    return 0;
}
