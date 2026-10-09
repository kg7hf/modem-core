// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// transmit_chain_tests.cpp - the transmit chain's tests (Part 2)
// =============================================================================
// Each test feeds known data through one stage of the transmitter and checks
// what comes out, in the order the article "The transmit chain" walks the chain.
// Run them from VS Code's Testing panel or with ctest; set a breakpoint and debug
// any one of them. The tx_demo program walks the same message and writes a WAV.
// Explained in the article series, Part 2 (the transmit chain).
// =============================================================================

#include "modem/common/bits.hpp"
#include "modem/common/convert.hpp"
#include "modem/common/fec/convolutional_k7.hpp"
#include "modem/common/types.hpp"
#include "modem/common/units/rates.hpp"
#include "modem/common/waveform/waveform.hpp"
#include "modem/m110a/body_waveform.hpp"
#include "modem/m110a/demapper.hpp"
#include "modem/m110a/interleaver.hpp"
#include "modem/m110a/scrambler.hpp"
#include "modem/m110a/transmitter.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <numbers>
#include <print>
#include <span>
#include <utility>
#include <vector>

// The article's walkthrough, section by section: run it with
//   transmit_chain_tests "[walkthrough]" --order decl
// and every number the article quotes prints in the article's order.

namespace
{

using ::modem::common::as_real;
using ::modem::common::bit_at;
using ::modem::common::BodyInterleave;
using ::modem::common::ChannelSidebandPolicy;
using ::modem::common::ConvolutionalEncoderK7;
using ::modem::common::DataRate;
using ::modem::common::IQSample;
using ::modem::common::WaveformConfig;
using ::modem::common::WaveformFamily;
using ::modem::common::widen;
using ::modem::m110a::BodyMode;
using ::modem::m110a::BodyTransmissionPlan;

constexpr BodyMode mode_600l{DataRate::bps600, BodyInterleave::long_block};
constexpr std::array<std::uint8_t, 2> hi{'H', 'i'};

// The transmitter's work buffers, sized from the plan by the caller: the library never
// allocates.
struct Rendered
{
    BodyTransmissionPlan plan{};
    std::vector<std::uint8_t> framed_bits{};
    std::vector<std::uint8_t> transmitted_tribits{};
    std::vector<std::uint8_t> coded_bits{};
    std::vector<std::uint8_t> interleaver_matrix{};
    std::vector<std::uint8_t> interleaved_bits{};
    std::vector<float> audio{};
};

// Plan the burst, size every buffer from the plan, then render it.
Rendered render_hi()
{
    const auto plan = ::modem::m110a::body_transmission_plan(mode_600l, hi.size());
    REQUIRE(plan.error().is_ok());

    Rendered out;
    out.plan = plan.value();
    out.framed_bits.resize(out.plan.framed_bits);
    out.transmitted_tribits.resize(out.plan.transmitted_symbols);
    out.coded_bits.resize(out.plan.block.coded_bits);
    out.interleaver_matrix.resize(out.plan.block.coded_bits);
    out.interleaved_bits.resize(out.plan.block.coded_bits);
    out.audio.resize(out.plan.audio_samples);

    const ::modem::m110a::BodyTransmissionScratch scratch{
        .framed_bits = out.framed_bits,
        .transmitted_tribits = out.transmitted_tribits,
        .coded_bits = out.coded_bits,
        .interleaver_matrix = out.interleaver_matrix,
        .interleaved_bits = out.interleaved_bits,
    };
    REQUIRE(::modem::m110a::generate_body_transmission_audio(out.plan, hi, scratch, out.audio).is_ok());
    return out;
}

// FNV-1a over the transmitted tribits: integers, so the hash is the same on every platform.
std::uint64_t fnv1a(std::span<const std::uint8_t> values)
{
    std::uint64_t hash = 14695981039346656037ULL;
    for (const auto value : values)
    {
        hash ^= value;
        hash *= 1099511628211ULL;
    }
    return hash;
}

} // namespace

// --- Plan the whole burst before you move a bit ---

TEST_CASE("The plan sizes the whole 600L burst before a bit moves", "[transmit][plan][walkthrough]")
{
    const auto plan = ::modem::m110a::body_transmission_plan(mode_600l, hi.size());
    REQUIRE(plan.error().is_ok());
    const auto& p = plan.value();
    std::println("\"Hi\" in 600L: {} payload bits, {} framed bits, {} body block, {} preamble symbols, {} symbols in all, {} samples ({:.1f} s at 48 kHz)", p.payload_bits, p.framed_bits,
                 p.body_blocks, p.preamble_symbols, p.transmitted_symbols, p.audio_samples, as_real(p.audio_samples) / 48000.0F);
    CHECK(p.payload_bits == 16U);
    CHECK(p.body_blocks == 1U);
    CHECK(p.transmitted_symbols == 23040U);
    CHECK(p.audio_samples == 460800U);
}

TEST_CASE("A request the plan cannot size or a buffer that does not match is refused", "[transmit][plan][walkthrough]")
{
    const auto too_big = ::modem::m110a::body_transmission_plan(mode_600l, std::numeric_limits<std::size_t>::max());
    const bool plan_refused = !too_big;
    std::println("plan for SIZE_MAX bytes: {}", plan_refused ? "refused" : "accepted");
    CHECK(plan_refused);

    auto rendered = render_hi();
    rendered.audio.resize(rendered.audio.size() - 1U);
    const ::modem::m110a::BodyTransmissionScratch scratch{
        .framed_bits = rendered.framed_bits,
        .transmitted_tribits = rendered.transmitted_tribits,
        .coded_bits = rendered.coded_bits,
        .interleaver_matrix = rendered.interleaver_matrix,
        .interleaved_bits = rendered.interleaved_bits,
    };
    const auto status = ::modem::m110a::generate_body_transmission_audio(rendered.plan, hi, scratch, rendered.audio);
    const bool render_refused = !status.is_ok();
    std::println("render into an audio buffer one sample short: {}", render_refused ? "refused" : "accepted");
    CHECK(render_refused);
}

// --- The FEC: spend bits to buy survival ---

TEST_CASE("One input bit stays in the encoder for seven coded pairs", "[transmit][fec][walkthrough]")
{
    // A single 1 into the all-zero K=7 encoder, then zeros: the pairs read out the
    // generators 133 and 171 (octal) one tap at a time. The bit is in the encoder for seven
    // pairs; each generator has five taps, so ten of those fourteen coded bits depend on it.
    ConvolutionalEncoderK7 encoder(0U);
    std::size_t last_touched{};
    std::size_t coded_bits_touched{};
    std::print("input 1 0 0 0 0 0 0 0 0 -> pairs:");
    for (std::size_t k = 0U; k < 9U; ++k)
    {
        const auto pair = encoder.push(k == 0U ? 1U : 0U);
        std::print(" {}{}", pair.t1, pair.t2);
        coded_bits_touched += std::size_t{pair.t1} + std::size_t{pair.t2};
        last_touched = (pair.t1 | pair.t2) != 0U ? k : last_touched;
    }
    std::println("\nthe bit is in the encoder for {} pairs; {} of their {} coded bits depend on it", last_touched + 1U, coded_bits_touched, 2U * (last_touched + 1U));
    CHECK(last_touched + 1U == 7U);
    CHECK(coded_bits_touched == 10U);
}

// --- Repetition and Walsh: more redundancy, lower rates ---

TEST_CASE("Each rate's block plan shows its repetitions and its data and probe frame", "[transmit][plan][walkthrough]")
{
    std::println(" rate  bits/symbol  pair copies  data + probe symbols per frame");
    for (const auto rate : {DataRate::bps75, DataRate::bps150, DataRate::bps300, DataRate::bps600, DataRate::bps1200, DataRate::bps2400})
    {
        const auto block = ::modem::m110a::body_block_plan({rate, BodyInterleave::short_block});
        REQUIRE(block.error().is_ok());
        const auto& b = block.value();
        if (b.frame_symbols() == 0U)
        {
            std::println("{:5}  {:11}  {:11}  Walsh: 32 symbols per coded pair, no probe frame", std::to_underlying(rate), b.information_bits_per_channel_symbol, b.fec_pair_repetitions);
        }
        else
        {
            std::println("{:5}  {:11}  {:11}  {} + {}", std::to_underlying(rate), b.information_bits_per_channel_symbol, b.fec_pair_repetitions, b.unknown_symbols_per_probe,
                         b.known_symbols_per_probe);
        }
        const std::size_t expected_copies = rate == DataRate::bps150 ? 4U : rate == DataRate::bps300 ? 2U : 1U;
        CHECK(b.fec_pair_repetitions == expected_copies);
    }
}

// --- Interleaving: scatter the damage ---

TEST_CASE("The interleaver moves every coded bit of a block exactly once", "[transmit][interleave][walkthrough]")
{
    const WaveformConfig config{WaveformFamily::serial_tone, DataRate::bps600, BodyInterleave::long_block, ChannelSidebandPolicy::detect};
    const auto spec = ::modem::common::interleaver_for(config);
    REQUIRE(spec.error().is_ok());
    const auto& s = spec.value();
    std::println("600L interleaver: {} rows x {} columns, load row increment {}, fetch column decrement {}", s.rows, s.columns, s.load_row_increment, s.fetch_column_decrement);

    // The interleaver moves bits, so it is run once per bit of the input position: the
    // outputs, read back bit by bit, say which input position landed where.
    const std::size_t size = s.input_bits;
    std::vector<std::uint32_t> source(size, 0U);
    std::vector<std::uint8_t> input(size);
    std::vector<std::uint8_t> scratch(size);
    std::vector<std::uint8_t> output(size);
    for (unsigned b = 0U; (std::size_t{1} << b) < size; ++b)
    {
        for (std::size_t i = 0U; i < size; ++i)
        {
            input[i] = bit_at(i, b);
        }
        REQUIRE(::modem::m110a::body_interleave(input, scratch, output, s).is_ok());
        for (std::size_t i = 0U; i < size; ++i)
        {
            source[i] |= std::uint32_t{output[i] & 1U} << b;
        }
    }

    std::vector<std::uint8_t> seen(size, 0U);
    for (const auto from : source)
    {
        REQUIRE(from < size);
        ++seen[from];
    }
    std::println("{} coded bits; the first out on the air came from positions {} {} {} {} {}", size, source[0], source[1], source[2], source[3], source[4]);
    CHECK(std::ranges::all_of(seen, [](std::uint8_t count) { return count == 1U; }));
}

// --- Bits to symbols: the constellation and a Gray map ---

TEST_CASE("The modified Gray map puts neighboring points one bit apart", "[transmit][map][walkthrough]")
{
    std::print("8-PSK (width 3): bits ->");
    std::array<std::uint8_t, 8> bits_at_point{};
    for (std::uint8_t value = 0U; value < 8U; ++value)
    {
        const auto tribit = ::modem::m110a::mapped_tribit(value, 3U);
        bits_at_point[tribit] = value;
        std::print(" {}", tribit);
    }
    std::println("");
    for (std::size_t point = 0U; point < 8U; ++point)
    {
        // Neighbors on the circle, 45 degrees apart, differ in exactly one bit.
        CHECK(std::popcount(widen<unsigned>(bits_at_point[point]) ^ widen<unsigned>(bits_at_point[(point + 1U) % 8U])) == 1);
    }

    std::println("QPSK (width 2): {} {} {} {}", ::modem::m110a::mapped_tribit(0U, 2U), ::modem::m110a::mapped_tribit(1U, 2U), ::modem::m110a::mapped_tribit(2U, 2U),
                 ::modem::m110a::mapped_tribit(3U, 2U));
    std::println("BPSK (width 1, 600L): coded 0 -> tribit {}, coded 1 -> tribit {}", ::modem::m110a::mapped_tribit(0U, 1U), ::modem::m110a::mapped_tribit(1U, 1U));
    CHECK(::modem::m110a::mapped_tribit(0U, 1U) == 0U);
    CHECK(::modem::m110a::mapped_tribit(1U, 1U) == 4U);
}

// --- Scrambling: whiten it, and exactly ---

TEST_CASE("The body randomizer starts from 0xBAD and adds on the 8-PSK ring", "[transmit][scramble][walkthrough]")
{
    ::modem::m110a::BodyDataRandomizer randomizer;
    randomizer.reset();
    std::println("load value: 0x{:03X}", randomizer.state());
    CHECK(randomizer.state() == 0xBADU);

    constexpr std::array<std::uint8_t, 6> first{0U, 2U, 4U, 3U, 3U, 6U};
    std::print("first tribits:");
    for (const auto expected : first)
    {
        const auto tribit = randomizer.next_tribit();
        std::print(" {}", tribit);
        CHECK(tribit == expected);
    }
    std::println("");

    std::println("(4 + 2) mod 8 = {}; (6 + 3) mod 8 = {}", ::modem::m110a::tribit_add(4U, 2U), ::modem::m110a::tribit_add(6U, 3U));
    CHECK(::modem::m110a::tribit_add(4U, 2U) == 6U);
    CHECK(::modem::m110a::tribit_add(6U, 3U) == 1U);
}

// --- Turning symbols into 1800 Hz ---

TEST_CASE("The carrier step is 13.5 degrees a sample down to the last bit", "[transmit][carrier][walkthrough]")
{
    const auto step = ::modem::m110a::body_carrier_radians_per_sample;
    std::println("2 pi x 1800 / 48000 = {:.7f} rad = {:.1f} degrees, float bits 0x{:08X}", step, step * 180.0F / std::numbers::pi_v<float>, std::bit_cast<std::uint32_t>(step));
    CHECK(std::bit_cast<std::uint32_t>(step) == 0x3E714639U);
}

TEST_CASE("The burst stays under the 0.98 no-clip limit", "[transmit][carrier][walkthrough]")
{
    const auto rendered = render_hi();
    float peak{};
    for (const auto sample : rendered.audio)
    {
        peak = std::max(peak, std::fabs(sample));
    }
    std::println("{} samples, peak magnitude {:.4f}", rendered.audio.size(), peak);
    CHECK(peak <= 0.98F);
}

// --- Walk a message through it ---

TEST_CASE("Walk: 'H' goes in LSB first and comes out as coded pairs", "[transmit][walk][walkthrough]")
{
    constexpr std::uint8_t h = 'H';
    constexpr std::array<std::uint8_t, 8> expected_bits{0U, 0U, 0U, 1U, 0U, 0U, 1U, 0U};
    constexpr std::array<std::uint8_t, 16> expected_pairs{0U, 0U, 0U, 0U, 0U, 0U, 1U, 1U, 0U, 1U, 1U, 1U, 0U, 0U, 0U, 1U};

    ConvolutionalEncoderK7 encoder(0U);
    std::print("'H' = 0x{:02X} -> bits:", h);
    std::array<std::uint8_t, 16> pairs{};
    for (unsigned position = 0U; position < 8U; ++position)
    {
        const auto bit = bit_at(h, position);
        std::print(" {}", bit);
        CHECK(bit == expected_bits[position]);
        const auto pair = encoder.push(bit);
        pairs[2U * position] = pair.t1;
        pairs[2U * position + 1U] = pair.t2;
    }
    std::print("\ncoded pairs:");
    for (std::size_t k = 0U; k < 8U; ++k)
    {
        std::print(" {}{}", pairs[2U * k], pairs[2U * k + 1U]);
    }
    std::println("");
    CHECK(pairs == expected_pairs);
}

TEST_CASE("Walk: a coded 1 plus randomizer 2 goes out as tribit 6 at 270 degrees", "[transmit][walk][walkthrough]")
{
    const auto tribit = ::modem::m110a::tribit_add(::modem::m110a::mapped_tribit(1U, 1U), 2U);
    const IQSample point = ::modem::m110a::psk8_symbol(tribit);
    std::println("coded 1 -> tribit {}, + randomizer 2 -> tribit {}: I = {:.3f}, Q = {:.3f}, magnitude {:.3f}", ::modem::m110a::mapped_tribit(1U, 1U), tribit, point.real(), point.imag(),
                 std::abs(point));
    CHECK(tribit == 6U);
    CHECK(std::fabs(point.real()) < 1.0e-6F);
    CHECK(std::fabs(point.imag() + 1.0F) < 1.0e-6F);
}

TEST_CASE("Walk: tribit 6 held steady is a clean 1800 Hz tone", "[transmit][walk][walkthrough]")
{
    // Forty symbols of tribit 6 from carrier phase zero; samples 241 to 248 sit well inside.
    const std::vector<std::uint8_t> held(40U, 6U);
    std::vector<float> audio(held.size() * ::modem::m110a::body_audio_samples_per_symbol);
    float carrier_phase{};
    REQUIRE(::modem::m110a::body_tribits_to_audio(held, audio, carrier_phase).is_ok());

    constexpr std::array<float, 8> article{0.126F, 0.244F, 0.349F, 0.435F, 0.497F, 0.531F, 0.536F, 0.511F};
    std::print("n=241..248:");
    for (std::size_t k = 0U; k < article.size(); ++k)
    {
        std::print(" {:.3f}", audio[241U + k]);
        CHECK(std::fabs(audio[241U + k] - article[k]) < 0.0005F);
    }
    std::println("");
}

TEST_CASE("Walk: the whole Hi burst matches its golden values", "[transmit][walk][golden][walkthrough]")
{
    const auto rendered = render_hi();
    const auto hash = fnv1a(rendered.transmitted_tribits);
    std::println("{} samples; first {:.5f} {:.5f} {:.5f} (preamble); {} transmitted tribits, FNV-1a {:016x}", rendered.audio.size(), rendered.audio[0], rendered.audio[1], rendered.audio[2],
                 rendered.transmitted_tribits.size(), hash);
    CHECK(hash == 0x6097e9da9fbb6255ULL);
    CHECK(std::fabs(rendered.audio[0] - 0.39122F) < 0.000005F);
    CHECK(std::fabs(rendered.audio[1] - 0.45497F) < 0.000005F);
    CHECK(std::fabs(rendered.audio[2] - 0.49206F) < 0.000005F);
}
