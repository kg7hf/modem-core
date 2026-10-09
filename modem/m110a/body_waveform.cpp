// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

/*
================================================================================
body_waveform.cpp - the serial-tone body waveform (build and parse a burst)
================================================================================

WHAT THIS FILE IS
-----------------
The structural backbone of the serial-tone body: the block plan, preamble
generation, per-block encode/decode, the probe framing, EOM framing, and audio
rendering. It is the integrator that wires the FEC, interleaver, demapper, and
scrambler into an on-air burst - and it computes the BodyBlockPlan that sizes every
receiver stage.

BODY BURST ANATOMY
------------------
  PREAMBLE (N x 480 sym, countdown) | BODY blocks | EOM (32b 0x4B65A5B2) | FLUSH (144b)
  Each block = data frames of [unknown DATA symbols | known PROBE symbols]; most rates
  20/20, 2400/4800 32/16; every short block = 1440 symbols (0.6 s), long = x8.

  INPUT  : (encode) payload bits + BodyBlockPlan; (decode) received symbols + plan.
  OUTPUT : transmitted tribits / audio; or soft metrics / info bits / payload octets.
  DEPENDS: fec, interleaver, demapper, scrambler, timing_recovery (RRC taps).

KEY FUNCTIONS
-------------
  body_block_plan()          THE per-mode geometry table (read this first).
  encode_body_block()        FEC -> interleave -> map+whiten + insert probes.
  decode_body_block()        mirror: received symbols back to information bits.
  body_block_soft_metrics()  extract+deinterleave+repetition-combine (no Viterbi), so
                             the receiver can run ONE continuous Viterbi over the burst.
  append_body_eom_and_flush / find_body_eom / pack_body_payload   framing + payload.
  BodyAudioStreamModulator   RRC pulse-shape + 1800 Hz upconvert (stateful, windowed).
  The receive side (decode_body_block, body_block_soft_metrics, find_body_eom,
  pack_body_payload) is declared in the header; it arrives with the receiver series.

Teaching walkthrough: modem/m110a/body-waveform-and-scrambler-explainer.md
================================================================================
*/

#include "modem/m110a/body_waveform.hpp"

#include "modem/common/bits.hpp"
#include "modem/common/constants.hpp"
#include "modem/common/dsp/timing_recovery.hpp"
#include "modem/common/fec/convolutional_k7.hpp"
#include "modem/m110a/demapper.hpp"
#include "modem/m110a/interleaver.hpp"
#include "modem/m110a/scrambler.hpp"

#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <tuple>

#include "modem/common/status.hpp"
namespace modem::m110a
{

using ::modem::common::bit_at;
using ::modem::common::BitSpan;
using ::modem::common::BodyInterleave;
using ::modem::common::ChannelSidebandPolicy;
using ::modem::common::ConvolutionalEncoderK7;
using ::modem::common::DataRate;
using ::modem::common::interleaver_for;
using ::modem::common::IQSample;
using ::modem::common::IQSampleSpan;
using ::modem::common::low_bits;
using ::modem::common::make_root_raised_cosine_taps;
using ::modem::common::MutableBitSpan;
using ::modem::common::MutableIQSampleSpan;
using ::modem::common::MutableSampleSpan;
using ::modem::common::narrow;
using ::modem::common::Result;
using ::modem::common::round_to;
using ::modem::common::shift_in_bit;
using ::modem::common::Status;
using ::modem::common::StatusCode;
using ::modem::common::WaveformConfig;
using ::modem::common::WaveformFamily;
using ::modem::common::with_bit;

constexpr std::array<std::array<std::uint8_t, 8>, 8> channel_symbol_patterns{{
    {{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U}},
    {{0U, 4U, 0U, 4U, 0U, 4U, 0U, 4U}},
    {{0U, 0U, 4U, 4U, 0U, 0U, 4U, 4U}},
    {{0U, 4U, 4U, 0U, 0U, 4U, 4U, 0U}},
    {{0U, 0U, 0U, 0U, 4U, 4U, 4U, 4U}},
    {{0U, 4U, 0U, 4U, 4U, 0U, 4U, 0U}},
    {{0U, 0U, 4U, 4U, 4U, 4U, 0U, 0U}},
    {{0U, 4U, 4U, 0U, 4U, 0U, 0U, 4U}},
}};

constexpr std::array<std::uint8_t, 9> preamble_prefix{0U, 1U, 3U, 0U, 1U, 3U, 1U, 2U, 0U};

static std::uint8_t bit_group(BitSpan bits, std::size_t offset, std::uint8_t width) noexcept
{
    std::uint8_t value{};

    for (std::uint8_t bit = 0U; bit < width; ++bit)
    {
        value = shift_in_bit(value, (bits[offset + bit] & 1U) != 0U);
    }

    return value;
}

static Status encode_coded_bits(BitSpan information_bits, const BodyBlockPlan& plan, MutableBitSpan coded, BodyEncodeState& state) noexcept
{
    if (coded.size() < plan.coded_bits)
    {
        return {StatusCode::buffer_too_small, "body coded-bit buffer too small"};
    }

    if (plan.mode.data_rate == DataRate::bps4800)
    {
        for (std::size_t index = 0U; index < information_bits.size(); ++index)
        {
            coded[index] = information_bits[index] & 1U;
        }

        return Status::success();
    }

    ConvolutionalEncoderK7 encoder(state.fec_state);
    std::size_t written{};

    for (const auto bit : information_bits)
    {
        const auto pair = encoder.push(bit);

        for (std::uint8_t repeat = 0U; repeat < plan.fec_pair_repetitions; ++repeat)
        {
            coded[written++] = pair.t1;
            coded[written++] = pair.t2;
        }
    }

    if (written != plan.coded_bits)
    {
        return {StatusCode::internal_error, "body FEC output length does not match block plan"};
    }

    state.fec_state = encoder.state();
    return Status::success();
}

// body_block_plan, body_designators, find_designator_row, and the preamble-symbol
// counts are constexpr in body_waveform.hpp so the whole geometry table folds at
// compile time (see body_scratch_max). The rate notes: 150 (x4) and 300 (x2) are the
// rate-1/2 REPETITION modes whose repeated coded copies body_block_soft_metrics sums;
// 600/1200/2400 are plain rate 1/2; 75 is Walsh orthogonal spreading; 4800 is uncoded.

std::uint8_t body_channel_symbol_tribit(unsigned channel_symbol, std::size_t spread_index) noexcept
{
    const auto& pattern = channel_symbol_patterns[channel_symbol & 7U];
    return pattern[spread_index % pattern.size()];
}

std::uint8_t body_75_spread_tribit(std::uint8_t information_dibit, bool exceptional_set, std::size_t spread_index) noexcept
{
    // MIL-STD-188-110B Table XIII first maps the raw information dibit to a
    // modified-Gray channel symbol. Tables XIVa/XIVb are indexed by that
    // channel symbol, not directly by the two interleaver-output bits.
    const auto channel_symbol = modified_gray_decode(information_dibit, 2U);
    const auto pattern_index = channel_symbol + (exceptional_set ? 4U : 0U);
    return body_channel_symbol_tribit(pattern_index, spread_index);
}

static void emit_body_preamble_segment(const BodyDesignators& designators, std::uint8_t countdown, SyncRandomizer& randomizer, MutableBitSpan output) noexcept
{
    std::array<unsigned, 15> channel_symbols{};

    for (std::size_t index = 0U; index < preamble_prefix.size(); ++index)
    {
        channel_symbols[index] = preamble_prefix[index];
    }

    channel_symbols[9] = designators.d1;
    channel_symbols[10] = designators.d2;
    channel_symbols[11] = 4U | ((countdown >> 4U) & 3U);
    channel_symbols[12] = 4U | ((countdown >> 2U) & 3U);
    channel_symbols[13] = 4U | (countdown & 3U);
    channel_symbols[14] = 0U;
    std::size_t written{};

    for (const auto channel_symbol : channel_symbols)
    {
        for (std::size_t spread = 0U; spread < 32U; ++spread)
        {
            output[written++] = tribit_add(body_channel_symbol_tribit(channel_symbol, spread), randomizer.next_tribit());
        }
    }
}

Status generate_body_preamble(BodyMode mode, MutableBitSpan output) noexcept
{
    return generate_body_preamble_tail(mode, body_preamble_segments(mode.interleave), output);
}

Status generate_body_preamble_tail(BodyMode mode, std::size_t tail_segments, MutableBitSpan output) noexcept
{
    const auto plan = body_block_plan(mode);
    const auto segments = body_preamble_segments(mode.interleave);

    if (!plan)
    {
        return plan.error();
    }

    if (tail_segments == 0U || tail_segments > segments)
    {
        return {StatusCode::invalid_argument, "body preamble tail segment count is invalid"};
    }

    if (output.size() < tail_segments * body_preamble_segment_symbols)
    {
        return {StatusCode::buffer_too_small, "body preamble output too small"};
    }

    // Every segment consumes 480 scrambler tribits, a whole multiple of the
    // 32-entry sync sequence, so the scramble alignment at any segment
    // boundary equals the alignment at the preamble start and a fresh
    // randomizer generates any segment-aligned suffix exactly.
    SyncRandomizer randomizer;
    std::size_t written{};

    for (std::size_t segment = segments - tail_segments; segment < segments; ++segment)
    {
        const auto countdown = low_bits<5>(segments - segment - 1U);
        emit_body_preamble_segment(plan.value().designators, countdown, randomizer, output.subspan(written, body_preamble_segment_symbols));
        written += body_preamble_segment_symbols;
    }

    return Status::success();
}

Status encode_body_block(BitSpan information_bits, const BodyBlockPlan& plan, BodyEncodeScratch scratch, MutableBitSpan transmitted_tribits) noexcept
{
    BodyEncodeState state{};
    return encode_body_block(information_bits, plan, scratch, transmitted_tribits, state);
}

// -----------------------------------------------------------------------------
// encode_body_block  (transmit one body block)
// -----------------------------------------------------------------------------
// 50K view: Build the transmitted symbols for one block: FEC -> interleave ->
//   map+whiten, inserting the known probe symbols each frame.
// Detailed view: encode_coded_bits (rate 1/2 x reps, threading BodyEncodeState) ->
//   body_interleave -> for each frame emit the unknown data symbols (modified-Gray
//   mapped + data-randomizer whitening) then the known probe symbols; the last two
//   frames' probes carry D1 then D2 for mid-body mode re-ID. 75 bps takes the Walsh
//   path. Its receive mirror, decode_body_block, is declared in the header.
// 5th-grade view: Encode the words, shuffle them, add the secret voice-shuffle, and
//   drop in the memorized checkpoint phrases.
// -----------------------------------------------------------------------------
Status encode_body_block(BitSpan information_bits, const BodyBlockPlan& plan, BodyEncodeScratch scratch, MutableBitSpan transmitted_tribits, BodyEncodeState& state) noexcept
{
    if (information_bits.size() != plan.information_bits || scratch.coded.size() < plan.coded_bits || scratch.interleaved.size() < plan.coded_bits ||
        transmitted_tribits.size() < plan.transmitted_symbols)
    {
        return {StatusCode::invalid_argument, "body encode buffers do not match block plan"};
    }

    const auto encode_status = encode_coded_bits(information_bits, plan, scratch.coded, state);

    if (!encode_status.is_ok())
    {
        return encode_status;
    }

    const WaveformConfig config{WaveformFamily::serial_tone, plan.mode.data_rate, plan.mode.interleave, ChannelSidebandPolicy::detect};
    const auto interleaver = interleaver_for(config);

    if (!interleaver)
    {
        return interleaver.error();
    }

    const auto interleave_status = body_interleave(scratch.coded.first(plan.coded_bits), scratch.interleaver_matrix, scratch.interleaved.first(plan.coded_bits), interleaver.value());

    if (!interleave_status.is_ok())
    {
        return interleave_status;
    }

    BodyDataRandomizer randomizer;
    std::size_t bit_index{};
    std::size_t written{};

    if (plan.mode.data_rate == DataRate::bps75)
    {
        for (std::size_t dibit = 0U; dibit < plan.data_channel_symbols; ++dibit)
        {
            const auto value = bit_group(scratch.interleaved, bit_index, 2U);
            const bool exceptional_set = dibit + 1U == plan.data_channel_symbols;

            for (std::size_t spread = 0U; spread < 32U; ++spread)
            {
                transmitted_tribits[written++] = tribit_add(body_75_spread_tribit(value, exceptional_set, spread), randomizer.next_tribit());
            }

            bit_index += 2U;
        }

        return written == plan.transmitted_symbols ? Status::success() : Status{StatusCode::internal_error, "75-bps transmit count mismatch"};
    }

    const auto frames = plan.data_channel_symbols / plan.unknown_symbols_per_probe;

    for (std::size_t frame = 0U; frame < frames; ++frame)
    {
        for (std::size_t symbol = 0U; symbol < plan.unknown_symbols_per_probe; ++symbol)
        {
            const auto value = bit_group(scratch.interleaved, bit_index, plan.information_bits_per_channel_symbol);
            const auto tribit = mapped_tribit(value, plan.information_bits_per_channel_symbol);
            transmitted_tribits[written++] = tribit_add(tribit, randomizer.next_tribit());
            bit_index += plan.information_bits_per_channel_symbol;
        }

        const std::uint8_t known_value = frame + 2U == frames ? plan.designators.d1 : frame + 1U == frames ? plan.designators.d2 : std::uint8_t{0U};

        for (std::size_t symbol = 0U; symbol < plan.known_symbols_per_probe; ++symbol)
        {
            const auto probe_tribit = plan.known_symbols_per_probe == 20U && symbol >= 16U ? 0U : body_channel_symbol_tribit(known_value, symbol);
            transmitted_tribits[written++] = tribit_add(probe_tribit, randomizer.next_tribit());
        }
    }

    return bit_index == plan.coded_bits && written == plan.transmitted_symbols ? Status::success() : Status{StatusCode::internal_error, "body transmit count mismatch"};
}

// -----------------------------------------------------------------------------
// append_body_eom_and_flush
// -----------------------------------------------------------------------------
// 50K view: Frame the payload with the end-of-message marker and trellis flush.
// Detailed view: payload bits, then the 32-bit EOM word 0x4B65A5B2 (MSB-first),
//   then 144 flush bits, zero-padded up to a whole block_information_bits multiple.
//   find_body_eom reverses it (minimum Hamming distance to the EOM word).
// 5th-grade view: Add "THE END" and some cleanup marks, padded to a whole page.
// -----------------------------------------------------------------------------
Result<std::size_t> append_body_eom_and_flush(BitSpan payload, std::size_t block_information_bits, MutableBitSpan framed_bits) noexcept
{
    if (block_information_bits == 0U)
    {
        return Status{StatusCode::invalid_argument, "body frame block size is zero"};
    }

    const auto unpadded = payload.size() + body_eom_bits + body_flush_bits;
    const auto required = ((unpadded + block_information_bits - 1U) / block_information_bits) * block_information_bits;

    if (framed_bits.size() < required)
    {
        return Status{StatusCode::buffer_too_small, "body frame output too small"};
    }

    std::size_t written{};

    for (const auto bit : payload)
    {
        framed_bits[written++] = bit & 1U;
    }

    for (unsigned shift = body_eom_bits; shift > 0U; --shift)
    {
        framed_bits[written++] = bit_at(body_eom_word, shift - 1U); // MSB first
    }

    while (written < required)
    {
        framed_bits[written++] = 0U;
    }

    return written;
}

Status body_tribits_to_iq(BitSpan tribits, MutableIQSampleSpan output) noexcept
{
    if (output.size() < tribits.size())
    {
        return {StatusCode::buffer_too_small, "body IQ output too small"};
    }

    for (std::size_t index = 0U; index < tribits.size(); ++index)
    {
        output[index] = psk8_symbol(tribits[index]);
    }

    return Status::success();
}

Status body_tribits_to_audio(BitSpan tribits, MutableSampleSpan output, float& carrier_phase_radians) noexcept
{
    BodyAudioStreamModulator modulator;
    auto status = modulator.initialize(carrier_phase_radians);

    if (!status.is_ok())
    {
        return status;
    }

    status = modulator.render_window(tribits, 0U, tribits.size(), output);
    carrier_phase_radians = modulator.carrier_phase_radians();
    return status;
}

Status BodyAudioStreamModulator::initialize(float carrier_phase_radians) noexcept
{
    constexpr float shaping_rolloff = 0.25F;
    const auto status = make_root_raised_cosine_taps(body_audio_samples_per_symbol, shaping_rolloff, mTaps);

    if (!status.is_ok())
    {
        return status;
    }

    float worst_polyphase_sum = 0.0F;

    for (std::size_t phase = 0U; phase < body_audio_samples_per_symbol; ++phase)
    {
        float phase_sum = 0.0F;

        for (std::size_t tap = phase; tap < mTaps.size(); tap += body_audio_samples_per_symbol)
        {
            phase_sum += std::fabs(mTaps[tap]);
        }

        worst_polyphase_sum = std::max(worst_polyphase_sum, phase_sum);
    }

    mAmplitudeScale = 0.98F / worst_polyphase_sum;
    // The starting phase is a whole number of 2 pi / 80 steps (0, or a value returned by carrier_phase_radians()).
    // Table entry n holds phase 3n steps, so the entry for u steps is u times the inverse of 3 mod 80.
    constexpr auto period = narrow<std::int64_t>(body_carrier_period_samples);
    constexpr std::int64_t inverse_of_three = 27;
    static_assert((3 * inverse_of_three) % period == 1, "27 is the inverse of 3 mod 80");
    const auto units = round_to<std::int64_t>(carrier_phase_radians / body_carrier_table_radians);
    mCarrierIndex = narrow<std::size_t>((((units % period) + period) % period) * inverse_of_three % period);
    mInitialized = true;
    return Status::success();
}

void BodyAudioStreamModulator::reset() noexcept
{
    mCarrierIndex = 0U;
}

// -----------------------------------------------------------------------------
// BodyAudioStreamModulator::render_window  (symbols -> 48 kHz audio)
// -----------------------------------------------------------------------------
// 50K view: Render a window of tribit symbols to real passband audio, stateful
//   across windows (retains carrier phase).
// Detailed view: Convolve the 8-PSK symbols with the root-raised-cosine taps
//   (rolloff 0.25, 20 samples/symbol), scale by mAmplitudeScale (0.98 of the worst
//   polyphase peak - headroom), and upconvert to the 1800 Hz carrier. Symbols outside
//   the window are treated as zero; carrier phase carries to the next call.
// 5th-grade view: Smooth each symbol into a pulse and ride it up onto the 1800 Hz
//   tone, remembering where the wave was so windows join seamlessly.
// -----------------------------------------------------------------------------
Status BodyAudioStreamModulator::render_window(BitSpan symbols, std::size_t first_output_symbol, std::size_t output_symbols, MutableSampleSpan output) noexcept
{
    if (!mInitialized)
    {
        return {StatusCode::invalid_argument, "body audio stream modulator is not initialized"};
    }

    if (first_output_symbol > symbols.size() || output_symbols > symbols.size() - first_output_symbol)
    {
        return {StatusCode::invalid_argument, "body audio output range is outside the symbol window"};
    }

    const auto required_samples = output_symbols * body_audio_samples_per_symbol;

    if (output.size() < required_samples)
    {
        return {StatusCode::buffer_too_small, "body audio output too small"};
    }

    constexpr auto samples_per_symbol = body_audio_samples_per_symbol;
    constexpr auto half_span = body_audio_shaping_span_symbols * body_audio_samples_per_symbol;
    const auto first_output_sample = first_output_symbol * samples_per_symbol;

    for (std::size_t output_index = 0U; output_index < required_samples; ++output_index)
    {
        IQSample baseband{};
        const auto window_output_index = first_output_sample + output_index;
        // Symbol s reaches this sample through tap window_output_index + half_span -
        // s * samples_per_symbol, inside the filter's [0, 2 * half_span] exactly for s
        // from ceil((window_output_index - half_span) / samples_per_symbol), or 0, to
        // floor((window_output_index + half_span) / samples_per_symbol). The symbols
        // are summed in increasing order, as the signed form of this loop did.
        const auto first_symbol = window_output_index > half_span ? (window_output_index - half_span + samples_per_symbol - 1U) / samples_per_symbol : 0U;
        const auto end_symbol = std::min((window_output_index + half_span) / samples_per_symbol + 1U, symbols.size());

        for (auto symbol = first_symbol; symbol < end_symbol; ++symbol)
        {
            const auto tap = window_output_index + half_span - symbol * samples_per_symbol;

            if (tap < mTaps.size())
            {
                baseband += psk8_symbol(symbols[symbol]) * mTaps[tap];
            }
        }

        output[output_index] = mAmplitudeScale * std::real(baseband * body_carrier_table[mCarrierIndex]);

        if (++mCarrierIndex == body_carrier_period_samples)
        {
            mCarrierIndex = 0U;
        }
    }

    return Status::success();
}

} // namespace modem::m110a
