// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#include "tests/walkthrough/front_end_steps.hpp"

#include "modem/common/convert.hpp"
#include "modem/common/dsp/timing_recovery.hpp"
#include "modem/m110a/body_audio_stream_frontend.hpp"
#include "modem/m110a/body_waveform.hpp"
#include "modem/m110a/scrambler.hpp"
#include "tests/support/front_end_harness.hpp"
#include "tests/support/test_waveform.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <print>
#include <vector>

namespace modem_test::front_end_steps
{

namespace
{

using ::modem::common::as_real;
using ::modem::common::CarrierTracker;
using ::modem::common::floor_to;
using ::modem::common::make_root_raised_cosine_taps;
using ::modem::common::narrow;
using ::modem::common::Status;
using ::modem::common::StatusCode;
using ::modem::common::widen;
using ::modem::m110a::body_carrier_period_samples;
using ::modem::m110a::body_carrier_table;
using ::modem::m110a::BodyAudioStreamFrontend;
using ::modem::m110a::early_late_timing_error;

constexpr std::size_t samples_per_symbol = ::modem::m110a::body_audio_samples_per_symbol; // 20 at 48 kHz
constexpr std::size_t half_symbol = samples_per_symbol / 2U;
constexpr float pi = std::numbers::pi_v<float>;

// The two-character message from Part 2, with the truth of what was sent.
Result<TestWaveform> hi_waveform()
{
    WaveformSpec spec;
    spec.payload = {'H', 'i'};
    return make_test_waveform(spec);
}

// The audio through a front end, 480 samples at a time unless the options say otherwise.
Result<FrontEndRunner> run(const std::vector<float>& audio, const FrontEndOptions& options = {})
{
    FrontEndRunner runner;
    auto status = runner.initialize(options);
    if (status.is_ok())
    {
        status = runner.feed(audio);
    }
    if (!status.is_ok())
    {
        return status;
    }
    return runner;
}

float degrees_0_to_360(IQSample value) noexcept
{
    auto degrees = std::arg(value) * 180.0F / pi;
    return degrees < 0.0F ? degrees + 360.0F : degrees;
}

bool same_bits(IQSample a, IQSample b) noexcept
{
    return std::bit_cast<std::uint32_t>(a.real()) == std::bit_cast<std::uint32_t>(b.real()) && std::bit_cast<std::uint32_t>(a.imag()) == std::bit_cast<std::uint32_t>(b.imag());
}

// The front end's on-time sample for burst symbol `symbol`, once the score has lined the
// stream up with what was sent.
IQSample on_time_of(const FrontEndRunner& runner, const BurstScore& score, std::size_t symbol)
{
    const auto k = narrow<std::int64_t>(symbol) + score.offset - narrow<std::int64_t>(runner.symbol_base());
    return runner.symbols()[narrow<std::size_t>(k)].on_time;
}

} // namespace

// --- Mixing down to baseband ---

CarrierTable carrier_table()
{
    // 1800 cycles a second at 48,000 samples a second is 3 cycles every 80 samples, so 80
    // phasors cover the carrier exactly; entry n turns 3n/80 of a cycle, 13.5 degrees a sample.
    CarrierTable step;
    for (std::size_t n = 0U; n < body_carrier_period_samples; ++n)
    {
        const auto cycles = widen<double>(as_real((3U * n) % body_carrier_period_samples)) / 80.0;
        const auto exact = std::polar(1.0, 2.0 * std::numbers::pi * cycles);
        const auto entry = body_carrier_table[n];
        step.worst_error = std::max({step.worst_error, std::fabs(widen<double>(entry.real()) - exact.real()), std::fabs(widen<double>(entry.imag()) - exact.imag())});

        if (n < 8U || n % 20U == 0U)
        {
            step.shown.push_back({n, 360.0 * cycles, entry});
        }
    }

    step.quadrants_exact = body_carrier_table[0] == IQSample{1.0F, 0.0F} && body_carrier_table[20] == IQSample{0.0F, -1.0F} && body_carrier_table[40] == IQSample{-1.0F, 0.0F} &&
                           body_carrier_table[60] == IQSample{0.0F, 1.0F};
    return step;
}

void print(const CarrierTable& step)
{
    std::println("  n   phase (deg)      cos        sin");
    for (const auto& row : step.shown)
    {
        std::println("{:3}   {:8.1f}   {:+9.6f}  {:+9.6f}", row.n, row.degrees, row.entry.real(), row.entry.imag());
    }
}

Result<ToneMix> tone_mix(std::uint32_t tone_hz)
{
    constexpr std::uint32_t sample_rate = 48000U;

    // The angle is reduced to one cycle in whole numbers first, so the tone is exact.
    std::vector<float> tone(sample_rate);
    for (std::uint32_t n = 0U; n < sample_rate; ++n)
    {
        tone[n] = 0.5F * std::cos(2.0F * pi * as_real((tone_hz * n) % sample_rate) / as_real(sample_rate));
    }

    const auto runner = run(tone);
    if (!runner)
    {
        return runner.error();
    }
    const auto& symbols = runner.value().symbols();
    constexpr std::size_t settled = 20U; // skip the filter filling up
    if (symbols.size() <= settled + 100U)
    {
        return Status{StatusCode::unavailable, "too few symbols from one second of tone"};
    }

    ToneMix step{.tone_hz = tone_hz, .smallest = 1.0e9F};
    float step_sum{};
    for (std::size_t k = settled + 1U; k < symbols.size(); ++k)
    {
        step_sum += std::arg(symbols[k].on_time * std::conj(symbols[k - 1U].on_time));
        step.smallest = std::min(step.smallest, std::abs(symbols[k].on_time));
        step.largest = std::max(step.largest, std::abs(symbols[k].on_time));
    }
    step.step = step_sum / as_real(symbols.size() - settled - 1U);
    step.expected_step = 2.0F * pi * (as_real(tone_hz) - 1800.0F) / 2400.0F;
    return step;
}

void print(const ToneMix& step)
{
    std::println("{} Hz tone: magnitude {:.4f} to {:.4f}, phase step {:+.6f} rad/symbol (expected {:+.6f})", step.tone_hz, step.smallest, step.largest, step.step, step.expected_step);
}

Result<Tribit6Mix> tribit6_mix()
{
    const auto made = hi_waveform();
    if (!made)
    {
        return made.error();
    }
    const auto& waveform = made.value();
    const auto& burst = waveform.bursts.front();

    // Part 2 ended on the randomizer's sixth value, 6, and its worked symbol was tribit 6, 270
    // degrees. In the Hi burst they meet at body symbol 5: a coded 0 plus randomizer value 6.
    // The transmitter puts each symbol's peak at 20 times its number.
    const auto symbol = burst.preamble_symbols + 5U;
    if (burst.tribits[symbol] != 6U)
    {
        return Status{StatusCode::invalid_argument, "body symbol 5 of the Hi burst is not tribit 6"};
    }
    const auto peak = burst.first_sample + symbol * samples_per_symbol;

    // Each mixed sample is the symbol plus a copy spinning at 3600 Hz (and a little of the
    // neighboring symbols); the matched filter averages the spin away.
    Tribit6Mix step;
    for (auto n = peak - 4U; n <= peak + 4U; ++n)
    {
        const auto carrier = body_carrier_table[n % body_carrier_period_samples];
        step.rows.push_back({n, waveform.audio[n], degrees_0_to_360(carrier), 2.0F * waveform.audio[n] * std::conj(carrier)});
    }

    const auto runner = run(waveform.audio);
    if (!runner)
    {
        return runner.error();
    }
    const auto score = score_burst(runner.value(), burst, 0U);
    if (!score.locked)
    {
        return Status{StatusCode::unavailable, "the timing loop did not lock"};
    }
    step.on_time = on_time_of(runner.value(), score, symbol);
    step.degrees = degrees_0_to_360(step.on_time);
    step.tribit = nearest_tribit(step.on_time);
    return step;
}

void print(const Tribit6Mix& step)
{
    std::println("   sample     audio   carrier (deg)   mixed I   mixed Q");
    for (const auto& row : step.rows)
    {
        std::println("{:9}  {:+8.4f}   {:12.1f}   {:+7.4f}   {:+7.4f}", row.sample, row.audio, row.carrier_degrees, row.mixed.real(), row.mixed.imag());
    }
    std::println("after the matched filter: on-time ({:+.3f}, {:+.3f}), {:.1f} degrees, tribit {}", step.on_time.real(), step.on_time.imag(), step.degrees, step.tribit);
}

// --- The matched filter ---

Result<ReceiveFilter> receive_filter()
{
    std::array<float, BodyAudioStreamFrontend::matched_filter_taps> taps{};
    const auto status = make_root_raised_cosine_taps(samples_per_symbol, BodyAudioStreamFrontend::matched_filter_rolloff, taps);
    if (!status.is_ok())
    {
        return status;
    }

    ReceiveFilter step{.taps = taps.size(), .middle = taps.size() / 2U};
    for (const auto tap : taps)
    {
        step.energy += tap * tap;
    }
    step.peak = taps[step.middle];
    step.peak_in_middle = std::ranges::max_element(taps) == taps.begin() + narrow<std::ptrdiff_t>(step.middle);
    for (std::size_t index = 0U; index < step.middle; ++index)
    {
        step.worst_asymmetry = std::max(step.worst_asymmetry, std::fabs(taps[index] - taps[taps.size() - 1U - index]));
    }
    return step;
}

void print(const ReceiveFilter& step)
{
    std::println("receive filter: {} taps, energy {:.6f}, peak {:.4f} at tap {}", step.taps, step.energy, step.peak, step.middle);
}

Result<PulseNeighbors> pulse_neighbors(float receive_rolloff)
{
    std::vector<float> transmit(::modem::m110a::body_audio_shaping_taps);
    std::vector<float> receive(BodyAudioStreamFrontend::matched_filter_taps);
    auto status = make_root_raised_cosine_taps(samples_per_symbol, 0.25F, transmit);
    if (status.is_ok())
    {
        status = make_root_raised_cosine_taps(samples_per_symbol, receive_rolloff, receive);
    }
    if (!status.is_ok())
    {
        return status;
    }

    // The transmit pulse through the receive filter: what one symbol looks like at the
    // filter's output, and how much of it lands on the other symbols' instants.
    std::vector<float> combined(transmit.size() + receive.size() - 1U);
    for (std::size_t i = 0U; i < transmit.size(); ++i)
    {
        for (std::size_t j = 0U; j < receive.size(); ++j)
        {
            combined[i + j] += transmit[i] * receive[j];
        }
    }

    const std::size_t center = combined.size() / 2U;
    PulseNeighbors step{.receive_rolloff = receive_rolloff, .peak = combined[center]};
    for (std::size_t offset = samples_per_symbol; offset <= center; offset += samples_per_symbol)
    {
        step.largest_neighbor = std::max({step.largest_neighbor, std::fabs(combined[center - offset]), std::fabs(combined[center + offset])});
    }
    step.decibels_below = 20.0F * std::log10(step.peak / step.largest_neighbor);
    return step;
}

void print(const PulseNeighbors& step)
{
    std::println("transmit 0.25, receive {:.2f}: peak {:.4f}, largest neighbor {:.5f} ({:.1f} dB below)", step.receive_rolloff, step.peak, step.largest_neighbor, step.decibels_below);
}

// --- Finding the sampling instant ---

Result<TimingError> timing_error()
{
    std::array<float, BodyAudioStreamFrontend::matched_filter_taps> taps{};
    const auto status = make_root_raised_cosine_taps(samples_per_symbol, BodyAudioStreamFrontend::matched_filter_rolloff, taps);
    if (!status.is_ok())
    {
        return status;
    }

    // The receive pulse itself stands in for the filtered signal: read it at the three
    // points, as the loop does, with the on-time point on or off the peak.
    const std::size_t peak = taps.size() / 2U;
    const auto error_at = [&taps](std::size_t on_time, float carrier_phase)
    {
        const auto turn = std::polar(1.0F, carrier_phase);
        return early_late_timing_error(IQSample{taps[on_time - half_symbol]} * turn, IQSample{taps[on_time]} * turn, IQSample{taps[on_time + half_symbol]} * turn);
    };
    return TimingError{
        .on_peak = error_at(peak, 0.0F),
        .early = error_at(peak - 4U, 0.0F),
        .late = error_at(peak + 4U, 0.0F),
        .early_turned = error_at(peak - 4U, 1.0F),
    };
}

void print(const TimingError& step)
{
    std::println("on the peak {:+.6f}, 4 samples early {:+.6f}, 4 samples late {:+.6f}, early and turned by 1 rad {:+.6f}", step.on_peak, step.early, step.late, step.early_turned);
}

// --- Keeping time for hours ---

FloatPosition float_position()
{
    FloatPosition step;
    step.position = 67108864.0F; // 2^26 samples, about 23 minutes at 48 kHz
    step.position += 20.0F;
    step.earlier = 16777216.0F; // 2^24
    step.earlier += 20.5F;

    // The front end's way: whole samples in an integer, the fraction in a float.
    step.whole = 67108864U;
    step.fraction = 0.5F;
    step.fraction += 20.5F;
    const auto whole_part = std::floor(step.fraction);
    step.whole += floor_to<std::uint64_t>(whole_part);
    step.fraction -= whole_part;
    return step;
}

void print(const FloatPosition& step)
{
    std::println("float: 2^26 + 20 = {:.1f}; 2^24 + 20.5 = {:.1f}", step.position, step.earlier);
    std::println("whole + fraction: {} + {:.1f}", step.whole, step.fraction);
}

// --- Any block size, the same symbols ---

Result<BlockSizes> block_sizes()
{
    const auto made = hi_waveform();
    if (!made)
    {
        return made.error();
    }
    const auto& audio = made.value().audio;
    const auto steady = run(audio, {.block_sizes = {480U}});
    const auto ragged = run(audio, {.block_sizes = {1U, 7U, 33U, 160U, 480U}});
    if (!steady || !ragged)
    {
        return steady ? ragged.error() : steady.error();
    }

    const auto& a = steady.value().symbols();
    const auto& b = ragged.value().symbols();
    BlockSizes step{.samples = audio.size(), .symbols = a.size()};
    for (std::size_t k = 0U; k < std::min(a.size(), b.size()); ++k)
    {
        step.identical += same_bits(a[k].early, b[k].early) && same_bits(a[k].on_time, b[k].on_time) ? 1U : 0U;
    }
    if (a.size() != b.size())
    {
        step.identical = 0U;
    }
    return step;
}

void print(const BlockSizes& step)
{
    std::println("{} samples, {} symbols, {} identical bit for bit", step.samples, step.symbols, step.identical);
}

// --- Carrier tracking, part one ---

Result<CarrierTracking> carrier_tracking()
{
    constexpr float proportional_gain = 0.02F; // the receiver's carrier-loop gains
    constexpr float integral_gain = 0.0002F;
    std::uint32_t state = 1U;
    const auto next_symbol = [&state]
    {
        state = state * 1664525U + 1013904223U; // a small fixed pseudo-random sequence
        return std::polar(1.0F, pi / 4.0F * as_real(state >> 29U));
    };

    CarrierTracker tracker;
    const auto status = tracker.configure(proportional_gain, integral_gain);
    if (!status.is_ok())
    {
        return status;
    }

    // Symbols turned by a fixed 0.5 rad: the loop should learn the turn.
    CarrierTracking step;
    tracker.reset();
    for (int k = 0; k < 2000; ++k)
    {
        const auto decision = next_symbol();
        const auto corrected = tracker.update(decision * std::polar(1.0F, 0.5F), decision);
        step.phase_residual = std::fabs(std::arg(corrected * std::conj(decision)));
    }
    step.learned_phase = tracker.phase_radians();

    // Symbols turning at 5 Hz: the loop should learn the frequency.
    const auto per_symbol = 2.0F * pi * 5.0F / 2400.0F;
    float angle{};
    tracker.reset();
    for (int k = 0; k < 5000; ++k)
    {
        const auto decision = next_symbol();
        const auto corrected = tracker.update(decision * std::polar(1.0F, angle), decision);
        step.frequency_residual = std::fabs(std::arg(corrected * std::conj(decision)));
        angle = std::remainder(angle + per_symbol, 2.0F * pi);
    }
    step.learned_hz = tracker.frequency_radians_per_symbol() * 2400.0F / (2.0F * pi);
    return step;
}

void print(const CarrierTracking& step)
{
    std::println("fixed 0.5 rad offset: learned {:.4f} rad, last symbol {:.5f} rad from its decision", step.learned_phase, step.phase_residual);
    std::println("5 Hz offset: learned {:.3f} Hz, last symbol {:.5f} rad from its decision", step.learned_hz, step.frequency_residual);
}

// --- Walk the symbols back out ---

Result<Walk> walk()
{
    const auto made = hi_waveform();
    if (!made)
    {
        return made.error();
    }
    const auto& burst = made.value().bursts.front();
    const auto runner = run(made.value().audio);
    if (!runner)
    {
        return runner.error();
    }
    const auto score = score_burst(runner.value(), burst, 0U);
    if (!score.locked)
    {
        return Status{StatusCode::unavailable, "the timing loop did not lock"};
    }

    ::modem::m110a::BodyDataRandomizer randomizer;
    randomizer.reset();
    Walk step{.every_tribit_matches = true, .data_is_bpsk = true};

    // 600L frames are 20 data symbols then 20 probe symbols; only data symbols carry coded
    // bits. Keep the first six body symbols, then the first that carries a coded 1.
    bool found_a_one = false;
    for (std::size_t j = 0U; j < 400U && !found_a_one; ++j)
    {
        const auto on_time = on_time_of(runner.value(), score, burst.preamble_symbols + j);
        WalkRow row{
            .body_symbol = j,
            .on_time = on_time,
            .degrees = degrees_0_to_360(on_time),
            .received = nearest_tribit(on_time),
            .sent = burst.tribits[burst.preamble_symbols + j],
            .randomizer = randomizer.next_tribit(),
        };
        row.data = narrow<std::uint8_t>((row.received + 8U - row.randomizer) % 8U);
        const bool is_data = j % 40U < 20U;
        step.every_tribit_matches = step.every_tribit_matches && row.received == row.sent;
        step.data_is_bpsk = step.data_is_bpsk && (!is_data || row.data == 0U || row.data == 4U);

        found_a_one = j >= 6U && is_data && row.data == 4U;
        if (j < 6U || found_a_one)
        {
            step.shown.push_back(row);
        }
    }
    return step;
}

void print(const Walk& step)
{
    std::println("body  on-time I, Q       angle  tribit  sent  randomizer  tribit - randomizer  coded bit");
    for (const auto& row : step.shown)
    {
        if (row.body_symbol >= 6U)
        {
            std::println(" ...");
        }
        std::println("{:4}  {:+7.3f} {:+7.3f}  {:6.1f}  {:6}  {:4}  {:10}  {:19}  {:9}", row.body_symbol, row.on_time.real(), row.on_time.imag(), row.degrees, row.received, row.sent, row.randomizer,
                     row.data, row.data == 4U ? 1 : 0);
    }
}

} // namespace modem_test::front_end_steps
