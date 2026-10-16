// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#include "tests/walkthrough/acquisition_steps.hpp"

#include "modem/common/convert.hpp"
#include "modem/m110a/body_audio_stream_frontend.hpp"
#include "modem/m110a/body_waveform.hpp"
#include "modem/m110a/demapper.hpp"
#include "modem/m110a/scrambler.hpp"
#include "modem/m110a/synchronization.hpp"
#include "tests/support/front_end_harness.hpp"
#include "tests/support/impairments.hpp"
#include "tests/support/test_channel.hpp"
#include "tests/support/test_waveform.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace modem_test::acquisition_steps
{

namespace
{

using ::modem::common::as_real;
using ::modem::common::BodyInterleave;
using ::modem::common::DataRate;
using ::modem::common::narrow;
using ::modem::common::round_to;
using ::modem::common::Status;
using ::modem::common::StatusCode;
using ::modem::common::widen;
using ::modem::m110a::acquire_body_preamble;
using ::modem::m110a::body_candidate_is_valid;
using ::modem::m110a::body_channel_symbol_tribit;
using ::modem::m110a::body_designators;
using ::modem::m110a::body_preamble_acquisition_prefix_symbols;
using ::modem::m110a::body_preamble_segment_symbols;
using ::modem::m110a::body_preamble_symbols;
using ::modem::m110a::body_prefix_correlation;
using ::modem::m110a::BodyAcquisition;
using ::modem::m110a::BodyAcquisitionConfig;
using ::modem::m110a::BodyAudioStreamFrontend;
using ::modem::m110a::BodyDesignatorState;
using ::modem::m110a::BodyMode;
using ::modem::m110a::characterize_body_candidate;
using ::modem::m110a::estimate_body_prefix_frequency_radians;
using ::modem::m110a::generate_body_preamble;
using ::modem::m110a::make_body_prefix_reference;
using ::modem::m110a::psk8_symbol;
using ::modem::m110a::recognize_body_designators;
using ::modem::m110a::SyncRandomizer;
using ::modem::m110a::TimingRateMode;
using ::modem::m110a::tribit_add;

constexpr float pi = std::numbers::pi_v<float>;
constexpr float symbol_rate_hz = 2400.0F;
constexpr std::size_t segment_symbols = body_preamble_segment_symbols;           // 480
constexpr std::size_t prefix_symbols = body_preamble_acquisition_prefix_symbols; // 288
constexpr std::size_t chips = 32U;                                               // per channel symbol
// Where the designators and the countdown sit among a segment's 15 channel symbols.
constexpr std::size_t d1_position = 9U;
constexpr std::size_t d2_position = 10U;
constexpr std::size_t c1_position = 11U;
constexpr std::size_t c2_position = 12U;
constexpr std::size_t c3_position = 13U;
constexpr BodyMode mode_600l{DataRate::bps600, BodyInterleave::long_block};
constexpr BodyMode mode_600s{DataRate::bps600, BodyInterleave::short_block};

// Half a second of line before the burst, so acquisition has noise to search through. The
// odd ten samples put the burst's symbol peaks where the front end starts sampling.
constexpr std::size_t lead_samples = 24010U;

// The acquisition settings the walkthrough uses: the operating point of the streaming
// receiver later in the series, a 0.35 correlation floor and a 120 Hz carrier gate, searching
// every start. (The library's
// default floor is a cautious 0.75.)
BodyAcquisitionConfig acquisition_config() noexcept
{
    BodyAcquisitionConfig config;
    config.minimum_correlation = 0.35F;
    return config;
}

const char* mode_name(::modem::m110a::BodyModeRecognition recognition) noexcept
{
    if (recognition.state != BodyDesignatorState::supported)
    {
        return recognition.state == BodyDesignatorState::recognized_unsupported ? "not implemented" : "invalid";
    }
    switch (recognition.mode.interleave)
    {
    case BodyInterleave::long_block:
        return "long interleave";
    case BodyInterleave::short_block:
        return "short interleave";
    default:
        return "no interleave";
    }
}

std::array<IQSample, prefix_symbols> prefix_reference_points() noexcept
{
    std::array<IQSample, prefix_symbols> reference{};
    make_body_prefix_reference(reference);
    return reference;
}

// The sync randomizer's 32 tribits. Its sequence is 32 long, so every 32-chip block of the
// preamble adds the same 32 values.
std::array<std::uint8_t, chips> sync_tribits() noexcept
{
    SyncRandomizer randomizer;
    std::array<std::uint8_t, chips> values{};
    for (auto& value : values)
    {
        value = randomizer.next_tribit();
    }
    return values;
}

// What a burst looks like to acquisition: the front end's on-time samples, one per symbol,
// and where the burst really is in them.
struct Received
{
    std::vector<IQSample> symbols{};
    std::size_t first_preamble{}; // stream symbol of the burst's first symbol
    std::size_t first_body{};
};

// The "Hi" burst from Parts 2 and 3 (600 bps, long interleave), after half a second of
// line, through the test channel and the receive front end.
Result<Received> received_hi(const ChannelSpec& channel)
{
    WaveformSpec spec;
    spec.payload = {'H', 'i'};
    auto made = make_test_waveform(spec);
    if (!made)
    {
        return made.error();
    }
    auto& waveform = made.value();
    waveform.audio.insert(waveform.audio.begin(), lead_samples, 0.0F);
    if (const auto status = apply_channel(waveform.audio, channel); !status.is_ok())
    {
        return status;
    }

    FrontEndRunner runner;
    auto status = runner.initialize();
    if (status.is_ok())
    {
        status = runner.feed(waveform.audio);
    }
    if (!status.is_ok())
    {
        return status;
    }

    Received received;
    received.symbols = on_time_symbols(runner);
    const auto& burst = waveform.bursts.front();
    received.first_preamble = narrow<std::size_t>(locate_burst(runner, burst, lead_samples));
    received.first_body = received.first_preamble + burst.preamble_symbols;
    return received;
}

// The walkthrough's line: noise 10 dB down and the carrier 37 Hz off.
constexpr float walkthrough_snr_db = 10.0F;
constexpr std::int32_t walkthrough_offset_hz = 37;

ChannelSpec walkthrough_channel(float snr_db = walkthrough_snr_db) noexcept
{
    ChannelSpec channel;
    channel.snr_db = snr_db;
    channel.carrier_offset_hz = walkthrough_offset_hz;
    channel.seed = 4U;
    return channel;
}

// A start through the scan's first two gates, as acquire_body_preamble tests it: the carrier
// estimate over the prefix, then the prefix correlation with that carrier removed.
struct StartTest
{
    float frequency_hz{};
    float correlation{};
    float phase{};
};

StartTest test_start(std::span<const IQSample> symbols, std::size_t start, std::span<const IQSample> reference) noexcept
{
    StartTest result;
    const auto radians = estimate_body_prefix_frequency_radians(symbols, start, reference);
    result.frequency_hz = radians * symbol_rate_hz / (2.0F * pi);
    result.correlation = body_prefix_correlation(symbols, start, reference, radians, result.phase);
    return result;
}

// --- The clock-offset loop: driving the rate integrator as the receiver does ---

constexpr std::size_t samples_per_symbol = ::modem::m110a::body_audio_samples_per_symbol; // 20 at 48 kHz
constexpr std::size_t clock_lead_samples = 5U;                                            // a few samples of silence first
constexpr std::size_t track_from_symbol = segment_symbols;                                // the first candidate: a segment in
constexpr std::size_t learning_window_symbols = 6U * segment_symbols;                     // 2880, the window the first design froze after
constexpr BodyMode mode_2400l{DataRate::bps2400, BodyInterleave::long_block};
constexpr BodyMode mode_2400s{DataRate::bps2400, BodyInterleave::short_block};

// The receiver's judgements of the integrator (its timing-tracker settings): the mean of its
// rate from 5 s after the first candidate, judged at 20, 30 and 40 s against a threshold that
// rises from 0.0025 to 0.0035 samples per symbol (125 to 175 ppm); below it, the integrator is
// retired, held at zero, for the rest of the burst. A burst over before 20 s keeps it.
constexpr std::size_t mean_from_symbols = 12000U;
constexpr std::array<std::size_t, 3> judge_at_symbols{48000U, 72000U, 96000U};
constexpr std::array<float, 3> retire_rates{0.0025F, 0.0030F, 0.0035F};

enum class Loop : std::uint8_t
{
    type_1,  // no rate integrator
    running, // the integrator tracks from the first candidate to the end of the burst
};

// Where the loop reads burst symbol k: the transmit clock stretches the grid and the receive
// filter delays it 80 samples.
double position_of_burst_symbol(std::size_t k, float clock_ppm)
{
    const double ratio = 1.0 + widen<double>(clock_ppm) * 1.0e-6;
    return widen<double>(as_real(clock_lead_samples)) + widen<double>(as_real(samples_per_symbol * k)) / ratio + 80.0;
}

// The stream sample just before the loop reads burst symbol k: a mode change here lands
// within a symbol of k.
std::uint64_t sample_before_burst_symbol(std::size_t k, float clock_ppm)
{
    return round_to<std::uint64_t>(position_of_burst_symbol(k, clock_ppm)) - samples_per_symbol / 2U;
}

struct LoopRun
{
    BurstTruth burst{};
    std::vector<GridOffset> offsets{};
    float grid{};        // the rate the clock really has, samples per symbol beyond 20
    float rate_at_end{}; // the integrator when the burst ended
};

// One burst through the channel and the front end, one symbol a block so the loop's position
// is kept at every symbol. The integrator, when it runs, tracks from the first candidate, a
// segment into the preamble, as the receiver starts it; retire_at_symbol holds it at zero from
// that burst symbol on, as the receiver's judgement does.
Result<LoopRun> run_loop(BodyMode mode, const ChannelSpec& channel, Loop loop, std::size_t body_seconds = 10U, std::optional<std::size_t> retire_at_symbol = std::nullopt)
{
    WaveformSpec spec;
    spec.mode = mode;
    spec.random_payload_bytes = std::to_underlying(mode.data_rate) * body_seconds / 8U;
    spec.seed = channel.seed;
    auto made = make_test_waveform(spec);
    if (!made)
    {
        return made.error();
    }
    auto audio = made.value().audio;
    if (const auto status = apply_channel(audio, channel); !status.is_ok())
    {
        return status;
    }
    std::vector<float> stream(clock_lead_samples, 0.0F);
    stream.insert(stream.end(), audio.begin(), audio.end());

    const auto ppm = channel.clock_ppm.value_or(0.0F);
    const auto& burst = made.value().bursts.front();
    FrontEndOptions options;
    options.block_sizes = {samples_per_symbol};
    if (loop == Loop::running)
    {
        options.timing_integral_gain = BodyAudioStreamFrontend::default_timing_integral_gain;
        options.timing_rate_changes.push_back({sample_before_burst_symbol(track_from_symbol, ppm), TimingRateMode::tracking});
        if (retire_at_symbol)
        {
            options.timing_rate_changes.push_back({sample_before_burst_symbol(*retire_at_symbol, ppm), TimingRateMode::held});
        }
    }

    FrontEndRunner runner;
    auto status = runner.initialize(options);
    if (status.is_ok())
    {
        status = runner.feed(stream);
    }
    if (!status.is_ok())
    {
        return status;
    }

    LoopRun run;
    run.burst = burst;
    run.offsets = grid_offsets(runner, burst, clock_lead_samples, ppm);
    run.grid = grid_rate(ppm);
    run.rate_at_end = runner.timing_rate();
    return run;
}

// The loop against the grid through the body proper, a segment in: its mean offset and how far
// it strayed from that.
std::pair<double, double> body_offset_and_wander(const LoopRun& run)
{
    const auto from = widen<double>(as_real(run.burst.preamble_symbols + segment_symbols));
    double sum{};
    std::size_t points{};
    for (const auto& point : run.offsets)
    {
        if (point.burst_symbol >= from)
        {
            sum += point.offset_samples;
            ++points;
        }
    }
    const auto mean = points == 0U ? 0.0 : sum / widen<double>(as_real(points));
    double wander{};
    for (const auto& point : run.offsets)
    {
        if (point.burst_symbol >= from)
        {
            wander = std::max(wander, std::fabs(point.offset_samples - mean));
        }
    }
    return {mean, wander};
}

// Whole symbols the loop gained or lost through the body, net. Each time its offset from the
// nearest peak jumps by more than half a symbol it has crossed to a neighbouring symbol's peak,
// one way or the other; a loop chattering across the edge crosses back, and those cancel.
std::size_t body_slips(const LoopRun& run)
{
    const auto from = widen<double>(as_real(run.burst.preamble_symbols + segment_symbols));
    const auto half_symbol = widen<double>(as_real(samples_per_symbol / 2U));
    std::int64_t net{};
    std::optional<double> previous{};
    for (const auto& point : run.offsets)
    {
        if (point.burst_symbol >= from)
        {
            if (previous && point.offset_samples - *previous > half_symbol)
            {
                --net;
            }
            else if (previous && point.offset_samples - *previous < -half_symbol)
            {
                ++net;
            }
            previous = point.offset_samples;
        }
    }
    return narrow<std::size_t>(net < 0 ? -net : net);
}

// The integrator as it stood when the loop reached burst symbol k (the last point before).
float rate_at_symbol(const LoopRun& run, std::size_t k)
{
    float rate{};
    for (const auto& point : run.offsets)
    {
        if (point.burst_symbol > widen<double>(as_real(k)))
        {
            break;
        }
        rate = point.timing_rate;
    }
    return rate;
}

// The integrator's mean rate over burst symbols [from, to), as the receiver judges it: the
// front end sums the rate every symbol, and the bench keeps it at every symbol.
double mean_rate(const LoopRun& run, std::size_t from, std::size_t to)
{
    double sum{};
    std::size_t points{};
    for (const auto& point : run.offsets)
    {
        if (point.burst_symbol >= widen<double>(as_real(from)) && point.burst_symbol < widen<double>(as_real(to)))
        {
            sum += widen<double>(point.timing_rate);
            ++points;
        }
    }
    return points == 0U ? 0.0 : sum / widen<double>(as_real(points));
}

float rate_error_ppm(float rate, float grid) noexcept
{
    return (rate - grid) / as_real(samples_per_symbol) * 1.0e6F;
}

double rate_ppm(double rate) noexcept
{
    return rate / widen<double>(as_real(samples_per_symbol)) * 1.0e6;
}

// A rate error for the tables, whole ppm with its sign; a rate within half a ppm is 0.
std::string ppm_off(float ppm)
{
    const auto whole = std::round(ppm);
    return whole == 0.0F ? std::string{"0 ppm off"} : std::format("{:+.0f} ppm off", whole);
}

std::string mode_label(BodyMode mode)
{
    return std::format("{}{}", std::to_underlying(mode.data_rate), mode.interleave == BodyInterleave::long_block ? "L" : "S");
}

// The type-1 loop started at one of the twenty sample phases of a symbol: the burst symbol
// from which it sits within a sample of the transmitter's grid for the rest of the preamble.
struct PullIn
{
    std::optional<std::size_t> pulled_in_at{};
    std::uint32_t moves{};
};

Result<PullIn> pull_in_from(BodyMode mode, const ChannelSpec& channel, std::size_t lead, bool search)
{
    WaveformSpec spec;
    spec.mode = mode;
    spec.random_payload_bytes = 1500U;
    spec.seed = channel.seed;
    auto made = make_test_waveform(spec);
    if (!made)
    {
        return made.error();
    }
    auto audio = made.value().audio;
    if (const auto status = apply_channel(audio, channel); !status.is_ok())
    {
        return status;
    }
    std::vector<float> stream(lead, 0.0F);
    stream.insert(stream.end(), audio.begin(), audio.end());

    FrontEndOptions options;
    options.block_sizes = {samples_per_symbol};
    options.phase_search = search;
    FrontEndRunner runner;
    auto status = runner.initialize(options);
    if (status.is_ok())
    {
        status = runner.feed(stream);
    }
    if (!status.is_ok())
    {
        return status;
    }

    const auto& burst = made.value().bursts.front();
    const auto preamble = widen<double>(as_real(burst.preamble_symbols));
    PullIn result{.moves = runner.phase_search_moves()};
    std::optional<std::size_t> candidate;
    for (const auto& point : grid_offsets(runner, burst, lead, 0.0F))
    {
        if (point.burst_symbol >= preamble)
        {
            break;
        }
        if (std::fabs(point.offset_samples) < 1.0)
        {
            if (!candidate)
            {
                candidate = narrow<std::size_t>(std::lround(point.burst_symbol));
            }
        }
        else
        {
            candidate.reset();
        }
    }
    result.pulled_in_at = candidate;
    return result;
}

} // namespace

// --- The preamble ---

Result<PreambleSegments> preamble_segments()
{
    std::vector<std::uint8_t> tribits(body_preamble_symbols(mode_600l.interleave));
    if (const auto status = generate_body_preamble(mode_600l, tribits); !status.is_ok())
    {
        return status;
    }

    // Read each 32-chip block back to the channel symbol whose code, plus the sync
    // randomizer, made those chips.
    const auto sync = sync_tribits();
    PreambleSegments step{.segments = tribits.size() / segment_symbols, .prefix_on_every_segment = true, .countdown_runs_down = true, .every_chip_is_a_code = true};
    constexpr std::array<std::uint8_t, 9> prefix{0U, 1U, 3U, 0U, 1U, 3U, 1U, 2U, 0U};

    for (std::size_t segment = 0U; segment < step.segments; ++segment)
    {
        SegmentRow row{.segment = segment};
        for (std::size_t position = 0U; position < row.channel_symbols.size(); ++position)
        {
            const auto block = segment * segment_symbols + position * chips;
            bool found = false;
            for (std::uint8_t code = 0U; code < 8U && !found; ++code)
            {
                bool all = true;
                for (std::size_t chip = 0U; chip < chips && all; ++chip)
                {
                    all = tribits[block + chip] == tribit_add(body_channel_symbol_tribit(code, chip), sync[chip]);
                }
                if (all)
                {
                    row.channel_symbols[position] = code;
                    found = true;
                }
            }
            step.every_chip_is_a_code = step.every_chip_is_a_code && found;
        }

        for (std::size_t position = 0U; position < prefix.size(); ++position)
        {
            step.prefix_on_every_segment = step.prefix_on_every_segment && row.channel_symbols[position] == prefix[position];
        }
        // C1, C2, C3 each carry two bits of the count, sent as 4 + the two bits.
        const auto& s = row.channel_symbols;
        row.countdown = narrow<std::uint8_t>(((s[c1_position] & 3U) << 4U) | ((s[c2_position] & 3U) << 2U) | (s[c3_position] & 3U));
        step.countdown_runs_down = step.countdown_runs_down && row.countdown == step.segments - 1U - segment;

        if (segment < 2U || segment + 2U >= step.segments)
        {
            step.shown.push_back(row);
        }
        if (segment == 0U)
        {
            step.d1 = row.channel_symbols[d1_position];
            step.d2 = row.channel_symbols[d2_position];
        }
    }

    step.recognition = recognize_body_designators(step.d1, step.d2, true);
    return step;
}

void print(const PreambleSegments& step)
{
    std::println("segment   0 1 3 0 1 3 1 2 0 | D1 D2 | C1 C2 C3 | 0   countdown");
    for (const auto& row : step.shown)
    {
        if (row.segment == step.segments - 2U)
        {
            std::println("  ...");
        }
        const auto& s = row.channel_symbols;
        std::println("{:7}   {} {} {} {} {} {} {} {} {} |  {}  {} |  {}  {}  {} | {}   {:9}", row.segment, s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8], s[9], s[10], s[11], s[12], s[13],
                     s[14], row.countdown);
    }
    std::println("{} segments; prefix on every one: {}; countdown runs down to 0: {}", step.segments, step.prefix_on_every_segment ? "yes" : "no", step.countdown_runs_down ? "yes" : "no");
    std::println("D1 {}, D2 {} on a long preamble: {} bps, {}", step.d1, step.d2, std::to_underlying(step.recognition.mode.data_rate), mode_name(step.recognition));
}

// --- The reference ---

Result<PrefixReference> prefix_reference()
{
    const auto reference = prefix_reference_points();
    const auto sync = sync_tribits();
    PrefixReference step{.chips = reference.size(), .matches_every_segment = true};

    for (std::size_t chip = 0U; chip < 8U; ++chip)
    {
        const auto code = body_channel_symbol_tribit(0U, chip);
        const auto tribit = tribit_add(code, sync[chip]);
        step.shown.push_back({chip, code, sync[chip], tribit, reference[chip]});
    }

    std::vector<std::uint8_t> tribits(body_preamble_symbols(mode_600l.interleave));
    if (const auto status = generate_body_preamble(mode_600l, tribits); !status.is_ok())
    {
        return status;
    }
    for (std::size_t segment = 0U; segment < tribits.size() / segment_symbols; ++segment)
    {
        for (std::size_t chip = 0U; chip < reference.size(); ++chip)
        {
            step.matches_every_segment = step.matches_every_segment && reference[chip] == psk8_symbol(tribits[segment * segment_symbols + chip]);
        }
    }
    return step;
}

void print(const PrefixReference& step)
{
    std::println("chip  code  randomizer  tribit       I       Q");
    for (const auto& row : step.shown)
    {
        std::println("{:4}  {:4}  {:10}  {:6}  {:+6.3f}  {:+6.3f}", row.chip, row.code, row.randomizer, row.tribit, row.point.real(), row.point.imag());
    }
    std::println(" ...  {} chips; the same as the first {} symbols of every transmitted segment: {}", step.chips, step.chips, step.matches_every_segment ? "yes" : "no");
}

// --- Correlating so a fade does not hide it ---

FadingPrefix fading_prefix()
{
    const auto reference = prefix_reference_points();
    // One correlation over the whole prefix, normalized the way body_prefix_correlation is.
    const auto coherent = [&](std::span<const IQSample> input)
    {
        IQSample sum{};
        float energy{};
        for (std::size_t chip = 0U; chip < reference.size(); ++chip)
        {
            sum += input[chip] * std::conj(reference[chip]);
            energy += std::norm(input[chip]);
        }
        return std::abs(sum) / std::sqrt(energy * as_real(reference.size()));
    };

    FadingPrefix step;
    float phase{};
    step.clean_coherent = coherent(reference);
    step.clean_blocks = body_prefix_correlation(reference, 0U, reference, 0.0F, phase);

    // A fade that swings the path's phase half a turn after chip 160, a block boundary.
    auto flipped = reference;
    for (std::size_t chip = 160U; chip < flipped.size(); ++chip)
    {
        flipped[chip] = -flipped[chip];
    }
    step.flipped_coherent = coherent(flipped);
    step.flipped_blocks = body_prefix_correlation(flipped, 0U, reference, 0.0F, phase);
    return step;
}

void print(const FadingPrefix& step)
{
    std::println("                            one sum   nine blocks");
    std::println("clean prefix                 {:5.3f}       {:5.3f}", step.clean_coherent, step.clean_blocks);
    std::println("last 128 chips turned 180    {:5.3f}       {:5.3f}", step.flipped_coherent, step.flipped_blocks);
}

Result<CorrelationComb> correlation_comb()
{
    const auto received = received_hi(walkthrough_channel());
    if (!received)
    {
        return received.error();
    }
    const auto& symbols = received.value().symbols;
    const auto first = received.value().first_preamble;
    const auto reference = prefix_reference_points();
    CorrelationComb step{.snr_db = walkthrough_snr_db, .carrier_offset_hz = walkthrough_offset_hz};

    // Every start from 300 symbols before the burst to the end of its third segment. A start
    // the carrier gate turns away never reaches the correlation test.
    const auto config = acquisition_config();
    const auto begin = first - 300U;
    const auto end = first + 3U * segment_symbols;
    std::vector<float> correlations;
    for (auto start = begin; start < end; ++start)
    {
        const auto tested = test_start(symbols, start, reference);
        const bool inside_gate = std::fabs(tested.frequency_hz) <= config.maximum_frequency_offset_hz;
        correlations.push_back(inside_gate ? tested.correlation : 0.0F);
        if (inside_gate && tested.correlation >= config.minimum_correlation)
        {
            step.peaks.push_back({narrow<std::ptrdiff_t>(start) - narrow<std::ptrdiff_t>(first), tested.correlation, tested.frequency_hz});
        }
    }
    for (std::size_t index = 0U; index < correlations.size(); ++index)
    {
        const auto start = narrow<std::ptrdiff_t>(begin + index) - narrow<std::ptrdiff_t>(first);
        const bool near_a_peak = std::ranges::any_of(step.peaks, [&](const CombPeak& peak) { return std::abs(start - peak.start) <= 2; });
        if (!near_a_peak && correlations[index] > step.largest_elsewhere)
        {
            step.largest_elsewhere = correlations[index];
            step.largest_elsewhere_start = start;
        }
    }
    return step;
}

void print(const CorrelationComb& step)
{
    std::println("{} dB SNR, carrier {:+} Hz off; every start that reaches the 0.35 floor:", step.snr_db, step.carrier_offset_hz);
    std::println("  start  correlation  carrier (Hz)");
    for (const auto& peak : step.peaks)
    {
        std::println("{:7}  {:11.3f}  {:+12.1f}", peak.start, peak.correlation, peak.frequency_hz);
    }
    std::println("largest correlation anywhere else, inside the carrier gate: {:.3f}, at start {}", step.largest_elsewhere, step.largest_elsewhere_start);
}

// --- The carrier offset ---

Result<CarrierRange> carrier_range()
{
    // One clean segment of the 600S preamble at symbol rate, turned by each offset.
    std::vector<std::uint8_t> tribits(body_preamble_symbols(mode_600s.interleave));
    if (const auto status = generate_body_preamble(mode_600s, tribits); !status.is_ok())
    {
        return status;
    }
    const auto reference = prefix_reference_points();
    const auto gate = acquisition_config().maximum_frequency_offset_hz;
    CarrierRange step;

    for (const auto offset_hz : {0.0F, 37.0F, -75.0F, 110.0F, 150.0F, 600.0F, 1300.0F})
    {
        std::vector<IQSample> input(segment_symbols);
        for (std::size_t n = 0U; n < input.size(); ++n)
        {
            input[n] = psk8_symbol(tribits[n]) * std::polar(1.0F, 2.0F * pi * offset_hz * as_real(n) / symbol_rate_hz);
        }
        const auto estimated = estimate_body_prefix_frequency_radians(input, 0U, reference) * symbol_rate_hz / (2.0F * pi);
        step.rows.push_back({offset_hz, estimated, std::fabs(estimated) <= gate});
    }
    return step;
}

void print(const CarrierRange& step)
{
    std::println("  true (Hz)  estimated (Hz)  inside the 120 Hz gate");
    for (const auto& row : step.rows)
    {
        std::println("{:+11.1f}  {:+14.1f}  {}", row.true_hz, row.estimated_hz, row.accepted ? "yes" : "no");
    }
}

// --- Reading the mode and the countdown ---

Result<ReadSegment> read_segment()
{
    const auto received = received_hi(walkthrough_channel());
    if (!received)
    {
        return received.error();
    }
    const auto& symbols = received.value().symbols;
    const auto reference = prefix_reference_points();
    const auto config = acquisition_config();
    ReadSegment step{.truth_preamble = received.value().first_preamble, .truth_body = received.value().first_body};

    // The scan's gates, start by start, until one passes all three.
    for (std::size_t start = 0U; start + segment_symbols <= symbols.size(); ++start)
    {
        const auto tested = test_start(symbols, start, reference);
        if (std::fabs(tested.frequency_hz) > config.maximum_frequency_offset_hz || tested.correlation < config.minimum_correlation)
        {
            continue;
        }
        BodyAcquisition candidate;
        candidate.first_preamble_symbol = start;
        candidate.frequency_offset_hz = tested.frequency_hz;
        candidate.carrier_phase_radians = tested.phase;
        candidate.normalized_correlation = tested.correlation;
        const auto read = characterize_body_candidate(symbols, candidate, config);
        if (!body_candidate_is_valid(read))
        {
            step.rejected.push_back({start, tested.correlation, tested.frequency_hz});
            continue;
        }

        step.start = start;
        step.correlation = tested.correlation;
        step.frequency_hz = tested.frequency_hz;
        step.countdown = read.preamble_countdown;
        step.recognition = read.recognition;
        step.first_body_symbol = read.first_body_symbol;
        step.first_preamble_symbol = read.first_preamble_symbol;
        if (const auto designators = body_designators(read.recognition.mode); designators && read.recognition.state == BodyDesignatorState::supported)
        {
            step.d1 = designators.value().d1;
            step.d2 = designators.value().d2;
        }
        return step;
    }
    return Status{StatusCode::unavailable, "no start passed all three gates"};
}

void print(const ReadSegment& step)
{
    for (const auto& rejected : step.rejected)
    {
        std::println("start {}: correlation {:.3f}, carrier {:+.1f} Hz, but not a valid segment", rejected.start, rejected.correlation, rejected.frequency_hz);
    }
    std::println("first start through all three gates: symbol {} (the burst starts at {})", step.start, step.truth_preamble);
    std::println("  correlation {:.3f}, carrier {:+.1f} Hz", step.correlation, step.frequency_hz);
    std::println("  D1 {}, D2 {}: {} bps, {}", step.d1, step.d2, std::to_underlying(step.recognition.mode.data_rate), mode_name(step.recognition));
    std::println("  countdown {}: the body starts {} segments on, at symbol {} (really {})", step.countdown, step.countdown + 1U, step.first_body_symbol, step.truth_body);
    std::println("  so the preamble began at symbol {}", step.first_preamble_symbol);
}

Result<WakeMidPreamble> wake_mid_preamble()
{
    const auto received = received_hi(walkthrough_channel());
    if (!received)
    {
        return received.error();
    }
    const auto& all = received.value().symbols;
    WakeMidPreamble step{.symbols_missed = 10U * segment_symbols + 137U};
    const auto cut = received.value().first_preamble + step.symbols_missed;
    const auto acquired = acquire_body_preamble(std::span<const IQSample>{all}.subspan(cut), acquisition_config());
    if (!acquired)
    {
        return acquired.error();
    }
    step.countdown = acquired.value().preamble_countdown;
    step.segment_read = (acquired.value().detected_segment_symbol + step.symbols_missed) / segment_symbols;
    step.first_body_symbol = acquired.value().first_body_symbol;
    step.truth_body = received.value().first_body - cut;
    step.preamble_start_in_buffer = acquired.value().preamble_start_in_buffer;
    return step;
}

void print(const WakeMidPreamble& step)
{
    std::println("the receiver starts {} symbols into the burst (segment {}, symbol {})", step.symbols_missed, step.symbols_missed / segment_symbols, step.symbols_missed % segment_symbols);
    std::println("  it read the countdown from burst segment {}: {}, so {} segments to the body", step.segment_read, step.countdown, step.countdown + 1U);
    std::println("  body starts at symbol {} of what it heard (really {})", step.first_body_symbol, step.truth_body);
    std::println("  preamble start in what it heard: {}", step.preamble_start_in_buffer ? "yes" : "no");
}

Result<UnsupportedMode> unsupported_mode()
{
    // A 600S preamble at symbol rate, then a second of random symbols.
    std::vector<std::uint8_t> tribits(body_preamble_symbols(mode_600s.interleave));
    if (const auto status = generate_body_preamble(mode_600s, tribits); !status.is_ok())
    {
        return status;
    }
    const auto sent = body_designators(mode_600s);
    if (!sent)
    {
        return sent.error();
    }

    // The same preamble with D1 and D2 changed to 7, 7 in every segment: each of those chips
    // loses its old code's tribit and gains code 7's.
    auto patched = tribits;
    for (std::size_t segment = 0U; segment < patched.size() / segment_symbols; ++segment)
    {
        for (const auto& [position, old_code] : {std::pair{d1_position, sent.value().d1}, std::pair{d2_position, sent.value().d2}})
        {
            for (std::size_t chip = 0U; chip < chips; ++chip)
            {
                auto& tribit = patched[segment * segment_symbols + position * chips + chip];
                tribit = tribit_add(tribit + 8U - body_channel_symbol_tribit(old_code, chip), body_channel_symbol_tribit(7U, chip));
            }
        }
    }

    DeterministicRng rng{11U};
    constexpr std::size_t body_symbols = 2400U;
    const auto to_symbols = [&](const std::vector<std::uint8_t>& preamble)
    {
        std::vector<IQSample> symbols;
        symbols.reserve(preamble.size() + body_symbols);
        for (const auto tribit : preamble)
        {
            symbols.push_back(psk8_symbol(tribit));
        }
        for (std::size_t n = 0U; n < body_symbols; ++n)
        {
            symbols.push_back(psk8_symbol(narrow<std::uint8_t>(rng.next_u64() & 7U)));
        }
        return symbols;
    };

    UnsupportedMode step;
    step.as_sent = recognize_body_designators(sent.value().d1, sent.value().d2, false).state;
    step.as_sent_acquired = acquire_body_preamble(to_symbols(tribits), acquisition_config()).has_value();
    step.patched = recognize_body_designators(7U, 7U, false).state;
    step.patched_acquired = acquire_body_preamble(to_symbols(patched), acquisition_config()).has_value();
    return step;
}

void print(const UnsupportedMode& step)
{
    const auto state = [](BodyDesignatorState value)
    {
        return value == BodyDesignatorState::supported ? "supported" : value == BodyDesignatorState::recognized_unsupported ? "recognized, not implemented" : "invalid";
    };
    std::println("D1 6, D2 6 (600S):        {:28}  acquired: {}", state(step.as_sent), step.as_sent_acquired ? "yes" : "no");
    std::println("D1 7, D2 7 (patched in):  {:28}  acquired: {}", state(step.patched), step.patched_acquired ? "yes" : "no");
}

// --- One segment, or all of them ---

Result<SegmentVote> segment_vote(float snr_db)
{
    const auto received = received_hi(walkthrough_channel(snr_db));
    if (!received)
    {
        return received.error();
    }
    const auto& symbols = received.value().symbols;
    const auto first = received.value().first_preamble;
    const auto reference = prefix_reference_points();
    const auto config = acquisition_config();
    SegmentVote step{.snr_db = snr_db};

    // Each segment read on its own, at its true start, as one segment's decode would be.
    for (std::size_t segment = 0U; segment < body_preamble_symbols(mode_600l.interleave) / segment_symbols; ++segment)
    {
        const auto start = first + segment * segment_symbols;
        const auto tested = test_start(symbols, start, reference);
        BodyAcquisition candidate;
        candidate.first_preamble_symbol = start;
        candidate.frequency_offset_hz = tested.frequency_hz;
        candidate.carrier_phase_radians = tested.phase;
        candidate.normalized_correlation = tested.correlation;
        const auto read = characterize_body_candidate(symbols, candidate, config);

        VoteRow row{
            .segment = segment,
            .correlation = tested.correlation,
            .valid = body_candidate_is_valid(read),
            .countdown = read.preamble_countdown,
            .rate = read.recognition.mode.data_rate,
            .first_body_symbol = read.first_body_symbol,
        };
        row.right =
            row.valid && read.recognition.mode.data_rate == mode_600l.data_rate && read.recognition.mode.interleave == mode_600l.interleave && read.first_body_symbol == received.value().first_body;
        step.right += row.right ? 1U : 0U;
        step.wrong += row.valid && !row.right ? 1U : 0U;
        step.invalid += row.valid ? 0U : 1U;
        step.rows.push_back(row);

        if (row.valid)
        {
            const auto same = std::ranges::find_if(step.groups, [&](const VoteGroup& group) { return group.first_body_symbol == row.first_body_symbol && group.rate == row.rate; });
            if (same == step.groups.end())
            {
                step.groups.push_back({row.first_body_symbol, row.rate, 1U});
            }
            else
            {
                ++same->supporters;
            }
        }
    }

    const auto acquired = acquire_body_preamble(symbols, config);
    step.acquired = acquired.has_value();
    step.acquisition_right = step.acquired && acquired.value().first_body_symbol == received.value().first_body && acquired.value().recognition.mode.data_rate == mode_600l.data_rate &&
                             acquired.value().recognition.mode.interleave == mode_600l.interleave;
    return step;
}

void print(const SegmentVote& step)
{
    std::println("{} dB SNR, each segment read on its own; the ones that did not read right:", step.snr_db);
    std::println("segment  correlation  countdown  reads as");
    for (const auto& row : step.rows)
    {
        if (row.right)
        {
            continue;
        }
        if (!row.valid)
        {
            std::println("{:7}  {:11.3f}  {:>9}  not a valid segment", row.segment, row.correlation, "-");
        }
        else
        {
            std::println("{:7}  {:11.3f}  {:9}  {} bps, body at {}{}", row.segment, row.correlation, row.countdown, std::to_underlying(row.rate), row.first_body_symbol, row.right ? "" : "  (wrong)");
        }
    }
    std::println("right {}, wrong {}, not valid {}", step.right, step.wrong, step.invalid);
    std::println("grouped by the burst they describe:");
    for (const auto& group : step.groups)
    {
        std::println("  {} bps, body at {}: {} segment{}", std::to_underlying(group.rate), group.first_body_symbol, group.supporters, group.supporters == 1U ? "" : "s");
    }
    std::println("acquisition: {}", !step.acquired ? "nothing" : step.acquisition_right ? "right" : "wrong");
}

// --- No burst at all ---

Result<NoiseOnly> noise_only()
{
    // Ten seconds of white noise at about the level a burst would arrive at.
    constexpr std::size_t samples = 480000U;
    std::vector<float> audio(samples);
    DeterministicRng rng{29U};
    for (auto& sample : audio)
    {
        sample = 0.3F * rng.gaussian();
    }

    FrontEndRunner runner;
    auto status = runner.initialize();
    if (status.is_ok())
    {
        status = runner.feed(audio);
    }
    if (!status.is_ok())
    {
        return status;
    }
    const auto symbols = on_time_symbols(runner);
    const auto reference = prefix_reference_points();
    const auto config = acquisition_config();
    NoiseOnly step{.seconds = as_real(samples) / 48000.0F};
    for (std::size_t start = 0U; start + segment_symbols <= symbols.size(); ++start)
    {
        const auto tested = test_start(symbols, start, reference);
        if (std::fabs(tested.frequency_hz) > config.maximum_frequency_offset_hz)
        {
            continue;
        }
        step.largest_correlation = std::max(step.largest_correlation, tested.correlation);
        if (tested.correlation < config.minimum_correlation)
        {
            continue;
        }
        ++step.over_floor;
        BodyAcquisition candidate;
        candidate.first_preamble_symbol = start;
        candidate.frequency_offset_hz = tested.frequency_hz;
        candidate.carrier_phase_radians = tested.phase;
        candidate.normalized_correlation = tested.correlation;
        step.valid += body_candidate_is_valid(characterize_body_candidate(symbols, candidate, config)) ? 1U : 0U;
    }
    step.acquired = acquire_body_preamble(symbols, config).has_value();
    return step;
}

void print(const NoiseOnly& step)
{
    std::println("{} s of noise: largest correlation {:.3f} against the 0.35 floor", step.seconds, step.largest_correlation);
    std::println("  starts through both gates {}, of those reading as a valid segment {}; acquired: {}", step.over_floor, step.valid, step.acquired ? "yes" : "no");
}

// --- Part 3's hang-up, fixed while acquiring ---

Result<PhaseSearch> phase_search()
{
    PhaseSearch step{.snr_db = 10.0F, .start_phases = samples_per_symbol};
    constexpr std::size_t in_time = 1200U; // of the short preamble's 1440 symbols
    ChannelSpec channel;
    channel.snr_db = step.snr_db;
    channel.seed = 1U;
    for (std::size_t lead = 0U; lead < samples_per_symbol; ++lead)
    {
        const auto plain = pull_in_from(mode_2400s, channel, lead, false);
        if (!plain)
        {
            return plain.error();
        }
        const auto searched = pull_in_from(mode_2400s, channel, lead, true);
        if (!searched)
        {
            return searched.error();
        }
        const auto on_time = [&](const PullIn& run) { return run.pulled_in_at && *run.pulled_in_at < in_time; };
        step.plain_in_time += on_time(plain.value()) ? 1U : 0U;
        step.searched_in_time += on_time(searched.value()) ? 1U : 0U;
        if (!on_time(plain.value()))
        {
            step.late_starts.push_back({lead, plain.value().pulled_in_at, searched.value().pulled_in_at, searched.value().moves});
        }
    }
    return step;
}

void print(const PhaseSearch& step)
{
    const auto at = [](const std::optional<std::size_t>& symbol) { return symbol ? std::to_string(*symbol) : std::string{"not in the preamble"}; };
    std::println("2400S, {} dB SNR, the burst started at each of the {} sample phases of a symbol", step.snr_db, step.start_phases);
    std::println("pulled in before symbol 1200 of the 1440-symbol preamble: {} of {} plain, {} of {} with the phase search", step.plain_in_time, step.start_phases, step.searched_in_time,
                 step.start_phases);
    std::println("the starts the plain loop missed:");
    std::println("lead  plain loop pulled in at   with the search   moves");
    for (const auto& start : step.late_starts)
    {
        std::println("{:4}  {:>22}  {:>16}  {:6}", start.lead, at(start.plain_pulled_in_at), at(start.searched_pulled_in_at), start.searched_moves);
    }
}

// --- The clock-offset loop ---

Result<ClockOffset> clock_offset()
{
    constexpr float snr_db = 20.0F;
    struct Case
    {
        BodyMode mode;
        float ppm;
        Loop loop;
    };
    constexpr std::array cases{
        Case{mode_2400l, 100.0F, Loop::type_1},   Case{mode_2400l, -100.0F, Loop::type_1},   Case{mode_2400l, 1000.0F, Loop::type_1},  Case{mode_2400l, 100.0F, Loop::running},
        Case{mode_2400l, 1000.0F, Loop::running}, Case{mode_2400l, -1000.0F, Loop::running}, Case{mode_2400s, 1000.0F, Loop::running},
    };

    ClockOffset step{.snr_db = snr_db};
    for (const auto& c : cases)
    {
        ChannelSpec channel;
        channel.snr_db = snr_db;
        channel.clock_ppm = c.ppm;
        channel.seed = 1U;
        const auto run = run_loop(c.mode, channel, c.loop);
        if (!run)
        {
            return run.error();
        }
        const auto [offset, wander] = body_offset_and_wander(run.value());
        ClockRow row{
            .clock_ppm = c.ppm,
            .mode = c.mode,
            .type_2 = c.loop == Loop::running,
            .body_offset = offset,
            .body_wander = wander,
            .slips = body_slips(run.value()),
        };
        if (row.type_2)
        {
            row.learned_ppm_off = rate_error_ppm(run.value().rate_at_end, run.value().grid);
        }
        step.rows.push_back(row);
    }
    return step;
}

void print(const ClockOffset& step)
{
    std::println("{} dB SNR; the loop against the symbol grid through a 10 s body", step.snr_db);
    std::println("clock (ppm)  mode   loop    offset (samples)  wander  symbols slipped  integrator at the end");
    for (const auto& row : step.rows)
    {
        const auto learned = row.learned_ppm_off ? ppm_off(*row.learned_ppm_off) : std::string{"-"};
        std::println("{:+11.0f}  {:5}  {}  {:+16.2f}  {:6.2f}  {:15}  {:>21}", row.clock_ppm, mode_label(row.mode), row.type_2 ? "type 2" : "type 1", row.body_offset, row.body_wander, row.slips,
                     learned);
    }
}

// The first symbol after the first candidate from which the rate stays within 100 ppm of the
// clock for a whole segment.
std::optional<std::size_t> settled_within_100_ppm(const LoopRun& run)
{
    std::size_t within_since{};
    for (const auto& point : run.offsets)
    {
        const auto symbol = narrow<std::size_t>(std::lround(point.burst_symbol));
        if (symbol < track_from_symbol)
        {
            continue;
        }
        if (std::fabs(rate_error_ppm(point.timing_rate, run.grid)) >= 100.0F)
        {
            within_since = 0U;
        }
        else if (within_since == 0U)
        {
            within_since = symbol;
        }
        else if (symbol >= within_since + segment_symbols)
        {
            return within_since - track_from_symbol;
        }
    }
    return std::nullopt;
}

Result<LearningTime> learning_time()
{
    LearningTime step{.mode = mode_600s, .clock_ppm = 1000.0F};
    for (const auto snr_db : {20.0F, 10.0F, 7.0F, 4.0F})
    {
        LearningRow row{.snr_db = snr_db};
        for (std::uint64_t seed = 1U; seed <= 3U; ++seed)
        {
            ChannelSpec channel;
            channel.snr_db = snr_db;
            channel.clock_ppm = step.clock_ppm;
            channel.seed = seed;
            const auto run = run_loop(step.mode, channel, Loop::running, 30U);
            if (!run)
            {
                return run.error();
            }
            row.error_at_window_ppm.push_back(rate_error_ppm(rate_at_symbol(run.value(), track_from_symbol + learning_window_symbols), run.value().grid));
            row.settled_at.push_back(settled_within_100_ppm(run.value()));
        }
        step.rows.push_back(row);
    }
    return step;
}

void print(const LearningTime& step)
{
    std::println("{}, the clock {:+.0f} ppm off, the integrator running from the first candidate, three seeds", mode_label(step.mode), step.clock_ppm);
    std::println("SNR (dB)  rate error at 2880 symbols (ppm)  within 100 ppm from symbol");
    for (const auto& row : step.rows)
    {
        std::string errors;
        for (const auto error : row.error_at_window_ppm)
        {
            errors += std::format(" {:+5.0f}", std::round(error) == 0.0F ? 0.0F : error);
        }
        std::string settled;
        for (const auto& symbol : row.settled_at)
        {
            settled += std::format(" {:>6}", symbol ? std::to_string(*symbol) : std::string{"never"});
        }
        std::println("{:8.0f}  {:>32}  {:>26}", row.snr_db, errors, settled);
    }
}

Result<LongBurstRule> long_burst_rule()
{
    LongBurstRule step{.snr_db = 20.0F, .delay_ms = 1.0F, .doppler_spread_hz = 0.5F, .body_seconds = 25U, .threshold_ppm = rate_ppm(widen<double>(retire_rates.front()))};
    for (const auto ppm : {0.0F, 350.0F})
    {
        for (std::uint64_t seed = 1U; seed <= 2U; ++seed)
        {
            ChannelSpec channel;
            channel.snr_db = step.snr_db;
            channel.fading = FadingSpec{.delay_ms = step.delay_ms, .doppler_spread_hz = step.doppler_spread_hz};
            if (ppm != 0.0F)
            {
                channel.clock_ppm = ppm;
            }
            channel.seed = seed;
            const auto running = run_loop(mode_2400l, channel, Loop::running, step.body_seconds);
            if (!running)
            {
                return running.error();
            }
            // The first judgement, 20 s after the first candidate, on the mean from 5 s.
            const auto from = track_from_symbol + mean_from_symbols;
            const auto at = track_from_symbol + judge_at_symbols.front();
            RuleRow row{.seed = seed, .clock_ppm = ppm};
            for (const auto& point : running.value().offsets)
            {
                if (point.burst_symbol >= widen<double>(as_real(from)) && point.burst_symbol < widen<double>(as_real(at)))
                {
                    row.largest_rate_ppm = std::max(row.largest_rate_ppm, rate_ppm(std::fabs(widen<double>(point.timing_rate))));
                }
            }
            row.judged_mean_ppm = rate_ppm(std::fabs(mean_rate(running.value(), from, at)));
            row.retired = row.judged_mean_ppm < step.threshold_ppm;
            step.rows.push_back(row);
        }
    }
    return step;
}

void print(const LongBurstRule& step)
{
    std::println("2400L, a {} s body through Watterson fading, {} ms and {} Hz, {} dB SNR; the integrator from 5 s to 20 s", step.body_seconds, step.delay_ms, step.doppler_spread_hz, step.snr_db);
    std::println("clock (ppm)  seed  largest momentary rate  mean rate  at 20 s (threshold {:.0f} ppm)", step.threshold_ppm);
    for (const auto& row : step.rows)
    {
        std::println("{:+11.0f}  {:4}  {:18.0f} ppm  {:5.0f} ppm  {}", row.clock_ppm, row.seed, row.largest_rate_ppm, row.judged_mean_ppm, row.retired ? "retired" : "kept");
    }
}

} // namespace modem_test::acquisition_steps
