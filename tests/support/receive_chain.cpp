// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#include "tests/support/receive_chain.hpp"

#include "modem/common/convert.hpp"
#include "modem/m110a/body_audio_stream_frontend.hpp"
#include "modem/m110a/body_waveform.hpp"
#include "tests/support/front_end_harness.hpp"
#include "tests/support/test_waveform.hpp"
#include "tests/support/wav_file.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <format>
#include <print>
#include <span>
#include <utility>
#include <vector>

namespace modem_test
{

namespace
{

using ::modem::common::as_real;
using ::modem::common::BodyInterleave;
using ::modem::common::DataRate;
using ::modem::common::narrow;
using ::modem::common::Result;
using ::modem::common::round_to;
using ::modem::common::Status;
using ::modem::common::StatusCode;
using ::modem::common::widen;
using ::modem::m110a::acquire_body_preamble;
using ::modem::m110a::BodyAcquisition;
using ::modem::m110a::BodyAcquisitionConfig;
using ::modem::m110a::BodyAudioStreamFrontend;
using ::modem::m110a::BodyMode;
using ::modem::m110a::TimingRateMode;

constexpr std::uint32_t sample_rate_hz = ::modem::m110a::body_audio_sample_rate_hz;       // 48000
constexpr std::size_t samples_per_symbol = ::modem::m110a::body_audio_samples_per_symbol; // 20
constexpr std::size_t segment_symbols = ::modem::m110a::body_preamble_segment_symbols;    // 480

// Half a second of line before a generated burst, as the walkthrough has it, so acquisition
// has noise to search through. The odd ten samples put the burst's symbol peaks where the
// front end starts sampling.
constexpr std::size_t lead_samples = 24010U;

// Where the front end reads its first on-time sample: the filter's length plus half a symbol.
constexpr std::uint64_t first_on_time_sample = BodyAudioStreamFrontend::matched_filter_taps - 1U + samples_per_symbol / 2U;

// The walkthrough's acquisition settings: a 0.35 correlation floor (the library's default
// is a cautious 0.75) and the 120 Hz carrier gate, searching every start.
BodyAcquisitionConfig acquisition_config() noexcept
{
    BodyAcquisitionConfig config;
    config.minimum_correlation = 0.35F;
    return config;
}

// The audio the receive stages are given, and, for a generated burst, what was sent.
struct Line
{
    std::vector<float> audio{};
    std::optional<BurstTruth> burst{}; // first_sample counts transmit samples from the line's start
};

// Part 2 and the channel: the message through the transmitter, half a second of line in
// front of it, then what the air does to both.
Result<Line> transmit(const ChainSpec& spec, ChainReport& report)
{
    if (spec.message.empty())
    {
        return Status{StatusCode::invalid_argument, "the message is empty"};
    }
    WaveformSpec waveform_spec;
    waveform_spec.mode = spec.mode;
    waveform_spec.payload.resize(spec.message.size());
    std::ranges::transform(spec.message, waveform_spec.payload.begin(), [](char c) { return std::bit_cast<std::uint8_t>(c); });
    auto made = make_test_waveform(waveform_spec);
    if (!made)
    {
        return made.error();
    }
    auto& waveform = made.value();
    Line line;
    line.burst = waveform.bursts.front();
    report.transmit = TransmitReport{.bytes = spec.message.size(),
                                     .preamble_symbols = line.burst->preamble_symbols,
                                     .body_symbols = line.burst->tribits.size() - line.burst->preamble_symbols,
                                     .audio_samples = waveform.audio.size()};

    line.audio.assign(lead_samples, 0.0F);
    line.audio.insert(line.audio.end(), waveform.audio.begin(), waveform.audio.end());
    line.burst->first_sample += lead_samples;
    report.lead_samples = lead_samples;
    if (const auto status = apply_channel(line.audio, spec.channel); !status.is_ok())
    {
        return status;
    }
    return line;
}

// A recording instead: the receive stages run at 48 kHz.
Result<Line> record(const std::string& path)
{
    auto wav = read_wav(path);
    if (!wav)
    {
        return wav.error();
    }
    if (wav.value().sample_rate_hz != sample_rate_hz)
    {
        return Status{StatusCode::invalid_argument, "the receive chain runs at 48 kHz: resample the recording first"};
    }
    Line line;
    line.audio = std::move(wav.value().samples);
    return line;
}

// How one pass of the front end over the line is set up. Neither, and the loop is Part 3's.
// The phase search alone is the front end as a receiver runs it while it looks for a preamble.
// With a commit, from that stream sample on the search is off and the rate integrator tracks,
// as a receiver runs it once it has committed to a burst (Part 4).
struct FrontEndPlan
{
    bool phase_search{};
    std::optional<std::uint64_t> commit_sample{};
};

Status run_front_end(FrontEndRunner& runner, std::span<const float> audio, const FrontEndPlan& plan)
{
    FrontEndOptions options;
    options.phase_search = plan.phase_search;
    if (plan.commit_sample)
    {
        options.phase_search_off_at_sample = plan.commit_sample;
        options.timing_integral_gain = BodyAudioStreamFrontend::default_timing_integral_gain;
        options.timing_rate_changes.push_back({.at_sample = *plan.commit_sample, .mode = TimingRateMode::tracking});
    }
    auto status = runner.initialize(options);
    if (status.is_ok())
    {
        status = runner.feed(audio);
    }
    return status;
}

// The loop through the body, against the transmitter's grid: its mean distance from the
// symbol peaks, and the symbols it gained or lost (the stream symbol it reads each burst
// symbol at stays the same while it keeps step).
GridReport grid_report(const FrontEndRunner& runner, const BurstTruth& burst, float clock_ppm)
{
    const auto body_start = widen<double>(as_real(burst.preamble_symbols));
    double sum{};
    std::size_t count{};
    std::optional<std::int64_t> first_lag{};
    std::int64_t last_lag{};
    for (const auto& point : grid_offsets(runner, burst, 0U, clock_ppm))
    {
        if (point.burst_symbol < body_start)
        {
            continue;
        }
        sum += point.offset_samples;
        ++count;
        const auto lag = narrow<std::int64_t>(point.symbol) - std::llround(point.burst_symbol);
        if (!first_lag)
        {
            first_lag = lag;
        }
        last_lag = lag;
    }
    return {.mean_offset_samples = count == 0U ? 0.0 : sum / widen<double>(as_real(count)), .slipped_symbols = first_lag ? last_lag - *first_lag : 0};
}

// The stream sample half a symbol before the loop reads stream symbol `symbol`, from where
// the search pass had it: the integrator's change of mode lands within a symbol of it.
std::uint64_t sample_before_symbol(const FrontEndRunner& runner, std::size_t symbol)
{
    std::uint64_t at = first_on_time_sample + symbol * samples_per_symbol;
    for (const auto& point : runner.timing_points())
    {
        if (point.symbol > symbol)
        {
            break;
        }
        at = point.position_sample + (symbol - point.symbol) * samples_per_symbol;
    }
    return at - samples_per_symbol / 2U;
}

// A rate the integrator holds, in samples per symbol beyond 20, as the transmit clock's
// offset: the inverse of grid_rate().
double clock_ppm_from_rate(double rate) noexcept
{
    const auto nominal = widen<double>(as_real(samples_per_symbol));
    return (nominal / (nominal + rate) - 1.0) * 1.0e6;
}

// Part 4: the front end with the phase search on, as a receiver runs it while it looks for a
// preamble, and acquisition over its symbols. Then the front end once more, committed at the
// first candidate, a segment into the preamble: the search off from there and the rate
// integrator tracking, as a streaming receiver would have run it. (The runner has the whole
// line, so it finds the candidate first and runs the front end again.)
Status acquire(const ChainSpec& spec, const Line& line, ChainReport& report)
{
    auto& acquisition = report.acquisition;
    FrontEndRunner search;
    if (const auto status = run_front_end(search, line.audio, FrontEndPlan{.phase_search = true}); !status.is_ok())
    {
        return status;
    }
    acquisition.phase_search_moves = search.phase_search_moves();
    const auto symbols = on_time_symbols(search);
    const auto acquired = acquire_body_preamble(symbols, acquisition_config());
    if (!acquired)
    {
        acquisition.status = acquired.error();
        return Status::success();
    }
    const auto& found = acquired.value();
    acquisition.acquired = found;

    const auto clock_ppm = spec.channel.clock_ppm.value_or(0.0F);
    std::size_t body_end = symbols.size();
    if (line.burst)
    {
        const auto burst_sample = round_to<std::uint64_t>(as_real(line.burst->first_sample) / (1.0F + clock_ppm * 1.0e-6F));
        const auto first = narrow<std::size_t>(locate_burst(search, *line.burst, burst_sample));
        acquisition.true_first_preamble = first;
        acquisition.true_first_body = first + line.burst->preamble_symbols;
        body_end = std::min(body_end, first + line.burst->tribits.size());
    }

    const auto first_candidate = found.preamble_start_in_buffer ? found.first_preamble_symbol : found.detected_segment_symbol;
    acquisition.committed_at_symbol = std::min(first_candidate + segment_symbols, symbols.size() - 1U);
    FrontEndRunner committed;
    const FrontEndPlan plan{.phase_search = true, .commit_sample = sample_before_symbol(search, acquisition.committed_at_symbol)};
    if (const auto status = run_front_end(committed, line.audio, plan); !status.is_ok())
    {
        return status;
    }

    // The integrator's mean rate through the body: the clock it learned. On a recording the
    // body runs to the end of the audio, so a long tail of noise after the burst counts too.
    double sum{};
    std::size_t count{};
    for (const auto& point : committed.timing_points())
    {
        if (point.symbol >= found.first_body_symbol && point.symbol < body_end)
        {
            sum += widen<double>(point.timing_rate);
            ++count;
        }
    }
    acquisition.learned_clock_ppm = count == 0U ? 0.0 : clock_ppm_from_rate(sum / widen<double>(as_real(count)));
    if (line.burst)
    {
        acquisition.grid = grid_report(committed, *line.burst, clock_ppm);
    }
    return Status::success();
}

std::string mode_description(BodyMode mode)
{
    const char* interleave = "no interleave";
    if (mode.interleave == BodyInterleave::long_block)
    {
        interleave = "long interleave";
    }
    else if (mode.interleave == BodyInterleave::short_block)
    {
        interleave = "short interleave";
    }
    return std::format("{} bps, {}", std::to_underlying(mode.data_rate), interleave);
}

// A clock offset, whole ppm with its sign; within half a ppm of zero is 0.
std::string ppm_text(double ppm)
{
    return std::fabs(ppm) < 0.5 ? std::string{"0"} : std::format("{:+.0f}", ppm);
}

// A carrier offset to a tenth of a hertz with its sign; within a twentieth of zero is 0.0.
std::string hz_text(float hz)
{
    return std::fabs(hz) < 0.05F ? std::string{"0.0"} : std::format("{:+.1f}", hz);
}

std::string channel_text(const ChainSpec& spec, std::size_t lead)
{
    const auto& channel = spec.channel;
    std::string text;
    const auto add = [&text](const std::string& part) { text += text.empty() ? part : ", " + part; };
    if (channel.snr_db)
    {
        add(std::format("{:g} dB SNR", *channel.snr_db));
    }
    if (channel.carrier_offset_hz)
    {
        add(std::format("carrier {:+} Hz", *channel.carrier_offset_hz));
    }
    if (channel.clock_ppm)
    {
        add(std::format("clock {:+g} ppm", *channel.clock_ppm));
    }
    if (channel.fading)
    {
        add(std::format("two-path fading, {:g} ms and {:g} Hz", channel.fading->delay_ms, channel.fading->doppler_spread_hz));
    }
    if (text.empty())
    {
        text = "clean";
    }
    return std::format("{}; {:.1f} s of line before the burst, seed {}", text, as_real(lead) / as_real(sample_rate_hz), channel.seed);
}

void print_grid(const GridReport& grid)
{
    if (grid.slipped_symbols == 0)
    {
        std::println("           through the body, against the transmitter's grid: {:.2f} samples off the symbol peaks, in step with every symbol", grid.mean_offset_samples);
        return;
    }
    // A loop that slips reads each symbol off a different peak, so its mean offset says nothing.
    std::println("           through the body, against the transmitter's grid: the loop slipped {} symbols", grid.slipped_symbols < 0 ? -grid.slipped_symbols : grid.slipped_symbols);
}

} // namespace

std::optional<BodyMode> parse_mode(const std::string& text)
{
    if (text == "4800")
    {
        return BodyMode{DataRate::bps4800, BodyInterleave::zero};
    }
    if (text.size() < 3U)
    {
        return std::nullopt;
    }
    const auto interleave_letter = text.back();
    const auto rate = text.substr(0U, text.size() - 1U);
    BodyInterleave interleave{};
    if (interleave_letter == 'L' || interleave_letter == 'l')
    {
        interleave = BodyInterleave::long_block;
    }
    else if (interleave_letter == 'S' || interleave_letter == 's')
    {
        interleave = BodyInterleave::short_block;
    }
    else
    {
        return std::nullopt;
    }
    for (const auto data_rate : {DataRate::bps75, DataRate::bps150, DataRate::bps300, DataRate::bps600, DataRate::bps1200, DataRate::bps2400})
    {
        if (rate == std::to_string(std::to_underlying(data_rate)))
        {
            return BodyMode{data_rate, interleave};
        }
    }
    return std::nullopt;
}

std::string mode_text(BodyMode mode)
{
    if (mode.interleave == BodyInterleave::zero)
    {
        return std::to_string(std::to_underlying(mode.data_rate));
    }
    return std::format("{}{}", std::to_underlying(mode.data_rate), mode.interleave == BodyInterleave::long_block ? "L" : "S");
}

Result<ChainReport> run_chain(const ChainSpec& spec)
{
    ChainReport report;
    report.spec = spec;
    auto line = spec.wav_in ? record(*spec.wav_in) : transmit(spec, report);
    if (!line)
    {
        return line.error();
    }
    if (spec.wav_out)
    {
        if (const auto status = write_wav(*spec.wav_out, line.value().audio, sample_rate_hz); !status.is_ok())
        {
            return status;
        }
    }

    FrontEndRunner part3;
    if (const auto status = run_front_end(part3, line.value().audio, FrontEndPlan{}); !status.is_ok())
    {
        return status;
    }
    report.front_end = FrontEndReport{.audio_samples = line.value().audio.size(), .sample_rate_hz = sample_rate_hz, .symbols = part3.symbols().size()};
    if (line.value().burst)
    {
        report.front_end.grid = grid_report(part3, *line.value().burst, spec.channel.clock_ppm.value_or(0.0F));
    }

    if (const auto status = acquire(spec, line.value(), report); !status.is_ok())
    {
        return status;
    }
    return report;
}

void print(const ChainReport& report)
{
    const auto& spec = report.spec;
    if (report.transmit)
    {
        const auto& sent = *report.transmit;
        std::println("== Part 2  transmit: \"{}\" in {} ({})", spec.message, mode_text(spec.mode), mode_description(spec.mode));
        std::println("           {} bytes -> {} preamble symbols ({} segments) + {} body symbols, {:.2f} s of audio", sent.bytes, sent.preamble_symbols, sent.preamble_symbols / segment_symbols,
                     sent.body_symbols, as_real(sent.audio_samples) / as_real(sample_rate_hz));
        std::println("== Channel: {}", channel_text(spec, report.lead_samples));
    }
    else
    {
        std::println("== Part 2  transmit: none; the audio is {}", spec.wav_in.value_or(""));
        std::println("== Channel: the recording's own");
    }

    const auto& front_end = report.front_end;
    std::println("== Part 3  front end, Part 3's loop: {:.2f} s of audio -> {} symbols", as_real(front_end.audio_samples) / as_real(front_end.sample_rate_hz), front_end.symbols);
    if (front_end.grid)
    {
        print_grid(*front_end.grid);
    }

    const auto& acquisition = report.acquisition;
    if (!acquisition.acquired)
    {
        std::println("== Part 4  acquisition: no burst found ({})", acquisition.status.message);
        return;
    }
    const auto& found = *acquisition.acquired;
    std::println("== Part 4  acquisition, with the phase search on (it moved the loop {} times): {}", acquisition.phase_search_moves, mode_description(found.recognition.mode));
    std::println("           read at symbol {}: countdown {}, correlation {:.3f}, carrier {} Hz", found.detected_segment_symbol, found.preamble_countdown, found.normalized_correlation,
                 hz_text(found.frequency_offset_hz));
    const auto preamble = found.preamble_start_in_buffer ? std::format("preamble from symbol {}", found.first_preamble_symbol) : std::string{"preamble started before the audio"};
    const auto truth = acquisition.true_first_preamble ? std::format(" (the bench: {} and {})", *acquisition.true_first_preamble, acquisition.true_first_body.value_or(0U)) : std::string{};
    std::println("           {}, body from symbol {}{}", preamble, found.first_body_symbol, truth);
    std::println("           committed at symbol {}: the phase search off, the rate integrator tracking", acquisition.committed_at_symbol);
    std::println("           the clock: the integrator learned {} ppm through the body", ppm_text(acquisition.learned_clock_ppm));
    if (acquisition.grid)
    {
        print_grid(*acquisition.grid);
    }
    std::println("== next: the first path (Part 5)");
}

} // namespace modem_test
