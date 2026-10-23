// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#include "tests/walkthrough/first_path_observe.hpp"

#include "modem/common/convert.hpp"
#include "modem/common/types.hpp"
#include "tests/support/front_end_harness.hpp"
#include "tests/support/wav_file.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <span>
#include <utility>

namespace modem_test::first_path
{

namespace
{

using ::modem::common::as_real;
using ::modem::common::IQSample;
using ::modem::common::narrow;
using ::modem::common::Result;
using ::modem::common::Status;
using ::modem::common::StatusCode;
using ::modem::common::widen;
using ::modem::m110a::acquire_body_preamble;
using ::modem::m110a::body_candidate_is_valid;
using ::modem::m110a::body_preamble_acquisition_prefix_symbols;
using ::modem::m110a::body_preamble_segment_symbols;
using ::modem::m110a::body_prefix_correlation;
using ::modem::m110a::BodyAcquisition;
using ::modem::m110a::BodyAcquisitionConfig;
using ::modem::m110a::characterize_body_candidate;
using ::modem::m110a::estimate_body_prefix_frequency_radians;
using ::modem::m110a::make_body_prefix_reference;

constexpr std::uint32_t sample_rate_hz = 48000U;
constexpr double samples_per_symbol = 20.0;
constexpr double filter_delay_samples = 80.0; // the receive filter's delay: a symbol read at p was sent at p - 80
constexpr double classify_samples = 20.0;     // one symbol either way
constexpr float symbol_rate_hz = 2400.0F;
constexpr std::size_t segment_symbols = body_preamble_segment_symbols;
constexpr std::size_t prefix_symbols = body_preamble_acquisition_prefix_symbols;
constexpr float correlation_floor = 0.35F;        // Part 4's walkthrough floor
constexpr std::size_t adaptive_window = 7U;       // anchor_minimum_path_delay_symbols for the adaptive equalizer
constexpr std::size_t turbo_window = 3U;          // and for the host's turbo decoder
constexpr std::size_t symbol_block_samples = 20U; // one symbol per block, so the loop's position is kept for each

std::array<IQSample, prefix_symbols> prefix_reference_points() noexcept
{
    std::array<IQSample, prefix_symbols> reference{};
    make_body_prefix_reference(reference);
    return reference;
}

// The walkthrough's acquisition settings: Part 4's 0.35 floor and 120 Hz gate, the scan
// limited to the burst's first 1200 symbols, the anchoring off or on with its window.
BodyAcquisitionConfig acquisition_config(std::size_t last_start, bool anchoring, std::size_t minimum_delay) noexcept
{
    BodyAcquisitionConfig config;
    config.minimum_correlation = correlation_floor;
    config.maximum_start_symbol = last_start;
    config.anchor_earliest_path = anchoring;
    config.anchor_minimum_path_delay_symbols = minimum_delay;
    return config;
}

// Where the front end read each stream symbol, in recording samples: the loop's position
// for that symbol less the receive filter's delay, so a symbol read on its peak sits at the
// centre the transmitter gave it.
class SymbolClock
{
public:
    explicit SymbolClock(const FrontEndRunner& runner, double phase) : mPoints(runner.timing_points()), mPhase(phase) {}

    // The recording sample of stream symbol k (the phase's zeros taken back out).
    [[nodiscard]] double sample_of(std::size_t k) const noexcept
    {
        const auto at = std::ranges::upper_bound(mPoints, widen<std::uint64_t>(k), {}, &TimingPoint::symbol);
        if (at == mPoints.begin())
        {
            return widen<double>(as_real(k)) * samples_per_symbol - mPhase;
        }
        const auto& point = *std::prev(at);
        const auto position = widen<double>(as_real(point.position_sample)) + widen<double>(point.position_fraction);
        return position + widen<double>(as_real(k - point.symbol)) * samples_per_symbol - filter_delay_samples - mPhase;
    }

    // The stream symbol read nearest a recording sample.
    [[nodiscard]] std::size_t symbol_at(double sample) const noexcept
    {
        std::size_t best{};
        double distance = std::numeric_limits<double>::infinity();
        for (const auto& point : mPoints)
        {
            const auto d = std::fabs(sample_of(narrow<std::size_t>(point.symbol)) - sample);
            if (d < distance)
            {
                distance = d;
                best = narrow<std::size_t>(point.symbol);
            }
        }
        return best;
    }

private:
    std::span<const TimingPoint> mPoints;
    double mPhase{};
};

Path classify(double body_sample, const Timeline& timeline) noexcept
{
    const auto direct_body = timeline.direct_symbol0_sample + samples_per_symbol * widen<double>(as_real(timeline.preamble_symbols));
    if (std::fabs(body_sample - direct_body) <= classify_samples)
    {
        return Path::direct;
    }
    if (std::fabs(body_sample - (direct_body + timeline.echo_delay_samples)) <= classify_samples)
    {
        return Path::echo;
    }
    return Path::neither;
}

// The prefix correlation at a start, with that start's own carrier estimate, as the scan and
// the walk test it.
float correlation_at(std::span<const IQSample> symbols, std::size_t start) noexcept
{
    const auto reference = prefix_reference_points();
    if (start + segment_symbols > symbols.size())
    {
        return 0.0F;
    }
    const auto radians = estimate_body_prefix_frequency_radians(symbols, start, reference);
    float phase{};
    return body_prefix_correlation(symbols, start, reference, radians, phase);
}

// Steps 1 and 2 of acquire_body_preamble(), from the outside: every start through the carrier
// gate, the floor and a valid read, grouped by the body boundary and mode they name.
void scan_groups(std::span<const IQSample> symbols, const BodyAcquisitionConfig& config, const SymbolClock& clock, const Timeline& timeline, Observation& seen)
{
    const auto reference = prefix_reference_points();
    struct Group
    {
        std::size_t body{};
        ::modem::m110a::BodyMode mode{};
        GroupObservation row{};
    };
    std::vector<Group> groups;
    const auto last = std::min(config.maximum_start_symbol, symbols.size() - segment_symbols);
    for (std::size_t start = 0U; start <= last; ++start)
    {
        const auto radians = estimate_body_prefix_frequency_radians(symbols, start, reference);
        const auto hz = radians * symbol_rate_hz / (2.0F * std::numbers::pi_v<float>);
        if (std::fabs(hz) > config.maximum_frequency_offset_hz)
        {
            continue;
        }
        BodyAcquisition candidate;
        candidate.first_preamble_symbol = start;
        candidate.frequency_offset_hz = hz;
        candidate.normalized_correlation = body_prefix_correlation(symbols, start, reference, radians, candidate.carrier_phase_radians);
        if (candidate.normalized_correlation < config.minimum_correlation)
        {
            continue;
        }
        const auto read = characterize_body_candidate(symbols, candidate, config);
        if (!body_candidate_is_valid(read))
        {
            continue;
        }
        const auto mode = read.recognition.mode;
        auto same = std::ranges::find_if(groups, [&](const Group& group)
                                         { return group.body == read.first_body_symbol && group.mode.data_rate == mode.data_rate && group.mode.interleave == mode.interleave; });
        if (same == groups.end())
        {
            const auto body_sample = clock.sample_of(read.first_body_symbol);
            groups.push_back({read.first_body_symbol, mode, GroupObservation{.body_sample = body_sample, .path = classify(body_sample, timeline)}});
            same = std::prev(groups.end());
        }
        ++same->row.supporters;
        same->row.correlation_sum += read.normalized_correlation;
        same->row.best_correlation = std::max(same->row.best_correlation, read.normalized_correlation);
    }

    for (const auto& group : groups)
    {
        seen.groups.push_back(group.row);
    }
    // strongest_group(): the most supporters, then the larger correlation sum, then the best one.
    for (std::size_t index = 1U; index < seen.groups.size(); ++index)
    {
        const auto& candidate = seen.groups[index];
        const auto& best = seen.groups[seen.strongest];
        if (candidate.supporters > best.supporters || (candidate.supporters == best.supporters && candidate.correlation_sum > best.correlation_sum) ||
            (candidate.supporters == best.supporters && candidate.correlation_sum == best.correlation_sum && candidate.best_correlation > best.best_correlation))
        {
            seen.strongest = index;
        }
    }
    std::size_t next{};
    for (std::size_t index = 0U; index < seen.groups.size(); ++index)
    {
        if (index != seen.strongest)
        {
            next = std::max(next, seen.groups[index].supporters);
        }
    }
    seen.strongest_margin = seen.groups.empty() ? 0U : seen.groups[seen.strongest].supporters - std::min(next, seen.groups[seen.strongest].supporters);
}

ProfileObservation profile(std::span<const IQSample> symbols, const BodyAcquisitionConfig& config, const SymbolClock& clock, const Timeline& timeline, const Result<BodyAcquisition>& strongest)
{
    ProfileObservation seen;
    const auto acquired = acquire_body_preamble(symbols, config);
    if (!acquired)
    {
        seen.anchor = Path::none;
        return seen;
    }
    const auto& found = acquired.value();
    seen.body_sample = clock.sample_of(found.first_body_symbol);
    seen.anchor = classify(seen.body_sample, timeline);
    seen.diagnostics = found.anchor;
    seen.anchor_correlation = correlation_at(symbols, found.detected_segment_symbol);
    if (strongest && strongest.value().first_body_symbol != found.first_body_symbol)
    {
        seen.moved_by = found.anchor.reanchored ? Move::precursor_search : Move::earlier_group;
    }

    // Where the walk tested the earliest peak: every segment start from the anchor's own
    // detected segment, reached back by that lag, up to the body boundary.
    const auto lag = found.anchor.precursor_candidates != 0U ? found.anchor.precursor_lag : 0U;
    if (lag != 0U && !found.anchor.reanchored)
    {
        auto start = found.detected_segment_symbol;
        while (start < lag)
        {
            start += segment_symbols;
        }
        for (auto shifted = start - lag; shifted + segment_symbols <= std::min(found.first_body_symbol, symbols.size()); shifted += segment_symbols)
        {
            seen.walk_lag_correlation_max = std::max(seen.walk_lag_correlation_max, correlation_at(symbols, shifted));
        }
    }
    return seen;
}

} // namespace

const char* path_name(Path path) noexcept
{
    switch (path)
    {
    case Path::direct:
        return "direct";
    case Path::echo:
        return "echo";
    case Path::neither:
        return "neither";
    default:
        return "none";
    }
}

const char* move_name(Move move) noexcept
{
    switch (move)
    {
    case Move::earlier_group:
        return "earlier_group";
    case Move::precursor_search:
        return "precursor_search";
    default:
        return "none";
    }
}

Result<std::vector<float>> read_recording(const std::string& path)
{
    auto wav = read_wav(path);
    if (!wav)
    {
        return wav.error();
    }
    if (wav.value().sample_rate_hz != sample_rate_hz)
    {
        return Status{StatusCode::invalid_argument, "the recording must be 48 kHz"};
    }
    return std::move(wav.value().samples);
}

Result<Observation> observe(const std::vector<float>& recording, const Timeline& timeline, std::size_t phase)
{
    std::vector<float> line(phase, 0.0F);
    line.insert(line.end(), recording.begin(), recording.end());

    // One symbol per 20-sample block, so the loop's position is kept for every symbol.
    FrontEndRunner runner;
    auto status = runner.initialize(FrontEndOptions{.block_sizes = {symbol_block_samples}, .phase_search = true});
    if (status.is_ok())
    {
        status = runner.feed(line);
    }
    if (!status.is_ok())
    {
        return status;
    }
    const auto symbols = on_time_symbols(runner);
    const SymbolClock clock(runner, widen<double>(as_real(phase)));

    Observation seen{.phase = phase};
    seen.direct_preamble_symbol = clock.symbol_at(timeline.direct_symbol0_sample);
    seen.scan_last_start = seen.direct_preamble_symbol + scan_symbols;
    scan_groups(symbols, acquisition_config(seen.scan_last_start, false, adaptive_window), clock, timeline, seen);

    const auto strongest = acquire_body_preamble(symbols, acquisition_config(seen.scan_last_start, false, adaptive_window));
    seen.off = profile(symbols, acquisition_config(seen.scan_last_start, false, adaptive_window), clock, timeline, strongest);
    seen.window_7 = profile(symbols, acquisition_config(seen.scan_last_start, true, adaptive_window), clock, timeline, strongest);
    seen.window_3 = profile(symbols, acquisition_config(seen.scan_last_start, true, turbo_window), clock, timeline, strongest);
    return seen;
}

} // namespace modem_test::first_path
