// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#include "tests/support/front_end_harness.hpp"

#include "modem/common/convert.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>
#include <optional>

namespace modem_test
{

using ::modem::common::as_real;
using ::modem::common::clamp_to_size;
using ::modem::common::IQSample;
using ::modem::common::mean;
using ::modem::common::narrow;
using ::modem::common::round_to;
using ::modem::common::Status;
using ::modem::common::StatusCode;
using ::modem::common::widen;
using ::modem::m110a::BodyAudioStreamFrontend;
using ::modem::m110a::BodyFrontendSymbol;
using ::modem::m110a::TimingRateMode;

namespace
{

constexpr float pi = std::numbers::pi_v<float>;
constexpr std::int64_t samples_per_symbol = 20;
// The receive filter's group delay: a symbol's peak comes out 80 samples after it went in.
constexpr std::uint64_t filter_delay_samples = 80U;

IQSample ideal(std::uint8_t tribit) noexcept
{
    return std::polar(1.0F, pi / 4.0F * as_real(tribit));
}

// The on-time sample for burst symbol j at the given offset, if the runner kept it.
std::optional<IQSample> on_time_at(const FrontEndRunner& runner, std::size_t j, std::int64_t offset)
{
    const auto stream = narrow<std::int64_t>(j) + offset - narrow<std::int64_t>(runner.symbol_base());

    if (stream < 0 || stream >= narrow<std::int64_t>(runner.symbols().size()))
    {
        return std::nullopt;
    }

    return runner.symbols()[narrow<std::size_t>(stream)].on_time;
}

} // namespace

std::vector<IQSample> on_time_symbols(const FrontEndRunner& runner)
{
    std::vector<IQSample> symbols;
    symbols.reserve(runner.symbols().size());

    for (const auto& symbol : runner.symbols())
    {
        symbols.push_back(symbol.on_time);
    }

    return symbols;
}

std::uint64_t locate_burst(const FrontEndRunner& runner, const BurstTruth& burst, std::uint64_t burst_first_sample)
{
    constexpr std::int64_t search = 40;
    constexpr std::size_t compared = 2000U;
    constexpr std::size_t group = 8U;
    // Where score_burst predicts burst symbol 0, before the timing loop has had any say.
    const auto predicted = (narrow<std::int64_t>(burst_first_sample) + 80 - 170) / samples_per_symbol;
    const auto first_compared = burst.tribits.size() - std::min(compared, burst.tribits.size());
    auto best_offset = predicted;
    float best = -1.0F;

    for (auto offset = predicted - search; offset <= predicted + search; ++offset)
    {
        float score{};
        IQSample sum{};

        for (std::size_t j = first_compared; j < burst.tribits.size(); ++j)
        {
            if (const auto on_time = on_time_at(runner, j, offset))
            {
                sum += *on_time * std::conj(ideal(burst.tribits[j]));
            }

            if ((j + 1U) % group == 0U)
            {
                score += std::abs(sum);
                sum = IQSample{};
            }
        }

        if (score > best)
        {
            best = score;
            best_offset = offset;
        }
    }

    return narrow<std::uint64_t>(std::max<std::int64_t>(best_offset, 0));
}

std::uint8_t nearest_tribit(IQSample symbol) noexcept
{
    const auto eighths = round_to<int>(std::arg(symbol) / (pi / 4.0F));
    return narrow<std::uint8_t>(((eighths % 8) + 8) % 8);
}

Status FrontEndRunner::initialize(const FrontEndOptions& options)
{
    if (options.block_sizes.empty())
    {
        return {StatusCode::invalid_argument, "front-end runner needs a block size"};
    }

    for (std::size_t index = 1U; index < options.timing_rate_changes.size(); ++index)
    {
        if (options.timing_rate_changes[index].at_sample < options.timing_rate_changes[index - 1U].at_sample)
        {
            return {StatusCode::invalid_argument, "front-end runner rate changes must be in order"};
        }
    }

    mOptions = options;
    mSymbols.clear();
    mTimingPoints.clear();
    mSymbolBase = 0U;
    mSamplesFed = 0U;
    mNextBlock = 0U;
    mNextChange = 0U;
    const auto status = mFrontEnd.initialize(BodyAudioStreamFrontend::default_timing_loop_gain, BodyAudioStreamFrontend::default_maximum_timing_step, options.receive_rolloff);

    if (status.is_ok())
    {
        mFrontEnd.configure_timing_tracker(options.timing_integral_gain.has_value(), options.timing_integral_gain.value_or(0.0F));
        mFrontEnd.set_phase_search(options.phase_search);
    }

    return status;
}

Status FrontEndRunner::feed(std::span<const float> audio)
{
    std::vector<BodyFrontendSymbol> block_output(BodyAudioStreamFrontend::maximum_block_symbols);
    const auto& changes = mOptions.timing_rate_changes;

    for (std::size_t start = 0U; start < audio.size();)
    {
        // A rate-mode change whose sample has come lands now, before that sample; the block
        // in front of the next one is cut short at its sample.
        while (mNextChange < changes.size() && changes[mNextChange].at_sample <= mSamplesFed)
        {
            const auto& change = changes[mNextChange];

            if (change.rate)
            {
                mFrontEnd.freeze_timing_rate(*change.rate);
            }
            else
            {
                mFrontEnd.set_timing_rate_mode(change.mode);
            }

            ++mNextChange;
        }

        // The search goes off the same way, before its sample.
        const auto& search_off_at = mOptions.phase_search_off_at_sample;

        if (search_off_at && mFrontEnd.phase_search() && *search_off_at <= mSamplesFed)
        {
            mFrontEnd.set_phase_search(false);
        }

        auto size = std::min({mOptions.block_sizes[mNextBlock], audio.size() - start, BodyAudioStreamFrontend::maximum_block_samples});

        if (mNextChange < changes.size())
        {
            size = clamp_to_size(changes[mNextChange].at_sample - mSamplesFed, size);
        }

        if (search_off_at && mFrontEnd.phase_search())
        {
            size = clamp_to_size(*search_off_at - mSamplesFed, size);
        }

        mNextBlock = (mNextBlock + 1U) % mOptions.block_sizes.size();
        std::size_t produced{};
        const auto status = mFrontEnd.process(audio.subspan(start, size), block_output, produced);

        if (!status.is_ok())
        {
            return status;
        }

        mSymbols.insert(mSymbols.end(), block_output.begin(), block_output.begin() + narrow<std::ptrdiff_t>(produced));

        if (produced > 0U)
        {
            mTimingPoints.push_back({.symbol = mFrontEnd.symbols_produced(),
                                     .position_sample = mFrontEnd.timing_position_sample(),
                                     .position_fraction = mFrontEnd.timing_position_fraction(),
                                     .timing_rate = mFrontEnd.timing_rate()});
        }

        start += size;
        mSamplesFed += size;
    }

    return Status::success();
}

void FrontEndRunner::discard_symbols() noexcept
{
    mSymbolBase += mSymbols.size();
    mSymbols.clear();
    mTimingPoints.clear();
}

BurstScore score_burst(const FrontEndRunner& runner, const BurstTruth& burst, std::uint64_t burst_first_sample)
{
    BurstScore score;
    const auto symbols = burst.tribits.size();

    // Where burst symbol 0 should appear: the transmitter puts symbol j's peak at sample
    // first + 20j, the matched filter delays it 80 samples, and the front end reads its
    // first symbol at sample 170. Search a few symbols either side of that.
    const auto predicted = (narrow<std::int64_t>(burst_first_sample) + 80 - 170) / samples_per_symbol;
    const auto window = std::min<std::size_t>(1000U, symbols / 2U);
    const auto window_begin = symbols - window;
    float best_coherence = -1.0F;
    IQSample derotate{1.0F, 0.0F};

    for (auto offset = predicted - 3; offset <= predicted + 3; ++offset)
    {
        IQSample sum{};
        float magnitude{};

        for (std::size_t j = window_begin; j < symbols; ++j)
        {
            if (const auto on_time = on_time_at(runner, j, offset))
            {
                sum += *on_time * std::conj(ideal(burst.tribits[j]));
                magnitude += std::abs(*on_time);
            }
        }

        const auto coherence = magnitude > 0.0F ? std::abs(sum) / magnitude : 0.0F;

        if (coherence > best_coherence)
        {
            best_coherence = coherence;
            score.offset = offset;
            derotate = std::abs(sum) > 0.0F ? std::conj(sum) / std::abs(sum) : IQSample{1.0F, 0.0F};
        }
    }

    score.rotation_degrees = -std::arg(derotate) * 180.0F / pi;
    // Locked: the timing loop sits on the symbol peaks, where the on-time samples are
    // largest on average. The first 200-symbol window whose mean magnitude reaches 99% of
    // the burst's settled mean (its last 1000 symbols) marks the lock. That does not depend
    // on the decisions, so a test can then hold the decisions to account.
    const auto mean_magnitude = [&](std::size_t begin, std::size_t end)
    {
        float sum{};
        std::size_t counted{};

        for (std::size_t j = begin; j < end; ++j)
        {
            if (const auto on_time = on_time_at(runner, j, score.offset))
            {
                sum += std::abs(*on_time);
                ++counted;
            }
        }

        return mean(sum, counted);
    };

    const auto settled = mean_magnitude(window_begin, symbols);
    constexpr std::size_t lock_window = 200U;

    for (std::size_t start = 0U; start + lock_window <= symbols && !score.locked && settled > 0.0F; start += lock_window / 2U)
    {
        if (mean_magnitude(start, start + lock_window) >= 0.99F * settled)
        {
            score.locked = true;
            score.locked_from = start;
        }
    }

    if (!score.locked)
    {
        return score;
    }

    float squared_error{};

    for (std::size_t j = score.locked_from; j < symbols; ++j)
    {
        const auto on_time = on_time_at(runner, j, score.offset);

        if (!on_time)
        {
            continue;
        }

        const auto corrected = *on_time * derotate;
        ++score.compared;
        score.correct += nearest_tribit(corrected) == burst.tribits[j] ? 1U : 0U;
        const auto error_degrees = std::arg(corrected * std::conj(ideal(burst.tribits[j]))) * 180.0F / pi;
        squared_error += error_degrees * error_degrees;
    }

    score.rms_phase_error_degrees = std::sqrt(mean(squared_error, score.compared));
    return score;
}

std::vector<GridOffset> grid_offsets(const FrontEndRunner& runner, const BurstTruth& burst, std::uint64_t waveform_start_sample, float clock_ppm)
{
    // The channel's clock replays the transmitter's samples input_per_output = 1 + ppm/1e6
    // at a time, so transmit sample t lands at receive sample t / ratio; the matched filter
    // then delays the peaks. Positions are taken relative to the waveform's start (exact in
    // double within 2^24 samples of it, which any one burst is).
    const double ratio = 1.0 + widen<double>(clock_ppm) * 1.0e-6;
    const auto symbols = widen<double>(as_real(burst.tribits.size()));
    std::vector<GridOffset> offsets;

    for (const auto& point : runner.timing_points())
    {
        if (point.position_sample < waveform_start_sample + filter_delay_samples)
        {
            continue;
        }

        const auto relative = widen<double>(as_real(point.position_sample - waveform_start_sample - filter_delay_samples)) + widen<double>(point.position_fraction);
        const auto transmit_sample = relative * ratio - widen<double>(as_real(burst.first_sample));
        const auto burst_symbol = transmit_sample / widen<double>(as_real(samples_per_symbol));

        if (burst_symbol < 0.0 || burst_symbol >= symbols)
        {
            continue;
        }

        const auto nearest = std::round(burst_symbol);
        offsets.push_back(
            {.symbol = point.symbol, .burst_symbol = burst_symbol, .offset_samples = (burst_symbol - nearest) * widen<double>(as_real(samples_per_symbol)) / ratio, .timing_rate = point.timing_rate});
    }

    return offsets;
}

float grid_rate(float clock_ppm) noexcept
{
    return as_real(samples_per_symbol) / (1.0F + clock_ppm * 1.0e-6F) - as_real(samples_per_symbol);
}

} // namespace modem_test
