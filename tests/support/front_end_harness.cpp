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
using ::modem::common::IQSample;
using ::modem::common::mean;
using ::modem::common::narrow;
using ::modem::common::round_to;
using ::modem::common::Status;
using ::modem::common::StatusCode;
using ::modem::m110a::BodyAudioStreamFrontend;
using ::modem::m110a::BodyFrontendSymbol;

namespace
{

constexpr float pi = std::numbers::pi_v<float>;
constexpr std::int64_t samples_per_symbol = 20;

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

    mOptions = options;
    mSymbols.clear();
    mSymbolBase = 0U;
    mSamplesFed = 0U;
    mNextBlock = 0U;
    return mFrontEnd.initialize(BodyAudioStreamFrontend::default_timing_loop_gain, BodyAudioStreamFrontend::default_maximum_timing_step, options.receive_rolloff);
}

Status FrontEndRunner::feed(std::span<const float> audio)
{
    std::vector<BodyFrontendSymbol> block_output(BodyAudioStreamFrontend::maximum_block_symbols);

    for (std::size_t start = 0U; start < audio.size();)
    {
        const auto size = std::min({mOptions.block_sizes[mNextBlock], audio.size() - start, BodyAudioStreamFrontend::maximum_block_samples});
        mNextBlock = (mNextBlock + 1U) % mOptions.block_sizes.size();
        std::size_t produced{};
        const auto status = mFrontEnd.process(audio.subspan(start, size), block_output, produced);

        if (!status.is_ok())
        {
            return status;
        }

        mSymbols.insert(mSymbols.end(), block_output.begin(), block_output.begin() + narrow<std::ptrdiff_t>(produced));
        start += size;
        mSamplesFed += size;
    }

    return Status::success();
}

void FrontEndRunner::discard_symbols() noexcept
{
    mSymbolBase += mSymbols.size();
    mSymbols.clear();
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

} // namespace modem_test
