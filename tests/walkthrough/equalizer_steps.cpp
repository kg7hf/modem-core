// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#include "tests/walkthrough/equalizer_steps.hpp"

#include "modem/common/constants.hpp"
#include "modem/common/convert.hpp"
#include "modem/m110a/demapper.hpp"
#include "tests/support/front_end_harness.hpp"
#include "tests/support/test_waveform.hpp"
#include "tests/walkthrough/first_path_observe.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <print>
#include <span>
#include <utility>

namespace modem_test::equalizer_steps
{

using ::modem::common::as_real;
using ::modem::common::BodyInterleave;
using ::modem::common::DataRate;
using ::modem::common::narrow;
using ::modem::common::Status;
using ::modem::common::StatusCode;
using ::modem::common::two_pi;
using ::modem::m110a::BodyAcquisitionConfig;
using ::modem::m110a::BodyAudioStreamFrontend;
using ::modem::m110a::BodyBlockPlan;
using ::modem::m110a::TimingRateMode;

namespace
{

constexpr std::size_t samples_per_symbol = ::modem::m110a::body_audio_samples_per_symbol; // 20
constexpr std::uint64_t first_on_time_sample = BodyAudioStreamFrontend::matched_filter_taps - 1U + samples_per_symbol / 2U;
constexpr std::size_t lead_samples = 24000U; // half a second of line in front of the clean burst
constexpr std::size_t maximum_delay_symbols = 15U;
constexpr std::size_t shown_frames = 24U;                 // the frames a step prints
const std::vector<std::uint8_t> hi_payload{0x48U, 0x69U}; // "Hi", the recordings' message

// Part 4's walkthrough settings: the 0.35 floor, every start searched, the anchoring on.
BodyAcquisitionConfig acquisition_config() noexcept
{
    BodyAcquisitionConfig config;
    config.minimum_correlation = 0.35F;
    return config;
}

// The stream sample half a symbol before the loop reads stream symbol `symbol`, from where
// the search pass had it (as the runner commits).
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

// The front end, the phase search on; committed from `commit_sample` when there is one.
Status run_front_end(FrontEndRunner& runner, std::span<const float> audio, std::optional<std::uint64_t> commit_sample)
{
    FrontEndOptions options;
    options.phase_search = true;
    if (commit_sample)
    {
        options.phase_search_off_at_sample = commit_sample;
        options.timing_integral_gain = BodyAudioStreamFrontend::default_timing_integral_gain;
        options.timing_rate_changes.push_back({.at_sample = *commit_sample, .mode = TimingRateMode::tracking});
    }
    auto status = runner.initialize(options);
    if (status.is_ok())
    {
        status = runner.feed(audio);
    }
    return status;
}

// Search, acquire, commit, and keep the committed pass's symbols, both samples of each.
Result<Burst> receive(std::string id, BodyMode mode, std::span<const float> audio, const TestWaveform& truth)
{
    FrontEndRunner search;
    if (const auto status = run_front_end(search, audio, std::nullopt); !status.is_ok())
    {
        return status;
    }
    const auto searched = on_time_symbols(search);
    const auto acquired = acquire_body_preamble(searched, acquisition_config());
    if (!acquired)
    {
        return acquired.error();
    }
    const auto& found = acquired.value();
    const auto first_candidate = found.preamble_start_in_buffer ? found.first_preamble_symbol : found.detected_segment_symbol;
    const auto commit_symbol = std::min(first_candidate + ::modem::m110a::body_preamble_segment_symbols, searched.size() - 1U);

    FrontEndRunner committed;
    if (const auto status = run_front_end(committed, audio, sample_before_symbol(search, commit_symbol)); !status.is_ok())
    {
        return status;
    }

    return burst_from(std::move(id), mode, committed, found, &truth.bursts.front());
}

Result<TestWaveform> transmit(BodyMode mode)
{
    return make_test_waveform(WaveformSpec{.mode = mode, .payload = hi_payload});
}

// A data decision against what was sent, in the mode's own alphabet. The scrambler turns
// each data symbol into an 8-PSK point, but the points the transmitter could have sent in
// its place are the sent one turned by 8 / 2^bits octants: two points at 600 bps and below,
// four at 1200, all eight at 2400. The decision is wrong when one of those others is nearer.
bool decided_wrong(IQSample output, std::uint8_t sent, std::uint8_t bits_per_symbol) noexcept
{
    const auto step = 8U >> std::clamp<unsigned>(bits_per_symbol, 1U, 3U);
    const auto own = std::norm(output - ::modem::m110a::psk8_symbol(sent));
    for (unsigned turn = step; turn < 8U; turn += step)
    {
        if (std::norm(output - ::modem::m110a::psk8_symbol(narrow<std::uint8_t>((sent + turn) & 7U))) < own)
        {
            return true;
        }
    }
    return false;
}

float hz(float radians_per_symbol, float symbol_rate_hz) noexcept
{
    return radians_per_symbol * symbol_rate_hz / two_pi;
}

// The three bursts the first two steps measure.
Result<std::vector<Burst>> three_bursts(const Pack& pack, std::string& diagnostic)
{
    std::vector<Burst> bursts;
    for (const auto id : {five_ms_case, two_ms_case})
    {
        auto burst = receive_recording(pack, id, diagnostic);
        if (!burst)
        {
            return burst.error();
        }
        bursts.push_back(std::move(burst.value()));
    }
    auto clean = receive_clean();
    if (!clean)
    {
        return clean.error();
    }
    bursts.push_back(std::move(clean.value()));
    return bursts;
}

Result<EngineWalk> walk_engine(const Pack& pack, BodyDataEqualizer engine, std::string& diagnostic)
{
    const auto burst = receive_recording(pack, five_ms_case, diagnostic);
    if (!burst)
    {
        return burst.error();
    }
    auto trained = train(burst.value(), engine);
    if (!trained)
    {
        return trained.error();
    }
    auto body = walk_body(burst.value(), trained.value());
    if (!body)
    {
        return body.error();
    }
    return EngineWalk{.id = burst.value().id, .engine = engine, .body = std::move(body.value())};
}

const char* engine_name(BodyDataEqualizer engine) noexcept
{
    return engine == BodyDataEqualizer::cir_mmse ? "CIR-MMSE" : "adaptive";
}

} // namespace

Result<Burst> receive_recording(const Pack& pack, std::string_view id, std::string& diagnostic)
{
    const auto* manifest = pack.cases.find(id);
    if (manifest == nullptr)
    {
        diagnostic = std::string{id} + " is not in the package";
        return Status{StatusCode::unavailable, "recording not in the package"};
    }
    const auto audio = ::modem_test::first_path::read_recording(pack.root + "/" + manifest->audio);
    if (!audio)
    {
        diagnostic = manifest->audio;
        return audio.error();
    }
    const auto truth = transmit(manifest->mode);
    if (!truth)
    {
        return truth.error();
    }
    return receive(std::string{id}, manifest->mode, audio.value(), truth.value());
}

Result<Burst> receive_audio(std::string id, BodyMode mode, std::span<const float> audio, const TestWaveform& truth)
{
    return receive(std::move(id), mode, audio, truth);
}

Burst burst_from(std::string id, BodyMode mode, const FrontEndRunner& committed, const BodyAcquisition& acquired, const BurstTruth* sent)
{
    Burst burst{.id = std::move(id), .mode = mode, .acquired = acquired};
    burst.symbols = on_time_symbols(committed);
    burst.half_symbols.reserve(committed.symbols().size() * 2U);
    for (const auto& symbol : committed.symbols())
    {
        burst.half_symbols.push_back(symbol.early);
        burst.half_symbols.push_back(symbol.on_time);
    }
    if (sent != nullptr)
    {
        burst.sent = sent->tribits;
        burst.sent_preamble_symbols = sent->preamble_symbols;
        burst.payload = sent->payload;
    }
    return burst;
}

Result<Burst> receive_clean()
{
    const BodyMode mode{DataRate::bps600, BodyInterleave::long_block};
    const auto truth = transmit(mode);
    if (!truth)
    {
        return truth.error();
    }
    std::vector<float> line(lead_samples, 0.0F);
    line.insert(line.end(), truth.value().audio.begin(), truth.value().audio.end());
    line.resize(line.size() + lead_samples, 0.0F);
    return receive(std::string{clean_case}, mode, line, truth.value());
}

Result<Trained> train(const Burst& burst, BodyDataEqualizer engine)
{
    const auto& acquired = burst.acquired;
    const auto preamble_symbols = ::modem::m110a::body_preamble_symbols(burst.mode.interleave);
    const auto first_body = acquired.first_body_symbol;
    const auto training_first = acquired.preamble_start_in_buffer ? acquired.first_preamble_symbol : acquired.detected_segment_symbol;
    if (first_body <= training_first || first_body - training_first > preamble_symbols || first_body * 2U > burst.half_symbols.size())
    {
        return Status{StatusCode::invalid_argument, "the acquired body boundary does not fit the preamble"};
    }
    const auto training_symbols = first_body - training_first;

    // The preamble's last training_symbols symbols, as the transmitter sent them.
    std::vector<std::uint8_t> preamble(preamble_symbols);
    if (const auto status = ::modem::m110a::generate_body_preamble(burst.mode, preamble); !status.is_ok())
    {
        return status;
    }
    const auto expected = std::span<const std::uint8_t>{preamble}.subspan(preamble_symbols - training_symbols, training_symbols);
    auto local = acquired;
    local.first_preamble_symbol = 0U;
    local.detected_segment_symbol = 0U;
    local.first_body_symbol = training_symbols;
    local.preamble_start_in_buffer = true;

    Trained trained{.receiver = std::make_unique<BodyAdaptiveReceiver>(), .engine = engine};
    auto& receiver = *trained.receiver;
    if (const auto status = receiver.select_data_equalizer(engine); !status.is_ok())
    {
        return status;
    }
    const auto training_samples = std::span<const IQSample>{burst.half_symbols}.subspan(training_first * 2U, training_symbols * 2U);
    const auto configure_and_train = [&](const BodyReceiverGeometry& geometry) -> Status
    {
        // 75 bps decides a 32-symbol group at once, so it has nothing to feed back.
        const auto feedback_taps = burst.mode.data_rate == DataRate::bps75 ? 0U : geometry.feedback_taps;
        auto status = receiver.configure_fractionally_spaced(geometry.feedforward_taps, feedback_taps, ::modem::m110a::body_receiver_equalizer_step_for(burst.mode.data_rate),
                                                             ::modem::m110a::body_receiver_carrier_proportional_gain, ::modem::m110a::body_receiver_carrier_integral_gain, geometry.cursor);
        return status.is_ok() ? receiver.train_preamble_fractional(training_samples, expected, local) : status;
    };

    // The default window first: the delay spread needs the frequency training chooses.
    trained.first_geometry = ::modem::m110a::select_body_receiver_geometry(0U);
    if (const auto status = configure_and_train(trained.first_geometry); !status.is_ok())
    {
        return status;
    }
    auto measured = acquired;
    measured.frequency_offset_hz = hz(receiver.selected_training_frequency_radians(), acquired.symbol_rate_hz);
    const auto spread = ::modem::m110a::estimate_body_preamble_delay_spread(burst.symbols, measured, maximum_delay_symbols, &trained.profile);
    trained.delay_spread = spread ? spread.value() : 0U;
    trained.geometry = ::modem::m110a::select_body_receiver_geometry(trained.delay_spread);
    if (trained.geometry.feedforward_taps != trained.first_geometry.feedforward_taps || trained.geometry.cursor != trained.first_geometry.cursor)
    {
        trained.retrained = true;
        if (const auto status = configure_and_train(trained.geometry); !status.is_ok())
        {
            return status;
        }
    }
    trained.race = receiver.last_preamble_race();
    trained.training_frequency_hz = hz(receiver.selected_training_frequency_radians(), acquired.symbol_rate_hz);
    return trained;
}

Result<Body> walk_body(const Burst& burst, Trained& trained)
{
    const auto plan_result = ::modem::m110a::body_block_plan(burst.mode);
    if (!plan_result)
    {
        return plan_result.error();
    }
    const BodyBlockPlan& plan = plan_result.value();
    if (plan.unknown_symbols_per_probe == 0U || plan.known_symbols_per_probe == 0U)
    {
        // 75 bps decides a 32-symbol group at once and sends no probes between them.
        return Status{StatusCode::unavailable, "this mode has no data frames and no probes"};
    }
    const auto block_symbols = plan.transmitted_symbols;
    const bool known = !burst.sent.empty();
    const auto first_body = burst.acquired.first_body_symbol;
    const auto heard_symbols = burst.half_symbols.size() / 2U;
    std::size_t blocks{};
    if (known)
    {
        blocks = (burst.sent.size() - burst.sent_preamble_symbols) / block_symbols;
    }
    else if (heard_symbols > first_body)
    {
        blocks = (heard_symbols - first_body) / block_symbols;
    }
    if ((first_body + blocks * block_symbols) * 2U > burst.half_symbols.size())
    {
        return Status{StatusCode::buffer_too_small, "the recording ends before the burst's last block"};
    }

    std::vector<IQSample> equalized(block_symbols);
    Body body{.blocks = blocks};
    float probe_error_sum{};

    for (std::size_t block = 0U; block < blocks; ++block)
    {
        const auto block_first = first_body + block * block_symbols;
        const auto samples = std::span<const IQSample>{burst.half_symbols}.subspan(block_first * 2U, block_symbols * 2U);
        if (const auto status = trained.receiver->equalize_block_fractional(samples, plan, equalized); !status.is_ok())
        {
            return status;
        }
        probe_error_sum += trained.receiver->last_block_probe_error();
        if (!known)
        {
            continue;
        }

        // Each frame's data decisions against what was sent, and its probe's squared error.
        const auto sent = std::span<const std::uint8_t>{burst.sent}.subspan(burst.sent_preamble_symbols + block * block_symbols, block_symbols);
        const auto frame_symbols = plan.frame_symbols();
        for (std::size_t frame = 0U; frame * frame_symbols < block_symbols; ++frame)
        {
            Frame row{.block = block, .frame = frame, .data_symbols = plan.unknown_symbols_per_probe};
            const auto first = frame * frame_symbols;
            for (std::size_t symbol = 0U; symbol < plan.unknown_symbols_per_probe; ++symbol)
            {
                row.data_errors += decided_wrong(equalized[first + symbol], sent[first + symbol], plan.information_bits_per_channel_symbol) ? 1U : 0U;
            }
            float probe_energy{};
            for (std::size_t symbol = plan.unknown_symbols_per_probe; symbol < frame_symbols; ++symbol)
            {
                probe_energy += std::norm(equalized[first + symbol] - ::modem::m110a::psk8_symbol(sent[first + symbol]));
            }
            row.probe_error = probe_energy / as_real(plan.known_symbols_per_probe);
            body.data_symbols += row.data_symbols;
            body.data_errors += row.data_errors;
            body.frames.push_back(row);
        }
    }
    body.probe_error = blocks == 0U ? 0.0F : probe_error_sum / as_real(blocks);
    return body;
}

Result<DelaySpread> delay_spread(const Pack& pack, std::string& diagnostic)
{
    const auto bursts = three_bursts(pack, diagnostic);
    if (!bursts)
    {
        return bursts.error();
    }
    DelaySpread step;
    for (const auto& burst : bursts.value())
    {
        const auto trained = train(burst, BodyDataEqualizer::adaptive);
        if (!trained)
        {
            diagnostic = burst.id;
            return trained.error();
        }
        const auto& profile = trained.value().profile;
        SpreadRow row{.id = burst.id, .spread = trained.value().delay_spread, .segments = profile.segments};
        for (std::size_t lag = 0U; lag <= maximum_delay_symbols; ++lag)
        {
            row.lag_fraction.push_back(profile.strongest_power > 0.0F ? profile.lag_powers[lag] / profile.strongest_power : 0.0F);
        }
        step.rows.push_back(std::move(row));
    }
    return step;
}

void print(const DelaySpread& step)
{
    std::println("   lag power behind the anchor, as a fraction of the strongest (lags 0 to 15; . under 0.05):");
    for (const auto& row : step.rows)
    {
        std::string profile;
        for (const auto fraction : row.lag_fraction)
        {
            profile += fraction < 0.05F ? "    ." : std::format(" {:4.2f}", fraction);
        }
        std::println("   {:<15}{}", row.id, profile);
        std::println("   {:<15} delay spread {} symbols, over {} preamble segments", "", row.spread, row.segments);
    }
}

Result<Window> window(const Pack& pack, std::string& diagnostic)
{
    const auto bursts = three_bursts(pack, diagnostic);
    if (!bursts)
    {
        return bursts.error();
    }
    Window step;
    for (const auto& burst : bursts.value())
    {
        const auto trained = train(burst, BodyDataEqualizer::adaptive);
        if (!trained)
        {
            diagnostic = burst.id;
            return trained.error();
        }
        step.rows.push_back(WindowRow{.id = burst.id, .spread = trained.value().delay_spread, .geometry = trained.value().geometry, .retrained = trained.value().retrained});
    }
    return step;
}

void print(const Window& step)
{
    for (const auto& row : step.rows)
    {
        const auto& g = row.geometry;
        const auto ahead = as_real(g.cursor) / 2.0F;
        const auto behind = as_real(g.feedforward_taps - g.cursor - 1U) / 2.0F;
        std::println("   {:<15} spread {:>2} -> {} taps, reference tap {}: {:.1f} symbols ahead, {:.1f} behind, {} feedback{}", row.id, row.spread, g.feedforward_taps, g.cursor, ahead, behind,
                     g.feedback_taps, row.retrained ? "; trained again on it" : "");
    }
}

Result<Training> training(const Pack& pack, std::string& diagnostic)
{
    const auto burst = receive_recording(pack, five_ms_case, diagnostic);
    if (!burst)
    {
        return burst.error();
    }
    const auto trained = train(burst.value(), BodyDataEqualizer::adaptive);
    if (!trained)
    {
        return trained.error();
    }
    return Training{
        .id = burst.value().id, .acquisition_frequency_hz = burst.value().acquired.frequency_offset_hz, .race = trained.value().race, .symbol_rate_hz = burst.value().acquired.symbol_rate_hz};
}

void print(const Training& step)
{
    const auto& race = step.race;
    std::println("   {}: acquisition's frequency {:+.2f} Hz", step.id, step.acquisition_frequency_hz);
    if (!race.escalated)
    {
        std::println("   its training reached the floor (error {:.3f}): no other guess trained", race.residual[0]);
        return;
    }
    std::println("   its error stayed above the floor, so the receiver trained every distinct guess:");
    for (std::size_t index = 0U; index < race.count; ++index)
    {
        std::println("   {:<26} {:+7.2f} Hz   error on the preamble's tail {:.3f}{}", index == 0U ? "acquisition's estimate" : "a new guess", hz(race.frequency_radians[index], step.symbol_rate_hz),
                     race.residual[index], index == race.winner ? "   <- kept" : "");
    }
    std::println("   (the new guesses: the whole-preamble estimate and the channel-fit search, each when it differs)");
}

Result<EngineWalk> adaptive(const Pack& pack, std::string& diagnostic)
{
    return walk_engine(pack, BodyDataEqualizer::adaptive, diagnostic);
}

Result<EngineWalk> cir_mmse(const Pack& pack, std::string& diagnostic)
{
    return walk_engine(pack, BodyDataEqualizer::cir_mmse, diagnostic);
}

void print(const EngineWalk& step)
{
    const auto& body = step.body;
    std::size_t clean_frames = 0U;
    for (const auto& frame : body.frames)
    {
        clean_frames += frame.data_errors == 0U ? 1U : 0U;
    }
    std::println("   {} on {}: {} of {} data symbols decided wrong before the decoder ({:.1f}%); {} of {} frames without one", engine_name(step.engine), step.id, body.data_errors, body.data_symbols,
                 100.0F * as_real(body.data_errors) / as_real(std::max<std::size_t>(body.data_symbols, 1U)), clean_frames, body.frames.size());
    std::println("   the first {} frames (wrong data symbols / probe error):", std::min<std::size_t>(body.frames.size(), shown_frames));
    std::string line = "  ";
    std::size_t shown = 0U;
    for (const auto& frame : body.frames)
    {
        if (shown == shown_frames)
        {
            break;
        }
        line += std::format(" {:2}/{:.2f}", frame.data_errors, frame.probe_error);
        if (++shown % 8U == 0U)
        {
            std::println("{}", line);
            line = "  ";
        }
    }
    if (line.size() > 2U)
    {
        std::println("{}", line);
    }
}

Result<Fade> fade(const Pack& pack, std::string& diagnostic)
{
    const auto adaptive_walk = adaptive(pack, diagnostic);
    if (!adaptive_walk)
    {
        return adaptive_walk.error();
    }
    const auto cir_walk = cir_mmse(pack, diagnostic);
    if (!cir_walk)
    {
        return cir_walk.error();
    }
    Fade step{.id = adaptive_walk.value().id, .adaptive = adaptive_walk.value().body, .cir_mmse = cir_walk.value().body};
    const auto frames = std::min(step.adaptive.frames.size(), step.cir_mmse.frames.size());
    for (std::size_t index = 0U; index < frames; ++index)
    {
        step.widest.push_back(index);
    }
    // How far CIR-MMSE trails the adaptive equalizer in a frame (negative: it leads).
    const auto lead = [&](std::size_t index) noexcept { return narrow<std::ptrdiff_t>(step.cir_mmse.frames[index].data_errors) - narrow<std::ptrdiff_t>(step.adaptive.frames[index].data_errors); };
    // The three frames where each engine trails the other most, in frame order within each.
    auto order = step.widest;
    std::ranges::stable_sort(order, [&](std::size_t left, std::size_t right) noexcept { return lead(left) > lead(right); });
    const auto take = std::min<std::size_t>(order.size() / 2U, 3U);
    std::vector<std::size_t> cir_behind(order.begin(), order.begin() + narrow<std::ptrdiff_t>(take));
    std::vector<std::size_t> adaptive_behind(order.end() - narrow<std::ptrdiff_t>(take), order.end());
    std::ranges::sort(cir_behind);
    std::ranges::sort(adaptive_behind);
    step.widest = cir_behind;
    step.widest.insert(step.widest.end(), adaptive_behind.begin(), adaptive_behind.end());
    return step;
}

void print(const Fade& step)
{
    std::println("   {}: where each engine trails the other most (wrong data symbols / probe error)", step.id);
    std::println("   block frame   adaptive      CIR-MMSE");
    const auto half = step.widest.size() / 2U;
    for (std::size_t row = 0U; row < step.widest.size(); ++row)
    {
        if (row == 0U || row == half)
        {
            std::println("   {}", row == 0U ? "CIR-MMSE behind:" : "adaptive behind:");
        }
        const auto& a = step.adaptive.frames[step.widest[row]];
        const auto& c = step.cir_mmse.frames[step.widest[row]];
        std::println("   {:5} {:5}   {:2} / {:.2f}     {:2} / {:.2f}", a.block, a.frame, a.data_errors, a.probe_error, c.data_errors, c.probe_error);
    }
}

} // namespace modem_test::equalizer_steps
