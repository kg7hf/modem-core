// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#include "tests/walkthrough/first_path_steps.hpp"

#include "modem/common/convert.hpp"
#include "modem/common/types.hpp"
#include "modem/m110a/synchronization.hpp"
#include "tests/support/front_end_harness.hpp"
#include "tests/support/manifest_json.hpp"
#include "tests/support/receive_chain.hpp"
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
#include <string_view>
#include <utility>
#include <vector>

namespace modem_test::first_path_steps
{

namespace
{

using ::modem::common::as_real;
using ::modem::common::IQSample;
using ::modem::common::narrow;
using ::modem::common::Status;
using ::modem::common::StatusCode;
using ::modem::common::widen;
using ::modem::m110a::acquire_body_preamble;
using ::modem::m110a::body_preamble_acquisition_prefix_symbols;
using ::modem::m110a::body_preamble_segment_symbols;
using ::modem::m110a::body_precursor_sidelobe_powers;
using ::modem::m110a::BodyAcquisitionConfig;
using ::modem::m110a::make_body_prefix_reference;

constexpr float pi = std::numbers::pi_v<float>;
constexpr float symbol_rate_hz = 2400.0F;
constexpr double samples_per_symbol = 20.0;
constexpr std::size_t segment_symbols = body_preamble_segment_symbols; // 480
constexpr std::size_t prefix_symbols = body_preamble_acquisition_prefix_symbols;
constexpr std::size_t prefix_blocks = 9U;
constexpr std::size_t chips = 32U;
constexpr std::size_t maximum_lag = 15U;
constexpr float significant_fraction = 0.125F;
// The sidelobe step's clean burst: half a second of silence, the odd ten samples putting
// the burst's symbol peaks where the front end starts sampling (Part 4).
constexpr std::size_t clean_lead_samples = 24010U;

// The Table XX row each recording was rendered on.
std::string_view channel_text(std::string_view id) noexcept
{
    if (id == "fp2-window" || id == "fp4-phase-short" || id == "fp6-phase-long")
    {
        return "Table XX's 600 bps channel: two paths 2 ms (4.8 symbols) apart, fading at 1 Hz, 7 dB";
    }
    return "Table XX's 300 bps channel: two paths 5 ms (12 symbols) apart, fading at 5 Hz, 7 dB";
}

// A body boundary in symbols from the direct path's.
double symbols_from_direct(double body_sample, const Case& manifest) noexcept
{
    const auto symbols = (body_sample - manifest.timeline.direct_body_symbol0_sample) / samples_per_symbol;
    return std::fabs(symbols) < 0.05 ? 0.0 : symbols; // printed to a tenth: no "-0.0"
}

std::string anchor_text(const ProfileObservation& seen, const Case& manifest)
{
    if (seen.anchor == Path::none)
    {
        return "nothing";
    }
    const auto name = seen.anchor == Path::direct ? std::string{"the direct path"} : seen.anchor == Path::echo ? std::string{"the echo"} : std::string{"neither path"};
    return std::format("{} (body {:+.1f} symbols from the direct path's)", name, symbols_from_direct(seen.body_sample, manifest));
}

std::string_view how(const ProfileObservation& seen) noexcept
{
    switch (seen.moved_by)
    {
    case Move::earlier_group:
        return "an earlier group";
    case Move::precursor_search:
        return "the precursor search";
    default:
        return "not moved";
    }
}

std::string move_text(const ProfileObservation& seen)
{
    switch (seen.moved_by)
    {
    case Move::earlier_group:
        return "moved by an earlier group";
    case Move::precursor_search:
        return std::format("moved by the precursor search, {} symbols", seen.diagnostics.reanchored_lag);
    default:
        return "not moved";
    }
}

void print_groups(const Observation& seen, const Case& manifest)
{
    std::println("the scan's groups (starts in the burst's first {} symbols):", scan_symbols);
    std::println("  body boundary, from the direct path's   supporters  correlation sum  best correlation");
    for (std::size_t index = 0U; index < seen.groups.size(); ++index)
    {
        const auto& group = seen.groups[index];
        const auto at = symbols_from_direct(group.body_sample, manifest);
        const auto label = group.path == Path::direct ? std::format("{:+.1f} symbols, the direct path", at)
                           : group.path == Path::echo ? std::format("{:+.1f} symbols, the echo", at)
                                                      : std::format("{:+.1f} symbols", at);
        std::println("  {:38}  {:10}  {:15.3f}  {:16.3f}{}", label, group.supporters, group.correlation_sum, group.best_correlation, index == seen.strongest ? "  strongest" : "");
    }
}

void print_profile(const ::modem::m110a::BodyAnchorDiagnostics& anchor)
{
    std::println("the precursor profile over {} segments, as a fraction of the anchored path (lag 0):", anchor.precursor_segments);
    std::print("  lag");
    for (std::size_t lag = 1U; lag <= maximum_lag; ++lag)
    {
        std::print("{:6}", lag);
    }
    std::print("\n     ");
    for (std::size_t lag = 1U; lag <= maximum_lag; ++lag)
    {
        std::print("{:6.3f}", anchor.precursor_powers[lag]);
    }
    std::println("");
    std::println("peaks at or above 1/8: {}{}", anchor.precursor_candidates,
                 anchor.precursor_candidates != 0U ? std::format(", the earliest at lag {} ({:.3f})", anchor.precursor_lag, anchor.precursor_fraction) : std::string{});
    std::println("the walk: {} segments tried; rejected by the carrier gate {}, the correlation floor {}, an invalid read {}, another mode {}", anchor.walk_segments, anchor.walk_rejected_frequency,
                 anchor.walk_rejected_correlation, anchor.walk_rejected_invalid, anchor.walk_rejected_mode);
}

std::array<IQSample, prefix_symbols> prefix_reference_points() noexcept
{
    std::array<IQSample, prefix_symbols> reference{};
    make_body_prefix_reference(reference);
    return reference;
}

// The precursor profile as step 5 of acquire_body_preamble() measures it, before the
// subtraction: at each lag before the anchor, the nine-block prefix correlation's power with
// the anchor's carrier taken out, summed over every segment from `first_segment` to the body.
struct Profile
{
    std::array<float, 16U> powers{};
    std::size_t segments{};
};

Profile precursor_profile(std::span<const IQSample> symbols, std::size_t first_segment, std::size_t body, float frequency_hz)
{
    const auto reference = prefix_reference_points();
    const auto radians = frequency_hz * 2.0F * pi / symbol_rate_hz;
    std::array<IQSample, prefix_symbols> rotated{};
    for (std::size_t index = 0U; index < prefix_symbols; ++index)
    {
        rotated[index] = std::conj(reference[index] * std::polar(1.0F, radians * as_real(index)));
    }
    Profile profile;
    for (auto start = first_segment; start + prefix_symbols <= std::min(body, symbols.size()); start += segment_symbols)
    {
        if (start < maximum_lag)
        {
            continue;
        }
        ++profile.segments;
        for (std::size_t lag = 0U; lag <= maximum_lag; ++lag)
        {
            for (std::size_t block = 0U; block < prefix_blocks; ++block)
            {
                IQSample sum{};
                for (std::size_t chip = 0U; chip < chips; ++chip)
                {
                    const auto index = block * chips + chip;
                    sum += symbols[start - lag + index] * rotated[index];
                }
                profile.powers[lag] += std::norm(sum);
            }
        }
    }
    return profile;
}

// The sidelobe model with nothing before the prefix: the reference against itself, zeros
// where it runs off the front.
std::array<float, 16U> prefix_only_sidelobes()
{
    const auto reference = prefix_reference_points();
    std::array<float, 16U> powers{};
    for (std::size_t lag = 0U; lag <= maximum_lag; ++lag)
    {
        for (std::size_t block = 0U; block < prefix_blocks; ++block)
        {
            IQSample sum{};
            for (std::size_t chip = 0U; chip < chips; ++chip)
            {
                const auto index = block * chips + chip;
                if (index >= lag)
                {
                    sum += reference[index - lag] * std::conj(reference[index]);
                }
            }
            powers[lag] += std::norm(sum);
        }
    }
    return powers;
}

float corrected_fraction(const Profile& profile, const std::array<float, 16U>& sidelobes, std::size_t lag) noexcept
{
    const auto power = std::max(0.0F, profile.powers[lag] - (sidelobes[lag] / sidelobes[0]) * profile.powers[0]);
    return power / profile.powers[0];
}

const ProfileObservation& profile_named(const Observation& seen, std::string_view name) noexcept
{
    return name == "off" ? seen.off : name == "window_7" ? seen.window_7 : seen.window_3;
}

char letter(Path path) noexcept
{
    switch (path)
    {
    case Path::direct:
        return 'D';
    case Path::echo:
        return 'E';
    case Path::neither:
        return 'N';
    default:
        return '-';
    }
}

} // namespace

Result<Pack> open_pack(const std::string& root, std::string& diagnostic)
{
    auto version = manifest::read_file(root + "/VERSION", diagnostic);
    if (!version)
    {
        return version.error();
    }
    if (version.value() != std::string{package_id} + "\n")
    {
        diagnostic = std::format("{}/VERSION: not {}", root, package_id);
        return Status{StatusCode::invalid_argument, "not the first-path-v1 package"};
    }
    auto cases = read_cases(root + "/cases.json", diagnostic);
    if (!cases)
    {
        return cases.error();
    }
    return Pack{.root = root, .cases = std::move(cases.value())};
}

Result<Recording> observe_case(const Pack& pack, std::string_view id, std::string& diagnostic)
{
    Recording recording;
    recording.manifest = pack.cases.find(id);
    if (recording.manifest == nullptr)
    {
        diagnostic = std::format("{}: the package has no case {}", pack.root, id);
        return Status{StatusCode::invalid_argument, "a case is missing from the package"};
    }
    const auto path = pack.root + "/" + recording.manifest->audio;
    const auto audio = read_recording(path);
    if (!audio)
    {
        diagnostic = std::format("{}: {}", path, audio.error().message);
        return audio.error();
    }
    for (const auto phase : recording.manifest->phase_offsets)
    {
        auto seen = observe(audio.value(), recording.manifest->observed_timeline(), phase);
        if (!seen)
        {
            diagnostic = std::format("{}, phase {}: {}", path, phase, seen.error().message);
            return seen.error();
        }
        recording.phases.push_back(std::move(seen.value()));
    }
    return recording;
}

std::vector<std::string> unmet(const Recording& recording)
{
    std::vector<std::string> failures;
    const auto& expect = recording.manifest->expect;
    const auto fail = [&](std::string what) { failures.push_back(std::format("{}: {}", recording.manifest->id, std::move(what))); };
    if (recording.phases.empty())
    {
        fail("no observation");
        return failures;
    }
    const auto& seen = recording.phases.front();

    for (const auto path : expect.groups_present)
    {
        if (std::ranges::none_of(seen.groups, [path](const GroupObservation& group) { return group.path == path; }))
        {
            fail(std::format("no group at {}", path_name(path)));
        }
    }
    for (const auto path : expect.groups_absent)
    {
        if (std::ranges::any_of(seen.groups, [path](const GroupObservation& group) { return group.path == path; }))
        {
            fail(std::format("a group at {}", path_name(path)));
        }
    }
    if (expect.strongest_group && (seen.groups.empty() || seen.groups[seen.strongest].path != *expect.strongest_group))
    {
        fail(std::format("the strongest group is not at {}", path_name(*expect.strongest_group)));
    }
    if (expect.strongest_margin_supporters_min && seen.strongest_margin < *expect.strongest_margin_supporters_min)
    {
        fail(std::format("the strongest group's margin {} is under {}", seen.strongest_margin, *expect.strongest_margin_supporters_min));
    }

    // The anchor in each setting the manifest names; moved_by, for every setting with the
    // anchoring on. The diagnostics' bounds are read on the first setting with it on.
    std::vector<const ProfileObservation*> on;
    const auto anchor = [&](const std::optional<Path>& expected, std::string_view name)
    {
        if (!expected)
        {
            return;
        }
        const auto& profile = profile_named(seen, name);
        if (profile.anchor != *expected)
        {
            fail(std::format("{} anchored on {}, not {}", name, path_name(profile.anchor), path_name(*expected)));
        }
        if (name != "off")
        {
            on.push_back(&profile);
        }
    };
    anchor(expect.anchor_off, "off");
    anchor(expect.anchor_window_7, "window_7");
    anchor(expect.anchor_window_3, "window_3");
    if (expect.moved_by)
    {
        for (const auto* profile : on)
        {
            if (profile->moved_by != *expect.moved_by)
            {
                fail(std::format("moved by {}, not {}", move_name(profile->moved_by), move_name(*expect.moved_by)));
            }
        }
    }
    if (!on.empty())
    {
        const auto& profile = *on.front();
        const auto& d = profile.diagnostics;
        if (expect.reanchored_lag && d.reanchored_lag != *expect.reanchored_lag)
        {
            fail(std::format("re-anchored by {}, not {}", d.reanchored_lag, *expect.reanchored_lag));
        }
        if (expect.precursor_lag && (d.precursor_candidates == 0U || d.precursor_lag != *expect.precursor_lag))
        {
            fail(std::format("the earliest precursor peak is not at lag {}", *expect.precursor_lag));
        }
        if (expect.precursor_fraction_min && widen<double>(d.precursor_fraction) < *expect.precursor_fraction_min)
        {
            fail(std::format("the precursor fraction {:.4f} is under {}", d.precursor_fraction, *expect.precursor_fraction_min));
        }
        if (expect.walk_segments_min && d.walk_segments < *expect.walk_segments_min)
        {
            fail(std::format("the walk tried {} segments, under {}", d.walk_segments, *expect.walk_segments_min));
        }
        const auto rejected = d.walk_rejected_frequency + d.walk_rejected_correlation + d.walk_rejected_invalid + d.walk_rejected_mode;
        if (expect.walk_rejected_total_min && rejected < *expect.walk_rejected_total_min)
        {
            fail(std::format("the walk rejected {} segments, under {}", rejected, *expect.walk_rejected_total_min));
        }
        if (expect.accepted_correlation_min && widen<double>(profile.anchor_correlation) < *expect.accepted_correlation_min)
        {
            fail(std::format("the accepted correlation {:.4f} is under {}", profile.anchor_correlation, *expect.accepted_correlation_min));
        }
        if (expect.rejected_correlation_max && widen<double>(profile.walk_lag_correlation_max) > *expect.rejected_correlation_max)
        {
            fail(std::format("a rejected start reached {:.4f}, over {}", profile.walk_lag_correlation_max, *expect.rejected_correlation_max));
        }
        if (expect.walk_accepted && d.reanchored != *expect.walk_accepted)
        {
            fail(d.reanchored ? "the walk accepted a segment" : "the walk accepted no segment");
        }
    }

    // The arrival phases: one setting over every phase.
    if (expect.profile)
    {
        std::string vector;
        std::vector<std::size_t> walked;
        for (const auto& phase : recording.phases)
        {
            const auto& profile = profile_named(phase, *expect.profile);
            vector += letter(profile.anchor);
            walked.push_back(profile.diagnostics.walk_segments);
        }
        const auto direct = narrow<std::size_t>(std::ranges::count(vector, 'D'));
        if (expect.direct_count && direct != *expect.direct_count)
        {
            fail(std::format("direct at {} phases, not {}", direct, *expect.direct_count));
        }
        if ((expect.direct_count_min && direct < *expect.direct_count_min) || (expect.direct_count_max && direct > *expect.direct_count_max))
        {
            fail(std::format("direct at {} phases, outside the range", direct));
        }
        if (expect.others && std::ranges::any_of(vector, [&](char c) { return c != 'D' && c != letter(*expect.others); }))
        {
            fail(std::format("a phase on neither the direct path nor {}", path_name(*expect.others)));
        }
        if (expect.phase_vector && vector != *expect.phase_vector)
        {
            fail(std::format("the phases read {}, not {}", vector, *expect.phase_vector));
        }
        if (!expect.walk_segments.empty() && walked != expect.walk_segments)
        {
            fail("the segments walked at each phase differ from the manifest's");
        }
    }
    return failures;
}

// --- Step 1 ---

Result<Recording> two_groups(const Pack& pack, std::string& diagnostic)
{
    return observe_case(pack, "fp1-echo-group", diagnostic);
}

void print_two_groups(const Recording& step)
{
    const auto& manifest = *step.manifest;
    const auto& seen = step.phases.front();
    std::println("{}, {} on {}", manifest.id, mode_text(manifest.mode), channel_text(manifest.id));
    print_groups(seen, manifest);
    std::println("without the anchoring: {}", anchor_text(seen.off, manifest));
    std::println("with it (the window from 7): {}, {}", anchor_text(seen.window_7, manifest), move_text(seen.window_7));
}

// --- Step 2 ---

Result<AnchorWindow> anchor_window(const Pack& pack, std::string& diagnostic)
{
    auto five = observe_case(pack, "fp1-echo-group", diagnostic);
    if (!five)
    {
        return five.error();
    }
    auto two = observe_case(pack, "fp2-window", diagnostic);
    if (!two)
    {
        return two.error();
    }
    return AnchorWindow{.five_ms = std::move(five.value()), .two_ms = std::move(two.value())};
}

void print(const AnchorWindow& step)
{
    std::println("the anchor, with the window from 7 symbols and from 3:");
    for (const auto* recording : {&step.five_ms, &step.two_ms})
    {
        const auto& manifest = *recording->manifest;
        const auto& seen = recording->phases.front();
        std::println("{}, {} on {}", manifest.id, mode_text(manifest.mode), channel_text(manifest.id));
        std::println("  without the anchoring:  {}", anchor_text(seen.off, manifest));
        std::println("  the window from 7:      {}, {}", anchor_text(seen.window_7, manifest), move_text(seen.window_7));
        std::println("  the window from 3:      {}, {}", anchor_text(seen.window_3, manifest), move_text(seen.window_3));
    }
}

// --- Step 3 ---

Result<Recording> faded_direct(const Pack& pack, std::string& diagnostic)
{
    return observe_case(pack, "fp3-faded-direct", diagnostic);
}

void print_faded_direct(const Recording& step)
{
    const auto& manifest = *step.manifest;
    const auto& seen = step.phases.front();
    std::println("{}, {} on {}", manifest.id, mode_text(manifest.mode), channel_text(manifest.id));
    print_groups(seen, manifest);
    std::println("without the anchoring: {}", anchor_text(seen.off, manifest));
    print_profile(seen.window_7.diagnostics);
    std::println("with the anchoring (the window from 7): {}, {}", anchor_text(seen.window_7, manifest), move_text(seen.window_7));
    std::println("the prefix correlation at the segment it read on: {:.3f}", seen.window_7.anchor_correlation);
}

// --- Step 4 ---

Result<SidelobeModel> sidelobe_model()
{
    // The "Hi" burst straight from the transmitter, through the front end: one path, no
    // noise, so everything the profile shows before lag 0 is the reference correlating with
    // itself and with the end of the segment before it.
    WaveformSpec spec;
    spec.payload = {'H', 'i'};
    auto made = make_test_waveform(spec);
    if (!made)
    {
        return made.error();
    }
    auto& audio = made.value().audio;
    audio.insert(audio.begin(), clean_lead_samples, 0.0F);
    FrontEndRunner runner;
    auto status = runner.initialize(FrontEndOptions{.phase_search = true});
    if (status.is_ok())
    {
        status = runner.feed(audio);
    }
    if (!status.is_ok())
    {
        return status;
    }
    const auto symbols = on_time_symbols(runner);
    BodyAcquisitionConfig config;
    config.minimum_correlation = 0.35F;
    config.anchor_earliest_path = false;
    const auto acquired = acquire_body_preamble(symbols, config);
    if (!acquired)
    {
        return acquired.error();
    }
    const auto& found = acquired.value();
    // Every segment of the burst, from its first.
    const auto profile = precursor_profile(symbols, found.first_preamble_symbol, found.first_body_symbol, found.frequency_offset_hz);
    const auto with_tail = body_precursor_sidelobe_powers(prefix_reference_points(), maximum_lag);
    const auto prefix_only = prefix_only_sidelobes();
    SidelobeModel step{.segments = profile.segments};
    for (std::size_t lag = 1U; lag <= maximum_lag; ++lag)
    {
        step.rows.push_back({lag, profile.powers[lag] / profile.powers[0], corrected_fraction(profile, prefix_only, lag), corrected_fraction(profile, with_tail, lag)});
    }
    return step;
}

void print(const SidelobeModel& step)
{
    std::println("one clean path, {} segments; precursor power as a fraction of lag 0 (* at or above 1/8):", step.segments);
    std::println("  lag  measured  less the prefix's sidelobes  less the model with the tail");
    const auto mark = [](float value) { return value >= significant_fraction ? '*' : ' '; };
    for (const auto& row : step.rows)
    {
        std::println("  {:3}  {:7.3f}{}  {:26.3f}{}  {:28.3f}{}", row.lag, row.measured, mark(row.measured), row.prefix_only, mark(row.prefix_only), row.with_tail, mark(row.with_tail));
    }
}

// --- Step 5 ---

Result<Recording> sidelobe_path(const Pack& pack, std::string& diagnostic)
{
    return observe_case(pack, "fp5-sidelobe-path", diagnostic);
}

void print_sidelobe_path(const Recording& step)
{
    const auto& manifest = *step.manifest;
    const auto& seen = step.phases.front();
    std::println("{}, {} on {}", manifest.id, mode_text(manifest.mode), channel_text(manifest.id));
    print_groups(seen, manifest);
    std::println("without the anchoring: {}", anchor_text(seen.off, manifest));
    print_profile(seen.window_7.diagnostics);
    std::println("the largest prefix correlation where the walk tested that lag: {:.3f}", seen.window_7.walk_lag_correlation_max);
    std::println("with the anchoring (the window from 7): {}, {}", anchor_text(seen.window_7, manifest), move_text(seen.window_7));
}

// --- Step 6 ---

Result<ArrivalPhase> arrival_phase(const Pack& pack, std::string& diagnostic)
{
    auto shorter = observe_case(pack, "fp4-phase-short", diagnostic);
    if (!shorter)
    {
        return shorter.error();
    }
    auto longer = observe_case(pack, "fp6-phase-long", diagnostic);
    if (!longer)
    {
        return longer.error();
    }
    return ArrivalPhase{.short_preamble = std::move(shorter.value()), .long_preamble = std::move(longer.value())};
}

void print(const ArrivalPhase& step)
{
    std::println("{}: a weak direct path; the window from 3; the burst arriving at each phase of a symbol", channel_text(step.short_preamble.manifest->id));
    std::println("  arrival  600S, 3 segments                              600L, 24 segments");
    std::println("    phase  anchored on  how                   walked     anchored on  how                   walked");
    std::size_t short_direct{};
    std::size_t long_direct{};
    for (std::size_t phase = 0U; phase < step.short_preamble.phases.size() && phase < step.long_preamble.phases.size(); ++phase)
    {
        const auto& s = step.short_preamble.phases[phase].window_3;
        const auto& l = step.long_preamble.phases[phase].window_3;
        short_direct += s.anchor == Path::direct ? 1U : 0U;
        long_direct += l.anchor == Path::direct ? 1U : 0U;
        std::println("  {:7}  {:11}  {:20}  {:6}     {:11}  {:20}  {:6}", step.short_preamble.phases[phase].phase, path_name(s.anchor), how(s), s.diagnostics.walk_segments, path_name(l.anchor),
                     how(l), l.diagnostics.walk_segments);
    }
    std::println("on the direct path: 600S at {} of {} phases, 600L at {}", short_direct, step.short_preamble.phases.size(), long_direct);
}

} // namespace modem_test::first_path_steps
