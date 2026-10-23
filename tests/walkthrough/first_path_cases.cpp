// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#include "tests/walkthrough/first_path_cases.hpp"

#include "modem/common/convert.hpp"
#include "tests/support/manifest_json.hpp"
#include "tests/support/receive_chain.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <initializer_list>
#include <ranges>
#include <utility>

namespace modem_test::first_path
{

namespace
{

using manifest::Kind;
using manifest::Value;
using ::modem::common::as_real;
using ::modem::common::Result;
using ::modem::common::round_to;
using ::modem::common::Status;
using ::modem::common::StatusCode;
using ::modem::common::widen;

constexpr std::size_t arrival_phases = 20U;
constexpr std::size_t lead_and_tail_samples = 24000U;
constexpr double samples_per_symbol = 20.0;
constexpr double equation_tolerance = 1.0e-6; // the timeline's equations hold to rounding
constexpr double delay_tolerance = 1.0;       // a delay within a sample of its row's
// The widest values a field may take: generous, so only nonsense is refused.
constexpr double largest_sample = 1.0e9;
constexpr double largest_chain_delay = 10000.0;
constexpr double largest_echo_delay = 1000.0;
constexpr double largest_uncertainty = 20.0;
constexpr double largest_fraction = 1.0e6;
constexpr std::size_t largest_preamble = 100000U;
constexpr std::size_t largest_count = 1000U;
constexpr std::size_t largest_lag = 15U;
constexpr std::size_t largest_step = 20U;

// What each case is: its mode, its second path's delay, its phases, and the expectation
// fields its spelling names (docs/development/first-path-test-data-v1.md, section 9).
struct CaseRule
{
    std::string_view id;
    std::string_view mode;
    double echo_delay_samples;
    bool phases;
    bool required;
    std::initializer_list<std::string_view> fields;
};

constexpr std::array<CaseRule, 6> case_rules{{
    {"fp1-echo-group", "300L", 240.0, false, true, {"groups_present", "strongest_group", "strongest_margin_supporters_min", "anchor", "moved_by"}},
    {"fp2-window", "600L", 96.0, false, true, {"strongest_group", "strongest_margin_supporters_min", "anchor"}},
    {"fp3-faded-direct",
     "300L",
     240.0,
     false,
     true,
     {"groups_absent", "anchor", "moved_by", "reanchored_lag", "precursor_fraction_min", "walk_segments_min", "walk_rejected_total_min", "accepted_correlation_min"}},
    {"fp4-phase-short", "600S", 96.0, true, true, {"profile", "direct_count_min", "direct_count_max", "others", "phase_vector", "walk_segments"}},
    {"fp5-sidelobe-path", "300L", 240.0, false, false, {"anchor", "precursor_lag", "precursor_fraction_min", "walk_accepted", "rejected_correlation_max"}},
    {"fp6-phase-long", "600L", 96.0, true, true, {"profile", "direct_count", "phase_vector", "walk_segments"}},
}};

// Reads typed fields out of the parsed manifest. The first failure is kept, with the
// path of the field it was found at; every later call then fails too, so the caller
// checks once at the end of a block.
class Reader
{
public:
    Reader() = default;

    [[nodiscard]] const std::string& diagnostic() const noexcept { return mDiagnostic; }

    [[nodiscard]] bool failed() const noexcept { return mFailed; }

    void fail(std::string_view where, std::string_view what)
    {
        if (!mFailed)
        {
            mDiagnostic = std::format("{}: {}", where, what);
            mFailed = true;
        }
    }

    const Value* member(const Value& object, std::string_view where, std::string_view key)
    {
        const auto* found = object.find(key);
        if (found == nullptr)
        {
            fail(where, std::format("\"{}\" is missing", key));
        }
        return found;
    }

    void only(const Value& object, std::string_view where, std::initializer_list<std::string_view> keys)
    {
        for (const auto& [name, value] : object.members)
        {
            if (std::ranges::find(keys, std::string_view{name}) == keys.end())
            {
                fail(where, std::format("\"{}\" is not a field of this object", name));
            }
        }
    }

    const Value* typed(const Value& object, std::string_view where, std::string_view key, Kind kind)
    {
        const auto* found = member(object, where, key);
        if (found != nullptr && found->kind != kind)
        {
            fail(std::format("{}.{}", where, key), "the wrong type");
            return nullptr;
        }
        return found;
    }

    std::string text(const Value& object, std::string_view where, std::string_view key)
    {
        const auto* found = typed(object, where, key, Kind::string);
        return found != nullptr ? found->text : std::string{};
    }

    double number(const Value& object, std::string_view where, std::string_view key, double minimum, double maximum)
    {
        const auto* found = typed(object, where, key, Kind::number);
        if (found == nullptr)
        {
            return 0.0;
        }
        if (found->number < minimum || found->number > maximum)
        {
            fail(std::format("{}.{}", where, key), std::format("{} is out of range [{}, {}]", found->number, minimum, maximum));
        }
        return found->number;
    }

    std::size_t integer(const Value& value, std::string_view where, std::size_t minimum, std::size_t maximum)
    {
        constexpr double largest = 1.0e9; // well inside size_t and exact in double
        if (value.kind != Kind::number || value.number != std::floor(value.number) || value.number < 0.0 || value.number > largest)
        {
            fail(where, std::format("expected a whole number in [{}, {}]", minimum, maximum));
            return 0U;
        }
        const auto whole = round_to<std::size_t>(value.number);
        if (whole < minimum || whole > maximum)
        {
            fail(where, std::format("expected a whole number in [{}, {}]", minimum, maximum));
            return 0U;
        }
        return whole;
    }

    std::size_t integer(const Value& object, std::string_view where, std::string_view key, std::size_t minimum, std::size_t maximum)
    {
        const auto* found = member(object, where, key);
        return found != nullptr ? integer(*found, std::format("{}.{}", where, key), minimum, maximum) : 0U;
    }

    Path path(const Value& value, std::string_view where)
    {
        if (value.kind == Kind::string)
        {
            for (const auto candidate : {Path::direct, Path::echo, Path::neither})
            {
                if (value.text == path_name(candidate))
                {
                    return candidate;
                }
            }
        }
        fail(where, R"(expected "direct", "echo" or "neither")");
        return Path::none;
    }

    // A file inside the package: relative, '/'-separated, no empty, '.' or '..' part.
    std::string package_path(const Value& object, std::string_view where, std::string_view key)
    {
        auto name = text(object, where, key);
        bool good = !name.empty() && name.front() != '/' && name.find_first_of("\\:") == std::string::npos;
        std::size_t start = 0U;
        while (good && start <= name.size())
        {
            const auto end = std::min(name.find('/', start), name.size());
            const auto part = std::string_view{name}.substr(start, end - start);
            good = !part.empty() && part != "." && part != "..";
            start = end + 1U;
        }
        if (!good)
        {
            fail(std::format("{}.{}", where, key), "not a relative path inside the package");
        }
        return name;
    }

private:
    std::string mDiagnostic;
    bool mFailed{};
};

void read_timeline(Reader& reader, const Value& object, std::string_view where, const CaseRule& rule, ::modem::m110a::BodyMode mode, CaseTimeline& out)
{
    reader.only(object, where,
                {"lead_samples", "tail_samples", "chain_delay_samples", "direct_start_sample", "tx_symbol0_center_offset_samples", "direct_symbol0_sample", "echo_delay_samples", "echo_start_sample",
                 "echo_symbol0_sample", "preamble_symbols", "direct_body_symbol0_sample", "echo_body_symbol0_sample", "uncertainty_samples", "calibration_id", "calibration_method", "operations"});
    // How the timeline was established (Stage 1 section 4): for the reader of the manifest, not
    // the walkthrough, but of the right type.
    reader.typed(object, where, "calibration_id", Kind::string);
    reader.typed(object, where, "calibration_method", Kind::string);
    reader.typed(object, where, "operations", Kind::array);
    out.lead_samples = reader.integer(object, where, "lead_samples", lead_and_tail_samples, lead_and_tail_samples);
    out.tail_samples = reader.integer(object, where, "tail_samples", lead_and_tail_samples, lead_and_tail_samples);
    out.chain_delay_samples = reader.number(object, where, "chain_delay_samples", 0.0, largest_chain_delay);
    out.direct_start_sample = reader.number(object, where, "direct_start_sample", 0.0, largest_sample);
    out.tx_symbol0_center_offset_samples = reader.number(object, where, "tx_symbol0_center_offset_samples", -largest_chain_delay, largest_chain_delay);
    out.direct_symbol0_sample = reader.number(object, where, "direct_symbol0_sample", 0.0, largest_sample);
    out.echo_delay_samples = reader.number(object, where, "echo_delay_samples", 0.0, largest_echo_delay);
    out.echo_start_sample = reader.number(object, where, "echo_start_sample", 0.0, largest_sample);
    out.echo_symbol0_sample = reader.number(object, where, "echo_symbol0_sample", 0.0, largest_sample);
    out.preamble_symbols = reader.integer(object, where, "preamble_symbols", 1U, largest_preamble);
    out.direct_body_symbol0_sample = reader.number(object, where, "direct_body_symbol0_sample", 0.0, largest_sample);
    out.echo_body_symbol0_sample = reader.number(object, where, "echo_body_symbol0_sample", 0.0, largest_sample);
    out.uncertainty_samples = reader.number(object, where, "uncertainty_samples", 0.0, largest_uncertainty);
    if (reader.failed())
    {
        return;
    }

    const auto check = [&](std::string_view field, double value, double expected)
    {
        if (std::fabs(value - expected) > equation_tolerance * std::max(1.0, std::fabs(expected)))
        {
            reader.fail(std::format("{}.{}", where, field), std::format("{} disagrees with the timeline's equation ({})", value, expected));
        }
    };
    const auto lead = widen<double>(as_real(out.lead_samples));
    check("direct_start_sample", out.direct_start_sample, lead + out.chain_delay_samples);
    check("direct_symbol0_sample", out.direct_symbol0_sample, out.direct_start_sample + out.tx_symbol0_center_offset_samples);
    check("echo_start_sample", out.echo_start_sample, out.direct_start_sample + out.echo_delay_samples);
    check("echo_symbol0_sample", out.echo_symbol0_sample, out.direct_symbol0_sample + out.echo_delay_samples);
    check("direct_body_symbol0_sample", out.direct_body_symbol0_sample, out.direct_symbol0_sample + samples_per_symbol * widen<double>(as_real(out.preamble_symbols)));
    check("echo_body_symbol0_sample", out.echo_body_symbol0_sample, out.direct_body_symbol0_sample + out.echo_delay_samples);
    if (out.preamble_symbols != ::modem::m110a::body_preamble_symbols(mode.interleave))
    {
        reader.fail(std::format("{}.preamble_symbols", where), "not this mode's preamble");
    }
    if (std::fabs(out.echo_delay_samples - rule.echo_delay_samples) > delay_tolerance)
    {
        reader.fail(std::format("{}.echo_delay_samples", where), std::format("not this case's channel ({} samples)", rule.echo_delay_samples));
    }
}

void read_expectations(Reader& reader, const Value& object, std::string_view where, const CaseRule& rule, CaseExpectations& out)
{
    reader.only(object, where, rule.fields);
    for (const auto field : rule.fields)
    {
        reader.member(object, where, field);
    }
    if (reader.failed())
    {
        return;
    }
    const auto paths = [&](std::string_view key, std::vector<Path>& into)
    {
        if (const auto* list = reader.typed(object, where, key, Kind::array))
        {
            for (std::size_t i = 0U; i < list->items.size(); ++i)
            {
                into.push_back(reader.path(list->items[i], std::format("{}.{}[{}]", where, key, i)));
            }
        }
    };
    const auto one_path = [&](std::string_view key) -> std::optional<Path>
    {
        if (const auto* found = object.find(key))
        {
            return reader.path(*found, std::format("{}.{}", where, key));
        }
        return std::nullopt;
    };
    const auto whole = [&](std::string_view key, std::size_t maximum) -> std::optional<std::size_t>
    {
        if (object.find(key) != nullptr)
        {
            return reader.integer(object, where, key, 0U, maximum);
        }
        return std::nullopt;
    };
    const auto fraction = [&](std::string_view key) -> std::optional<double>
    {
        if (object.find(key) != nullptr)
        {
            return reader.number(object, where, key, 0.0, largest_fraction);
        }
        return std::nullopt;
    };

    if (object.find("groups_present") != nullptr)
    {
        paths("groups_present", out.groups_present);
    }
    if (object.find("groups_absent") != nullptr)
    {
        paths("groups_absent", out.groups_absent);
    }
    out.strongest_group = one_path("strongest_group");
    out.strongest_margin_supporters_min = whole("strongest_margin_supporters_min", largest_count);
    if (const auto* anchor = object.find("anchor"))
    {
        const auto at = std::format("{}.anchor", where);
        if (anchor->kind != Kind::object || anchor->members.empty())
        {
            reader.fail(at, "expected an object naming off, window_7 or window_3");
        }
        else
        {
            reader.only(*anchor, at, {"off", "window_7", "window_3"});
            if (const auto* v = anchor->find("off"))
            {
                out.anchor_off = reader.path(*v, std::format("{}.off", at));
            }
            if (const auto* v = anchor->find("window_7"))
            {
                out.anchor_window_7 = reader.path(*v, std::format("{}.window_7", at));
            }
            if (const auto* v = anchor->find("window_3"))
            {
                out.anchor_window_3 = reader.path(*v, std::format("{}.window_3", at));
            }
        }
    }
    if (object.find("moved_by") != nullptr)
    {
        const auto moved = reader.text(object, where, "moved_by");
        for (const auto candidate : {Move::none, Move::earlier_group, Move::precursor_search})
        {
            if (moved == move_name(candidate))
            {
                out.moved_by = candidate;
            }
        }
        if (!out.moved_by)
        {
            reader.fail(std::format("{}.moved_by", where), R"(expected "none", "earlier_group" or "precursor_search")");
        }
    }
    out.reanchored_lag = whole("reanchored_lag", largest_lag);
    out.precursor_lag = whole("precursor_lag", largest_lag);
    out.precursor_fraction_min = fraction("precursor_fraction_min");
    out.walk_segments_min = whole("walk_segments_min", largest_count);
    out.walk_rejected_total_min = whole("walk_rejected_total_min", largest_count);
    if (object.find("accepted_correlation_min") != nullptr)
    {
        out.accepted_correlation_min = reader.number(object, where, "accepted_correlation_min", 0.0, 1.0);
    }
    if (object.find("rejected_correlation_max") != nullptr)
    {
        out.rejected_correlation_max = reader.number(object, where, "rejected_correlation_max", 0.0, 1.0);
    }
    if (object.find("walk_accepted") != nullptr)
    {
        if (const auto* accepted = reader.typed(object, where, "walk_accepted", Kind::boolean))
        {
            out.walk_accepted = accepted->boolean;
        }
    }
    if (object.find("profile") != nullptr)
    {
        out.profile = reader.text(object, where, "profile");
        if (*out.profile != "off" && *out.profile != "window_7" && *out.profile != "window_3")
        {
            reader.fail(std::format("{}.profile", where), R"(expected "off", "window_7" or "window_3")");
        }
    }
    out.direct_count_min = whole("direct_count_min", arrival_phases);
    out.direct_count_max = whole("direct_count_max", arrival_phases);
    out.direct_count = whole("direct_count", arrival_phases);
    out.others = one_path("others");
    if (object.find("phase_vector") != nullptr)
    {
        out.phase_vector = reader.text(object, where, "phase_vector");
        if (out.phase_vector->size() != arrival_phases || out.phase_vector->find_first_not_of("DEN-") != std::string::npos)
        {
            reader.fail(std::format("{}.phase_vector", where), "expected 20 letters, each D, E, N or -");
        }
    }
    if (object.find("walk_segments") != nullptr)
    {
        if (const auto* list = reader.typed(object, where, "walk_segments", Kind::array))
        {
            if (list->items.size() != arrival_phases)
            {
                reader.fail(std::format("{}.walk_segments", where), "expected 20 counts");
            }
            for (std::size_t i = 0U; i < list->items.size(); ++i)
            {
                out.walk_segments.push_back(reader.integer(list->items[i], std::format("{}.walk_segments[{}]", where, i), 0U, largest_count));
            }
        }
    }
}

Result<CaseSet> read_set(const Value& root, std::string& diagnostic)
{
    Reader reader;
    const auto fail = [&reader, &diagnostic]
    {
        diagnostic = reader.diagnostic();
        return Status{StatusCode::invalid_argument, "the case manifest is not usable"};
    };
    if (root.kind != Kind::object)
    {
        reader.fail("the manifest", "expected an object at the top");
        return fail();
    }
    reader.only(root, "the manifest", {"schema", "package_id", "cases"});
    if (reader.text(root, "the manifest", "schema") != cases_schema)
    {
        reader.fail("schema", std::format("not {} (unsupported version)", cases_schema));
    }
    if (reader.text(root, "the manifest", "package_id") != package_id)
    {
        reader.fail("package_id", std::format("not {}", package_id));
    }
    const auto* list = reader.typed(root, "the manifest", "cases", Kind::array);
    if (reader.failed() || list == nullptr)
    {
        return fail();
    }

    CaseSet set;
    for (std::size_t index = 0U; index < list->items.size() && !reader.failed(); ++index)
    {
        const auto& item = list->items[index];
        auto where = std::format("cases[{}]", index);
        if (item.kind != Kind::object)
        {
            reader.fail(where, "expected an object");
            break;
        }
        Case one;
        one.id = reader.text(item, where, "id");
        const auto rule = std::ranges::find(case_rules, std::string_view{one.id}, &CaseRule::id);
        if (rule == case_rules.end())
        {
            reader.fail(std::format("{}.id", where), std::format("\"{}\" is not a case of this package", one.id));
            break;
        }
        if (set.find(one.id) != nullptr)
        {
            reader.fail(std::format("{}.id", where), std::format("\"{}\" appears twice", one.id));
            break;
        }
        where = std::format("cases[{}] ({})", index, one.id);
        reader.only(item, where, {"id", "purpose", "article_steps", "audio", "clean_reference", "provenance", "mode", "payload_hex", "channel", "timeline", "phase_offsets_samples", "expectations"});
        one.purpose = reader.text(item, where, "purpose");
        if (const auto* steps = reader.typed(item, where, "article_steps", Kind::array))
        {
            for (std::size_t i = 0U; i < steps->items.size(); ++i)
            {
                one.article_steps.push_back(reader.integer(steps->items[i], std::format("{}.article_steps[{}]", where, i), 1U, largest_step));
            }
        }
        one.audio = reader.package_path(item, where, "audio");
        one.clean_reference = reader.package_path(item, where, "clean_reference");
        one.provenance = reader.package_path(item, where, "provenance");
        const auto mode_text_value = reader.text(item, where, "mode");
        const auto mode = parse_mode(mode_text_value);
        if (!reader.failed() && (!mode || mode_text_value != rule->mode))
        {
            reader.fail(std::format("{}.mode", where), std::format("expected {}", rule->mode));
        }
        one.mode = mode.value_or(::modem::m110a::BodyMode{});
        one.payload_hex = reader.text(item, where, "payload_hex");
        if (!reader.failed() && one.payload_hex != "4869")
        {
            reader.fail(std::format("{}.payload_hex", where), "expected 4869 (\"Hi\")");
        }
        reader.typed(item, where, "channel", Kind::object);
        if (const auto* timeline = reader.typed(item, where, "timeline", Kind::object); timeline != nullptr && !reader.failed())
        {
            read_timeline(reader, *timeline, std::format("{}.timeline", where), *rule, one.mode, one.timeline);
        }
        if (const auto* phases = reader.typed(item, where, "phase_offsets_samples", Kind::array); phases != nullptr && !reader.failed())
        {
            for (std::size_t i = 0U; i < phases->items.size(); ++i)
            {
                one.phase_offsets.push_back(reader.integer(phases->items[i], std::format("{}.phase_offsets_samples[{}]", where, i), 0U, arrival_phases - 1U));
            }
            const bool all = one.phase_offsets.size() == arrival_phases && std::ranges::equal(one.phase_offsets, std::views::iota(std::size_t{0U}, arrival_phases));
            const bool none = one.phase_offsets.size() == 1U && one.phase_offsets.front() == 0U;
            if (!reader.failed() && (rule->phases ? !all : !none))
            {
                reader.fail(std::format("{}.phase_offsets_samples", where), rule->phases ? "expected 0 to 19" : "expected [0]");
            }
        }
        if (const auto* expect = reader.typed(item, where, "expectations", Kind::object); expect != nullptr && !reader.failed())
        {
            read_expectations(reader, *expect, std::format("{}.expectations", where), *rule, one.expect);
        }
        set.cases.push_back(std::move(one));
    }
    for (const auto& rule : case_rules)
    {
        if (!reader.failed() && rule.required && set.find(rule.id) == nullptr)
        {
            reader.fail("cases", std::format("\"{}\" is missing", rule.id));
        }
    }
    if (reader.failed())
    {
        return fail();
    }
    return set;
}

} // namespace

Timeline Case::observed_timeline() const noexcept
{
    return Timeline{.direct_symbol0_sample = timeline.direct_symbol0_sample, .echo_delay_samples = timeline.echo_delay_samples, .preamble_symbols = timeline.preamble_symbols};
}

const Case* CaseSet::find(std::string_view id) const noexcept
{
    const auto found = std::ranges::find(cases, id, &Case::id);
    return found != cases.end() ? &*found : nullptr;
}

Result<CaseSet> parse_cases(std::string_view text, std::string& diagnostic)
{
    auto root = manifest::parse(text, diagnostic);
    if (!root)
    {
        return root.error();
    }
    return read_set(root.value(), diagnostic);
}

Result<CaseSet> read_cases(const std::string& path, std::string& diagnostic)
{
    auto bytes = manifest::read_file(path, diagnostic);
    if (!bytes)
    {
        return bytes.error();
    }
    auto set = parse_cases(bytes.value(), diagnostic);
    if (!set)
    {
        diagnostic = std::format("{}: {}", path, diagnostic);
    }
    return set;
}

} // namespace modem_test::first_path
