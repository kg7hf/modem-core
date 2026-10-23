// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "tests/walkthrough/first_path_cases.hpp"
#include "tests/walkthrough/first_path_observe.hpp"

#include <cstddef>
#include <string>
#include <vector>

// =============================================================================
// first_path_steps.hpp - the Part 5 walkthrough, one function per step
// =============================================================================
// The steps run on the article's recordings: the "Hi" message through Table XX's
// two-path channels, rendered with pathsim and selected because each shows one
// situation the article explains. Each step observes one or two recordings through
// observe() (Part 3's front end, the scan's groups, and acquire_body_preamble() with
// the anchoring off and on) and returns what it found; print() writes it the way the
// article quotes it. first_path_demo runs the steps in article order, and
// first_path_tests checks every recording against the expectations its manifest
// carries (unmet() below), so the walkthrough is under regression too.
// Explained in the article series, Part 5 (the first path).
// =============================================================================

namespace modem_test::first_path_steps
{

using ::modem::common::Result;
using namespace ::modem_test::first_path;

// The recordings: the package's folder and its manifest, read and checked.
struct Pack
{
    std::string root{};
    CaseSet cases{};
};

[[nodiscard]] Result<Pack> open_pack(const std::string& root, std::string& diagnostic);

// One recording observed at its phases (one, or all 20).
struct Recording
{
    const Case* manifest{};
    std::vector<Observation> phases{};
};

[[nodiscard]] Result<Recording> observe_case(const Pack& pack, std::string_view id, std::string& diagnostic);

// Each expectation in the case's manifest that the observation does not meet; none, and
// the recording does what the article says it does.
[[nodiscard]] std::vector<std::string> unmet(const Recording& recording);

// --- Step 1: two paths, two groups (fp1) ---
[[nodiscard]] Result<Recording> two_groups(const Pack& pack, std::string& diagnostic);
void print_two_groups(const Recording& step);

// --- Step 2: the window (fp1 and fp2) ---
struct AnchorWindow
{
    Recording five_ms{};
    Recording two_ms{};
};
[[nodiscard]] Result<AnchorWindow> anchor_window(const Pack& pack, std::string& diagnostic);
void print(const AnchorWindow& step);

// --- Step 3: a path with no group of its own (fp3) ---
[[nodiscard]] Result<Recording> faded_direct(const Pack& pack, std::string& diagnostic);
void print_faded_direct(const Recording& step);

// --- Step 4: the reference's own sidelobes (no recording: one clean path) ---
struct SidelobeRow
{
    std::size_t lag{};
    float measured{};    // a single path's precursor power at this lag, as a fraction of lag 0
    float prefix_only{}; // left after subtracting the sidelobes of the prefix alone
    float with_tail{};   // left after subtracting the model with the previous segment's tail
};

struct SidelobeModel
{
    std::size_t segments{};
    std::vector<SidelobeRow> rows{}; // lags 1 to 15
};

[[nodiscard]] Result<SidelobeModel> sidelobe_model();
void print(const SidelobeModel& step);

// --- Step 5: a later path's sidelobe, and the walk (fp5) ---
[[nodiscard]] Result<Recording> sidelobe_path(const Pack& pack, std::string& diagnostic);
void print_sidelobe_path(const Recording& step);

// --- Step 6: where in a symbol the burst arrives (fp4 and fp6) ---
struct ArrivalPhase
{
    Recording short_preamble{};
    Recording long_preamble{};
};
[[nodiscard]] Result<ArrivalPhase> arrival_phase(const Pack& pack, std::string& diagnostic);
void print(const ArrivalPhase& step);

} // namespace modem_test::first_path_steps
