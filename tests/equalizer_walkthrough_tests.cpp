// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// equalizer_walkthrough_tests - the Part 6 walkthrough's steps, each checked
// =============================================================================
// The clean burst's cases need no recording. The recordings' cases ([recordings])
// receive, train and equalize each recording exactly as equalizer_demo does; they need
// Part 5's recordings, so they are registered only when the build is configured with
// MODEM_ARTICLE_DATA=ON, and a missing or unusable package fails them as a setup error.
// Explained in the article series, Part 6 (the equalizer).
// =============================================================================

#include "modem/common/convert.hpp"
#include "tests/walkthrough/equalizer_steps.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <string>
#include <utility>

using namespace modem_test::equalizer_steps;

namespace
{

// The package, or a failed test that says why it is not usable.
Pack pack()
{
    std::string diagnostic;
    auto opened = open_pack(MODEM_FIRST_PATH_DATA, diagnostic);
    INFO("the recordings are not usable: " << diagnostic);
    REQUIRE(opened.has_value());
    return std::move(opened.value());
}

Burst clean_burst()
{
    auto burst = receive_clean();
    REQUIRE(burst.has_value());
    return std::move(burst.value());
}

float error_rate(const Body& body)
{
    REQUIRE(body.data_symbols > 0U);
    return ::modem::common::as_real(body.data_errors) / ::modem::common::as_real(body.data_symbols);
}

} // namespace

TEST_CASE("On a clean burst the delay spread is 0, the default window stands, and training stops at the first guess", "[equalizer]")
{
    const auto burst = clean_burst();
    const auto trained = train(burst, BodyDataEqualizer::adaptive);
    REQUIRE(trained.has_value());
    CHECK(trained.value().delay_spread == 0U);
    CHECK(trained.value().geometry.feedforward_taps == ::modem::m110a::body_receiver_feedforward_taps);
    CHECK_FALSE(trained.value().retrained);
    CHECK_FALSE(trained.value().race.escalated);
}

TEST_CASE("On a clean burst both engines decide every data symbol right", "[equalizer]")
{
    const auto burst = clean_burst();
    for (const auto engine : {BodyDataEqualizer::adaptive, BodyDataEqualizer::cir_mmse})
    {
        auto trained = train(burst, engine);
        REQUIRE(trained.has_value());
        const auto body = walk_body(burst, trained.value());
        REQUIRE(body.has_value());
        CHECK(body.value().data_symbols > 0U);
        CHECK(body.value().data_errors == 0U);
    }
}

TEST_CASE("The 5 ms recording reads 12 symbols behind the anchor and is trained again on the long window", "[recordings]")
{
    std::string diagnostic;
    const auto step = delay_spread(pack(), diagnostic);
    INFO(diagnostic);
    REQUIRE(step.has_value());
    REQUIRE(step.value().rows.size() == 3U);
    const auto& five_ms = step.value().rows[0];
    CHECK(five_ms.id == five_ms_case);
    CHECK(five_ms.spread >= 11U);
    CHECK(five_ms.spread <= 13U);

    const auto window_step = window(pack(), diagnostic);
    REQUIRE(window_step.has_value());
    CHECK(window_step.value().rows[0].geometry.feedforward_taps == 32U);
    CHECK(window_step.value().rows[0].geometry.feedback_taps == 14U);
    CHECK(window_step.value().rows[0].retrained);
}

TEST_CASE("The 2 ms recording, anchored on its echo, reads no path behind the anchor and keeps the default window", "[recordings]")
{
    std::string diagnostic;
    const auto step = window(pack(), diagnostic);
    INFO(diagnostic);
    REQUIRE(step.has_value());
    const auto& two_ms = step.value().rows[1];
    CHECK(two_ms.id == two_ms_case);
    CHECK(two_ms.spread == 0U);
    CHECK(two_ms.geometry.feedforward_taps == ::modem::m110a::body_receiver_feedforward_taps);
    CHECK_FALSE(two_ms.retrained);
}

TEST_CASE("On the 5 ms recording training tries more than acquisition's frequency and keeps a better guess", "[recordings]")
{
    std::string diagnostic;
    const auto step = training(pack(), diagnostic);
    INFO(diagnostic);
    REQUIRE(step.has_value());
    const auto& race = step.value().race;
    CHECK(race.escalated);
    CHECK(race.count >= 2U);
    CHECK(race.winner != 0U);
    CHECK(race.residual[race.winner] <= race.residual[0]);
}

TEST_CASE("Through the 5 ms body the adaptive equalizer decides fewer data symbols wrong than CIR-MMSE", "[recordings]")
{
    std::string diagnostic;
    const auto adaptive_walk = adaptive(pack(), diagnostic);
    INFO(diagnostic);
    REQUIRE(adaptive_walk.has_value());
    const auto cir_walk = cir_mmse(pack(), diagnostic);
    REQUIRE(cir_walk.has_value());
    const auto adaptive_rate = error_rate(adaptive_walk.value().body);
    const auto cir_rate = error_rate(cir_walk.value().body);
    INFO("adaptive " << adaptive_rate << ", CIR-MMSE " << cir_rate);
    CHECK(adaptive_rate < cir_rate);
    CHECK(adaptive_rate > 0.05);
    CHECK(adaptive_rate < 0.30);
}

TEST_CASE("On the 5 ms recording each engine has frames where it trails the other", "[recordings]")
{
    std::string diagnostic;
    const auto step = fade(pack(), diagnostic);
    INFO(diagnostic);
    REQUIRE(step.has_value());
    const auto& widest = step.value().widest;
    REQUIRE(widest.size() == 6U);
    for (std::size_t row = 0U; row < widest.size(); ++row)
    {
        const auto adaptive_errors = step.value().adaptive.frames[widest[row]].data_errors;
        const auto cir_errors = step.value().cir_mmse.frames[widest[row]].data_errors;
        INFO("row " << row);
        if (row < 3U)
        {
            CHECK(cir_errors > adaptive_errors);
        }
        else
        {
            CHECK(adaptive_errors > cir_errors);
        }
    }
}
