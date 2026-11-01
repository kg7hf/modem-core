// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

// =============================================================================
// delay_spread_tests.cpp - the preamble's delay-spread estimate and the equalizer
// geometry it selects. Explained in the article series, Part 6 (the equalizer).
// -----------------------------------------------------------------------------
// A single-path channel reports no echo; a 2 ms class echo (5 symbols) stays in the
// default geometry; a 5 ms class echo (12 symbols) selects the long-delay geometry.
// The known prefix's own autocorrelation sidelobes near lags 8 and 9 must not
// masquerade as an echo. A strong echo at 5 symbols with a weaker one at 12 (still above
// the significance fraction of the strongest lag) reports the farthest significant path,
// 12, not the strongest one, 5, so the long-delay geometry is selected; the lag
// power profile the estimator thresholded is checked to prove the case tells the two
// rules apart. Each echo casts the same period-8 sidelobes the main path does, 8 and 9
// symbols behind it; the estimator cancels them, strongest echo first, so they are not
// taken for a path. The cancellation has a limit: a real path 9 symbols behind a
// strong echo has to clear the significance fraction by the sidelobe's share, about 1 dB.
// =============================================================================

#include "modem/m110a/body_waveform.hpp"
#include "modem/m110a/receiver.hpp"
#include "modem/m110a/synchronization.hpp"
#include "tests/support/impairments.hpp"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

using ::modem::common::BodyInterleave;
using ::modem::common::DataRate;
using ::modem::common::IQSample;
using ::modem::m110a::BodyDelaySpreadProfile;

namespace
{

constexpr std::size_t preamble_symbols = 1440U;
// The channel: the direct path's gain, the echo's quadrature part (its in-phase part is
// each case's echo gain), and the noise.
constexpr float direct_gain = 0.8F;
constexpr float echo_quadrature = 0.2F;
constexpr float noise_sigma = 0.01F;
constexpr std::uint64_t noise_seed = 0xDE1A0ULL;

// One echo of the direct path: how many symbols later it arrives and its in-phase gain.
struct Echo
{
    std::size_t delay_symbols{};
    float gain{};
};

// The estimator's significance fraction of the strongest lag power (about -9 dB), the
// threshold the farthest significant path has to reach.
constexpr float significant_path_fraction = 0.125F;
// The estimator's lag reach, its default: lags 0 to 15.
constexpr std::size_t maximum_delay_symbols = 15U;
// The most an echo's own sidelobe may leave in the profile, as a share of that echo's lag
// power. The cancelled lags measure 0 to 0.2 percent; the uncancelled lag 14 sidelobe was
// 7.4 percent, so 2 percent separates the two with room on both sides.
constexpr float residual_sidelobe_share = 0.02F;

// The 600S preamble through a direct path and its echoes, with a little noise; the delay
// spread the receiver estimates from it (and, when asked, the lag powers it decided on).
// Each echo carries the same quadrature part, and the noise seed follows the first echo's delay.
std::size_t estimate_for(std::span<const Echo> echoes, BodyDelaySpreadProfile* profile = nullptr)
{
    const ::modem::m110a::BodyMode mode{DataRate::bps600, BodyInterleave::short_block};
    std::array<std::uint8_t, preamble_symbols> tribits{};
    REQUIRE(::modem::m110a::generate_body_preamble(mode, tribits).is_ok());
    std::vector<IQSample> clean(preamble_symbols);
    REQUIRE(::modem::m110a::body_tribits_to_iq(tribits, clean).is_ok());

    std::vector<IQSample> received(preamble_symbols);
    for (std::size_t index = 0U; index < clean.size(); ++index)
    {
        received[index] = direct_gain * clean[index];
        for (const auto& echo : echoes)
        {
            if (echo.delay_symbols != 0U && index >= echo.delay_symbols)
            {
                received[index] += IQSample{echo.gain, echo_quadrature} * clean[index - echo.delay_symbols];
            }
        }
    }

    modem_test::DeterministicRng noise{noise_seed + echoes.front().delay_symbols};
    modem_test::add_awgn(received, noise_sigma, noise);
    const auto acquisition = ::modem::m110a::acquire_body_preamble(received, {2400.0F, 0.55F, 40.0F, 1U});
    REQUIRE(acquisition);

    const auto estimate = ::modem::m110a::estimate_body_preamble_delay_spread(received, acquisition.value(), maximum_delay_symbols, profile);
    REQUIRE(estimate);
    return estimate.value();
}

// The one-echo channel the first three cases use.
std::size_t estimate_for(std::size_t echo_delay, float echo_gain)
{
    const std::array<Echo, 1U> echoes{Echo{echo_delay, echo_gain}};
    return estimate_for(echoes);
}

} // namespace

TEST_CASE("A single path reports no echo and keeps the default geometry", "[delay_spread]")
{
    const auto spread = estimate_for(0U, 0.0F);
    CHECK(spread <= 2U);
    CHECK(::modem::m110a::select_body_receiver_geometry(spread).feedforward_taps == ::modem::m110a::body_receiver_feedforward_taps);
}

TEST_CASE("A 2 ms class echo (5 symbols) is measured and stays in the default geometry", "[delay_spread]")
{
    const auto spread = estimate_for(5U, 0.5F);
    CHECK(spread >= 4U);
    CHECK(spread <= 6U);
    CHECK(::modem::m110a::select_body_receiver_geometry(spread).feedforward_taps == ::modem::m110a::body_receiver_feedforward_taps);
}

TEST_CASE("A 5 ms class echo (12 symbols) at -3 dB selects the long-delay geometry", "[delay_spread]")
{
    // The echo-combining countdown decode survives a -3 dB echo at twelve symbols (the
    // case that defeated the lag-0 correlator and was first softened to -6 dB).
    const auto spread = estimate_for(12U, 0.5F);
    CHECK(spread >= 11U);
    CHECK(spread <= 13U);
    const auto geometry = ::modem::m110a::select_body_receiver_geometry(spread);
    CHECK(geometry.feedforward_taps == 32U);
    CHECK(geometry.feedback_taps == 14U);
}

TEST_CASE("A weaker echo at 12 symbols behind a strong one at 5 is the delay spread, not the strongest echo", "[delay_spread]")
{
    // The equalizer window has to reach the farthest significant path. The echo at 12
    // symbols sits about 8 to 9 dB below the strongest lag, above the significance
    // fraction; the strongest echo (5 symbols) would have reported 5 and kept the default
    // geometry, which cannot hold the 12-symbol path.
    constexpr std::size_t strong_echo_delay = 5U;
    constexpr std::size_t weak_echo_delay = 12U;
    constexpr float strong_echo_gain = 0.5F;
    constexpr float weak_echo_gain = 0.21F;
    const std::array<Echo, 2U> echoes{Echo{strong_echo_delay, strong_echo_gain}, Echo{weak_echo_delay, weak_echo_gain}};
    BodyDelaySpreadProfile profile{};
    const auto spread = estimate_for(echoes, &profile);
    const float threshold = significant_path_fraction * profile.strongest_power;
    CAPTURE(profile.lag_powers[strong_echo_delay], profile.lag_powers[weak_echo_delay], profile.strongest_power, threshold);
    CHECK(spread >= 11U);
    CHECK(spread <= 13U);
    const auto geometry = ::modem::m110a::select_body_receiver_geometry(spread);
    CHECK(geometry.feedforward_taps == 32U);
    CHECK(geometry.feedback_taps == 14U);
    // The profile proves the case discriminates: the 5-symbol echo is the stronger lag, so
    // the strongest-echo rule reports 5, while the 12-symbol lag still reaches the threshold.
    CHECK(profile.lag_powers[strong_echo_delay] > profile.lag_powers[weak_echo_delay]);
    CHECK(profile.lag_powers[weak_echo_delay] >= threshold);
}

TEST_CASE("An echo's own sidelobes are not a path", "[delay_spread]")
{
    // An echo at 5 symbols repeats at lags 13 and 14 (5 + 8, 5 + 9), the reference's
    // period-8 autocorrelation sidelobes, at a few percent of its power.
    // The estimator cancels them before it scans for the largest lag. The estimate alone
    // cannot fail here: on this clean integer-delay render the sidelobe tops out near 7
    // percent of the strongest lag, under the 12.5 percent fraction; on faded two-path
    // bursts it crossed the fraction, which is why the cancellation exists.
    // The profile check is what fails on the old code: lag 14 read 7.4 percent of lag 5.
    constexpr std::size_t echo_delay = 5U;
    constexpr float echo_gain = 1.0F;
    constexpr std::size_t first_sidelobe_lag = echo_delay + 8U;
    constexpr std::size_t second_sidelobe_lag = echo_delay + 9U;
    constexpr std::size_t third_sidelobe_lag = echo_delay + 10U;
    const std::array<Echo, 1U> echoes{Echo{echo_delay, echo_gain}};
    BodyDelaySpreadProfile profile{};
    const auto spread = estimate_for(echoes, &profile);
    const float threshold = significant_path_fraction * profile.strongest_power;
    const float bound = residual_sidelobe_share * profile.lag_powers[echo_delay];
    CAPTURE(profile.lag_powers[0U], profile.lag_powers[echo_delay], profile.lag_powers[first_sidelobe_lag], profile.lag_powers[second_sidelobe_lag], profile.lag_powers[third_sidelobe_lag],
            profile.strongest_power, threshold, bound);
    CHECK(spread >= 4U);
    CHECK(spread <= 6U);
    CHECK(::modem::m110a::select_body_receiver_geometry(spread).feedforward_taps == ::modem::m110a::body_receiver_feedforward_taps);
    CHECK(profile.lag_powers[first_sidelobe_lag] < bound);
    CHECK(profile.lag_powers[second_sidelobe_lag] < bound);
    CHECK(profile.lag_powers[third_sidelobe_lag] < bound);
}

TEST_CASE("A real path eight symbols behind the echo survives the cancellation", "[delay_spread]")
{
    // The echo at 5 symbols casts a sidelobe on lag 13, exactly where the real path
    // sits. Cancelling the sidelobe takes its share off the lag, not the lag itself, so
    // the real path (about 8 dB below the strongest lag) still reaches the fraction and
    // the long geometry is selected.
    constexpr std::size_t echo_delay = 5U;
    constexpr std::size_t path_delay = 13U;
    constexpr float echo_gain = 0.5F;
    constexpr float path_gain = 0.25F;
    const std::array<Echo, 2U> echoes{Echo{echo_delay, echo_gain}, Echo{path_delay, path_gain}};
    BodyDelaySpreadProfile profile{};
    const auto spread = estimate_for(echoes, &profile);
    const float threshold = significant_path_fraction * profile.strongest_power;
    CAPTURE(profile.lag_powers[echo_delay], profile.lag_powers[path_delay], profile.strongest_power, threshold);
    CHECK(spread >= 12U);
    CHECK(spread <= 14U);
    const auto geometry = ::modem::m110a::select_body_receiver_geometry(spread);
    CHECK(geometry.feedforward_taps == 32U);
    CHECK(geometry.feedback_taps == 14U);
    CHECK(profile.lag_powers[path_delay] >= threshold);
}

TEST_CASE("A real path nine symbols behind the echo is the cancellation's limit", "[delay_spread]")
{
    // The echo at 5 symbols casts a sidelobe on lag 14 of about 8 percent of its power, and
    // the cancellation takes that off lag 14. A real path at 14 therefore has to clear the
    // fraction by that much more. The second section is a
    // documented limit of the cancellation, not a goal: a path 9 symbols behind a strong
    // echo needs to sit about 1 dB further above the fraction, and a path only about
    // 1 dB above it is lost behind the echo's sidelobe and reads as the echo.
    constexpr std::size_t echo_delay = 5U;
    constexpr std::size_t path_delay = 14U;
    constexpr float echo_gain = 0.5F;

    SECTION("A path clear of the sidelobe's share is the delay spread")
    {
        constexpr float path_gain = 0.30F;
        const std::array<Echo, 2U> echoes{Echo{echo_delay, echo_gain}, Echo{path_delay, path_gain}};
        BodyDelaySpreadProfile profile{};
        const auto spread = estimate_for(echoes, &profile);
        const float threshold = significant_path_fraction * profile.strongest_power;
        CAPTURE(profile.lag_powers[echo_delay], profile.lag_powers[path_delay], profile.strongest_power, threshold);
        CHECK(spread >= 13U);
        CHECK(spread <= 15U);
        const auto geometry = ::modem::m110a::select_body_receiver_geometry(spread);
        CHECK(geometry.feedforward_taps == 32U);
        CHECK(geometry.feedback_taps == 14U);
    }

    SECTION("A path about 1 dB above the fraction is lost behind the echo's sidelobe")
    {
        constexpr float path_gain = 0.25F;
        const std::array<Echo, 2U> echoes{Echo{echo_delay, echo_gain}, Echo{path_delay, path_gain}};
        BodyDelaySpreadProfile profile{};
        const auto spread = estimate_for(echoes, &profile);
        const float threshold = significant_path_fraction * profile.strongest_power;
        CAPTURE(profile.lag_powers[echo_delay], profile.lag_powers[path_delay], profile.strongest_power, threshold);
        CHECK(spread >= 4U);
        CHECK(spread <= 6U);
        CHECK(::modem::m110a::select_body_receiver_geometry(spread).feedforward_taps == ::modem::m110a::body_receiver_feedforward_taps);
        CHECK(profile.lag_powers[path_delay] < threshold);
    }
}
