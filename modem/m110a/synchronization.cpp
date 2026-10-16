// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

/*
================================================================================
synchronization.cpp - body preamble acquisition
================================================================================

WHAT THIS FILE IS
-----------------
The front door of the MIL-STD-188-110B serial-tone body receiver. Before any
payload bit can be decoded, this file answers, from complex baseband samples
alone: is a burst present, where does it start, what is its carrier
frequency/phase, what mode/rate is it, and where does the data body begin.

  INPUT  : complex baseband at the SYMBOL RATE (one complex value per
           2400-baud symbol), from the audio front end.
  OUTPUT : a BodyAcquisition (start, frequency, phase, mode, first_body_symbol).
  DEPENDS: body_waveform.hpp (preamble structure, body_channel_symbol_tribit,
           designator/countdown decode), scrambler.hpp (SyncRandomizer),
           demapper.hpp (psk8_symbol).
  NOT here: no payload decode. This produces the locked reference the equalizer,
           demapper, and FEC decoder are built on top of.

50,000-FOOT VIEW
----------------
The transmitter always begins with a known preamble made of repeating 480-symbol
segments. The first 288 chips of each segment are a fixed acquisition prefix.
This file correlates the incoming samples against that known prefix to find the
burst, estimates the carrier offset differentially, decodes the D1/D2 mode
designators and the C1/C2/C3 countdown, and back-computes where the body starts.
It groups the segments that agree, and lets every segment vote on the mode and
the boundary, so one low-SNR mis-decode cannot mis-frame training.

5th-GRADE VERSION
-----------------
A friend always sings the same jingle before a secret message. Listen for the
jingle; when you hear it clearly you learn it is starting, how high their voice
is (carrier frequency), which code they are using (mode), and a countdown to the
real words.

WORKED EXAMPLE / DEEP DIVE
--------------------------
The full step-by-step walkthrough - with toy numbers, equations, fading-failure
modes, and debugging signals - lives in the standalone explainer:
    modem/m110a/synchronization-and-acquisition-explainer.md
Explained in the article series, Part 4 (acquisition: finding the burst); its
unit tests are tests/acquisition_tests.cpp.

HIGH-LEVEL PASS THROUGH THIS FILE
---------------------------------
 1. make_body_prefix_reference()             build the known 288-chip template.
 2. estimate_body_prefix_frequency_radians() differential carrier estimate.
 3. body_prefix_correlation()                segmented NONCOHERENT match score.
 4. decode_channel_symbol()                  8-code + echo-combining symbol decode.
 5. characterize_body_candidate()            D1/D2 + countdown -> body boundary.
 6. acquire_body_preamble()                  scan -> consensus -> vote, one named
                                             function a step.
================================================================================
*/

#include "modem/m110a/synchronization.hpp"

#include "modem/common/constants.hpp"
#include "modem/common/convert.hpp"
#include "modem/m110a/demapper.hpp"
#include "modem/m110a/scrambler.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

#include "modem/common/status.hpp"
namespace modem::m110a
{

using ::modem::common::as_real;
using ::modem::common::IQSample;
using ::modem::common::IQSampleSpan;
using ::modem::common::low_bits;
using ::modem::common::pi;
using ::modem::common::Result;
using ::modem::common::rmath;
using ::modem::common::Status;
using ::modem::common::StatusCode;
using ::modem::common::two_pi;

constexpr std::array<std::uint8_t, 9> acquisition_prefix{0U, 1U, 3U, 0U, 1U, 3U, 1U, 2U, 0U};
constexpr std::size_t prefix_symbols = body_preamble_acquisition_prefix_symbols;
constexpr std::size_t first_segment_symbols = body_preamble_segment_symbols;
// Every preamble channel symbol is one of eight orthogonal 32-chip codes.
constexpr std::size_t channel_symbol_chips = 32U;
constexpr std::uint8_t channel_symbol_codes = 8U;
// Where the designators and the countdown sit among a segment's channel symbols.
constexpr std::size_t d1_position = 9U;
constexpr std::size_t d2_position = 10U;
constexpr std::size_t c1_position = 11U;
constexpr std::size_t c2_position = 12U;
constexpr std::size_t c3_position = 13U;

static float wrap_phase(float phase) noexcept
{
    while (phase > pi)
    {
        phase -= two_pi;
    }

    while (phase < -pi)
    {
        phase += two_pi;
    }

    return phase;
}

// -----------------------------------------------------------------------------
// make_body_prefix_reference
// -----------------------------------------------------------------------------
// 50K view: Build the known 288-chip acquisition-prefix template that every
//   correlation in this file compares against. Constant for a given build.
// Detailed view: Walk the 9 prefix channel symbols {0,1,3,0,1,3,1,2,0}; spread
//   each over 32 chips (body_channel_symbol_tribit), add the running
//   SyncRandomizer tribit (the sync scramble), reduce mod 8, and map to an 8-PSK
//   point (psk8_symbol). Result: 288 complex reference symbols.
// 5th-grade view: Write down the exact jingle on paper so you can compare what
//   you hear against it.
// -----------------------------------------------------------------------------
void make_body_prefix_reference(std::span<IQSample, prefix_symbols> reference) noexcept
{
    SyncRandomizer randomizer;
    std::size_t index{};

    for (const auto channel_symbol : acquisition_prefix)
    {
        for (std::size_t spread = 0U; spread < channel_symbol_chips; ++spread)
        {
            const auto tribit = tribit_add(body_channel_symbol_tribit(channel_symbol, spread), randomizer.next_tribit());
            reference[index++] = psk8_symbol(tribit);
        }
    }
}

// The reference never changes, so the scanners build it once, on first use, into
// static storage (as psk8_symbol does its table): it costs no stack, and no time
// after the first call.
static const std::array<IQSample, prefix_symbols>& prefix_reference() noexcept
{
    static const auto reference = []() noexcept
    {
        std::array<IQSample, prefix_symbols> table{};
        make_body_prefix_reference(table);
        return table;
    }();
    return reference;
}

// -----------------------------------------------------------------------------
// estimate_body_prefix_frequency_radians
// -----------------------------------------------------------------------------
// 50K view: Estimate the carrier frequency error over the prefix, in radians per
//   symbol, at a candidate start.
// Detailed view: Differential (Kay-style) estimator. Multiply each received
//   sample by the conjugate of the previous one AND strip the known reference
//   modulation: r[k]*conj(r[k-1])*conj(c[k])*c[k-1]. When the received signal is
//   the reference times a carrier e^{j*omega*k}, the known parts cancel to a
//   positive magnitude and the sum's angle is ~= omega. arg(sum) returns omega.
// 5th-grade view: Figure out how much the pitch drifted by comparing each note
//   to the one before it.
// -----------------------------------------------------------------------------
float estimate_body_prefix_frequency_radians(IQSampleSpan input, std::size_t start, IQSampleSpan reference) noexcept
{
    IQSample differential{};

    for (std::size_t index = 1U; index < reference.size(); ++index)
    {
        differential += input[start + index] * std::conj(input[start + index - 1U]) * std::conj(reference[index]) * reference[index - 1U];
    }

    return std::arg(differential);
}

// -----------------------------------------------------------------------------
// body_prefix_correlation
// -----------------------------------------------------------------------------
// 50K view: Return a normalized [0,1] "is the prefix here?" score and, as a side
//   effect, the coherent carrier phase for equalizer seeding.
// Detailed view: Segmented NONCOHERENT correlation - correlate each 32-chip
//   block coherently (derotated by the frequency estimate) and sum the block
//   MAGNITUDES. On a static channel this equals a full coherent correlation;
//   under fast fading the coherent sum self-cancels but the per-block magnitudes
//   survive (the measured 5 Hz graceful-degradation behavior). See the inline
//   note below for the fading rationale.
// 5th-grade view: Instead of matching the whole jingle at once (which a wavering
//   signal ruins), match it in nine short pieces and add up how good each was.
// -----------------------------------------------------------------------------
float body_prefix_correlation(IQSampleSpan input, std::size_t start, IQSampleSpan reference, float frequency_radians, float& phase) noexcept
{
    // Segmented noncoherent prefix correlation: each 32-chip sub-block is
    // correlated coherently and the block magnitudes are summed. On a static
    // channel the block phases align, so the metric equals the full coherent
    // correlation; under fast fading (a 5 Hz spread decorrelates the channel
    // well inside the 120 ms prefix) the coherent sum self-cancels while the
    // per-block gains survive, which is exactly the measured 5 Hz
    // acquisition loss. The reported carrier phase remains the coherent
    // estimate for equalizer seeding.
    IQSample coherent{};
    IQSample block_correlation{};
    float noncoherent{};
    float input_energy{};

    for (std::size_t index = 0U; index < reference.size(); ++index)
    {
        const auto rotation = std::polar(1.0F, frequency_radians * as_real(index));
        const auto product = input[start + index] * std::conj(reference[index] * rotation);
        coherent += product;
        block_correlation += product;
        input_energy += std::norm(input[start + index]);

        if ((index + 1U) % channel_symbol_chips == 0U)
        {
            noncoherent += std::abs(block_correlation);
            block_correlation = IQSample{};
        }
    }

    noncoherent += std::abs(block_correlation);
    phase = std::arg(coherent);
    constexpr float energy_floor = 1.0e-12F;
    return noncoherent / rmath::sqrt(std::max(input_energy * as_real(reference.size()), energy_floor));
}

// -----------------------------------------------------------------------------
// decode_channel_symbol
// -----------------------------------------------------------------------------
// 50K view: Decode one preamble channel symbol (a D1/D2 designator or a C1/C2/C3
//   countdown digit) by asking which of the 8 orthogonal spreading codes matches.
// Detailed view: For each candidate 0..7 rebuild the expected 32-chip spread
//   (code + sync scramble + within-block frequency ramp) and correlate. The 8
//   codes are orthogonal, so max-magnitude correlation is the ML decision under
//   an unknown constant phase. NONCOHERENT echo combining scores each candidate
//   as |Lag0|^2 + max_{3<=lag<=15}|Lag|^2 (direct + best echo) so a long-delay
//   two-path channel with a faded direct path can still read the symbol.
// 5th-grade view: Hold up all 8 possible letters, see which the sound matches
//   best, and if the direct sound is weak let the wall-echo vote too.
// -----------------------------------------------------------------------------
static std::uint8_t decode_channel_symbol(IQSampleSpan input, std::size_t start, std::size_t channel_index, float frequency_radians) noexcept
{
    std::array<float, channel_symbol_codes> correlation_energies{};
    SyncRandomizer randomizer;
    const auto spread_offset = channel_index * channel_symbol_chips;

    for (std::size_t index = 0U; index < spread_offset; ++index)
    {
        randomizer.advance();
    }

    std::array<std::uint8_t, channel_symbol_chips> randomizer_values{};

    for (auto& value : randomizer_values)
    {
        value = randomizer.next_tribit();
    }

    // The eight 32-chip spreading sequences are mutually orthogonal, so the
    // maximum-magnitude correlation is the maximum-likelihood decision under an
    // unknown constant carrier phase. Extrapolating an absolute phase from the
    // acquisition prefix across the whole segment is not reliable over a fading
    // channel; only the residual frequency ramp within one 32-symbol block is
    // applied, which tolerates several hertz of frequency-estimate error.
    //
    // Long-delay two-path channels (a 5 ms echo is twelve symbols) put nearly
    // half the received energy in a delayed copy the lag-0 correlator treats
    // as interference, and a faded direct path then loses the countdown
    // decode entirely. Each candidate is therefore scored with noncoherent
    // direct-plus-echo combining over a small lag window, without needing the
    // (yet unknown) delay; on a clean channel the echo term adds one noise
    // sample and the orthogonal-code margin absorbs it.
    constexpr std::size_t maximum_echo_lag = 15U;

    for (std::uint8_t candidate = 0U; candidate < channel_symbol_codes; ++candidate)
    {
        std::array<IQSample, maximum_echo_lag + 1U> lag_correlations{};

        for (std::size_t spread = 0U; spread < channel_symbol_chips; ++spread)
        {
            const auto tribit = tribit_add(body_channel_symbol_tribit(candidate, spread), randomizer_values[spread]);
            const auto expected = psk8_symbol(tribit) * std::polar(1.0F, frequency_radians * as_real(spread));

            for (std::size_t lag = 0U; lag <= maximum_echo_lag; ++lag)
            {
                const auto sample_index = start + spread_offset + spread + lag;

                if (sample_index < input.size())
                {
                    lag_correlations[lag] += input[sample_index] * std::conj(expected);
                }
            }
        }

        float best_echo_energy{};

        for (std::size_t lag = 3U; lag <= maximum_echo_lag; ++lag)
        {
            best_echo_energy = std::max(best_echo_energy, std::norm(lag_correlations[lag]));
        }

        correlation_energies[candidate] = std::norm(lag_correlations[0]) + best_echo_energy;
    }

    std::uint8_t best{};

    for (std::uint8_t candidate = 1U; candidate < channel_symbol_codes; ++candidate)
    {
        if (correlation_energies[candidate] > correlation_energies[best])
        {
            best = candidate;
        }
    }

    return best;
}

// -----------------------------------------------------------------------------
// characterize_body_candidate
// -----------------------------------------------------------------------------
// 50K view: Turn a raw "there might be a prefix at start" into a fully described
//   candidate: mode, countdown, preamble start, and body boundary.
// Detailed view: Decode D1(9), D2(10), C1/C2/C3(11/12/13). Each countdown symbol
//   is sent as 4|digit, so a decoded value < 4 means "not a credible segment".
//   Pack the countdown, resolve the mode (recognize_body_designators, LONG tried
//   first), then back-extrapolate: body begins (countdown+1)*480 symbols after
//   this segment; the true preamble start is elapsed_segments*480 earlier (with
//   the carrier phase rewound to match).
// 5th-grade view: Read the dialect and the "...2 more" countdown, then do the
//   arithmetic for where the real words start and where the jingle began.
// -----------------------------------------------------------------------------
BodyAcquisition characterize_body_candidate(IQSampleSpan input, BodyAcquisition candidate, const BodyAcquisitionConfig& config) noexcept
{
    const auto frequency_radians = candidate.frequency_offset_hz * two_pi / config.symbol_rate_hz;
    const auto detected_segment = candidate.first_preamble_symbol;
    const auto d1 = decode_channel_symbol(input, detected_segment, d1_position, frequency_radians);
    const auto d2 = decode_channel_symbol(input, detected_segment, d2_position, frequency_radians);
    const auto c1 = decode_channel_symbol(input, detected_segment, c1_position, frequency_radians);
    const auto c2 = decode_channel_symbol(input, detected_segment, c2_position, frequency_radians);
    const auto c3 = decode_channel_symbol(input, detected_segment, c3_position, frequency_radians);

    // Every countdown symbol is transmitted as 4 | digit; a decoded value
    // below 4 means this position is not a credible preamble segment.
    if (c1 < 4U || c2 < 4U || c3 < 4U)
    {
        candidate.detected_segment_symbol = detected_segment;
        candidate.preamble_countdown = std::numeric_limits<std::uint8_t>::max();
        candidate.symbol_rate_hz = config.symbol_rate_hz;
        candidate.recognition = {BodyDesignatorState::invalid, {}};
        return candidate;
    }

    const auto countdown = low_bits<6>(((c1 & 3U) << 4U) | ((c2 & 3U) << 2U) | (c3 & 3U)); // three 2-bit digits
    const auto short_recognition = recognize_body_designators(d1, d2, false);
    const auto long_recognition = recognize_body_designators(d1, d2, true);
    candidate.detected_segment_symbol = detected_segment;
    candidate.preamble_countdown = countdown;
    candidate.symbol_rate_hz = config.symbol_rate_hz;

    if (long_recognition.state == BodyDesignatorState::supported)
    {
        candidate.recognition = long_recognition;
    }
    else if (short_recognition.state == BodyDesignatorState::supported)
    {
        candidate.recognition = short_recognition;
    }
    else if (long_recognition.state == BodyDesignatorState::recognized_unsupported || short_recognition.state == BodyDesignatorState::recognized_unsupported)
    {
        candidate.recognition = {BodyDesignatorState::recognized_unsupported, {}};
    }
    else
    {
        candidate.recognition = {BodyDesignatorState::invalid, {}};
    }

    if (candidate.recognition.state == BodyDesignatorState::supported)
    {
        const auto segments = body_preamble_segments(candidate.recognition.mode.interleave);

        if (countdown < segments)
        {
            const auto elapsed_segments = segments - countdown - 1U;
            const auto elapsed_symbols = elapsed_segments * first_segment_symbols;
            candidate.first_body_symbol = detected_segment + (countdown + 1U) * first_segment_symbols;

            if (candidate.first_preamble_symbol >= elapsed_symbols)
            {
                candidate.first_preamble_symbol -= elapsed_symbols;
                candidate.preamble_start_in_buffer = true;
                candidate.carrier_phase_radians = wrap_phase(candidate.carrier_phase_radians - frequency_radians * as_real(elapsed_symbols));
            }
        }
    }

    return candidate;
}

bool body_candidate_is_valid(const BodyAcquisition& candidate) noexcept
{
    if (candidate.recognition.state != BodyDesignatorState::supported)
    {
        return false;
    }

    const auto segments = body_preamble_segments(candidate.recognition.mode.interleave);
    return candidate.preamble_countdown < segments && candidate.first_body_symbol >= candidate.detected_segment_symbol + first_segment_symbols;
}

// -----------------------------------------------------------------------------
// acquire_body_preamble, step by step
// -----------------------------------------------------------------------------
// The batch scanner is a short sequence of named steps; each is a function below.
//   1. scan_for_consensus        every start through three gates (carrier offset
//                                in range, prefix correlation over the floor, a
//                                valid mode and countdown), and the survivors
//                                grouped by the transmission they describe.
//   2. strongest_group           the group the most segments support.
//   3. vote_mode_and_boundary    every segment votes on the mode and boundary.
// -----------------------------------------------------------------------------

// Candidates that describe the same transmission: the same body boundary and mode.
struct AcquisitionConsensus
{
    BodyAcquisition representative{};
    std::size_t supporting_segments{};
    float correlation_sum{};
    float best_correlation{};
};

struct ConsensusScan
{
    static constexpr std::size_t capacity = 32U;
    std::array<AcquisitionConsensus, capacity> groups{};
    std::size_t count{};
};

static bool same_mode(const BodyAcquisition& first, const BodyAcquisition& second) noexcept
{
    return first.recognition.mode.data_rate == second.recognition.mode.data_rate && first.recognition.mode.interleave == second.recognition.mode.interleave;
}

static bool same_transmission(const BodyAcquisition& first, const BodyAcquisition& second) noexcept
{
    return first.first_body_symbol == second.first_body_symbol && same_mode(first, second);
}

// Step 1's test of one start, cheapest gate first: the carrier offset in range,
// the prefix correlation over the floor, then a valid mode and countdown.
static std::optional<BodyAcquisition> evaluate_start(IQSampleSpan input, std::size_t start, IQSampleSpan reference, const BodyAcquisitionConfig& config) noexcept
{
    const auto frequency_radians = estimate_body_prefix_frequency_radians(input, start, reference);
    const auto frequency_hz = frequency_radians * config.symbol_rate_hz / two_pi;

    if (std::fabs(frequency_hz) > config.maximum_frequency_offset_hz)
    {
        return std::nullopt;
    }

    float phase{};
    const auto correlation = body_prefix_correlation(input, start, reference, frequency_radians, phase);

    if (correlation < config.minimum_correlation)
    {
        return std::nullopt;
    }

    BodyAcquisition candidate{};
    candidate.first_preamble_symbol = start;
    candidate.frequency_offset_hz = frequency_hz;
    candidate.carrier_phase_radians = phase;
    candidate.normalized_correlation = correlation;
    candidate = characterize_body_candidate(input, candidate, config);

    if (!body_candidate_is_valid(candidate))
    {
        return std::nullopt;
    }

    return candidate;
}

// Step 1: every start through the gates. Candidates that agree on the body
// boundary and mode join one group, so each clean segment of a preamble adds a
// supporter to its transmission's group.
static ConsensusScan scan_for_consensus(IQSampleSpan input, IQSampleSpan reference, std::size_t search_starts, const BodyAcquisitionConfig& config) noexcept
{
    ConsensusScan scan{};

    for (std::size_t start = 0U; start < search_starts; ++start)
    {
        const auto candidate = evaluate_start(input, start, reference, config);

        if (!candidate)
        {
            continue;
        }

        std::size_t group_index{};

        while (group_index < scan.count && !same_transmission(scan.groups[group_index].representative, *candidate))
        {
            ++group_index;
        }

        if (group_index == scan.count)
        {
            if (scan.count == scan.groups.size())
            {
                continue;
            }

            ++scan.count;
        }

        auto& group = scan.groups[group_index];

        // The representative supplies the frequency/phase estimates the
        // equalizer trains with, and every member already back-extrapolates
        // its position to the full preamble start - so quality, not
        // earliness, must pick it. A fade-distorted early segment that
        // clears the threshold with a wrong frequency estimate otherwise
        // displaces a clean later segment and poisons the training
        // (measured: a +7.6 Hz representative lost a block a 1.0 Hz
        // same-group member decoded exactly).
        if (group.supporting_segments == 0U || candidate->normalized_correlation > group.representative.normalized_correlation)
        {
            group.representative = *candidate;
        }

        ++group.supporting_segments;
        group.correlation_sum += candidate->normalized_correlation;
        group.best_correlation = std::max(group.best_correlation, candidate->normalized_correlation);
    }

    return scan;
}

// Step 2: the group with the most supporting segments; a tie goes to the larger
// correlation sum, then to the larger single correlation.
static std::size_t strongest_group(const ConsensusScan& scan) noexcept
{
    std::size_t best_group{};

    for (std::size_t group = 1U; group < scan.count; ++group)
    {
        const auto& candidate = scan.groups[group];
        const auto& best = scan.groups[best_group];

        if (candidate.supporting_segments > best.supporting_segments || (candidate.supporting_segments == best.supporting_segments && candidate.correlation_sum > best.correlation_sum) ||
            (candidate.supporting_segments == best.supporting_segments && candidate.correlation_sum == best.correlation_sum && candidate.best_correlation > best.best_correlation))
        {
            best_group = group;
        }
    }

    return best_group;
}

// Step 3: consensus over the whole preamble, mode AND body boundary.
// The mode designators and the countdown come from one segment's decode at the
// lowest SNR the standard allows; a single mis-decode (measured at 75L: a wrong
// mode, or a countdown two segments off -> preamble training on a mis-framed
// reference, residual ~1.0, whole payload lost) is invisible to the consensus
// scan, which only covers the first two or three segments. Every later segment
// carries the same designators and a consistent boundary, so decode them all
// from the winner's segment on and let the majority rule; segments past the true
// boundary decode as invalid and abstain.
static BodyAcquisition vote_mode_and_boundary(IQSampleSpan input, IQSampleSpan reference, const BodyAcquisition& current, const BodyAcquisitionConfig& config) noexcept
{
    struct BoundaryVote
    {
        BodyAcquisition representative{};
        std::size_t votes{};
    };
    constexpr std::size_t boundary_capacity = 8U;
    std::array<BoundaryVote, boundary_capacity> boundaries{};
    std::size_t boundary_count = 0U;
    const auto current_frequency_radians = current.frequency_offset_hz * two_pi / config.symbol_rate_hz;

    for (std::size_t segment_start = current.detected_segment_symbol; segment_start + first_segment_symbols <= input.size(); segment_start += first_segment_symbols)
    {
        BodyAcquisition probe{};
        probe.first_preamble_symbol = segment_start;
        probe.frequency_offset_hz = current.frequency_offset_hz;
        float phase{};
        probe.normalized_correlation = body_prefix_correlation(input, segment_start, reference, current_frequency_radians, phase);
        probe.carrier_phase_radians = phase;
        probe = characterize_body_candidate(input, probe, config);

        if (!body_candidate_is_valid(probe))
        {
            continue;
        }

        std::size_t index = 0U;

        while (index < boundary_count && !same_transmission(boundaries[index].representative, probe))
        {
            ++index;
        }

        if (index == boundary_count)
        {
            if (boundary_count == boundaries.size())
            {
                continue;
            }

            boundaries[index].representative = probe;
            ++boundary_count;
        }
        else if (probe.normalized_correlation > boundaries[index].representative.normalized_correlation)
        {
            boundaries[index].representative = probe;
        }

        ++boundaries[index].votes;
    }

    std::size_t best = boundary_count;

    for (std::size_t index = 0U; index < boundary_count; ++index)
    {
        if (best == boundary_count || boundaries[index].votes > boundaries[best].votes)
        {
            best = index;
        }
    }

    if (best == boundary_count || boundaries[best].votes < 2U || same_transmission(boundaries[best].representative, current))
    {
        return current;
    }

    // The majority disagrees with the winner: keep the winner's frequency but
    // adopt the majority mode, boundary and the preamble start they imply.
    auto corrected = boundaries[best].representative;
    corrected.frequency_offset_hz = current.frequency_offset_hz;
    return corrected;
}

// -----------------------------------------------------------------------------
// acquire_body_preamble  (the main acquisition entry point)
// -----------------------------------------------------------------------------
// 50K view: Scan every candidate start, keep the ones past the frequency,
//   correlation and decode gates, group them by which transmission they describe
//   (consensus voting), pick the strongest group, then repair single-segment
//   mis-decodes with a whole-preamble majority vote.
// Detailed view: the numbered steps above, in order.
// 5th-grade view: Listen everywhere, keep the clear catches, group the agreeing
//   ones, pick the strongest, and let the other segments outvote a segment that
//   misheard the countdown.
// -----------------------------------------------------------------------------
Result<BodyAcquisition> acquire_body_preamble(IQSampleSpan symbol_rate_input, const BodyAcquisitionConfig& config) noexcept
{
    if (config.symbol_rate_hz <= 0.0F || config.minimum_correlation <= 0.0F || config.minimum_correlation > 1.0F || config.maximum_frequency_offset_hz <= 0.0F ||
        symbol_rate_input.size() < first_segment_symbols)
    {
        return Status{StatusCode::invalid_argument, "invalid body acquisition request"};
    }

    const auto& reference = prefix_reference();
    const auto available_starts = symbol_rate_input.size() - first_segment_symbols + 1U;
    const auto search_starts = config.maximum_start_symbol == 0U ? available_starts : std::min(available_starts, config.maximum_start_symbol + 1U);
    const auto scan = scan_for_consensus(symbol_rate_input, reference, search_starts, config);

    if (scan.count == 0U)
    {
        return Status{StatusCode::unavailable, "body preamble countdown not acquired"};
    }

    const auto& strongest = scan.groups[strongest_group(scan)].representative;
    return vote_mode_and_boundary(symbol_rate_input, reference, strongest, config);
}

} // namespace modem::m110a
