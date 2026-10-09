// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "modem/common/types.hpp"
#include "modem/m110a/body_audio_stream_frontend.hpp"
#include "tests/support/test_waveform.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

// =============================================================================
// front_end_harness.hpp - run audio through the receive front end and score it
// =============================================================================
// FrontEndRunner feeds audio to a BodyAudioStreamFrontend in whatever block sizes a
// test asks for and keeps every recovered symbol, numbered from the start of the
// stream. score_burst() lines those symbols up with what the transmitter sent and
// counts the decisions that match once the timing loop has locked. The front end
// does not track the carrier (a later stage does), so the score removes one fixed
// phase rotation per burst before deciding.
// =============================================================================

namespace modem_test
{

struct FrontEndOptions
{
    std::vector<std::size_t> block_sizes{480U}; // used in turn, repeating
    float receive_rolloff{modem::m110a::BodyAudioStreamFrontend::matched_filter_rolloff};
};

class FrontEndRunner
{
public:
    [[nodiscard]] modem::common::Status initialize(const FrontEndOptions& options = {});
    [[nodiscard]] modem::common::Status feed(std::span<const float> audio);

    // Symbols kept so far; symbols()[k] is stream symbol symbol_base() + k.
    [[nodiscard]] const std::vector<modem::m110a::BodyFrontendSymbol>& symbols() const noexcept { return mSymbols; }

    [[nodiscard]] std::uint64_t symbol_base() const noexcept { return mSymbolBase; }

    [[nodiscard]] std::uint64_t samples_fed() const noexcept { return mSamplesFed; }

    // Drops the kept symbols (a long run scores each burst, then discards it); the
    // numbering carries on.
    void discard_symbols() noexcept;

private:
    modem::m110a::BodyAudioStreamFrontend mFrontEnd{};
    FrontEndOptions mOptions{};
    std::vector<modem::m110a::BodyFrontendSymbol> mSymbols{};
    std::uint64_t mSymbolBase{};
    std::uint64_t mSamplesFed{};
    std::size_t mNextBlock{};
};

struct BurstScore
{
    bool locked{};
    std::size_t locked_from{}; // burst symbol where the comparison starts
    std::int64_t offset{};     // stream symbol = burst symbol + offset
    std::size_t compared{};
    std::size_t correct{};
    float rotation_degrees{}; // the fixed carrier rotation removed
    float rms_phase_error_degrees{};
};

// Scores one burst whose first sample sits at stream sample burst_first_sample.
[[nodiscard]] BurstScore score_burst(const FrontEndRunner& runner, const BurstTruth& burst, std::uint64_t burst_first_sample);

// The nearest of the eight phases, as a tribit (tribit t sits at t x 45 degrees).
[[nodiscard]] std::uint8_t nearest_tribit(modem::common::IQSample symbol) noexcept;

} // namespace modem_test
