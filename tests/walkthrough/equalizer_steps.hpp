// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#pragma once

#include "modem/common/status.hpp"
#include "modem/common/types.hpp"
#include "modem/m110a/body_waveform.hpp"
#include "modem/m110a/receiver.hpp"
#include "modem/m110a/synchronization.hpp"
#include "tests/support/test_waveform.hpp"
#include "tests/walkthrough/first_path_steps.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// =============================================================================
// equalizer_steps.hpp - the Part 6 walkthrough, one function per step
// =============================================================================
// The steps run on Part 5's recordings (the "Hi" message through Table XX's two-path
// channels) and on one clean burst straight from the transmitter. Each burst is received
// as the runner receives it: Part 3's front end with the phase search on, Part 4 and 5's
// acquisition, then the front end once more, committed a segment into the preamble. The
// receiver is then trained on the preamble the way a receiver runs it (the default window
// first, the delay spread measured with the frequency training chose, the long window
// only if the measurement asks for it), and each engine walks the body. What was sent
// comes from the transmitter itself (make_test_waveform reproduces the recordings'
// transmit side), so every decision can be checked against the symbol behind it.
// equalizer_demo runs the steps in article order; equalizer_walkthrough_tests checks them.
// Explained in the article series, Part 6 (the equalizer).
// =============================================================================

namespace modem_test::equalizer_steps
{

using ::modem::common::IQSample;
using ::modem::common::Result;
using ::modem::m110a::BodyAcquisition;
using ::modem::m110a::BodyAdaptiveReceiver;
using ::modem::m110a::BodyDataEqualizer;
using ::modem::m110a::BodyDelaySpreadProfile;
using ::modem::m110a::BodyMode;
using ::modem::m110a::BodyReceiverGeometry;
using ::modem_test::first_path_steps::open_pack;
using ::modem_test::first_path_steps::Pack;

// The recordings each step uses, and the name of the clean burst.
inline constexpr std::string_view five_ms_case = "fp1-echo-group"; // 300L, 5 ms and 5 Hz
inline constexpr std::string_view two_ms_case = "fp2-window";      // 600L, 2 ms and 1 Hz, anchored on the echo
inline constexpr std::string_view clean_case = "clean";            // 600L, no channel

// One burst received and ready for the equalizer.
struct Burst
{
    std::string id{};
    BodyMode mode{};
    std::vector<IQSample> symbols{};      // on time, one per symbol: acquisition and the delay spread
    std::vector<IQSample> half_symbols{}; // early then on time, two per symbol: the equalizer
    BodyAcquisition acquired{};
    std::vector<std::uint8_t> sent{}; // every transmitted tribit, preamble and body
    std::size_t sent_preamble_symbols{};
    std::vector<std::uint8_t> payload{};
};

[[nodiscard]] Result<Burst> receive_recording(const Pack& pack, std::string_view id, std::string& diagnostic);
[[nodiscard]] Result<Burst> receive_clean();
// Any burst's audio, received the same way; `truth` is the transmitter's record of what it sent
// (equalizer_compare renders its own bursts through the bench's channel and uses this).
[[nodiscard]] Result<Burst> receive_audio(std::string id, BodyMode mode, std::span<const float> audio, const ::modem_test::TestWaveform& truth);

// The receiver trained on a burst's preamble.
struct Trained
{
    std::unique_ptr<BodyAdaptiveReceiver> receiver{};
    BodyDataEqualizer engine{};
    std::size_t delay_spread{};
    BodyDelaySpreadProfile profile{};
    BodyReceiverGeometry first_geometry{};
    BodyReceiverGeometry geometry{};
    bool retrained{};
    BodyAdaptiveReceiver::PreambleRaceDiagnostics race{}; // of the training that stands
    float training_frequency_hz{};
};

[[nodiscard]] Result<Trained> train(const Burst& burst, BodyDataEqualizer engine);

// One data frame of the body: its data symbols' decisions against what was sent, and the
// mean squared error on the probe that follows it.
struct Frame
{
    std::size_t block{};
    std::size_t frame{};
    std::size_t data_symbols{};
    std::size_t data_errors{};
    float probe_error{};
};

// The body walked block by block. Decoding the symbols back into the message is Part 7's
// (soft demapping and the decoder), so the walk stops at the equalizer's decisions.
struct Body
{
    std::vector<Frame> frames{};
    std::size_t data_symbols{};
    std::size_t data_errors{};
};

[[nodiscard]] Result<Body> walk_body(const Burst& burst, Trained& trained);

// --- Step 1: how far behind the anchor (fp1, fp2, the clean burst) ---
struct SpreadRow
{
    std::string id{};
    std::size_t spread{};
    std::size_t segments{};
    std::vector<float> lag_fraction{}; // lags 0 to 15, each as a fraction of the strongest
};
struct DelaySpread
{
    std::vector<SpreadRow> rows{};
};
[[nodiscard]] Result<DelaySpread> delay_spread(const Pack& pack, std::string& diagnostic);
void print(const DelaySpread& step);

// --- Step 2: a window sized from the measurement ---
struct WindowRow
{
    std::string id{};
    std::size_t spread{};
    BodyReceiverGeometry geometry{};
    bool retrained{};
};
struct Window
{
    std::vector<WindowRow> rows{};
};
[[nodiscard]] Result<Window> window(const Pack& pack, std::string& diagnostic);
void print(const Window& step);

// --- Step 3: training on the preamble (fp1) ---
struct Training
{
    std::string id{};
    float acquisition_frequency_hz{};
    BodyAdaptiveReceiver::PreambleRaceDiagnostics race{};
    float symbol_rate_hz{};
};
[[nodiscard]] Result<Training> training(const Pack& pack, std::string& diagnostic);
void print(const Training& step);

// --- Steps 4 and 5: each engine through fp1's body ---
struct EngineWalk
{
    std::string id{};
    BodyDataEqualizer engine{};
    Body body{};
};
[[nodiscard]] Result<EngineWalk> adaptive(const Pack& pack, std::string& diagnostic);
[[nodiscard]] Result<EngineWalk> cir_mmse(const Pack& pack, std::string& diagnostic);
void print(const EngineWalk& step);

// --- Step 6: when the channel moves (fp1 at 5 Hz, both engines frame by frame) ---
struct Fade
{
    std::string id{};
    Body adaptive{};
    Body cir_mmse{};
    std::vector<std::size_t> widest{}; // the frames where the engines differ most, in order
};
[[nodiscard]] Result<Fade> fade(const Pack& pack, std::string& diagnostic);
void print(const Fade& step);

} // namespace modem_test::equalizer_steps
