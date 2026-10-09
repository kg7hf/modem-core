// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Paul R. Decker (KG7HF)

#include "tests/support/test_waveform.hpp"

#include "modem/common/convert.hpp"
#include "tests/support/impairments.hpp"

#include <span>
#include <utility>

namespace modem_test
{

using ::modem::common::narrow;
using ::modem::common::Result;

Result<TestWaveform> make_test_waveform(const WaveformSpec& spec)
{
    using namespace ::modem::m110a;
    constexpr std::uint64_t low_byte = 0xFFU;
    TestWaveform waveform;
    DeterministicRng rng{spec.seed};

    for (std::size_t burst = 0U; burst < spec.bursts; ++burst)
    {
        BurstTruth truth;
        truth.payload = spec.payload;

        if (truth.payload.empty())
        {
            truth.payload.resize(spec.random_payload_bytes);

            for (auto& byte : truth.payload)
            {
                byte = narrow<std::uint8_t>(rng.next_u64() & low_byte);
            }
        }

        const auto plan_result = body_transmission_plan(spec.mode, truth.payload.size());

        if (!plan_result)
        {
            return plan_result.error();
        }

        const auto& plan = plan_result.value();
        std::vector<std::uint8_t> framed_bits(plan.framed_bits);
        std::vector<std::uint8_t> coded_bits(plan.block.coded_bits);
        std::vector<std::uint8_t> interleaver_matrix(plan.block.coded_bits);
        std::vector<std::uint8_t> interleaved_bits(plan.block.coded_bits);
        truth.tribits.assign(plan.transmitted_symbols, 0U);
        truth.preamble_symbols = plan.preamble_symbols;

        if (burst > 0U)
        {
            waveform.audio.insert(waveform.audio.end(), spec.gap_samples, 0.0F);
        }

        truth.first_sample = waveform.audio.size();
        waveform.audio.resize(truth.first_sample + plan.audio_samples);

        const BodyTransmissionScratch scratch{
            .framed_bits = framed_bits,
            .transmitted_tribits = truth.tribits,
            .coded_bits = coded_bits,
            .interleaver_matrix = interleaver_matrix,
            .interleaved_bits = interleaved_bits,
        };
        const auto audio = std::span<float>{waveform.audio}.subspan(truth.first_sample, plan.audio_samples);
        const auto status = generate_body_transmission_audio(plan, truth.payload, scratch, audio);

        if (!status.is_ok())
        {
            return status;
        }

        waveform.bursts.push_back(std::move(truth));
    }

    return waveform;
}

} // namespace modem_test
