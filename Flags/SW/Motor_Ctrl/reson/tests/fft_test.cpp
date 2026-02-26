#include <gtest/gtest.h>
#include <cmath>
#include <algorithm>
#include "../include/core/frame.hpp"
#include "../include/core/types.hpp"
#include "../include/core/spectre.hpp"
#include "../include/dsp/fft.hpp"
#include "../include/dsp/helpers.hpp"
#include "generator.hpp"

// Test Frame and Spectre indexing
TEST(CoreFrameSpectre, IndexingAndLength) {
    constexpr size_t N = 512;
    reson::core::Frame<N> frame;
    reson::core::Spectre<N> spectre;

    EXPECT_EQ(frame.length(), N);
    // Spectre length varies by implementation

    frame[0] = 1.0f;
    frame[N - 1] = 2.0f;
    EXPECT_FLOAT_EQ(frame[0], 1.0f);
    EXPECT_FLOAT_EQ(frame[N - 1], 2.0f);
}

// Test that impulse at zero gives flat spectrum
TEST(FFT, ImpulseAtZeroGivesFlatSpectrum) {
    constexpr size_t N = 512;
    reson::core::Frame<N> frame;
    reson::core::Spectre<N> spectre;
    
    frame.samples.fill(0.0f);
    frame[0] = 1.0f;

    reson::dsp::FFT<N> fft;
    fft.process(frame, spectre);

    // All bins should have approximately the same magnitude (1.0)
    for (size_t i = 0; i < spectre.length(); ++i) {
        EXPECT_NEAR(std::abs(spectre[i]), 1.0f, 0.01f);
    }
}

// Test that constant signal has energy only at DC
TEST(FFT, ConstantSignalHasDCOnly) {
    constexpr size_t N = 512;
    reson::core::Frame<N> frame;
    reson::core::Spectre<N> spectre;
    
    frame.samples.fill(1.0f);

    reson::dsp::FFT<N> fft;
    fft.process(frame, spectre);

    // DC component should be N
    EXPECT_NEAR(std::abs(spectre[0]), static_cast<float>(N), 0.1f);
    
    // All other bins should be near zero
    for (size_t i = 1; i < spectre.length(); ++i) {
        EXPECT_NEAR(std::abs(spectre[i]), 0.0f, 0.01f);
    }
}

// Test Parseval's theorem with power spectrum
TEST(Helpers, ParsevalHoldsWithPowerSpectrum) {
    constexpr size_t N = 512;
    reson::core::Frame<N> frame = create_single_sinusoid_frame<N>(1.0f, 440.0f, 16000.0f);
    reson::core::Spectre<N> spectre;

    reson::dsp::FFT<N> fft;
    fft.process(frame, spectre);

    float time_domain_energy = 0.0f;
    for (size_t i = 0; i < frame.length(); ++i) {
        time_domain_energy += frame[i] * frame[i];
    }

    float freq_domain_energy = 0.0f;
    auto freq_energies = reson::dsp::power_spectrum(spectre);
    for (const auto& energy : freq_energies) {
        freq_domain_energy += energy;
    }

    // Parseval: time domain energy ≈ frequency domain energy
    EXPECT_NEAR(time_domain_energy, freq_domain_energy, time_domain_energy * 0.01f);
}

// Test mel conversions are rough inverses
TEST(Helpers, MelConversionsRoughInverse) {
    float freq_hz = 1000.0f;
    float mel = reson::dsp::hz_to_mel(freq_hz);
    float freq_back = reson::dsp::mel_to_hz(mel);
    
    EXPECT_NEAR(freq_hz, freq_back, 0.1f);
}

// Test log compression and DCT return finite values
TEST(Helpers, LogCompressionAndDctReturnFinite) {
    std::vector<float> values = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    
    auto logged = reson::dsp::log_compression(values);
    for (auto val : logged) {
        EXPECT_TRUE(std::isfinite(val));
    }

    auto dct_result = reson::dsp::dct(values, 3);
    EXPECT_EQ(dct_result.size(), 3u);
    for (auto val : dct_result) {
        EXPECT_TRUE(std::isfinite(val));
    }
}
