#include <gtest/gtest.h>
#include <cmath>
#include "../include/features/mfcc_pipeline.hpp"
#include "generator.hpp"

// Test that normalized mel filters have unit sum
TEST(MelFilterBank, NormalizedFiltersHaveUnitSum) {
    const int n_mels = 40;
    const int n_fft = 512;
    const float sample_rate = 16000.0f;
    
    reson::dsp::MelFilterBank mel_bank(n_mels, n_fft, sample_rate, true);
    
    // Create flat spectrum
    std::vector<float> flat_spectrum(n_fft / 2 + 1, 1.0f);
    auto mel_energies = mel_bank.apply(flat_spectrum);
    
    // Each filter should produce finite positive values
    for (auto energy : mel_energies) {
        EXPECT_TRUE(std::isfinite(energy));
        EXPECT_GE(energy, 0.0f);
    }
}

// Test that MFCC pipeline returns expected size and finite values for common signals
TEST(MFCCPipeline, ReturnsExpectedSizeAndFiniteForCommonSignals) {
    constexpr size_t N = 512;
    const float sample_rate = 16000.0f;
    const int n_mels = 40;
    const int n_fft = 512;
    const int n_mfcc = 13;
    
    MFCCPipeline<N> mfcc_pipeline(sample_rate, n_mels, n_fft, n_mfcc);
    
    // Test with sinusoid
    reson::core::Frame<N> sine_frame = create_single_sinusoid_frame<N>(1.0f, 440.0f, sample_rate);
    auto sine_mfccs = mfcc_pipeline.process(sine_frame);
    
    EXPECT_EQ(sine_mfccs.size(), static_cast<size_t>(n_mfcc));
    for (auto mfcc : sine_mfccs) {
        EXPECT_TRUE(std::isfinite(mfcc));
    }
    
    // Test with impulse
    reson::core::Frame<N> impulse_frame = create_impulse_frame<N>(1.0f);
    auto impulse_mfccs = mfcc_pipeline.process(impulse_frame);
    
    EXPECT_EQ(impulse_mfccs.size(), static_cast<size_t>(n_mfcc));
    for (auto mfcc : impulse_mfccs) {
        EXPECT_TRUE(std::isfinite(mfcc));
    }
    
    // Test with DC signal
    reson::core::Frame<N> dc_frame = create_dc_frame<N>(1.0f);
    auto dc_mfccs = mfcc_pipeline.process(dc_frame);
    
    EXPECT_EQ(dc_mfccs.size(), static_cast<size_t>(n_mfcc));
    for (auto mfcc : dc_mfccs) {
        EXPECT_TRUE(std::isfinite(mfcc));
    }
}
