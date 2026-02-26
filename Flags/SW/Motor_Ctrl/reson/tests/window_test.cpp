#include <gtest/gtest.h>
#include <cmath>
#include "../include/core/frame.hpp"
#include "../include/core/types.hpp"
#include "../include/dsp/window.hpp"
#include "generator.hpp"

// Test that Hann window has zero endpoints when applied to ones
TEST(Window, HannHasZeroEndpointsOnOnes) {
    constexpr size_t N = 512;
    reson::core::Frame<N> frame;
    frame.samples.fill(1.0f);

    reson::dsp::Window<N> hann_window(reson::dsp::WindowType::Hann);
    hann_window.apply_window(frame);

    // Hann window should have zero at endpoints
    EXPECT_NEAR(frame[0], 0.0f, 1e-5f);
    EXPECT_NEAR(frame[N - 1], 0.0f, 1e-5f);
    
    // Middle should be close to 1.0
    EXPECT_NEAR(frame[N / 2], 1.0f, 0.01f);
}

// Test that Hamming window has ~0.08 endpoints when applied to ones
TEST(Window, HammingHasApprox008EndpointsOnOnes) {
    constexpr size_t N = 512;
    reson::core::Frame<N> frame;
    frame.samples.fill(1.0f);

    reson::dsp::Window<N> hamming_window(reson::dsp::WindowType::Hamming);
    hamming_window.apply_window(frame);

    // Hamming window should have ~0.08 at endpoints
    EXPECT_NEAR(frame[0], 0.08f, 0.01f);
    EXPECT_NEAR(frame[N - 1], 0.08f, 0.01f);
    
    // Middle should be close to 1.0
    EXPECT_NEAR(frame[N / 2], 1.0f, 0.01f);
}

// Test that apply_window scales samples properly
TEST(Window, ApplyWindowScalesSamples) {
    constexpr size_t N = 512;
    reson::core::Frame<N> frame = create_single_sinusoid_frame<N>(1.0f, 440.0f, 16000.0f);
    reson::core::Frame<N> original_frame = frame;

    reson::dsp::Window<N> hann_window(reson::dsp::WindowType::Hann);
    hann_window.apply_window(frame);

    // After windowing, samples should be scaled
    // Endpoints should be reduced to near zero
    
    // Energy should be reduced overall
    float original_energy = 0.0f;
    float windowed_energy = 0.0f;
    for (size_t i = 0; i < N; ++i) {
        original_energy += original_frame[i] * original_frame[i];
        windowed_energy += frame[i] * frame[i];
    }
    EXPECT_LT(windowed_energy, original_energy);
}
