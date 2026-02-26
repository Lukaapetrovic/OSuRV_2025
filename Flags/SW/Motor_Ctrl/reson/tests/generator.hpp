#include <cmath>
#include <array>
#include <vector>
#include <cstddef>
#include <random>
#include "../include/core/frame.hpp"
#include "../include/core/types.hpp"
#include "../include/core/spectre.hpp"


template<size_t N>
reson::core::Frame<N> create_single_sinusoid_frame(float amplitude, float frequency, float sample_rate) {
    reson::core::Frame<N> frame;
    for (size_t i = 0; i < N; ++i) {
        frame[i] = amplitude * std::sin(2 * reson::core::PI * frequency * i / sample_rate);
    }
    return frame;
}

template <size_t N>
reson::core::Frame<N> create_sum_sinusoids_frame(const std::vector<float>& freqs, float amplitude, float sampleRate) {
  reson::core::Frame<N> frame;
  frame.samples.fill(0.0f);
  for (float f : freqs) {
      for (size_t n = 0; n < N; ++n) {
          frame[n] += amplitude * std::sin(2.0 * M_PI * f * n / sampleRate);
      }
  }
  return frame;
}

template<size_t N>
reson::core::Frame<N> create_impulse_frame(float amplitude) {
    reson::core::Frame<N> frame;
    frame.samples.fill(0.0f);
    frame[N / 2] = amplitude;
    return frame;
}

template<size_t N>
reson::core::Frame<N> create_dc_frame(float amplitude) {
    reson::core::Frame<N> frame;
    frame.samples.fill(amplitude);
    return frame;
}

template<size_t N>
reson::core::Frame<N> create_zero_frame() {
    reson::core::Frame<N> frame;
    frame.samples.fill(0.0f);
    return frame;
}

template<size_t N>
reson::core::Frame<N> create_ramp_frame(float start = 0.0f, float end = 1.0f) {
    reson::core::Frame<N> frame;
    float step = (end - start) / (N - 1);
    for (size_t i = 0; i < N; ++i) {
        frame[i] = start + i * step;
    }
    return frame;
}

template<size_t N>
reson::core::Frame<N> create_white_noise_frame(float amplitude) {
    reson::core::Frame<N> frame;
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(-amplitude, amplitude);
    for (size_t i = 0; i < N; ++i) {
        frame[i] = distribution(generator);
    }
    return frame;
}