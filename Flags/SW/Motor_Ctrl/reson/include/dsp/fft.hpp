#pragma once
#include <array>
#include <complex>
#include <cmath>
#include "../core/frame.hpp"
#include "../core/spectre.hpp"
#include "../core/types.hpp"


namespace reson::dsp{

template<size_t N>

/**
 * @ingroup dsp
 * @brief Radix-2 Cooley–Tukey FFT.
 *
 * Converts a real input frame (`reson::core::Frame<N>`) into a complex spectrum
 * (`reson::core::Spectre<N>`).
 *
 * @tparam N FFT size (must be a power of 2).
 */
class FFT{

public:

    FFT() {
        compute_twiddle();
    }

    /**
     * @brief Compute the FFT of a single frame.
     * @param in Input time-domain frame.
     * @param out Output complex spectrum.
     */
    void process(const core::Frame<N>& in, core::Spectre<N>& out) const {

        for(size_t i = 0; i < N; i++){
            buffer[i] = { in[i], 0.0f };
        }
        
        bit_reverse(buffer);

        compute_fft(buffer);

        for(size_t i = 0; i < N; i ++){
            out[i] = buffer[i];
        }
    }

private:

    using Complex = core::ComplexSample;
    mutable std::array<Complex, N> buffer;
    std::array<Complex, N/2> twiddle;

    void compute_twiddle(){
        for(size_t i = 0; i < N / 2; i++){
            float angle = -2.0f * core::PI * i / N;
            twiddle[i] = { std::cos(angle), std::sin(angle) };
        }
    }

    void bit_reverse(std::array<Complex, N>& data) const {
        size_t j = 0;
        for (size_t i = 1; i < N; ++i) {
            size_t bit = N >> 1;
            while (j & bit) {
                j ^= bit;
                bit >>= 1;
            }
            j |= bit;
            if (i < j)
                std::swap(data[i], data[j]);
        }
    }

    void compute_fft(std::array<Complex, N>& data) const {
        for (size_t len = 2; len <= N; len <<= 1) {
            size_t half = len >> 1;
            size_t step = N / len;

            for (size_t i = 0; i < N; i += len) {
                for (size_t j = 0; j < half; ++j) {
                    const auto& w = twiddle[j * step];
                    Complex u = data[i + j];
                    Complex v = data[i + j + half] * w;

                    data[i + j] = u + v;
                    data[i + j + half] = u - v;
                }
            }
        }
    }

};


}

