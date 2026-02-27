#pragma once
#include <cstddef>
#include <array>
#include <complex>

namespace reson::core{

    /**
     * @ingroup core
     * @brief Fixed-size complex frequency-domain spectrum.
     *
     * Produced by `reson::dsp::FFT<N>`.
     *
     * @tparam N Number of complex FFT bins.
     */
    template<size_t N>
    struct Spectre{
        std::array<std::complex<float>, N> bins;

        Spectre() = default;

        std::complex<float>& operator[](size_t i) { return bins[i]; }
        const std::complex<float>& operator[](size_t i) const { return bins[i]; }

        size_t length() const { return N; }
    };
}