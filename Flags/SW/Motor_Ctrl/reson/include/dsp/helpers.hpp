#pragma once
#include <cmath>
#include <vector>
#include "../core/types.hpp"
#include "../core/frame.hpp"
#include "../core/spectre.hpp"


namespace reson::dsp {

    /**
     * @ingroup dsp
     * @brief Convert frequency in Hz to Mel scale.
     */
    inline float hz_to_mel(float hz) {
        return 2595.0f * std::log10(1.0f + hz / 700.0f);
    }

    /**
     * @ingroup dsp
     * @brief Convert Mel scale value back to Hz.
     */
    inline float mel_to_hz(float mel) {
        return 700.0f * (std::pow(10.0f, mel / 2595.0f) - 1.0f);
    }

    template<size_t N>
    /**
     * @ingroup dsp
     * @brief Compute power spectrum from a complex spectrum.
     *
     * Returns an array of length `N` where each bin is `|X[k]|^2 / N`.
     */
    std::array<float, N> power_spectrum(const reson::core::Spectre<N>& spec) {
        std::array<float, N> out{};
        for (size_t i = 0; i < N; ++i)
            out[i] = std::norm(spec[i]) / N;
        return out;
    }

    inline int clamp_int(int v,int lo,int hi){ return std::min(hi,std::max(lo,v)); }

    /**
     * @ingroup dsp
     * @brief Apply log compression to Mel energies.
     */
    inline std::vector<float> log_compression(const std::vector<float>& mel){
        std::vector<float> out(mel.size());
        for(size_t i=0;i<mel.size();++i) out[i] = std::log(mel[i]+1e-10f);
        return out;
    }
    
    /**
     * @ingroup dsp
     * @brief Compute orthonormal DCT-II of log-Mel energies.
     * @param log_mel Log-compressed Mel energies.
     * @param n_coeffs Number of coefficients to return.
     * @return Vector of MFCC coefficients (size = `n_coeffs`).
     */
    inline std::vector<float> dct(const std::vector<float>& log_mel, int n_coeffs) {
        int M = log_mel.size();
        std::vector<float> mfcc(n_coeffs,0.0f);
        for(int n=0; n<n_coeffs; ++n) {
            float factor = (n == 0) ? std::sqrt(1.0f / M) : std::sqrt(2.0f / M);
            for(int m=0; m<M; ++m){
                mfcc[n] += log_mel[m] * std::cos(M_PI * n * (m + 0.5) / M);
            }
            mfcc[n] *= factor;
        }
        return mfcc;
    }

}
