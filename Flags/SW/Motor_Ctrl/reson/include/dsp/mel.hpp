#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include "helpers.hpp"

namespace reson::dsp {

/**
 * @ingroup dsp
 * @brief Triangular Mel filter bank projection for power spectra.
 */
class MelFilterBank {
public:
    /**
     * @brief Construct a Mel filter bank.
     * @param sample_rate Input signal sample rate (Hz).
     * @param n_fft FFT size.
     * @param n_mels Number of Mel filters.
     * @param fmin_hz Lower frequency bound (Hz).
     * @param fmax_hz Upper frequency bound (Hz). If <= 0, defaults to Nyquist.
     * @param normalize_by_sum If true, normalize each filter by sum of weights.
     */
    MelFilterBank(int sample_rate, int n_fft, int n_mels,
                  float fmin_hz=0.0f, float fmax_hz=-1.0f,
                  bool normalize_by_sum=true)
        : sample_rate_(sample_rate), n_fft_(n_fft), n_mels_(n_mels),
          fmin_hz_(fmin_hz), fmax_hz_(fmax_hz), normalize_by_sum_(normalize_by_sum)
    {
        if (fmax_hz_ <= 0) fmax_hz_ = sample_rate_/2.0f;
        build_filterbank();
    }

    /**
     * @brief Apply filter bank to a power spectrum.
     * @param power_spectrum Vector of size `n_fft/2 + 1`.
     * @return Vector of Mel energies (size = `n_mels`).
     */
    std::vector<float> apply(const std::vector<float>& power_spectrum) const {
        const size_t n_bins = filterbank_[0].size();
        if (power_spectrum.size() != n_bins)
            throw std::invalid_argument("power_spectrum size mismatch");

        std::vector<float> mel(n_mels_, 0.0f);
        for (int m=0; m<n_mels_; ++m)
            for (size_t k=0; k<n_bins; ++k)
                mel[m] += power_spectrum[k] * filterbank_[m][k];
        return mel;
    }

    /**
     * @brief Access raw filter bank weights.
     */
    const std::vector<std::vector<float>>& get_filterbank() const { return filterbank_; }

private:
    int sample_rate_;
    int n_fft_;
    int n_mels_;
    float fmin_hz_;
    float fmax_hz_;
    bool normalize_by_sum_;
    std::vector<std::vector<float>> filterbank_;

    static double hz_to_mel(double hz) { return 2595.0 * std::log10(1.0 + hz/700.0); }
    static double mel_to_hz(double mel) { return 700.0 * (std::pow(10.0, mel/2595.0)-1.0); }
    static int clamp_int(int v, int lo, int hi) { return std::min(hi,std::max(lo,v)); }

    void build_filterbank() {
        const int n_bins = n_fft_/2 + 1;
        const double mel_min = hz_to_mel(fmin_hz_);
        const double mel_max = hz_to_mel(fmax_hz_);

        std::vector<double> mel_points(n_mels_+2);
        for (int i=0; i<n_mels_+2; ++i)
            mel_points[i] = mel_min + i*(mel_max-mel_min)/(n_mels_+1);

        std::vector<int> bin_points(n_mels_+2);
        for (int i=0;i<n_mels_+2;++i) {
            int bin = static_cast<int>(std::floor(mel_to_hz(mel_points[i]) * n_fft_ / sample_rate_));
            bin_points[i] = clamp_int(bin, 0, n_bins-1);
        }

        filterbank_.assign(n_mels_, std::vector<float>(n_bins,0.0f));

        for (int m=0;m<n_mels_;++m) {
            int left = bin_points[m];
            int center = bin_points[m+1];
            int right = bin_points[m+2];

            if(left==center) center = std::min(center+1, n_bins-1);
            if(center==right) right = std::min(right+1, n_bins-1);

            for(int k=left;k<center;++k)
                filterbank_[m][k] = static_cast<float>((double(k)-left)/double(center-left));

            for(int k=center;k<right;++k)
                filterbank_[m][k] = static_cast<float>((right-double(k))/double(right-center));

            if(normalize_by_sum_) {
                double sum = 0.0;
                for(auto val : filterbank_[m]) sum+=val;
                if(sum>0.0) for(auto &val : filterbank_[m]) val/=sum;
            }
        }
    }
};

}
