#pragma once
#include "../core/frame.hpp"
#include "../core/spectre.hpp"
#include "../core/types.hpp"
#include "../dsp/fft.hpp"
#include "../dsp/helpers.hpp"
#include "../dsp/window.hpp"
#include "../dsp/mel.hpp"


template<size_t N>
/**
 * @ingroup features
 * @brief End-to-end MFCC feature extraction pipeline for a single frame.
 *
 * Steps:
 * - windowing (Hann)
 * - FFT
 * - power spectrum
 * - Mel filter bank
 * - log compression
 * - DCT (keep first `n_mfcc`)
 *
 * @tparam N Frame size.
 */
class MFCCPipeline {
    public:
      
        MFCCPipeline(int sample_rate, int n_mels, int n_fft, int n_mfcc, int fmin_hz=0, int fmax_hz=-1)
            : fft_(),
              window_(reson::dsp::WindowType::Hann),
              mel_filter_bank_(sample_rate, n_fft, n_mels, fmin_hz, fmax_hz, true),
              n_mfcc_(n_mfcc),
              sample_rate_(sample_rate),
              n_mels_(n_mels),
              n_fft_(n_fft),
              fmin_hz_(fmin_hz),
              fmax_hz_(fmax_hz)
        {
          if(fmax_hz_ == -1) {
              fmax_hz_ = sample_rate_ / 2;
          }

        }

        /**
         * @brief Run MFCC extraction on one frame.
         * @param frame Input time-domain frame.
         * @return Vector of MFCC coefficients (size = `n_mfcc`).
         */
        std::vector<float> process(const reson::core::Frame<N>& frame) {
            reson::core::Frame<N> windowed_frame = frame;
            window_.apply_window(windowed_frame);

            reson::core::Spectre<N> spectre;
            fft_.process(windowed_frame, spectre);

            auto power_spec = reson::dsp::power_spectrum(spectre);
            std::vector<float> power_spec_vec(power_spec.begin(), power_spec.begin() + N/2 + 1);

            auto mel_energies = mel_filter_bank_.apply(power_spec_vec);
            auto log_mel = reson::dsp::log_compression(mel_energies);
            auto mfccs = reson::dsp::dct(log_mel, n_mfcc_);

            return mfccs;
        }

    private:
        reson::dsp::FFT<N> fft_;
        reson::dsp::Window<N> window_;
        reson::dsp::MelFilterBank mel_filter_bank_;
        int n_mfcc_;
        int sample_rate_;
        int n_mels_;
        int n_fft_;
        int fmin_hz_;
        int fmax_hz_;
};