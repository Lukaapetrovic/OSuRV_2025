#pragma once
#include <array>
#include <cstddef>
#include "../core/frame.hpp"
#include "../core/types.hpp"


namespace reson::dsp{
    
/**
 * @ingroup dsp
 * @brief Supported window functions.
 */
enum WindowType{
    Hann,
    Hamming
};

template<size_t N>
/**
 * @ingroup dsp
 * @brief Windowing function applied to a `Frame<N>`.
 *
 * Windowing reduces spectral leakage before FFT.
 *
 * @tparam N Frame size.
 */
class Window{
        
public:

    explicit Window(WindowType type_) : type(type_) {
        compute_coefficients();
    }

    /**
     * @brief Multiply the frame samples by the window coefficients.
     * @param frame Frame to be modified in-place.
     */
    void apply_window(reson::core::Frame<N>& frame) const{
        for(size_t i = 0; i < N; i++){
            frame[i] *= coeffs[i];
        }
    }

private:
    WindowType type;
    std::array<reson::core::Sample, N> coeffs;

    void compute_coefficients(){

        if (N == 0) {
            return;
        }
        if (N == 1) {
            coeffs[0] = 0.0f;
            return;
        }
        if (N == 2) {
            coeffs[0] = 0.0f;
            coeffs[1] = 0.0f;
            return;
        }


        switch(type) {
            case WindowType::Hann:

                for(size_t n = 0; n < N; n++){
                    coeffs[n] = 0.5f * (1.0f - std::cos(2.0f * core::PI * n / (N - 1))); 
                }
            
            break;

            case WindowType::Hamming:
                
                for(size_t n = 0; n < N; n++){
                    coeffs[n] = 0.54f  - 0.46f * std::cos(2.0f * core::PI * n / (N - 1));; 
                }

            break;

            default:
                coeffs.fill(1.0f);
            break;
        }

    }
    

};


}