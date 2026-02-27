#pragma once
#include <cstddef>
#include <complex>

namespace reson::core{
    /**
     * @defgroup core Core
     * @brief Core data types shared across the library.
     */

    /**
     * @defgroup dsp DSP
     * @brief Low-level DSP building blocks (FFT, windowing, helpers, Mel filter bank).
     */

    /**
     * @defgroup features Features
     * @brief Higher-level feature extraction modules (e.g., MFCC pipeline).
     */

    /** @ingroup core */
    using Sample = float;
    /** @ingroup core */
    using ComplexSample = std::complex<float>;

    /** @ingroup core */
    constexpr size_t DEFAULT_FRAME_SIZE = 1024;
    /** @ingroup core */
    constexpr float DEFAULT_SAMPLE_RATE = 44100.0f; 
    /** @ingroup core */
    constexpr float PI = 3.14159265359f;



}
 
