#pragma once
#include <cstddef>
#include <array>
#include "types.hpp"

namespace reson::core{

    /**
     * @ingroup core
     * @brief Fixed-size time-domain audio frame.
     *
     * Lightweight wrapper around `std::array<float, N>`.
     *
     * @tparam N Number of samples in the frame.
     */
    template<size_t N>
    struct Frame{
        std::array<float, N> samples;
        
        float& operator[](size_t i) { return samples[i]; }
        const float& operator[](size_t i) const { return samples[i]; }
        
        size_t length() const { return N; }
    };

}
