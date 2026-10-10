/**
 * @file noise-filter.hpp
 * @brief Noise filtering algorithms for SCOPE calibration.
 */

#ifndef SRC_SCOPE_NOISE_FILTER_NOISE_FILTER_HPP_
#define SRC_SCOPE_NOISE_FILTER_NOISE_FILTER_HPP_

#include "common/pipeline/stages.hpp"

#include "scope/common/style.hpp"

namespace scope {

/**
 * Reduces a set of frames to one image (a dark frame).
 */
class NoiseFilterAlgorithm : public found::FunctionStage<Images, Image> {};

/**
 * Estimates fixed-pattern noise as the per-pixel median across frames.
 */
class DarkScreenFilter : public NoiseFilterAlgorithm {
 public:
    /**
     * Computes the per-pixel median of the frames (the lower median for an
     * even count).
     *
     * @param images Frames with identical dimensions and channel counts.
     *
     * @return The median image. The caller must std::free its pixel buffer.
     *
     * @throws std::invalid_argument if images is empty.
     * @throws std::runtime_error if an image is null, the dimensions do not
     *         match, or the output buffer cannot be allocated.
     */
    Image Run(const Images &images) override;
};

}  // namespace scope

#endif  // SRC_SCOPE_NOISE_FILTER_NOISE_FILTER_HPP_
