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
 * Reduces a set of input frames to a single representative image
 * (e.g. a dark frame / fixed-pattern noise estimate).
 */
class NoiseFilterAlgorithm : public found::FunctionStage<Images, Image> {
 public:
    NoiseFilterAlgorithm() = default;
    virtual ~NoiseFilterAlgorithm() {}
};

/**
 * Computes a per-pixel median across frames to estimate fixed-pattern noise.
 */
class DarkScreenFilter : public NoiseFilterAlgorithm {
 public:
    DarkScreenFilter() = default;
    ~DarkScreenFilter() override = default;

    /**
     * Computes the per-pixel median of the input frames.
     *
     * @param images Frames with identical dimensions and channel counts.
     *
     * @return The median image; the caller owns the pixel buffer.
     *
     * @throws std::invalid_argument if images is empty.
     * @throws std::runtime_error if any image is null or has mismatched dimensions.
     */
    Image Run(const Images &images) override;
};

}  // namespace scope

#endif  // SRC_SCOPE_NOISE_FILTER_NOISE_FILTER_HPP_
