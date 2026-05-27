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
 *
 * For an even number of input frames the lower median (n - 1) / 2 is returned.
 */
class DarkScreenFilter : public NoiseFilterAlgorithm {
 public:
    DarkScreenFilter() = default;
    ~DarkScreenFilter() override = default;

    /**
     * Computes the per-pixel median of the input frames. For even-count
     * inputs, returns the lower median.
     *
     * @param images Frames with identical dimensions and channel counts.
     *
     * @return The median image. The caller owns the pixel buffer and must
     *         release it with std::free (matching FOUND's stb_image-allocated
     *         input buffers).
     *
     * @throws std::invalid_argument if images is empty.
     * @throws std::runtime_error if any image is null, has mismatched dimensions,
     *         or if the output buffer cannot be allocated.
     */
    Image Run(const Images &images) override;
};

}  // namespace scope

#endif  // SRC_SCOPE_NOISE_FILTER_NOISE_FILTER_HPP_
