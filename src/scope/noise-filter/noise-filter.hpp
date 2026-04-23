#ifndef SRC_SCOPE_NOISE_FILTER_NOISE_FILTER_HPP_
#define SRC_SCOPE_NOISE_FILTER_NOISE_FILTER_HPP_

#include <vector>

#include "common/pipeline/stages.hpp"
#include "common/style.hpp"

namespace scope {

/// Alias for images used by SCOPE noise filtering stages.
using Image = found::Image;
/// Alias for the image collection used as stage input.
using Images = std::vector<Image>;

/**
 * Houses noise filtering algorithms used in SCOPE calibration.
 */
class NoiseFilterAlgorithm : public found::FunctionStage<Images, Image> {
 public:
    NoiseFilterAlgorithm() = default;
    virtual ~NoiseFilterAlgorithm() {}
};

/**
 * Builds a dark-frame estimate using the per-pixel median across input images.
 */
class DarkScreenFilter : public NoiseFilterAlgorithm {
 public:
    /// Constructs a new DarkScreenFilter.
    DarkScreenFilter() = default;
    /// Destroys the filter.
    ~DarkScreenFilter() override = default;

    /**
     * Computes a median dark frame from input images.
     *
     * @param images The set of raw frames used to estimate fixed-pattern noise.
     * @return The median image with newly allocated pixel storage.
     */
    Image Run(const Images& images) override;
};

}  // namespace scope

#endif  // SRC_SCOPE_NOISE_FILTER_NOISE_FILTER_HPP_
