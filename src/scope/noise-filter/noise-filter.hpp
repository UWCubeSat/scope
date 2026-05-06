/// @file noise-filter.hpp
/// @brief Noise filtering algorithms for SCOPE calibration.
///
/// This header defines the interface for noise filtering stages in SCOPE's
/// image calibration pipeline.

#ifndef SRC_SCOPE_NOISE_FILTER_NOISE_FILTER_HPP_
#define SRC_SCOPE_NOISE_FILTER_NOISE_FILTER_HPP_

#include <vector>

#include "common/pipeline/stages.hpp"
#include "common/style.hpp"

namespace scope {

using Image = found::Image;
using Images = std::vector<Image>;

/// Base class for noise filtering algorithms.
class NoiseFilterAlgorithm : public found::FunctionStage<Images, Image> {
 public:
    NoiseFilterAlgorithm() = default;
    virtual ~NoiseFilterAlgorithm() {}
};

/// Computes a per-pixel median filter to estimate fixed-pattern noise.
class DarkScreenFilter : public NoiseFilterAlgorithm {
 public:
    DarkScreenFilter() = default;
    ~DarkScreenFilter() override = default;

    /// @param images Raw frames with identical dimensions and channel counts.
    /// @return The median image with newly allocated pixel storage.
    ///         The caller must free the image buffer.
    /// @throws std::invalid_argument if images collection is empty.
    /// @throws std::runtime_error if images have mismatched dimensions.
    Image Run(const Images& images) override;
};

}  // namespace scope

#endif  // SRC_SCOPE_NOISE_FILTER_NOISE_FILTER_HPP_
