/// @file star-centroid.hpp
/// @brief Star centroid algorithms for SCOPE calibration.
///
/// This header defines the star-centroid pipeline stage: its abstract
/// base and concrete implementations.

#ifndef SRC_SCOPE_STAR_CENTROID_STAR_CENTROID_HPP_
#define SRC_SCOPE_STAR_CENTROID_STAR_CENTROID_HPP_

#include <vector>

#include "common/pipeline/stages.hpp"

#include "scope/command-line/parsing/options.hpp"
#include "scope/common/style.hpp"

namespace scope {

/// Pipeline stage that extracts star centroids from a calibrated image.
class StarCentroidAlgorithm : public found::FunctionStage<Image, std::vector<float>> {
 public:
    StarCentroidAlgorithm() = default;
    virtual ~StarCentroidAlgorithm() {}
};

/// A priori ROI + center-of-intensity star locator. Stub implementation
/// pending the real algorithm.
class ROIFilterAlgorithm : public StarCentroidAlgorithm {
 public:
    ROIFilterAlgorithm() = default;
    explicit ROIFilterAlgorithm([[maybe_unused]] const RecalibrationOptions &options) {}
    ~ROIFilterAlgorithm() override = default;

    std::vector<float> Run(const Image &darkScreen) override;
};

}  // namespace scope

#endif  // SRC_SCOPE_STAR_CENTROID_STAR_CENTROID_HPP_
