/**
 * @file star-centroid.hpp
 * @brief Star centroid algorithms for SCOPE calibration.
 */

#ifndef SRC_SCOPE_STAR_CENTROID_STAR_CENTROID_HPP_
#define SRC_SCOPE_STAR_CENTROID_STAR_CENTROID_HPP_

#include <vector>

#include "common/pipeline/stages.hpp"

#include "scope/command-line/parsing/options.hpp"
#include "scope/common/style.hpp"

namespace scope {

/**
 * Extracts star centroids from a calibrated image.
 */
class StarCentroidAlgorithm : public found::FunctionStage<Image, std::vector<float>> {
 public:
    StarCentroidAlgorithm() = default;
    virtual ~StarCentroidAlgorithm() {}
};

/**
 * A priori ROI + center-of-intensity star locator. Stub pending the real algorithm.
 */
class ROIFilterAlgorithm : public StarCentroidAlgorithm {
 public:
    ROIFilterAlgorithm() = default;

    /**
     * Constructs a new ROIFilterAlgorithm.
     *
     * @param options Parsed recalibration options (currently unused).
     */
    explicit ROIFilterAlgorithm([[maybe_unused]] const RecalibrationOptions &options) {}

    ~ROIFilterAlgorithm() override = default;

    /**
     * Locates star centroids in the given image.
     *
     * @param darkScreen Image produced by the noise-filter stage.
     *
     * @return Flattened centroid coordinates. The stub returns an empty vector.
     */
    std::vector<float> Run(const Image &darkScreen) override;
};

}  // namespace scope

#endif  // SRC_SCOPE_STAR_CENTROID_STAR_CENTROID_HPP_
