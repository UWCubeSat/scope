/**
 * @file star-centroid.hpp
 * @brief Star centroid algorithms for SCOPE calibration.
 */

#ifndef SRC_SCOPE_STAR_CENTROID_STAR_CENTROID_HPP_
#define SRC_SCOPE_STAR_CENTROID_STAR_CENTROID_HPP_

#include <utility>
#include <vector>

#include "common/pipeline/stages.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/catalog/catalog.hpp"
#include "scope/command-line/parsing/options.hpp"
#include "scope/common/style.hpp"

namespace scope {

/**
 * Extracts star centroids from the star-field images, pairing each measured
 * (distorted) pixel with the catalog star it came from.
 */
class StarCentroidAlgorithm : public found::FunctionStage<Image, CentroidObservations> {
 public:
    StarCentroidAlgorithm() = default;
    virtual ~StarCentroidAlgorithm() {}
};

/**
 * A priori ROI + center-of-intensity star locator (Orion paper §"Image
 * Processing and Star Centroiding").
 *
 * For each star image, every catalog star is projected to its expected pixel
 * using the per-image prior attitude and the prior calibration. Stars that fall
 * within the sensor (with ROI margin) are centroided via ExtractCentroid on the
 * dark-subtracted image; successful centroids become Observations.
 */
class ROIFilterAlgorithm : public StarCentroidAlgorithm {
 public:
    /**
     * Constructs a new ROIFilterAlgorithm.
     *
     * @param options Parsed recalibration options (prior intrinsics + distortion,
     *                star images, and centroid threshold). Copied so the stage is
     *                self-contained.
     * @param catalog The star catalog to project and match against. Owned by the
     *                stage; the produced CentroidObservations references it.
     * @param attitudes One prior attitude per star image (rotates inertial
     *                  directions into the camera frame).
     */
    ROIFilterAlgorithm(const RecalibrationOptions &options, Catalog catalog, std::vector<found::Quaternion> attitudes)
        : options_(options), catalog_(std::move(catalog)), attitudes_(std::move(attitudes)) {}

    ~ROIFilterAlgorithm() override = default;

    /**
     * Locates star centroids across all star images.
     *
     * @param darkFrame The dark frame produced by the noise-filter stage; it is
     *                  subtracted from each star image before centroiding.
     *
     * @return The gathered observations, the forwarded attitudes, and a pointer
     *         to this stage's catalog.
     *
     * @throws std::runtime_error if the attitude count does not match the star
     *         image count, or a star image's dimensions do not match the dark
     *         frame.
     */
    CentroidObservations Run(const Image &darkFrame) override;

 private:
    /// Captured calibration options (prior parameters + star images + threshold).
    const RecalibrationOptions options_;
    /// The catalog the produced observations index into.
    const Catalog catalog_;
    /// One prior attitude per star image.
    const std::vector<found::Quaternion> attitudes_;
};

}  // namespace scope

#endif  // SRC_SCOPE_STAR_CENTROID_STAR_CENTROID_HPP_
