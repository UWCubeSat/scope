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
#include "scope/projection/projection.hpp"

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
 * For each star image, every catalog star bright enough to clear the magnitude
 * threshold is projected to its expected pixel using the per-image prior attitude
 * and the prior calibration. Stars behind the camera or outside the sensor (with
 * ROI margin) are skipped; the rest are centroided via ExtractCentroid on the
 * dark-subtracted image, searching a window of options.roiSize pixels around the
 * predicted position. Successful centroids become Observations, except that
 * candidates whose centroids collide within one image (two catalog stars landing
 * on the same blob) are dropped as ambiguous.
 */
class ROIFilterAlgorithm : public StarCentroidAlgorithm {
 public:
    /**
     * Constructs a new ROIFilterAlgorithm.
     *
     * @param options Parsed recalibration options (star images, centroid and
     *                magnitude thresholds, and ROI size). Copied so the stage is
     *                self-contained. Its prior camera fields are not read here;
     *                the prior comes from camera.
     * @param camera The prior intrinsics and distortion coefficients, used to
     *               predict where each catalog star lands.
     * @param catalog The star catalog to project and match against. Owned by the
     *                stage; each observation gets a copy of its star's direction,
     *                so the output does not refer back to it.
     * @param attitudes One prior attitude per star image (rotates inertial
     *                  directions into the camera frame).
     */
    ROIFilterAlgorithm(const RecalibrationOptions &options,
                       const CameraParameters &camera,
                       Catalog catalog,
                       std::vector<found::Quaternion> attitudes)
        : options_(options), camera_(camera), catalog_(std::move(catalog)), attitudes_(std::move(attitudes)) {}

    ~ROIFilterAlgorithm() override = default;

    /**
     * Locates star centroids across all star images.
     *
     * @param darkFrame The dark frame produced by the noise-filter stage; it is
     *                  subtracted from each star image before centroiding.
     *
     * @return The gathered observations and the forwarded attitudes.
     *
     * @throws std::runtime_error if the attitude count does not match the star
     *         image count, the ROI size is below 1, or a star image's dimensions
     *         do not match the dark frame.
     */
    CentroidObservations Run(const Image &darkFrame) override;

 private:
    /// Captured calibration options (star images + thresholds + ROI size).
    const RecalibrationOptions options_;
    /// The prior camera parameters star positions are predicted with.
    const CameraParameters camera_;
    /// The catalog whose stars are projected and matched.
    const Catalog catalog_;
    /// One prior attitude per star image.
    const std::vector<found::Quaternion> attitudes_;
};

}  // namespace scope

#endif  // SRC_SCOPE_STAR_CENTROID_STAR_CENTROID_HPP_
