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
 * Extracts star centroids from the star images, pairing each measured pixel
 * with the catalog star it came from.
 */
class StarCentroidAlgorithm : public found::FunctionStage<Image, CentroidObservations> {};

/**
 * A priori ROI + center-of-intensity star locator (Orion paper §"Image
 * Processing and Star Centroiding").
 *
 * For each star image, every catalog star brighter than the magnitude threshold
 * is projected with the prior attitude and calibration. Those that land on the
 * sensor (less an ROI margin) are centroided with ExtractCentroid on the
 * dark-subtracted image. Centroids that collide within one image (two catalog
 * stars on the same blob) are dropped as ambiguous.
 */
class ROIFilterAlgorithm : public StarCentroidAlgorithm {
 public:
    /**
     * @param options Supplies the star images, the centroid and magnitude
     *                thresholds and the ROI size. Its camera fields are ignored.
     * @param camera The prior intrinsics and distortion coefficients.
     * @param catalog The star catalog to project.
     * @param attitudes One prior attitude per star image, in SCOPE's camera frame.
     */
    ROIFilterAlgorithm(const RecalibrationOptions &options,
                       const CameraParameters &camera,
                       Catalog catalog,
                       std::vector<found::Quaternion> attitudes)
        : options_(options), camera_(camera), catalog_(std::move(catalog)), attitudes_(std::move(attitudes)) {}

    /**
     * Locates star centroids across all star images.
     *
     * @param darkFrame The dark frame to subtract from each star image.
     *
     * @return The observations, with the prior attitudes forwarded.
     *
     * @throws std::runtime_error if the attitude and star image counts differ,
     *         the ROI size is below 1, or a star image's dimensions do not match
     *         the dark frame.
     */
    CentroidObservations Run(const Image &darkFrame) override;

 private:
    /// Star images, thresholds and ROI size.
    const RecalibrationOptions options_;
    /// The prior camera parameters.
    const CameraParameters camera_;
    /// The stars to project.
    const Catalog catalog_;
    /// One prior attitude per star image.
    const std::vector<found::Quaternion> attitudes_;
};

}  // namespace scope

#endif  // SRC_SCOPE_STAR_CENTROID_STAR_CENTROID_HPP_
