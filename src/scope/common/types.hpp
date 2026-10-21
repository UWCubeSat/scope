/**
 * @file types.hpp
 * @brief Plain data passed between SCOPE's stages, independent of FOUND's
 * pipeline and of the command line.
 */

#ifndef SRC_SCOPE_COMMON_TYPES_HPP_
#define SRC_SCOPE_COMMON_TYPES_HPP_

#include <array>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/projection/projection.hpp"

namespace scope {

/**
 * One catalog star as measured in one star image. It carries the star's
 * direction, so it stands on its own without the catalog.
 */
struct Observation {
    /// Index of the star image this came from.
    int imageIndex;
    /// The star's HR number (CatalogStar::name), for tracing a residual to a star.
    int starName;
    /// Unit line of sight to the star in the inertial frame (CatalogStar::spatial).
    found::Vec3 inertialDirection;
    /// The measured (distorted) centroid [u', v'], in pixels.
    found::Vec2 measuredPixel;
};

/**
 * The star-centroid stage's output.
 */
struct CentroidObservations {
    /// The measurements from every star image.
    std::vector<Observation> observations;
    /// One prior attitude per star image, indexed by Observation::imageIndex.
    std::vector<found::Quaternion> attitudes;
};

/**
 * The optimization stage's output. Default-constructed, it is an empty,
 * unconverged result.
 */
struct CalibrationResult {
    /// The fitted intrinsics and distortion coefficients.
    CameraParameters camera;
    /// Which parameters were estimated, in kCameraParameterFields order; the
    /// rest are the prior's.
    std::array<bool, kCameraParameterCount> estimated{};
    /// One-sigma uncertainty of each parameter: zero if fixed, NaN if the fit
    /// could not determine it.
    CameraParameters uncertainty;
    /// One refined attitude per star image, in the prior attitudes' order and
    /// frame.
    std::vector<found::Quaternion> attitudes;
    /// The mean of the per-image corrections refined * prior^-1: with two
    /// cameras, the correction to their alignment.
    found::Quaternion alignmentCorrection = found::Quaternion::Identity();
    /// Measured minus predicted pixel [du, dv] for every observation, in order.
    std::vector<found::Vec2> residuals;
    /// Whether each observation was used in the final fit.
    std::vector<bool> inliers;
    /// RMS of the inliers' residual u and v components pooled together, in pixels.
    decimal residualRms = DECIMAL(0.0);
    /// Levenberg-Marquardt steps taken, over all outlier-rejection rounds.
    int iterations = 0;
    /// Whether the fit converged.
    bool converged = false;
};

}  // namespace scope

#endif  // SRC_SCOPE_COMMON_TYPES_HPP_
