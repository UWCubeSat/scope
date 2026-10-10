/**
 * @file types.hpp
 * @brief Plain data passed between SCOPE's stages, independent of FOUND's
 * pipeline and of the command line.
 */

#ifndef SRC_SCOPE_COMMON_TYPES_HPP_
#define SRC_SCOPE_COMMON_TYPES_HPP_

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
    /// One refined attitude per star image, in the prior attitudes' order and frame.
    std::vector<found::Quaternion> attitudes;
    /// Measured minus predicted pixel [du, dv] for each observation, in order.
    std::vector<found::Vec2> residuals;
    /// RMS of the residuals' u and v components pooled together, in pixels.
    decimal residualRms = DECIMAL(0.0);
    /// Whether the fit converged.
    bool converged = false;
};

}  // namespace scope

#endif  // SRC_SCOPE_COMMON_TYPES_HPP_
