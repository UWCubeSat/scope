/**
 * @file types.hpp
 * @brief Plain data passed between SCOPE's stages.
 *
 * Nothing here depends on FOUND's pipeline or on the command line, so the
 * algorithms can be written and tested as plain functions over these types.
 */

#ifndef SRC_SCOPE_COMMON_TYPES_HPP_
#define SRC_SCOPE_COMMON_TYPES_HPP_

#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/projection/projection.hpp"

namespace scope {

/**
 * A single measured star: where a catalog star was actually observed (the raw,
 * distorted pixel) in one of the star images. The star's inertial direction is
 * carried along, so an observation stands on its own without the catalog.
 */
struct Observation {
    /// Index of the star image this observation came from.
    int imageIndex;
    /// The star's catalog identifier (CatalogStar::name, the HR number). Not
    /// part of the measurement model; it lets a residual be traced to a star.
    int starName;
    /// Unit line of sight to the star in the inertial (equatorial J2000) frame:
    /// the e_I of the measurement model (CatalogStar::spatial).
    found::Vec3 inertialDirection;
    /// The measured (distorted) centroid in pixel coordinates [u', v'].
    found::Vec2 measuredPixel;
};

/**
 * The star-centroid stage's output: the measurements and the per-image prior
 * attitudes, which are forwarded for the optimization stage.
 */
struct CentroidObservations {
    /// All star measurements gathered across every star image.
    std::vector<Observation> observations;
    /// One prior attitude per star image; Observation::imageIndex indexes it.
    std::vector<found::Quaternion> attitudes;
};

/**
 * The optimization stage's output: the fitted calibration and how well it fits
 * the observations. A default-constructed value is an empty, unconverged result.
 */
struct CalibrationResult {
    /// The fitted intrinsics and distortion coefficients.
    CameraParameters camera;
    /// One refined attitude per star image, in the order and frame of the prior
    /// attitudes (inertial into camera, boresight on +z).
    std::vector<found::Quaternion> attitudes;
    /// One residual per observation, in observation order: the measured pixel
    /// minus the pixel the fitted model predicts, [du, dv].
    std::vector<found::Vec2> residuals;
    /// Root mean square of the residuals' u and v components pooled together,
    /// in pixels.
    decimal residualRms = DECIMAL(0.0);
    /// Whether the fit converged.
    bool converged = false;
};

}  // namespace scope

#endif  // SRC_SCOPE_COMMON_TYPES_HPP_
