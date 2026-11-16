/**
 * @file measurement-model.hpp
 * @brief Partial derivatives of a predicted star centroid, for the fit.
 */

#ifndef SRC_SCOPE_OPTIMIZATION_MEASUREMENT_MODEL_HPP_
#define SRC_SCOPE_OPTIMIZATION_MEASUREMENT_MODEL_HPP_

#include <Eigen/Core>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/projection/projection.hpp"

namespace scope {

/**
 * The Jacobian of one predicted centroid u' (ProjectStarToPixel), split into
 * the columns every image shares and the columns of the star's own image.
 */
struct MeasurementJacobian {
    /// Derivatives with respect to the camera parameters, in
    /// kCameraParameterFields order.
    Eigen::Matrix<decimal, 2, kCameraParameterCount> camera;
    /// Derivatives with respect to the image's attitude correction, at zero.
    Eigen::Matrix<decimal, 2, 3> attitude;
};

/**
 * The Jacobian of ProjectStarToPixel for one star.
 *
 * The attitude columns are for a correction applied as
 * AttitudeCorrection(dtheta) * attitude, which moves the camera-frame
 * direction by e_C x dtheta to first order.
 *
 * @param eC The star's unit line of sight in the camera frame, with the
 *           current attitude applied. Must have z > 0.
 * @param camera The camera parameters to evaluate at.
 *
 * @return The partial derivatives of [u', v'].
 */
MeasurementJacobian ComputeMeasurementJacobian(const found::Vec3 &eC, const CameraParameters &camera);

}  // namespace scope

#endif  // SRC_SCOPE_OPTIMIZATION_MEASUREMENT_MODEL_HPP_
