/**
 * @file projection.hpp
 * @brief Forward camera projection (pinhole + Brown distortion + intrinsics).
 *
 * Implements the Orion paper's camera model (Christian et al. 2016, §"Camera
 * Model"). The image plane is normalized at z = 1, so the focal length is carried
 * entirely by the intrinsic parameters d_x and d_y (in pixels); these are
 * RecalibrationOptions::focalLengthX / focalLengthY. This is the standard
 * computer-vision convention and matches what the CLI options represent.
 */

#ifndef SRC_SCOPE_PROJECTION_PROJECTION_HPP_
#define SRC_SCOPE_PROJECTION_PROJECTION_HPP_

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/command-line/parsing/options.hpp"

namespace scope {

/**
 * Applies the Brown radial + decentering distortion model (paper Eq. 6) to an
 * ideal (undistorted) image-plane point.
 *
 * @param ideal Ideal pinhole image-plane point [x_i, y_i] (normalized, z = 1).
 * @param k1,k2,k3 Radial distortion coefficients.
 * @param p1,p2 Decentering (tangential) distortion coefficients.
 *
 * @return The distorted image-plane point [x_i', y_i'].
 */
found::Vec2 BrownDistort(const found::Vec2 &ideal, decimal k1, decimal k2, decimal k3, decimal p1, decimal p2);

/**
 * Projects an inertial-frame line-of-sight direction to a raw (distorted) pixel.
 *
 * Full forward model: rotate e_I into the camera frame by the prior attitude
 * (paper Eq. 1 frame), pinhole-project onto the normalized image plane, apply
 * Brown distortion (Eq. 6), then apply the camera intrinsics (Eq. 7).
 *
 * @param eI Unit line-of-sight direction in the inertial frame (a CatalogStar's
 *           spatial vector).
 * @param attitude Prior attitude that rotates a vector from the inertial frame
 *                 into the camera frame (e_C = attitude * e_I). The factory
 *                 currently injects identity; LOST integration will supply real
 *                 per-image attitudes at this seam.
 * @param options Calibration options carrying the prior intrinsics and
 *                distortion coefficients.
 *
 * @return The predicted distorted pixel coordinate [u', v']. Only meaningful
 *         when the star is in front of the camera; callers should reject
 *         out-of-sensor results with InSensorWithMargin.
 */
found::Vec2 ProjectStarToPixel(const found::Vec3 &eI,
                               const found::Quaternion &attitude,
                               const RecalibrationOptions &options);

/**
 * Tests whether a pixel lies inside the sensor with a margin on every side.
 *
 * @param pixel Pixel coordinate [u, v].
 * @param width Image width, in pixels.
 * @param height Image height, in pixels.
 * @param margin Margin to keep clear of every edge. Use (ROI / 2) + 1 so a full
 *               ROI drawn around the pixel never overruns the image.
 *
 * @return true iff pixel is in [margin, width - margin) x [margin, height - margin).
 */
bool InSensorWithMargin(const found::Vec2 &pixel, int width, int height, int margin);

}  // namespace scope

#endif  // SRC_SCOPE_PROJECTION_PROJECTION_HPP_
