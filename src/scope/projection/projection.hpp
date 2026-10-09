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

#include <optional>

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
 * Converts a LOST attitude into SCOPE's camera-frame convention.
 *
 * LOST and FOUND put the optical axis (boresight) on camera +x: their forward
 * model is pixel = (c_x - f*y/x, c_y - f*z/x) and they assert x > 0 (see FOUND
 * common/spatial/camera.cpp and LOST camera.cpp SpatialToCamera). SCOPE's
 * ProjectStarToPixel instead puts the boresight on +z and projects x/z, y/z.
 * Both share the equatorial inertial frame and both apply the attitude as
 * e_C = attitude * e_I, so the conventions differ only by a fixed rotation of the
 * camera frame. Requiring SCOPE's pixel to equal LOST's for every line of sight
 * forces the signed permutation (x, y, z)_lost -> (-y, -z, x)_scope.
 *
 * Feeding a raw LOST quaternion to ProjectStarToPixel without this conversion
 * produces plausible-but-wrong pixels (the boresight star projects to z = 0 and
 * is dropped, while stars 90 deg off-axis are accepted), so every per-image
 * attitude arriving from LOST MUST pass through here first. This is the loud
 * counterpart to the LOST-integration seam in providers/factory.hpp.
 *
 * @param lostAttitude Attitude from LOST: rotates an inertial line of sight into
 *                     LOST's x-boresight camera frame (e_C = lostAttitude * e_I).
 *
 * @return The equivalent attitude in SCOPE's z-boresight camera frame, ready to
 *         hand to ProjectStarToPixel.
 */
found::Quaternion LostAttitudeToScopeFrame(const found::Quaternion &lostAttitude);

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
 * @return The predicted distorted pixel coordinate [u', v'], or std::nullopt if
 *         the star is at or behind the image plane (camera-frame z <= 0) and so
 *         cannot be imaged. A returned pixel is not guaranteed in-sensor; callers
 *         must still reject out-of-frame results with InSensorWithMargin.
 */
std::optional<found::Vec2> ProjectStarToPixel(const found::Vec3 &eI,
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
