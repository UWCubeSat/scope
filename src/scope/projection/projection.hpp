/**
 * @file projection.hpp
 * @brief Forward camera projection (pinhole + Brown distortion + intrinsics).
 *
 * The Orion paper's camera model (Christian et al. 2016, §"Camera Model"). The
 * image plane is normalized at z = 1, so the focal length lives entirely in the
 * intrinsics d_x and d_y, in pixels.
 */

#ifndef SRC_SCOPE_PROJECTION_PROJECTION_HPP_
#define SRC_SCOPE_PROJECTION_PROJECTION_HPP_

#include <optional>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

namespace scope {

/**
 * The ten parameters of the paper's camera model: the intrinsics
 * k = [d_x, alpha, d_y, u_p, v_p] (Eq. 13) followed by the Brown distortion
 * coefficients xi = [k1, k2, k3, p1, p2] (Eq. 14), in that order.
 */
struct CameraParameters {
    /// d_x: focal length along u, in pixels.
    decimal focalLengthX = DECIMAL(0.0);
    /// alpha: skew, the contribution of y' to u, in pixels.
    decimal alpha = DECIMAL(0.0);
    /// d_y: focal length along v, in pixels.
    decimal focalLengthY = DECIMAL(0.0);
    /// u_p: principal point column, in pixels.
    decimal principalX = DECIMAL(0.0);
    /// v_p: principal point row, in pixels.
    decimal principalY = DECIMAL(0.0);
    /// Radial distortion coefficient on r^2.
    decimal k1 = DECIMAL(0.0);
    /// Radial distortion coefficient on r^4.
    decimal k2 = DECIMAL(0.0);
    /// Radial distortion coefficient on r^6.
    decimal k3 = DECIMAL(0.0);
    /// First decentering (tangential) coefficient.
    decimal p1 = DECIMAL(0.0);
    /// Second decentering (tangential) coefficient.
    decimal p2 = DECIMAL(0.0);
};

/**
 * Applies Brown radial and decentering distortion (paper Eq. 6).
 *
 * @param ideal Undistorted point [x, y] on the normalized image plane.
 * @param k1,k2,k3 Radial distortion coefficients.
 * @param p1,p2 Decentering (tangential) distortion coefficients.
 *
 * @return The distorted image-plane point [x', y'].
 */
found::Vec2 BrownDistort(const found::Vec2 &ideal, decimal k1, decimal k2, decimal k3, decimal p1, decimal p2);

/**
 * Converts a LOST attitude into SCOPE's camera-frame convention.
 *
 * LOST and FOUND put the boresight on camera +x, with
 * pixel = (c_x - f*y/x, c_y - f*z/x) (FOUND common/spatial/camera.cpp, LOST
 * camera.cpp SpatialToCamera). SCOPE puts it on +z and projects x/z, y/z. Both
 * use the equatorial inertial frame and e_C = attitude * e_I, so they differ
 * only by a fixed rotation of the camera frame. Matching the two pixels for
 * every line of sight forces (x, y, z)_lost -> (-y, -z, x)_scope.
 *
 * A raw LOST quaternion gives plausible but wrong pixels, so every attitude
 * from LOST must pass through here. The --attitudes file and ProvideAttitudes
 * expect SCOPE's frame and do not call this.
 *
 * @param lostAttitude Attitude from LOST, in its x-boresight camera frame.
 *
 * @return The same attitude in SCOPE's z-boresight camera frame.
 */
found::Quaternion LostAttitudeToScopeFrame(const found::Quaternion &lostAttitude);

/**
 * Projects an inertial line of sight to a raw (distorted) pixel: rotate into
 * the camera frame, pinhole-project (paper Eq. 1), distort (Eq. 6), apply the
 * intrinsics (Eq. 7).
 *
 * @param eI Unit line of sight in the inertial frame.
 * @param attitude Rotation from the inertial frame into the camera frame
 *                 (e_C = attitude * e_I), boresight on +z.
 * @param camera The intrinsics and distortion coefficients.
 *
 * @return The predicted pixel [u', v'], or std::nullopt if the star is at or
 *         behind the image plane (camera-frame z <= 0). The pixel may lie
 *         outside the sensor; check it with InSensorWithMargin.
 */
std::optional<found::Vec2> ProjectStarToPixel(const found::Vec3 &eI,
                                              const found::Quaternion &attitude,
                                              const CameraParameters &camera);

/**
 * Tests whether a pixel lies inside the sensor with a margin on every side.
 *
 * @param pixel Pixel coordinate [u, v].
 * @param width Image width, in pixels.
 * @param height Image height, in pixels.
 * @param margin Margin to keep clear of every edge, in pixels.
 *
 * @return true iff pixel is in [margin, width - margin) x [margin, height - margin).
 */
bool InSensorWithMargin(const found::Vec2 &pixel, int width, int height, int margin);

}  // namespace scope

#endif  // SRC_SCOPE_PROJECTION_PROJECTION_HPP_
