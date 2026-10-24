/**
 * @file distortion.hpp
 * @brief Inverse camera model and distortion map.
 *
 * ProjectStarToPixel (projection.hpp) takes a line of sight to a raw pixel.
 * This file goes the other way, from a raw pixel back to an undistorted
 * point, a line of sight or the pixel an ideal pinhole camera would have
 * produced. It also samples the distortion over the sensor, which is the last
 * block of the Orion paper's calibration flow (Christian et al. 2016, Fig. 1,
 * "Compute distortion map"; Fig. 6).
 *
 * As elsewhere in SCOPE, an integer pixel coordinate is the centre of a pixel.
 */

#ifndef SRC_SCOPE_DISTORTION_DISTORTION_HPP_
#define SRC_SCOPE_DISTORTION_DISTORTION_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/projection/projection.hpp"

namespace scope {

/// Default iteration limit for BrownUndistort. Newton's method converges in a
/// handful of steps for any lens a star camera would use.
constexpr int kDefaultUndistortIterations = 20;

/**
 * The Jacobian of BrownDistort with respect to the undistorted point,
 * d[x', y'] / d[x, y] (paper Eqs. 36-37).
 *
 * @param ideal Undistorted point [x, y] on the normalized image plane.
 * @param camera Supplies the distortion coefficients k1, k2, k3, p1 and p2;
 *               its intrinsics are ignored.
 *
 * @return The 2 x 2 Jacobian.
 */
found::Mat2 BrownDistortJacobian(const found::Vec2 &ideal, const CameraParameters &camera);

/**
 * Inverts BrownDistort (paper Eq. 6) by Newton's method, starting from the
 * distorted point.
 *
 * Where the distortion polynomial folds back on itself (the Jacobian's
 * determinant reaches zero), the inverse is not unique; such points are
 * rejected rather than mapped onto the far branch.
 *
 * @param distorted Distorted point [x', y'] on the normalized image plane.
 * @param camera Supplies the distortion coefficients k1, k2, k3, p1 and p2;
 *               its intrinsics are ignored.
 * @param maxIterations The most Newton steps to take.
 *
 * @return The undistorted point [x, y], or std::nullopt if an iterate reached
 *         a point where the Jacobian's determinant is not positive, or the
 *         method did not converge within maxIterations.
 */
std::optional<found::Vec2> BrownUndistort(const found::Vec2 &distorted,
                                          const CameraParameters &camera,
                                          int maxIterations = kDefaultUndistortIterations);

/**
 * Converts a raw pixel to the distorted point on the normalized image plane,
 * inverting the camera calibration matrix (paper Eq. 7).
 *
 * @param pixel Raw pixel [u', v'] (column, row).
 * @param camera Supplies the intrinsics d_x, alpha, d_y, u_p and v_p.
 *
 * @return The distorted image-plane point [x', y'].
 *
 * @throws std::invalid_argument if d_x or d_y is zero.
 */
found::Vec2 PixelToImagePlane(const found::Vec2 &pixel, const CameraParameters &camera);

/**
 * Converts a raw pixel to the unit line of sight it observes, in the camera
 * frame (boresight on +z): the inverse of ProjectStarToPixel with an identity
 * attitude.
 *
 * @param pixel Raw pixel [u', v'] (column, row).
 * @param camera The intrinsics and distortion coefficients.
 *
 * @return The unit line of sight, or std::nullopt if BrownUndistort fails.
 *
 * @throws std::invalid_argument if d_x or d_y is zero.
 */
std::optional<found::Vec3> PixelToLineOfSight(const found::Vec2 &pixel, const CameraParameters &camera);

/**
 * Converts a raw pixel to the pixel at which an undistorted camera with the
 * same intrinsics would have seen the same line of sight.
 *
 * @param pixel Raw pixel [u', v'] (column, row).
 * @param camera The intrinsics and distortion coefficients.
 *
 * @return The undistorted pixel [u, v], or std::nullopt if BrownUndistort
 *         fails.
 *
 * @throws std::invalid_argument if d_x or d_y is zero.
 */
std::optional<found::Vec2> UndistortPixel(const found::Vec2 &pixel, const CameraParameters &camera);

/**
 * One point of a distortion map.
 */
struct DistortionSample {
    /// The raw pixel [u', v'] the sample was taken at.
    found::Vec2 pixel;
    /// UndistortPixel(pixel) - pixel: what to add to a measured pixel to remove
    /// the distortion, in pixels. std::nullopt where UndistortPixel fails.
    std::optional<found::Vec2> correction;
};

/**
 * Samples the distortion correction on a regular grid of raw pixels covering
 * the sensor (paper Fig. 6).
 *
 * Columns are taken at 0, spacing, 2 * spacing, ... and always include the
 * last column, width - 1, so that the corners are sampled; rows likewise. The
 * samples are in row-major order.
 *
 * @param camera The intrinsics and distortion coefficients.
 * @param width Image width, in pixels.
 * @param height Image height, in pixels.
 * @param spacing Grid spacing, in pixels.
 *
 * @return The samples.
 *
 * @throws std::invalid_argument if width, height or spacing is below 1, or
 *         d_x or d_y is zero.
 */
std::vector<DistortionSample> ComputeDistortionMap(const CameraParameters &camera,
                                                   int width,
                                                   int height,
                                                   int spacing);

/**
 * Summary statistics of a distortion map.
 */
struct DistortionSummary {
    /// Largest correction magnitude over the valid samples, in pixels.
    decimal maxMagnitude = DECIMAL(0.0);
    /// Root mean square of the correction magnitudes over the valid samples,
    /// in pixels.
    decimal rmsMagnitude = DECIMAL(0.0);
    /// Number of samples with a correction.
    std::size_t validCount = 0;
    /// Number of samples without one.
    std::size_t invalidCount = 0;
};

/**
 * Summarizes a distortion map.
 *
 * @param samples The map, from ComputeDistortionMap.
 *
 * @return The summary; all zeros if no sample has a correction.
 */
DistortionSummary SummarizeDistortionMap(const std::vector<DistortionSample> &samples);

/**
 * Formats a distortion map as CSV with the header `u,v,du,dv`, one sample per
 * line. A sample without a correction has du and dv left empty.
 *
 * @param samples The map, from ComputeDistortionMap.
 *
 * @return The CSV text, ending in a newline.
 */
std::string FormatDistortionMapCsv(const std::vector<DistortionSample> &samples);

}  // namespace scope

#endif  // SRC_SCOPE_DISTORTION_DISTORTION_HPP_
