#ifndef TEST_SCOPE_COMMON_CAMERAS_HPP_
#define TEST_SCOPE_COMMON_CAMERAS_HPP_

#include "common/decimal.hpp"

#include "scope/projection/projection.hpp"

namespace scope {

/// The candidate sensor (XIMEA MC031MG-SY-FL, Sony IMX252).
constexpr int kSensorWidth = 2064;
constexpr int kSensorHeight = 1544;

/// The paper's Monte Carlo sensor: 2592 x 2048 pixels, 20 x 16 degrees.
constexpr int kPaperSensorWidth = 2592;
constexpr int kPaperSensorHeight = 2048;

/// Stage 2's edge margin at the default 31-pixel window.
constexpr int kDefaultSearchMargin = 16;

/// The Edmund #35-142 lens's datasheet values on the candidate sensor
/// (LENS.md §4), the prior a first calibration would start from.
inline CameraParameters DatasheetCamera() {
    CameraParameters camera;
    camera.focalLengthX = DECIMAL(1739.13);
    camera.focalLengthY = DECIMAL(1739.13);
    camera.principalX = DECIMAL(1031.5);
    camera.principalY = DECIMAL(771.5);
    camera.k1 = DECIMAL(-0.157);
    camera.k2 = DECIMAL(0.082);
    return camera;
}

/// A plausible as-built unit of the same lens, with every parameter off the
/// datasheet's and none zero, for fits to recover.
inline CameraParameters AsBuiltCamera() {
    CameraParameters camera;
    camera.focalLengthX = DECIMAL(1745.0);
    camera.alpha = DECIMAL(0.4);
    camera.focalLengthY = DECIMAL(1744.2);
    camera.principalX = DECIMAL(1031.5);
    camera.principalY = DECIMAL(771.5);
    camera.k1 = DECIMAL(-0.152);
    camera.k2 = DECIMAL(0.077);
    camera.k3 = DECIMAL(0.004);
    camera.p1 = DECIMAL(1e-4);
    camera.p2 = DECIMAL(-5e-5);
    return camera;
}

/// The camera of the paper's Monte Carlo (Table 1, "Actual Value"). Table 1
/// has no principal point, so it sits at the image centre.
inline CameraParameters PaperCamera() {
    CameraParameters camera;
    camera.focalLengthX = DECIMAL(7308.2);
    camera.alpha = DECIMAL(-1.4);
    camera.focalLengthY = DECIMAL(7307.2);
    camera.principalX = DECIMAL(1295.5);
    camera.principalY = DECIMAL(1023.5);
    camera.k1 = DECIMAL(-0.075);
    camera.k2 = DECIMAL(0.883);
    camera.k3 = DECIMAL(-12.60);
    camera.p1 = DECIMAL(0.0012);
    camera.p2 = DECIMAL(0.0016);
    return camera;
}

}  // namespace scope

#endif  // TEST_SCOPE_COMMON_CAMERAS_HPP_
