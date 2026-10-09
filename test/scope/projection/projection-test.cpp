#include <gtest/gtest.h>

#include <optional>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/command-line/parsing/options.hpp"
#include "scope/projection/projection.hpp"

namespace scope {

namespace {

/// Builds options with the given pinhole parameters and no distortion or skew.
RecalibrationOptions PinholeOptions(decimal focalX, decimal focalY, decimal px, decimal py) {
    RecalibrationOptions options;
    options.focalLengthX = focalX;
    options.focalLengthY = focalY;
    options.principalX = px;
    options.principalY = py;
    options.alpha = DECIMAL(0.0);
    options.k1 = DECIMAL(0.0);
    options.k2 = DECIMAL(0.0);
    options.k3 = DECIMAL(0.0);
    options.p1 = DECIMAL(0.0);
    options.p2 = DECIMAL(0.0);
    return options;
}

const decimal kTol = DECIMAL(1e-4);

// LOST/FOUND forward model with the optical axis on +x: a camera-frame vector
// projects to pixel (c_x - f*y/x, c_y - f*z/x) (FOUND common/spatial/camera.cpp,
// LOST camera.cpp SpatialToCamera). Replicated here so the adapter is pinned
// against LOST's published convention rather than against itself.
found::Vec2 LostPixel(const found::Vec3 &cam, decimal f, decimal cx, decimal cy) {
    return found::Vec2{cx - f * cam.y() / cam.x(), cy - f * cam.z() / cam.x()};
}

}  // namespace

// An on-axis star with identity attitude and no distortion projects to the
// principal point.
TEST(ProjectStarToPixelTest, OnAxisStarMapsToPrincipalPoint) {
    RecalibrationOptions options = PinholeOptions(DECIMAL(100.0), DECIMAL(100.0), DECIMAL(320.0), DECIMAL(240.0));
    found::Vec3 eI(DECIMAL(0.0), DECIMAL(0.0), DECIMAL(1.0));

    std::optional<found::Vec2> pixel = ProjectStarToPixel(eI, found::Quaternion::Identity(), options);

    ASSERT_TRUE(pixel.has_value());
    EXPECT_NEAR(pixel->x(), DECIMAL(320.0), kTol);
    EXPECT_NEAR(pixel->y(), DECIMAL(240.0), kTol);
}

// An off-axis star with no distortion lands at principal + focal * (x/z, y/z).
TEST(ProjectStarToPixelTest, OffAxisStarPinhole) {
    RecalibrationOptions options = PinholeOptions(DECIMAL(100.0), DECIMAL(100.0), DECIMAL(320.0), DECIMAL(240.0));
    found::Vec3 eI(DECIMAL(0.1), DECIMAL(-0.05), DECIMAL(1.0));

    std::optional<found::Vec2> pixel = ProjectStarToPixel(eI, found::Quaternion::Identity(), options);

    ASSERT_TRUE(pixel.has_value());
    EXPECT_NEAR(pixel->x(), DECIMAL(330.0), kTol);  // 100 * 0.1 + 320
    EXPECT_NEAR(pixel->y(), DECIMAL(235.0), kTol);  // 100 * -0.05 + 240
}

// The skew term alpha contributes alpha * y' to the u coordinate.
TEST(ProjectStarToPixelTest, SkewAffectsUCoordinate) {
    RecalibrationOptions options = PinholeOptions(DECIMAL(100.0), DECIMAL(100.0), DECIMAL(320.0), DECIMAL(240.0));
    options.alpha = DECIMAL(10.0);
    found::Vec3 eI(DECIMAL(0.1), DECIMAL(0.2), DECIMAL(1.0));

    std::optional<found::Vec2> pixel = ProjectStarToPixel(eI, found::Quaternion::Identity(), options);

    ASSERT_TRUE(pixel.has_value());
    EXPECT_NEAR(pixel->x(), DECIMAL(332.0), kTol);  // 100 * 0.1 + 10 * 0.2 + 320
    EXPECT_NEAR(pixel->y(), DECIMAL(260.0), kTol);  // 100 * 0.2 + 240
}

// A non-identity attitude rotates the inertial direction into the camera frame
// before projection: here eI = (1,0,0) is rotated onto the boresight.
TEST(ProjectStarToPixelTest, AttitudeRotatesIntoCameraFrame) {
    RecalibrationOptions options = PinholeOptions(DECIMAL(100.0), DECIMAL(100.0), DECIMAL(320.0), DECIMAL(240.0));
    found::Vec3 eI(DECIMAL(1.0), DECIMAL(0.0), DECIMAL(0.0));
    // Rotate -90 deg about +y maps the inertial x-axis onto the camera +z boresight.
    const found::Vec3 yAxis(DECIMAL(0.0), DECIMAL(1.0), DECIMAL(0.0));
    found::Quaternion attitude(found::AngleAxis(-DECIMAL_M_PI_2, yAxis));

    std::optional<found::Vec2> pixel = ProjectStarToPixel(eI, attitude, options);

    ASSERT_TRUE(pixel.has_value());
    EXPECT_NEAR(pixel->x(), DECIMAL(320.0), kTol);
    EXPECT_NEAR(pixel->y(), DECIMAL(240.0), kTol);
}

// A star rotated to sit behind the camera (camera-frame z <= 0) cannot be
// imaged, so the projection reports nullopt rather than a folded-over pixel.
TEST(ProjectStarToPixelTest, RejectsStarBehindCamera) {
    RecalibrationOptions options = PinholeOptions(DECIMAL(100.0), DECIMAL(100.0), DECIMAL(320.0), DECIMAL(240.0));
    // Anti-boresight: directly behind the camera. Without the z-guard this would
    // divide by a negative z and project straight onto the principal point.
    found::Vec3 eI(DECIMAL(0.0), DECIMAL(0.0), DECIMAL(-1.0));

    EXPECT_FALSE(ProjectStarToPixel(eI, found::Quaternion::Identity(), options).has_value());

    // A star 30 deg off the anti-boresight is still behind the image plane.
    found::Vec3 offAxisBehind(DECIMAL(0.5), DECIMAL(0.0), DECIMAL(-1.0));
    EXPECT_FALSE(ProjectStarToPixel(offAxisBehind.normalized(), found::Quaternion::Identity(), options).has_value());
}

// Pure radial distortion (k1 != 0) scales an off-axis point outward.
TEST(BrownDistortTest, PureRadial) {
    // r^2 = 0.01, radial = 1 + 0.5 * 0.01 = 1.005, so x' = 0.1005, y' = 0.
    found::Vec2 distorted = BrownDistort(
        found::Vec2(DECIMAL(0.1), DECIMAL(0.0)), DECIMAL(0.5), DECIMAL(0.0), DECIMAL(0.0), DECIMAL(0.0), DECIMAL(0.0));

    EXPECT_NEAR(distorted.x(), DECIMAL(0.1005), kTol);
    EXPECT_NEAR(distorted.y(), DECIMAL(0.0), kTol);
}

// Decentering distortion (p1, p2 != 0) adds the tangential terms of Eq. 6.
TEST(BrownDistortTest, Decentering) {
    found::Vec2 distorted = BrownDistort(found::Vec2(DECIMAL(0.1), DECIMAL(0.2)),
                                         DECIMAL(0.0),
                                         DECIMAL(0.0),
                                         DECIMAL(0.0),
                                         DECIMAL(0.01),
                                         DECIMAL(0.02));

    // dx = 2*p1*x*y + p2*(r2 + 2x^2) = 0.0004 + 0.02*0.07 = 0.0018 -> x' = 0.1018
    // dy = p1*(r2 + 2y^2) + 2*p2*x*y = 0.01*0.13 + 0.0008 = 0.0021 -> y' = 0.2021
    EXPECT_NEAR(distorted.x(), DECIMAL(0.1018), kTol);
    EXPECT_NEAR(distorted.y(), DECIMAL(0.2021), kTol);
}

// Zero distortion coefficients leave the point unchanged.
TEST(BrownDistortTest, ZeroCoefficientsAreIdentity) {
    found::Vec2 ideal(DECIMAL(0.3), DECIMAL(-0.2));
    found::Vec2 distorted = BrownDistort(ideal, DECIMAL(0.0), DECIMAL(0.0), DECIMAL(0.0), DECIMAL(0.0), DECIMAL(0.0));

    EXPECT_NEAR(distorted.x(), ideal.x(), kTol);
    EXPECT_NEAR(distorted.y(), ideal.y(), kTol);
}

// InSensorWithMargin accepts the inclusive lower edge and rejects the exclusive
// upper edge and anything past the margin.
TEST(InSensorWithMarginTest, EdgeBehavior) {
    const int w = 64;
    const int h = 64;
    const int margin = 16;

    EXPECT_TRUE(InSensorWithMargin(found::Vec2(DECIMAL(16.0), DECIMAL(16.0)), w, h, margin));
    EXPECT_TRUE(InSensorWithMargin(found::Vec2(DECIMAL(32.0), DECIMAL(32.0)), w, h, margin));
    EXPECT_TRUE(InSensorWithMargin(found::Vec2(DECIMAL(47.9), DECIMAL(47.9)), w, h, margin));

    // Just inside the lower margin -> rejected.
    EXPECT_FALSE(InSensorWithMargin(found::Vec2(DECIMAL(15.9), DECIMAL(30.0)), w, h, margin));
    EXPECT_FALSE(InSensorWithMargin(found::Vec2(DECIMAL(30.0), DECIMAL(15.9)), w, h, margin));
    // On the exclusive upper edge -> rejected.
    EXPECT_FALSE(InSensorWithMargin(found::Vec2(DECIMAL(48.0), DECIMAL(30.0)), w, h, margin));
    EXPECT_FALSE(InSensorWithMargin(found::Vec2(DECIMAL(30.0), DECIMAL(48.0)), w, h, margin));
}

// Under an identity LOST attitude the camera frame equals the inertial frame, so
// the adapter alone reconciles LOST's x-boresight with SCOPE's z-boresight. A
// star on the LOST optical axis (inertial +x) must land on the principal point.
TEST(LostAttitudeToScopeFrameTest, OnAxisStarMapsToPrincipalPoint) {
    RecalibrationOptions options = PinholeOptions(DECIMAL(100.0), DECIMAL(100.0), DECIMAL(320.0), DECIMAL(240.0));
    const found::Quaternion attitude = LostAttitudeToScopeFrame(found::Quaternion::Identity());
    found::Vec3 eI(DECIMAL(1.0), DECIMAL(0.0), DECIMAL(0.0));

    std::optional<found::Vec2> pixel = ProjectStarToPixel(eI, attitude, options);

    ASSERT_TRUE(pixel.has_value());
    EXPECT_NEAR(pixel->x(), DECIMAL(320.0), kTol);
    EXPECT_NEAR(pixel->y(), DECIMAL(240.0), kTol);
}

// The adapter must reproduce LOST's exact pixel for off-axis stars, pinning the
// axis permutation AND its signs. Under the identity LOST attitude, a +camera-y
// then a +camera-z inertial nudge must match LOST's SpatialToCamera to the last
// sign -- a swapped or sign-flipped permutation would mirror the field.
TEST(LostAttitudeToScopeFrameTest, MatchesLostForwardModelSigns) {
    const decimal f = DECIMAL(100.0);
    const decimal cx = DECIMAL(320.0);
    const decimal cy = DECIMAL(240.0);
    RecalibrationOptions options = PinholeOptions(f, f, cx, cy);
    const found::Quaternion attitude = LostAttitudeToScopeFrame(found::Quaternion::Identity());

    // +camera-y: LOST sends this toward -u (left); SCOPE must agree.
    found::Vec3 plusY(DECIMAL(1.0), DECIMAL(0.1), DECIMAL(0.0));
    std::optional<found::Vec2> py = ProjectStarToPixel(plusY, attitude, options);
    ASSERT_TRUE(py.has_value());
    found::Vec2 pyExpected = LostPixel(plusY, f, cx, cy);
    EXPECT_NEAR(py->x(), pyExpected.x(), kTol);
    EXPECT_NEAR(py->y(), pyExpected.y(), kTol);

    // +camera-z: LOST sends this toward -v; SCOPE must agree.
    found::Vec3 plusZ(DECIMAL(1.0), DECIMAL(0.0), DECIMAL(0.1));
    std::optional<found::Vec2> pz = ProjectStarToPixel(plusZ, attitude, options);
    ASSERT_TRUE(pz.has_value());
    found::Vec2 pzExpected = LostPixel(plusZ, f, cx, cy);
    EXPECT_NEAR(pz->x(), pzExpected.x(), kTol);
    EXPECT_NEAR(pz->y(), pzExpected.y(), kTol);
}

// A star on the LOST anti-boresight (inertial -x) lands behind SCOPE's image
// plane after conversion (camera-frame z < 0), so the projection rejects it
// rather than folding it onto the sensor.
TEST(LostAttitudeToScopeFrameTest, AntiBoresightIsRejected) {
    RecalibrationOptions options = PinholeOptions(DECIMAL(100.0), DECIMAL(100.0), DECIMAL(320.0), DECIMAL(240.0));
    const found::Quaternion attitude = LostAttitudeToScopeFrame(found::Quaternion::Identity());
    found::Vec3 eI(DECIMAL(-1.0), DECIMAL(0.0), DECIMAL(0.0));

    EXPECT_FALSE(ProjectStarToPixel(eI, attitude, options).has_value());
}

}  // namespace scope
