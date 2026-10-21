#include <gtest/gtest.h>

#include <cmath>
#include <optional>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/projection/projection.hpp"

namespace scope {

namespace {

/// Builds camera parameters with the given pinhole parameters and no distortion or skew.
CameraParameters PinholeCamera(decimal focalX, decimal focalY, decimal px, decimal py) {
    CameraParameters camera;
    camera.focalLengthX = focalX;
    camera.focalLengthY = focalY;
    camera.principalX = px;
    camera.principalY = py;
    camera.alpha = DECIMAL(0.0);
    camera.k1 = DECIMAL(0.0);
    camera.k2 = DECIMAL(0.0);
    camera.k3 = DECIMAL(0.0);
    camera.p1 = DECIMAL(0.0);
    camera.p2 = DECIMAL(0.0);
    return camera;
}

const decimal kTol = DECIMAL(1e-4);

// LOST/FOUND forward model, boresight on +x: pixel = (c_x - f*y/x, c_y - f*z/x)
// (FOUND common/spatial/camera.cpp, LOST camera.cpp SpatialToCamera). Replicated
// so the adapter is tested against LOST's convention, not against itself.
found::Vec2 LostPixel(const found::Vec3 &cam, decimal f, decimal cx, decimal cy) {
    return found::Vec2{cx - f * cam.y() / cam.x(), cy - f * cam.z() / cam.x()};
}

}  // namespace

TEST(ProjectStarToPixelTest, OnAxisStarMapsToPrincipalPoint) {
    CameraParameters camera = PinholeCamera(DECIMAL(100.0), DECIMAL(100.0), DECIMAL(320.0), DECIMAL(240.0));
    found::Vec3 eI(DECIMAL(0.0), DECIMAL(0.0), DECIMAL(1.0));

    std::optional<found::Vec2> pixel = ProjectStarToPixel(eI, found::Quaternion::Identity(), camera);

    ASSERT_TRUE(pixel.has_value());
    EXPECT_NEAR(pixel->x(), DECIMAL(320.0), kTol);
    EXPECT_NEAR(pixel->y(), DECIMAL(240.0), kTol);
}

// An off-axis star with no distortion lands at principal + focal * (x/z, y/z).
TEST(ProjectStarToPixelTest, OffAxisStarPinhole) {
    CameraParameters camera = PinholeCamera(DECIMAL(100.0), DECIMAL(100.0), DECIMAL(320.0), DECIMAL(240.0));
    found::Vec3 eI(DECIMAL(0.1), DECIMAL(-0.05), DECIMAL(1.0));

    std::optional<found::Vec2> pixel = ProjectStarToPixel(eI, found::Quaternion::Identity(), camera);

    ASSERT_TRUE(pixel.has_value());
    EXPECT_NEAR(pixel->x(), DECIMAL(330.0), kTol);  // 100 * 0.1 + 320
    EXPECT_NEAR(pixel->y(), DECIMAL(235.0), kTol);  // 100 * -0.05 + 240
}

// The skew term alpha contributes alpha * y' to the u coordinate.
TEST(ProjectStarToPixelTest, SkewAffectsUCoordinate) {
    CameraParameters camera = PinholeCamera(DECIMAL(100.0), DECIMAL(100.0), DECIMAL(320.0), DECIMAL(240.0));
    camera.alpha = DECIMAL(10.0);
    found::Vec3 eI(DECIMAL(0.1), DECIMAL(0.2), DECIMAL(1.0));

    std::optional<found::Vec2> pixel = ProjectStarToPixel(eI, found::Quaternion::Identity(), camera);

    ASSERT_TRUE(pixel.has_value());
    EXPECT_NEAR(pixel->x(), DECIMAL(332.0), kTol);  // 100 * 0.1 + 10 * 0.2 + 320
    EXPECT_NEAR(pixel->y(), DECIMAL(260.0), kTol);  // 100 * 0.2 + 240
}

// Here eI = (1, 0, 0) is rotated onto the boresight.
TEST(ProjectStarToPixelTest, AttitudeRotatesIntoCameraFrame) {
    CameraParameters camera = PinholeCamera(DECIMAL(100.0), DECIMAL(100.0), DECIMAL(320.0), DECIMAL(240.0));
    found::Vec3 eI(DECIMAL(1.0), DECIMAL(0.0), DECIMAL(0.0));
    // Rotate -90 deg about +y maps the inertial x-axis onto the camera +z boresight.
    const found::Vec3 yAxis(DECIMAL(0.0), DECIMAL(1.0), DECIMAL(0.0));
    found::Quaternion attitude(found::AngleAxis(-DECIMAL_M_PI_2, yAxis));

    std::optional<found::Vec2> pixel = ProjectStarToPixel(eI, attitude, camera);

    ASSERT_TRUE(pixel.has_value());
    EXPECT_NEAR(pixel->x(), DECIMAL(320.0), kTol);
    EXPECT_NEAR(pixel->y(), DECIMAL(240.0), kTol);
}

TEST(ProjectStarToPixelTest, RejectsStarBehindCamera) {
    CameraParameters camera = PinholeCamera(DECIMAL(100.0), DECIMAL(100.0), DECIMAL(320.0), DECIMAL(240.0));
    // Anti-boresight: without the z-guard this lands on the principal point.
    found::Vec3 eI(DECIMAL(0.0), DECIMAL(0.0), DECIMAL(-1.0));

    EXPECT_FALSE(ProjectStarToPixel(eI, found::Quaternion::Identity(), camera).has_value());

    // A star 30 deg off the anti-boresight is still behind the image plane.
    found::Vec3 offAxisBehind(DECIMAL(0.5), DECIMAL(0.0), DECIMAL(-1.0));
    EXPECT_FALSE(ProjectStarToPixel(offAxisBehind.normalized(), found::Quaternion::Identity(), camera).has_value());
}

// The paper's Table 1 lens (k3 = -12.6) folds back about 26 degrees off axis,
// so a star at 32 degrees, far outside its 12.8 degree half-diagonal, would
// land back on the sensor. It is rejected; one inside the field is not.
TEST(ProjectStarToPixelTest, RejectsStarPastDistortionFold) {
    CameraParameters camera = PinholeCamera(DECIMAL(7308.2), DECIMAL(7307.2), DECIMAL(1295.5), DECIMAL(1023.5));
    camera.k1 = DECIMAL(-0.075);
    camera.k2 = DECIMAL(0.883);
    camera.k3 = DECIMAL(-12.60);
    const auto offAxis = [](decimal degrees) {
        const decimal angle = found::DegToRad(degrees);
        return found::Vec3(DECIMAL_SIN(angle), DECIMAL(0.0), DECIMAL_COS(angle));
    };

    EXPECT_TRUE(ProjectStarToPixel(offAxis(DECIMAL(10.0)), found::Quaternion::Identity(), camera).has_value());
    EXPECT_FALSE(ProjectStarToPixel(offAxis(DECIMAL(32.0)), found::Quaternion::Identity(), camera).has_value());
}

TEST(DistortionFoldRadiusSquaredTest, FindsTheFold) {
    CameraParameters paper = PinholeCamera(DECIMAL(7308.2), DECIMAL(7307.2), DECIMAL(1295.5), DECIMAL(1023.5));
    paper.k1 = DECIMAL(-0.075);
    paper.k2 = DECIMAL(0.883);
    paper.k3 = DECIMAL(-12.60);
    CameraParameters lens = PinholeCamera(DECIMAL(1739.13), DECIMAL(1739.13), DECIMAL(1031.5), DECIMAL(771.5));
    lens.k1 = DECIMAL(-0.157);
    lens.k2 = DECIMAL(0.082);

    // The paper's lens folds where the slope 1 + 3 k1 s + 5 k2 s^2 + 7 k3 s^3
    // first crosses zero, near s = 0.239 (26 degrees off axis).
    const decimal fold = DistortionFoldRadiusSquared(paper);
    const decimal slope = DECIMAL(1.0) + DECIMAL(3.0) * paper.k1 * fold + DECIMAL(5.0) * paper.k2 * fold * fold +
                          DECIMAL(7.0) * paper.k3 * fold * fold * fold;
    EXPECT_NEAR(fold, DECIMAL(0.239), DECIMAL(0.001));
    EXPECT_NEAR(slope, DECIMAL(0.0), DECIMAL(1e-9));
    const decimal r = DECIMAL_SQRT(fold);
    const decimal tiny = DECIMAL(1e-6);
    EXPECT_TRUE(ProjectStarToPixel(found::Vec3(r - tiny, DECIMAL(0.0), DECIMAL(1.0)), found::Quaternion::Identity(),
                                   paper)
                    .has_value());
    EXPECT_FALSE(ProjectStarToPixel(found::Vec3(r + tiny, DECIMAL(0.0), DECIMAL(1.0)), found::Quaternion::Identity(),
                                    paper)
                     .has_value());
    EXPECT_TRUE(std::isinf(DistortionFoldRadiusSquared(lens)));
    EXPECT_TRUE(std::isinf(DistortionFoldRadiusSquared(PinholeCamera(DECIMAL(1.0), DECIMAL(1.0), DECIMAL(0.0),
                                                                     DECIMAL(0.0)))));
}

// The radial map can fold and then turn upward again, so a star whose own
// slope is positive is still rejected if a fold lies between it and the axis.
// Here the slope 1 - 3 s + 1.4 s^3 (s = r^2) is negative near s = 0.5 and
// positive again at s = 2.
TEST(ProjectStarToPixelTest, RejectsStarBeyondFoldThatUnfolds) {
    CameraParameters camera = PinholeCamera(DECIMAL(100.0), DECIMAL(100.0), DECIMAL(320.0), DECIMAL(240.0));
    camera.k1 = DECIMAL(-1.0);
    camera.k3 = DECIMAL(0.2);

    // r^2 = 2 and r^2 = 0.04.
    EXPECT_FALSE(ProjectStarToPixel(found::Vec3(DECIMAL(1.0), DECIMAL(1.0), DECIMAL(1.0)).normalized(),
                                    found::Quaternion::Identity(), camera)
                     .has_value());
    EXPECT_TRUE(ProjectStarToPixel(found::Vec3(DECIMAL(0.2), DECIMAL(0.0), DECIMAL(1.0)).normalized(),
                                   found::Quaternion::Identity(), camera)
                    .has_value());
}

// Barrel distortion that never folds (the datasheet fit for the 6 mm lens)
// keeps stars however far off axis.
TEST(ProjectStarToPixelTest, KeepsStarsWhenDistortionNeverFolds) {
    CameraParameters camera = PinholeCamera(DECIMAL(1739.13), DECIMAL(1739.13), DECIMAL(1031.5), DECIMAL(771.5));
    camera.k1 = DECIMAL(-0.157);
    camera.k2 = DECIMAL(0.082);

    EXPECT_TRUE(ProjectStarToPixel(found::Vec3(DECIMAL(2.0), DECIMAL(1.5), DECIMAL(1.0)).normalized(),
                                   found::Quaternion::Identity(), camera)
                    .has_value());
}

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

// Under an identity LOST attitude, a star on LOST's optical axis (inertial +x)
// must land on the principal point.
TEST(LostAttitudeToScopeFrameTest, OnAxisStarMapsToPrincipalPoint) {
    CameraParameters camera = PinholeCamera(DECIMAL(100.0), DECIMAL(100.0), DECIMAL(320.0), DECIMAL(240.0));
    const found::Quaternion attitude = LostAttitudeToScopeFrame(found::Quaternion::Identity());
    found::Vec3 eI(DECIMAL(1.0), DECIMAL(0.0), DECIMAL(0.0));

    std::optional<found::Vec2> pixel = ProjectStarToPixel(eI, attitude, camera);

    ASSERT_TRUE(pixel.has_value());
    EXPECT_NEAR(pixel->x(), DECIMAL(320.0), kTol);
    EXPECT_NEAR(pixel->y(), DECIMAL(240.0), kTol);
}

// Off-axis stars must match LOST's pixel exactly, which pins the axis
// permutation and its signs; a wrong one would mirror the field.
TEST(LostAttitudeToScopeFrameTest, MatchesLostForwardModelSigns) {
    const decimal f = DECIMAL(100.0);
    const decimal cx = DECIMAL(320.0);
    const decimal cy = DECIMAL(240.0);
    CameraParameters camera = PinholeCamera(f, f, cx, cy);
    const found::Quaternion attitude = LostAttitudeToScopeFrame(found::Quaternion::Identity());

    // +camera-y: LOST sends this toward -u (left); SCOPE must agree.
    found::Vec3 plusY(DECIMAL(1.0), DECIMAL(0.1), DECIMAL(0.0));
    std::optional<found::Vec2> py = ProjectStarToPixel(plusY, attitude, camera);
    ASSERT_TRUE(py.has_value());
    found::Vec2 pyExpected = LostPixel(plusY, f, cx, cy);
    EXPECT_NEAR(py->x(), pyExpected.x(), kTol);
    EXPECT_NEAR(py->y(), pyExpected.y(), kTol);

    // +camera-z: LOST sends this toward -v; SCOPE must agree.
    found::Vec3 plusZ(DECIMAL(1.0), DECIMAL(0.0), DECIMAL(0.1));
    std::optional<found::Vec2> pz = ProjectStarToPixel(plusZ, attitude, camera);
    ASSERT_TRUE(pz.has_value());
    found::Vec2 pzExpected = LostPixel(plusZ, f, cx, cy);
    EXPECT_NEAR(pz->x(), pzExpected.x(), kTol);
    EXPECT_NEAR(pz->y(), pzExpected.y(), kTol);
}

// A star on LOST's anti-boresight (inertial -x) ends up behind SCOPE's image
// plane and is rejected.
TEST(LostAttitudeToScopeFrameTest, AntiBoresightIsRejected) {
    CameraParameters camera = PinholeCamera(DECIMAL(100.0), DECIMAL(100.0), DECIMAL(320.0), DECIMAL(240.0));
    const found::Quaternion attitude = LostAttitudeToScopeFrame(found::Quaternion::Identity());
    found::Vec3 eI(DECIMAL(-1.0), DECIMAL(0.0), DECIMAL(0.0));

    EXPECT_FALSE(ProjectStarToPixel(eI, attitude, camera).has_value());
}

}  // namespace scope
