#include <gtest/gtest.h>

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/distortion/distortion.hpp"
#include "scope/projection/projection.hpp"

namespace scope {

namespace {

/// The candidate flight camera from TODO.md: Edmund Optics #35-142 (6 mm) on a
/// 3.45 um, 2064 x 1544 sensor, with the distortion read from the datasheet.
constexpr int kLensWidth = 2064;
constexpr int kLensHeight = 1544;

CameraParameters LensCamera() {
    CameraParameters camera;
    camera.focalLengthX = DECIMAL(1739.0);
    camera.focalLengthY = DECIMAL(1739.0);
    camera.principalX = DECIMAL(1031.5);
    camera.principalY = DECIMAL(771.5);
    camera.k1 = DECIMAL(-0.157);
    camera.k2 = DECIMAL(0.082);
    return camera;
}

/// The true camera of the paper's Monte Carlo study (Table 1), on its
/// 2592 x 2048 sensor. Every parameter is nonzero.
constexpr int kPaperWidth = 2592;
constexpr int kPaperHeight = 2048;

CameraParameters PaperCamera() {
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

/// A pinhole camera with no distortion or skew.
CameraParameters PinholeCamera() {
    CameraParameters camera;
    camera.focalLengthX = DECIMAL(100.0);
    camera.focalLengthY = DECIMAL(120.0);
    camera.principalX = DECIMAL(32.0);
    camera.principalY = DECIMAL(24.0);
    return camera;
}

/// A camera whose radial distortion folds back at r = 1/sqrt(3), so that
/// distorted radii above 2 / (3 sqrt(3)) ~ 0.385 have no preimage.
CameraParameters FoldingCamera() {
    CameraParameters camera = PinholeCamera();
    camera.focalLengthY = DECIMAL(100.0);
    camera.principalX = DECIMAL(0.0);
    camera.principalY = DECIMAL(0.0);
    camera.k1 = DECIMAL(-1.0);
    return camera;
}

found::Vec2 Distort(const found::Vec2 &ideal, const CameraParameters &camera) {
    return BrownDistort(ideal, camera.k1, camera.k2, camera.k3, camera.p1, camera.p2);
}

/// Raw pixels spread over a sensor, corners included.
std::vector<found::Vec2> SensorPixels(int width, int height) {
    std::vector<found::Vec2> pixels;
    for (int i = 0; i <= 4; ++i) {
        for (int j = 0; j <= 4; ++j) {
            pixels.push_back(found::Vec2{DECIMAL(width - 1) * DECIMAL(i) / DECIMAL(4.0),
                                         DECIMAL(height - 1) * DECIMAL(j) / DECIMAL(4.0)});
        }
    }
    return pixels;
}

const decimal kPixelTol = DECIMAL(1e-6);

}  // namespace

// The analytic Jacobian (Eqs. 36-37) matches central differences of
// BrownDistort, with every distortion term active.
TEST(BrownDistortJacobianTest, MatchesFiniteDifferences) {
    CameraParameters camera = PaperCamera();
    // Exaggerate the decentering so its columns are not lost in the radial terms.
    camera.p1 = DECIMAL(0.03);
    camera.p2 = DECIMAL(-0.02);
    const decimal h = DECIMAL(1e-6);

    for (const found::Vec2 &ideal : {found::Vec2{DECIMAL(0.1), DECIMAL(-0.15)},
                                     found::Vec2{DECIMAL(-0.2), DECIMAL(0.05)},
                                     found::Vec2{DECIMAL(0.0), DECIMAL(0.12)}}) {
        const found::Mat2 jacobian = BrownDistortJacobian(ideal, camera);
        for (int column = 0; column < 2; ++column) {
            found::Vec2 offset = found::Vec2::Zero();
            offset(column) = h;
            const found::Vec2 numeric =
                (Distort(ideal + offset, camera) - Distort(ideal - offset, camera)) / (DECIMAL(2.0) * h);
            EXPECT_NEAR(jacobian(0, column), numeric.x(), DECIMAL(1e-7));
            EXPECT_NEAR(jacobian(1, column), numeric.y(), DECIMAL(1e-7));
        }
    }
}

// With no distortion, BrownDistort is the identity.
TEST(BrownDistortJacobianTest, IdentityWithoutDistortion) {
    const found::Mat2 jacobian = BrownDistortJacobian(found::Vec2{DECIMAL(0.3), DECIMAL(-0.2)}, PinholeCamera());
    EXPECT_TRUE(jacobian.isApprox(found::Mat2::Identity()));
}

// Undistorting a distorted point recovers it, for both the real lens and the
// paper's camera, across the whole sensor.
TEST(BrownUndistortTest, InvertsBrownDistort) {
    for (const CameraParameters &camera : {LensCamera(), PaperCamera()}) {
        for (decimal x = DECIMAL(-0.6); x <= DECIMAL(0.6); x += DECIMAL(0.1)) {
            for (decimal y = DECIMAL(-0.45); y <= DECIMAL(0.45); y += DECIMAL(0.15)) {
                // The paper camera's field is far smaller; keep it on its sensor.
                const decimal scale = camera.focalLengthX > DECIMAL(5000.0) ? DECIMAL(0.3) : DECIMAL(1.0);
                const found::Vec2 ideal{x * scale, y * scale};

                const std::optional<found::Vec2> recovered = BrownUndistort(Distort(ideal, camera), camera);

                ASSERT_TRUE(recovered.has_value());
                EXPECT_NEAR(recovered->x(), ideal.x(), DECIMAL(1e-12));
                EXPECT_NEAR(recovered->y(), ideal.y(), DECIMAL(1e-12));
            }
        }
    }
}

// With no distortion the first Newton step lands on the answer.
TEST(BrownUndistortTest, IdentityWithoutDistortion) {
    const found::Vec2 point{DECIMAL(0.25), DECIMAL(-0.4)};
    const std::optional<found::Vec2> recovered = BrownUndistort(point, PinholeCamera(), 1);
    ASSERT_TRUE(recovered.has_value());
    EXPECT_EQ(*recovered, point);
}

// A distorted radius past the fold has no preimage on the near branch.
TEST(BrownUndistortTest, RejectsPointsPastTheFold) {
    EXPECT_FALSE(BrownUndistort(found::Vec2{DECIMAL(1.0), DECIMAL(0.0)}, FoldingCamera()).has_value());
    // Inside the fold it still works.
    const found::Vec2 ideal{DECIMAL(0.3), DECIMAL(0.0)};
    const std::optional<found::Vec2> recovered = BrownUndistort(Distort(ideal, FoldingCamera()), FoldingCamera());
    ASSERT_TRUE(recovered.has_value());
    EXPECT_NEAR(recovered->x(), ideal.x(), DECIMAL(1e-12));
}

// Too few iterations to converge gives no answer rather than a wrong one.
TEST(BrownUndistortTest, ReportsNonConvergence) {
    const CameraParameters camera = LensCamera();
    const found::Vec2 distorted = Distort(found::Vec2{DECIMAL(0.6), DECIMAL(0.45)}, camera);
    EXPECT_FALSE(BrownUndistort(distorted, camera, 1).has_value());
    EXPECT_FALSE(BrownUndistort(distorted, camera, 0).has_value());
    EXPECT_TRUE(BrownUndistort(distorted, camera).has_value());
}

// PixelToImagePlane inverts the calibration matrix of Eq. 7, skew included.
TEST(PixelToImagePlaneTest, InvertsCalibrationMatrix) {
    const CameraParameters camera = PaperCamera();
    const found::Vec2 imagePlane{DECIMAL(0.05), DECIMAL(-0.08)};
    const found::Vec2 pixel{
        camera.focalLengthX * imagePlane.x() + camera.alpha * imagePlane.y() + camera.principalX,
        camera.focalLengthY * imagePlane.y() + camera.principalY};

    const found::Vec2 recovered = PixelToImagePlane(pixel, camera);

    EXPECT_NEAR(recovered.x(), imagePlane.x(), DECIMAL(1e-12));
    EXPECT_NEAR(recovered.y(), imagePlane.y(), DECIMAL(1e-12));
}

TEST(PixelToImagePlaneTest, RejectsZeroFocalLength) {
    CameraParameters camera = PinholeCamera();
    camera.focalLengthX = DECIMAL(0.0);
    EXPECT_THROW(PixelToImagePlane(found::Vec2::Zero(), camera), std::invalid_argument);

    camera = PinholeCamera();
    camera.focalLengthY = DECIMAL(0.0);
    EXPECT_THROW(PixelToImagePlane(found::Vec2::Zero(), camera), std::invalid_argument);
}

// PixelToLineOfSight undoes ProjectStarToPixel with an identity attitude.
TEST(PixelToLineOfSightTest, InvertsProjection) {
    for (const CameraParameters &camera : {LensCamera(), PaperCamera()}) {
        for (const found::Vec3 &direction : {found::Vec3(DECIMAL(0.0), DECIMAL(0.0), DECIMAL(1.0)),
                                             found::Vec3(DECIMAL(0.03), DECIMAL(-0.02), DECIMAL(1.0)),
                                             found::Vec3(DECIMAL(-0.1), DECIMAL(0.08), DECIMAL(1.0))}) {
            const found::Vec3 eC = direction.normalized();
            const std::optional<found::Vec2> pixel =
                ProjectStarToPixel(eC, found::Quaternion::Identity(), camera);
            ASSERT_TRUE(pixel.has_value());

            const std::optional<found::Vec3> lineOfSight = PixelToLineOfSight(*pixel, camera);

            ASSERT_TRUE(lineOfSight.has_value());
            EXPECT_NEAR(lineOfSight->norm(), DECIMAL(1.0), DECIMAL(1e-12));
            EXPECT_NEAR((*lineOfSight - eC).norm(), DECIMAL(0.0), DECIMAL(1e-12));
        }
    }
}

TEST(PixelToLineOfSightTest, RejectsPixelsPastTheFold) {
    EXPECT_FALSE(PixelToLineOfSight(found::Vec2{DECIMAL(100.0), DECIMAL(0.0)}, FoldingCamera()).has_value());
}

// UndistortPixel gives the pixel a pinhole camera with the same intrinsics
// would see the same line of sight at.
TEST(UndistortPixelTest, MatchesPinholeProjection) {
    const CameraParameters camera = PaperCamera();
    CameraParameters pinhole = camera;
    pinhole.k1 = pinhole.k2 = pinhole.k3 = pinhole.p1 = pinhole.p2 = DECIMAL(0.0);

    const found::Vec3 eC = found::Vec3(DECIMAL(0.12), DECIMAL(-0.09), DECIMAL(1.0)).normalized();
    const std::optional<found::Vec2> raw = ProjectStarToPixel(eC, found::Quaternion::Identity(), camera);
    const std::optional<found::Vec2> ideal = ProjectStarToPixel(eC, found::Quaternion::Identity(), pinhole);
    ASSERT_TRUE(raw.has_value());
    ASSERT_TRUE(ideal.has_value());

    const std::optional<found::Vec2> undistorted = UndistortPixel(*raw, camera);

    ASSERT_TRUE(undistorted.has_value());
    EXPECT_NEAR(undistorted->x(), ideal->x(), kPixelTol);
    EXPECT_NEAR(undistorted->y(), ideal->y(), kPixelTol);
}

TEST(UndistortPixelTest, IdentityWithoutDistortion) {
    for (const found::Vec2 &pixel : SensorPixels(64, 48)) {
        const std::optional<found::Vec2> undistorted = UndistortPixel(pixel, PinholeCamera());
        ASSERT_TRUE(undistorted.has_value());
        EXPECT_NEAR(undistorted->x(), pixel.x(), kPixelTol);
        EXPECT_NEAR(undistorted->y(), pixel.y(), kPixelTol);
    }
}

TEST(UndistortPixelTest, RejectsPixelsPastTheFold) {
    EXPECT_FALSE(UndistortPixel(found::Vec2{DECIMAL(100.0), DECIMAL(0.0)}, FoldingCamera()).has_value());
}

// Undistorting then re-projecting returns the raw pixel over the whole sensor.
TEST(UndistortPixelTest, RoundTripsOverSensor) {
    const CameraParameters camera = LensCamera();
    for (const found::Vec2 &pixel : SensorPixels(kLensWidth, kLensHeight)) {
        const std::optional<found::Vec3> lineOfSight = PixelToLineOfSight(pixel, camera);
        ASSERT_TRUE(lineOfSight.has_value());

        const std::optional<found::Vec2> reprojected =
            ProjectStarToPixel(*lineOfSight, found::Quaternion::Identity(), camera);

        ASSERT_TRUE(reprojected.has_value());
        EXPECT_NEAR(reprojected->x(), pixel.x(), kPixelTol);
        EXPECT_NEAR(reprojected->y(), pixel.y(), kPixelTol);
    }
}

// The grid steps by the spacing and always includes the last row and column.
TEST(ComputeDistortionMapTest, GridIncludesEdges) {
    const std::vector<DistortionSample> samples = ComputeDistortionMap(PinholeCamera(), 64, 48, 20);

    // Columns 0, 20, 40, 60, 63; rows 0, 20, 40, 47.
    ASSERT_EQ(samples.size(), 20u);
    EXPECT_EQ(samples[0].pixel, found::Vec2(DECIMAL(0.0), DECIMAL(0.0)));
    EXPECT_EQ(samples[1].pixel, found::Vec2(DECIMAL(20.0), DECIMAL(0.0)));
    EXPECT_EQ(samples[4].pixel, found::Vec2(DECIMAL(63.0), DECIMAL(0.0)));
    EXPECT_EQ(samples[5].pixel, found::Vec2(DECIMAL(0.0), DECIMAL(20.0)));
    EXPECT_EQ(samples[19].pixel, found::Vec2(DECIMAL(63.0), DECIMAL(47.0)));
}

// When the spacing divides the last coordinate, the edge is not repeated.
TEST(ComputeDistortionMapTest, GridDoesNotRepeatEdges) {
    const std::vector<DistortionSample> samples = ComputeDistortionMap(PinholeCamera(), 41, 1, 20);

    ASSERT_EQ(samples.size(), 3u);
    EXPECT_EQ(samples[2].pixel, found::Vec2(DECIMAL(40.0), DECIMAL(0.0)));
}

TEST(ComputeDistortionMapTest, RejectsBadArguments) {
    EXPECT_THROW(ComputeDistortionMap(PinholeCamera(), 0, 10, 5), std::invalid_argument);
    EXPECT_THROW(ComputeDistortionMap(PinholeCamera(), 10, 0, 5), std::invalid_argument);
    EXPECT_THROW(ComputeDistortionMap(PinholeCamera(), 10, 10, 0), std::invalid_argument);
    CameraParameters camera = PinholeCamera();
    camera.focalLengthX = DECIMAL(0.0);
    EXPECT_THROW(ComputeDistortionMap(camera, 10, 10, 5), std::invalid_argument);
}

TEST(ComputeDistortionMapTest, ZeroWithoutDistortion) {
    for (const DistortionSample &sample : ComputeDistortionMap(PinholeCamera(), 64, 48, 8)) {
        ASSERT_TRUE(sample.correction.has_value());
        EXPECT_NEAR(sample.correction->norm(), DECIMAL(0.0), kPixelTol);
    }
}

// Pure barrel distortion pulls points inward, so every correction points away
// from the principal point, and pixels at the same radius get the same size of
// correction.
TEST(ComputeDistortionMapTest, BarrelCorrectionIsRadialAndOutward) {
    CameraParameters camera = LensCamera();
    camera.focalLengthY = camera.focalLengthX;
    const found::Vec2 principal{camera.principalX, camera.principalY};

    for (const DistortionSample &sample : ComputeDistortionMap(camera, kLensWidth, kLensHeight, 200)) {
        ASSERT_TRUE(sample.correction.has_value());
        const found::Vec2 radial = sample.pixel - principal;
        if (radial.norm() < DECIMAL(1.0)) {
            continue;
        }
        // Parallel to the radius and pointing outward.
        EXPECT_NEAR(radial.x() * sample.correction->y() - radial.y() * sample.correction->x(), DECIMAL(0.0),
                    DECIMAL(1e-6) * radial.norm());
        EXPECT_GT(radial.dot(*sample.correction), DECIMAL(0.0));
    }

    // Mirror-image pixels about the principal point get mirror-image corrections.
    const std::optional<found::Vec2> left = UndistortPixel(found::Vec2{DECIMAL(0.0), DECIMAL(0.0)}, camera);
    const std::optional<found::Vec2> right =
        UndistortPixel(found::Vec2{DECIMAL(kLensWidth - 1), DECIMAL(kLensHeight - 1)}, camera);
    ASSERT_TRUE(left.has_value());
    ASSERT_TRUE(right.has_value());
    EXPECT_NEAR((*left - principal).norm(), (*right - principal).norm(), kPixelTol);
}

// Sanity check against TODO.md, which reads the real lens's datasheet as
// putting stars 92 px off in the corners and 53 px off at the left and right
// edges when distortion is ignored.
TEST(ComputeDistortionMapTest, LensCornerCorrectionMatchesDatasheet) {
    const std::vector<DistortionSample> samples = ComputeDistortionMap(LensCamera(), kLensWidth, kLensHeight, 100);
    const DistortionSummary summary = SummarizeDistortionMap(samples);

    EXPECT_EQ(summary.invalidCount, 0u);
    EXPECT_NEAR(summary.maxMagnitude, DECIMAL(92.0), DECIMAL(1.0));
    const std::optional<found::Vec2> edge = UndistortPixel(found::Vec2{DECIMAL(0.0), DECIMAL(771.5)}, LensCamera());
    ASSERT_TRUE(edge.has_value());
    EXPECT_NEAR((*edge - found::Vec2{DECIMAL(0.0), DECIMAL(771.5)}).norm(), DECIMAL(53.0), DECIMAL(1.0));
    // The largest correction is at a corner.
    const decimal corner = samples.front().correction->norm();
    EXPECT_NEAR(corner, summary.maxMagnitude, kPixelTol);
}

TEST(SummarizeDistortionMapTest, ComputesStatistics) {
    const std::vector<DistortionSample> samples = {
        {found::Vec2{DECIMAL(0.0), DECIMAL(0.0)}, found::Vec2{DECIMAL(3.0), DECIMAL(4.0)}},
        {found::Vec2{DECIMAL(1.0), DECIMAL(0.0)}, std::nullopt},
        {found::Vec2{DECIMAL(2.0), DECIMAL(0.0)}, found::Vec2{DECIMAL(0.0), DECIMAL(-1.0)}},
        {found::Vec2{DECIMAL(3.0), DECIMAL(0.0)}, found::Vec2{DECIMAL(1.0), DECIMAL(0.0)}},
    };

    const DistortionSummary summary = SummarizeDistortionMap(samples);

    EXPECT_EQ(summary.validCount, 3u);
    EXPECT_EQ(summary.invalidCount, 1u);
    EXPECT_NEAR(summary.maxMagnitude, DECIMAL(5.0), kPixelTol);
    EXPECT_NEAR(summary.rmsMagnitude, DECIMAL_SQRT(DECIMAL(27.0) / DECIMAL(3.0)), kPixelTol);
}

TEST(SummarizeDistortionMapTest, ZeroWithoutValidSamples) {
    const DistortionSummary empty = SummarizeDistortionMap({});
    EXPECT_EQ(empty.validCount, 0u);
    EXPECT_EQ(empty.maxMagnitude, DECIMAL(0.0));
    EXPECT_EQ(empty.rmsMagnitude, DECIMAL(0.0));

    const DistortionSummary invalid =
        SummarizeDistortionMap({{found::Vec2{DECIMAL(1.0), DECIMAL(2.0)}, std::nullopt}});
    EXPECT_EQ(invalid.invalidCount, 1u);
    EXPECT_EQ(invalid.rmsMagnitude, DECIMAL(0.0));
}

TEST(FormatDistortionMapCsvTest, WritesHeaderAndRows) {
    const std::vector<DistortionSample> samples = {
        {found::Vec2{DECIMAL(0.0), DECIMAL(10.0)}, found::Vec2{DECIMAL(1.5), DECIMAL(-0.25)}},
        {found::Vec2{DECIMAL(20.0), DECIMAL(10.0)}, std::nullopt},
    };

    EXPECT_EQ(FormatDistortionMapCsv(samples), "u,v,du,dv\n0,10,1.5,-0.25\n20,10,,\n");
    EXPECT_EQ(FormatDistortionMapCsv({}), "u,v,du,dv\n");
}

}  // namespace scope
