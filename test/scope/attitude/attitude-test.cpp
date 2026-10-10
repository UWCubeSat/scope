#include <gtest/gtest.h>

#include <optional>
#include <stdexcept>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/attitude/attitude.hpp"
#include "scope/projection/projection.hpp"

#include "test/scope/common/test-files.hpp"

namespace scope {

namespace {

const decimal kTol = DECIMAL(1e-5);

/// Checks a quaternion's components against (w, x, y, z).
void ExpectQuaternion(const found::Quaternion &actual, decimal w, decimal x, decimal y, decimal z) {
    EXPECT_NEAR(actual.w(), w, kTol);
    EXPECT_NEAR(actual.x(), x, kTol);
    EXPECT_NEAR(actual.y(), y, kTol);
    EXPECT_NEAR(actual.z(), z, kTol);
}

}  // namespace

// Each line is one quaternion, real part first, returned in file order.
TEST(LoadAttitudesTest, LoadsQuaternionsInFileOrder) {
    TempFile file("attitudes-order.txt", "1 0 0 0\n0 1 0 0\n0.5 0.5 0.5 0.5\n");

    std::vector<found::Quaternion> attitudes = LoadAttitudes(file.Path());

    ASSERT_EQ(attitudes.size(), 3u);
    ExpectQuaternion(attitudes[0], DECIMAL(1.0), DECIMAL(0.0), DECIMAL(0.0), DECIMAL(0.0));
    ExpectQuaternion(attitudes[1], DECIMAL(0.0), DECIMAL(1.0), DECIMAL(0.0), DECIMAL(0.0));
    ExpectQuaternion(attitudes[2], DECIMAL(0.5), DECIMAL(0.5), DECIMAL(0.5), DECIMAL(0.5));
}

// A quaternion that is not unit length is normalized rather than rejected.
TEST(LoadAttitudesTest, NormalizesEachQuaternion) {
    TempFile file("attitudes-normalize.txt", "2 0 0 0\n3 0 4 0\n");

    std::vector<found::Quaternion> attitudes = LoadAttitudes(file.Path());

    ASSERT_EQ(attitudes.size(), 2u);
    ExpectQuaternion(attitudes[0], DECIMAL(1.0), DECIMAL(0.0), DECIMAL(0.0), DECIMAL(0.0));
    ExpectQuaternion(attitudes[1], DECIMAL(0.6), DECIMAL(0.0), DECIMAL(0.8), DECIMAL(0.0));
}

// Comments, blank lines and Windows line endings do not produce attitudes.
TEST(LoadAttitudesTest, SkipsCommentsAndBlankLines) {
    TempFile file("attitudes-comments.txt", "# w x y z\n\n1 0 0 0  # first image\n   \t\n0 0 0 1\r\n");

    std::vector<found::Quaternion> attitudes = LoadAttitudes(file.Path());

    ASSERT_EQ(attitudes.size(), 2u);
    ExpectQuaternion(attitudes[0], DECIMAL(1.0), DECIMAL(0.0), DECIMAL(0.0), DECIMAL(0.0));
    ExpectQuaternion(attitudes[1], DECIMAL(0.0), DECIMAL(0.0), DECIMAL(0.0), DECIMAL(1.0));
}

// Commas work as separators, with or without spaces.
TEST(LoadAttitudesTest, AcceptsCommaSeparators) {
    TempFile file("attitudes-commas.txt", "1,0,0,0\n0, 0, 1, 0\n");

    std::vector<found::Quaternion> attitudes = LoadAttitudes(file.Path());

    ASSERT_EQ(attitudes.size(), 2u);
    ExpectQuaternion(attitudes[0], DECIMAL(1.0), DECIMAL(0.0), DECIMAL(0.0), DECIMAL(0.0));
    ExpectQuaternion(attitudes[1], DECIMAL(0.0), DECIMAL(0.0), DECIMAL(1.0), DECIMAL(0.0));
}

// A file with no attitude lines yields no attitudes.
TEST(LoadAttitudesTest, EmptyFileYieldsNoAttitudes) {
    TempFile file("attitudes-empty.txt", "# nothing here\n");

    EXPECT_TRUE(LoadAttitudes(file.Path()).empty());
}

// The attitude rotates inertial into SCOPE's +z-boresight camera frame, so it
// can be handed straight to ProjectStarToPixel. This one turns inertial +x onto
// the boresight, which puts a star at (ra = 0, dec = 0) on the principal point.
TEST(LoadAttitudesTest, AttitudeRotatesInertialIntoCameraFrame) {
    TempFile file("attitudes-frame.txt", "0.7071067811865476 0 -0.7071067811865476 0\n");

    std::vector<found::Quaternion> attitudes = LoadAttitudes(file.Path());
    ASSERT_EQ(attitudes.size(), 1u);

    CameraParameters camera;
    camera.focalLengthX = DECIMAL(100.0);
    camera.focalLengthY = DECIMAL(100.0);
    camera.principalX = DECIMAL(32.0);
    camera.principalY = DECIMAL(24.0);

    const found::Vec3 inertialX(DECIMAL(1.0), DECIMAL(0.0), DECIMAL(0.0));
    std::optional<found::Vec2> pixel = ProjectStarToPixel(inertialX, attitudes[0], camera);

    ASSERT_TRUE(pixel.has_value());
    EXPECT_NEAR(pixel->x(), DECIMAL(32.0), DECIMAL(1e-3));
    EXPECT_NEAR(pixel->y(), DECIMAL(24.0), DECIMAL(1e-3));
}

// A path that cannot be opened throws.
TEST(LoadAttitudesTest, MissingFileThrows) {
    EXPECT_THROW(LoadAttitudes("test/fixtures/does-not-exist.txt"), std::runtime_error);
}

// A line with fewer than four numbers throws.
TEST(LoadAttitudesTest, TooFewNumbersThrows) {
    TempFile file("attitudes-short.txt", "1 0 0 0\n1 0 0\n");

    EXPECT_THROW(LoadAttitudes(file.Path()), std::runtime_error);
}

// A line with more than four numbers throws.
TEST(LoadAttitudesTest, TooManyNumbersThrows) {
    TempFile file("attitudes-long.txt", "1 0 0 0 0\n");

    EXPECT_THROW(LoadAttitudes(file.Path()), std::runtime_error);
}

// A line holding something other than numbers throws.
TEST(LoadAttitudesTest, NonNumericThrows) {
    TempFile file("attitudes-text.txt", "1 0 zero 0\n");

    EXPECT_THROW(LoadAttitudes(file.Path()), std::runtime_error);
}

// The zero quaternion is not a rotation and throws.
TEST(LoadAttitudesTest, ZeroQuaternionThrows) {
    TempFile file("attitudes-zero.txt", "0 0 0 0\n");

    EXPECT_THROW(LoadAttitudes(file.Path()), std::runtime_error);
}

}  // namespace scope
