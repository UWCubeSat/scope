#include <gtest/gtest.h>

#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
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

TEST(LoadAttitudesTest, AcceptsCommaSeparators) {
    TempFile file("attitudes-commas.txt", "1,0,0,0\n0, 0, 1, 0\n");

    std::vector<found::Quaternion> attitudes = LoadAttitudes(file.Path());

    ASSERT_EQ(attitudes.size(), 2u);
    ExpectQuaternion(attitudes[0], DECIMAL(1.0), DECIMAL(0.0), DECIMAL(0.0), DECIMAL(0.0));
    ExpectQuaternion(attitudes[1], DECIMAL(0.0), DECIMAL(0.0), DECIMAL(1.0), DECIMAL(0.0));
}

// A loaded attitude goes straight into ProjectStarToPixel. This one turns
// inertial +x onto the boresight, so a star at (ra = 0, dec = 0) lands on the
// principal point.
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

TEST(LoadAttitudesTest, MissingFileThrows) {
    EXPECT_THROW(LoadAttitudes("test/fixtures/does-not-exist.txt"), std::runtime_error);
}

TEST(LoadAttitudesTest, TooFewNumbersThrows) {
    TempFile file("attitudes-short.txt", "1 0 0 0\n1 0 0\n");

    EXPECT_THROW(LoadAttitudes(file.Path()), std::runtime_error);
}

TEST(LoadAttitudesTest, TooManyNumbersThrows) {
    TempFile file("attitudes-long.txt", "1 0 0 0 0\n");

    EXPECT_THROW(LoadAttitudes(file.Path()), std::runtime_error);
}

TEST(LoadAttitudesTest, NonNumericThrows) {
    TempFile file("attitudes-text.txt", "1 0 zero 0\n");

    EXPECT_THROW(LoadAttitudes(file.Path()), std::runtime_error);
}

TEST(LoadAttitudesTest, ZeroQuaternionThrows) {
    TempFile file("attitudes-zero.txt", "0 0 0 0\n");

    EXPECT_THROW(LoadAttitudes(file.Path()), std::runtime_error);
}

TEST(WriteAttitudesTest, RoundTripsThroughLoadAttitudes) {
    const std::vector<found::Quaternion> attitudes = {
        found::Quaternion(DECIMAL(0.5), DECIMAL(0.5), DECIMAL(-0.5), DECIMAL(0.5)),
        found::Quaternion(
            found::AngleAxis(DECIMAL(1.234567890123), found::Vec3(DECIMAL(0.6), DECIMAL(0.0), DECIMAL(0.8)))),
    };
    TempFile file("attitudes-written.txt", "");

    WriteAttitudes(file.Path(), attitudes, "first line\nsecond line");
    const std::vector<found::Quaternion> loaded = LoadAttitudes(file.Path());

    ASSERT_EQ(loaded.size(), attitudes.size());
    for (std::size_t i = 0; i < attitudes.size(); ++i) {
        // LoadAttitudes renormalizes, which can move the last bit.
        EXPECT_LT((loaded[i].coeffs() - attitudes[i].coeffs()).norm(), DECIMAL(1e-14));
    }
    std::ifstream written(file.Path());
    std::stringstream contents;
    contents << written.rdbuf();
    EXPECT_EQ(contents.str().rfind("# first line\n# second line\n", 0), 0u);
}

TEST(WriteAttitudesTest, UnwritablePathThrows) {
    EXPECT_THROW(WriteAttitudes("test/fixtures/no-such-directory/attitudes.txt", {}, ""), std::runtime_error);
}

// /dev/full opens but fails every write, as a full disk would.
TEST(WriteAttitudesTest, FailedWriteThrows) {
    if (!std::ifstream("/dev/full")) {
        GTEST_SKIP() << "needs /dev/full";
    }
    EXPECT_THROW(WriteAttitudes("/dev/full", {found::Quaternion::Identity()}, "header"), std::runtime_error);
}

// To first order the correction is I - [dtheta x] (paper Eq. 38), which moves
// a direction by e x dtheta.
TEST(AttitudeCorrectionTest, MatchesPaperFirstOrderForm) {
    const found::Vec3 dtheta(DECIMAL(2e-6), DECIMAL(-3e-6), DECIMAL(1e-6));
    const found::Vec3 e = found::Vec3(DECIMAL(0.3), DECIMAL(-0.2), DECIMAL(1.0)).normalized();

    const found::Vec3 corrected = AttitudeCorrection(dtheta) * e;

    const found::Vec3 expected = e - dtheta.cross(e);
    EXPECT_NEAR((corrected - expected).norm(), DECIMAL(0.0), DECIMAL(1e-11));
}

TEST(AttitudeCorrectionTest, ZeroIsIdentity) {
    EXPECT_EQ(AttitudeCorrection(found::Vec3::Zero()).coeffs(), found::Quaternion::Identity().coeffs());
    EXPECT_EQ(CorrectionRotationVector(found::Quaternion::Identity()), found::Vec3::Zero());
}

TEST(AttitudeCorrectionTest, RotationVectorRoundTrips) {
    for (const found::Vec3 &dtheta : {found::Vec3(DECIMAL(1e-4), DECIMAL(-2e-4), DECIMAL(5e-5)),
                                      found::Vec3(DECIMAL(0.3), DECIMAL(1.1), DECIMAL(-2.0)),
                                      found::Vec3(DECIMAL(0.0), DECIMAL(0.0), DECIMAL(-3.0))}) {
        const found::Vec3 recovered = CorrectionRotationVector(AttitudeCorrection(dtheta));
        EXPECT_NEAR((recovered - dtheta).norm(), DECIMAL(0.0), DECIMAL(1e-12));
        // The sign of the quaternion does not matter.
        const found::Quaternion negated(-AttitudeCorrection(dtheta).coeffs());
        EXPECT_NEAR((CorrectionRotationVector(negated) - dtheta).norm(), DECIMAL(0.0), DECIMAL(1e-12));
    }
}

TEST(AverageQuaternionsTest, OneQuaternionIsItsOwnAverage) {
    const found::Quaternion q = AttitudeCorrection(found::Vec3(DECIMAL(0.1), DECIMAL(0.2), DECIMAL(0.3)));

    EXPECT_NEAR(AverageQuaternions({q}).angularDistance(q), DECIMAL(0.0), DECIMAL(1e-12));
}

// q and -q are the same rotation, so a flipped sign must not pull the
// average away; a plain mean of the components would give zero.
TEST(AverageQuaternionsTest, IgnoresSign) {
    const found::Quaternion q = AttitudeCorrection(found::Vec3(DECIMAL(0.4), DECIMAL(-0.1), DECIMAL(0.2)));
    const found::Quaternion negated(-q.coeffs());

    const found::Quaternion average = AverageQuaternions({q, negated, q});

    EXPECT_NEAR(average.angularDistance(q), DECIMAL(0.0), DECIMAL(1e-12));
    EXPECT_GE(average.w(), DECIMAL(0.0));
}

// Rotations about one axis average to the mean angle when they are small.
TEST(AverageQuaternionsTest, AveragesSmallRotationsAboutOneAxis) {
    const found::Vec3 axis = found::Vec3(DECIMAL(1.0), DECIMAL(2.0), DECIMAL(-1.0)).normalized();
    const std::vector<found::Quaternion> rotations = {AttitudeCorrection(DECIMAL(1e-4) * axis),
                                                      AttitudeCorrection(DECIMAL(3e-4) * axis),
                                                      AttitudeCorrection(DECIMAL(-1e-4) * axis),
                                                      AttitudeCorrection(DECIMAL(5e-4) * axis)};

    const found::Vec3 average = CorrectionRotationVector(AverageQuaternions(rotations));

    EXPECT_NEAR((average - DECIMAL(2e-4) * axis).norm(), DECIMAL(0.0), DECIMAL(1e-10));
}

// Equal and opposite rotations about different axes average to the identity.
TEST(AverageQuaternionsTest, SymmetricSetAveragesToIdentity) {
    const std::vector<found::Quaternion> rotations = {
        AttitudeCorrection(found::Vec3(DECIMAL(0.2), DECIMAL(0.0), DECIMAL(0.0))),
        AttitudeCorrection(found::Vec3(DECIMAL(-0.2), DECIMAL(0.0), DECIMAL(0.0))),
        AttitudeCorrection(found::Vec3(DECIMAL(0.0), DECIMAL(0.0), DECIMAL(0.3))),
        AttitudeCorrection(found::Vec3(DECIMAL(0.0), DECIMAL(0.0), DECIMAL(-0.3)))};

    EXPECT_NEAR(
        AverageQuaternions(rotations).angularDistance(found::Quaternion::Identity()), DECIMAL(0.0), DECIMAL(1e-12));
}

// Whichever sign the decomposition picks, the result has a non-negative real
// part.
TEST(AverageQuaternionsTest, RealPartIsNeverNegative) {
    for (int i = 0; i < 40; ++i) {
        const decimal t = DECIMAL(i);
        const found::Quaternion q = found::Quaternion(DECIMAL_COS(t),
                                                      DECIMAL_SIN(DECIMAL(1.7) * t),
                                                      DECIMAL_COS(DECIMAL(2.3) * t),
                                                      DECIMAL_SIN(DECIMAL(0.6) * t))
                                        .normalized();

        const found::Quaternion average = AverageQuaternions({q});

        EXPECT_GE(average.w(), DECIMAL(0.0));
        EXPECT_NEAR(average.angularDistance(q), DECIMAL(0.0), DECIMAL(1e-12));
    }
}

TEST(AverageQuaternionsTest, EmptyThrows) {
    EXPECT_THROW(AverageQuaternions({}), std::invalid_argument);
}

}  // namespace scope
