#include <gtest/gtest.h>

#include <cstddef>

#include <optional>
#include <random>
#include <stdexcept>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/common/types.hpp"
#include "scope/evaluation/evaluation.hpp"
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

/// The paper's Monte Carlo camera (Table 1) without its distortion: a narrow
/// field, about 20 x 16 degrees.
constexpr int kPaperWidth = 2592;
constexpr int kPaperHeight = 2048;

CameraParameters PaperPinholeCamera() {
    CameraParameters camera;
    camera.focalLengthX = DECIMAL(7308.2);
    camera.focalLengthY = DECIMAL(7307.2);
    camera.principalX = DECIMAL(1295.5);
    camera.principalY = DECIMAL(1023.5);
    return camera;
}

/// A rotation of angle radians about axis.
found::Quaternion Rotation(decimal angle, const found::Vec3 &axis) {
    return found::Quaternion(found::AngleAxis(angle, axis.normalized()));
}

/// Angle between two attitudes, in radians.
decimal AngleBetween(const found::Quaternion &a, const found::Quaternion &b) {
    return found::AngleAxis(a * b.conjugate()).angle();
}

/// Half the field of view of a camera on the normalized image plane, given its
/// sensor size in pixels.
found::Vec2 HalfField(const CameraParameters &camera, int width, int height) {
    return found::Vec2{DECIMAL(width) / (DECIMAL(2.0) * camera.focalLengthX),
                       DECIMAL(height) / (DECIMAL(2.0) * camera.focalLengthY)};
}

/// Adds a star image to observations: stars on a 5 x 4 grid over 90% of the
/// field, measured exactly as the camera would see them at trueAttitude, with
/// prior as the image's attitude.
void AddImage(CentroidObservations *observations,
              const CameraParameters &camera,
              const found::Vec2 &halfField,
              const found::Quaternion &trueAttitude,
              const found::Quaternion &prior) {
    const int image = static_cast<int>(observations->attitudes.size());
    observations->attitudes.push_back(prior);
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 4; ++j) {
            // From -90% to +90% of the half field on each axis.
            const decimal x = DECIMAL(0.9) * halfField.x() * (DECIMAL(i) / DECIMAL(2.0) - DECIMAL(1.0));
            const decimal y = DECIMAL(0.9) * halfField.y() * (DECIMAL(j) / DECIMAL(1.5) - DECIMAL(1.0));
            const found::Vec3 eC = found::Vec3(x, y, DECIMAL(1.0)).normalized();
            const found::Vec3 eI = trueAttitude.conjugate() * eC;
            const std::optional<found::Vec2> pixel = ProjectStarToPixel(eI, trueAttitude, camera);
            ASSERT_TRUE(pixel.has_value());
            observations->observations.push_back(Observation{image, i * 4 + j, eI, *pixel});
        }
    }
}

/// The true attitudes of the test images.
std::vector<found::Quaternion> TrueAttitudes() {
    return {Rotation(DECIMAL(0.3), found::Vec3(DECIMAL(1.0), DECIMAL(2.0), DECIMAL(3.0))),
            Rotation(DECIMAL(2.0), found::Vec3(DECIMAL(-2.0), DECIMAL(0.5), DECIMAL(1.0)))};
}

/// One image at each of TrueAttitudes, each with a prior about 100 arcseconds
/// off on every axis.
CentroidObservations TwoImages(const CameraParameters &camera, const found::Vec2 &halfField) {
    const decimal priorError = found::ArcSecToRad(DECIMAL(100.0));
    const found::Vec3 errorAxes[] = {found::Vec3(DECIMAL(1.0), DECIMAL(-1.0), DECIMAL(1.0)),
                                     found::Vec3(DECIMAL(-1.0), DECIMAL(1.0), DECIMAL(1.0))};

    CentroidObservations observations;
    const std::vector<found::Quaternion> truth = TrueAttitudes();
    for (std::size_t k = 0; k < truth.size(); ++k) {
        AddImage(&observations, camera, halfField, truth[k], Rotation(priorError, errorAxes[k]) * truth[k]);
    }
    return observations;
}

/// TwoImages for the flight camera, over its whole sensor.
CentroidObservations LensImages() { return TwoImages(LensCamera(), HalfField(LensCamera(), kLensWidth, kLensHeight)); }

const decimal kPixelTol = DECIMAL(1e-6);

}  // namespace

// Known values: components 1, 2, 3, -2 have mean 1 and central moments
// m2 = 3.5, m3 = -4.5, m4 = 24.5.
TEST(ComputeResidualStatisticsTest, ComputesMoments) {
    const ResidualStatistics statistics =
        ComputeResidualStatistics({found::Vec2{DECIMAL(1.0), DECIMAL(2.0)}, found::Vec2{DECIMAL(3.0), DECIMAL(-2.0)}});

    EXPECT_EQ(statistics.count, 2u);
    EXPECT_NEAR(statistics.mean, DECIMAL(1.0), DECIMAL(1e-12));
    EXPECT_NEAR(statistics.standardDeviation, DECIMAL_SQRT(DECIMAL(3.5)), DECIMAL(1e-12));
    EXPECT_NEAR(statistics.skewness, DECIMAL(-4.5) / DECIMAL_POW(DECIMAL(3.5), DECIMAL(1.5)), DECIMAL(1e-12));
    EXPECT_NEAR(statistics.kurtosis, DECIMAL(2.0), DECIMAL(1e-12));
}

// Normally distributed residuals have skewness 0 and kurtosis 3, the
// convention the paper's Fig. 7 reports in.
TEST(ComputeResidualStatisticsTest, NormalResidualsHaveKurtosisThree) {
    std::mt19937 generator(42);
    std::normal_distribution<decimal> noise(DECIMAL(0.0), DECIMAL(0.15));
    std::vector<found::Vec2> residuals(50000);
    for (found::Vec2 &residual : residuals) {
        residual = found::Vec2{noise(generator), noise(generator)};
    }

    const ResidualStatistics statistics = ComputeResidualStatistics(residuals);

    EXPECT_NEAR(statistics.mean, DECIMAL(0.0), DECIMAL(0.005));
    EXPECT_NEAR(statistics.standardDeviation, DECIMAL(0.15), DECIMAL(0.005));
    EXPECT_NEAR(statistics.skewness, DECIMAL(0.0), DECIMAL(0.05));
    EXPECT_NEAR(statistics.kurtosis, DECIMAL(3.0), DECIMAL(0.1));
}

TEST(ComputeResidualStatisticsTest, ZeroWithoutSpread) {
    const ResidualStatistics empty = ComputeResidualStatistics({});
    EXPECT_EQ(empty.count, 0u);
    EXPECT_EQ(empty.mean, DECIMAL(0.0));
    EXPECT_EQ(empty.standardDeviation, DECIMAL(0.0));

    const ResidualStatistics constant =
        ComputeResidualStatistics({found::Vec2{DECIMAL(0.5), DECIMAL(0.5)}, found::Vec2{DECIMAL(0.5), DECIMAL(0.5)}});
    EXPECT_NEAR(constant.mean, DECIMAL(0.5), DECIMAL(1e-12));
    EXPECT_EQ(constant.standardDeviation, DECIMAL(0.0));
    EXPECT_EQ(constant.skewness, DECIMAL(0.0));
    EXPECT_EQ(constant.kurtosis, DECIMAL(0.0));
}

// With the true calibration, refining each image's attitude removes the prior
// attitude error and leaves no residual.
TEST(EvaluateHoldoutTest, TrueCalibrationLeavesNoResidual) {
    const CameraParameters camera = LensCamera();
    const CentroidObservations observations = LensImages();

    const HoldoutEvaluation evaluation = EvaluateHoldout(observations, camera);

    ASSERT_EQ(evaluation.residuals.size(), observations.observations.size());
    for (const std::optional<found::Vec2> &residual : evaluation.residuals) {
        ASSERT_TRUE(residual.has_value());
        EXPECT_LT(residual->norm(), kPixelTol);
    }

    const std::vector<found::Quaternion> truth = TrueAttitudes();
    ASSERT_EQ(evaluation.attitudes.size(), truth.size());
    for (std::size_t k = 0; k < truth.size(); ++k) {
        ASSERT_TRUE(evaluation.attitudes[k].has_value());
        EXPECT_LT(AngleBetween(*evaluation.attitudes[k], truth[k]), DECIMAL(1e-9));
    }

    EXPECT_EQ(evaluation.statistics.count, observations.observations.size());
    EXPECT_LT(evaluation.statistics.standardDeviation, kPixelTol);
}

// Leaving out the lens's distortion leaves residuals no attitude can remove.
TEST(EvaluateHoldoutTest, WrongCalibrationLeavesResidual) {
    const CentroidObservations observations = LensImages();
    CameraParameters noDistortion = LensCamera();
    noDistortion.k1 = DECIMAL(0.0);
    noDistortion.k2 = DECIMAL(0.0);

    const HoldoutEvaluation evaluation = EvaluateHoldout(observations, noDistortion);

    EXPECT_EQ(evaluation.statistics.count, observations.observations.size());
    EXPECT_GT(evaluation.statistics.standardDeviation, DECIMAL(10.0));
}

// Over a narrow field a shifted principal point is almost exactly an attitude
// error (paper Eqs. 45-48), so the refined attitudes absorb nearly all of it.
// The flight lens's field is about three times wider and strongly distorted,
// which leaves several times more behind: there the principal point can be
// told apart from the attitude.
TEST(EvaluateHoldoutTest, PrincipalPointShiftIsAbsorbedOnlyInNarrowFields) {
    const decimal shift = DECIMAL(5.0);

    const CameraParameters paper = PaperPinholeCamera();
    CameraParameters paperShifted = paper;
    paperShifted.principalX += shift;
    paperShifted.principalY -= shift;
    const CentroidObservations paperImages = TwoImages(paper, HalfField(paper, kPaperWidth, kPaperHeight));
    const HoldoutEvaluation narrow = EvaluateHoldout(paperImages, paperShifted);

    CameraParameters lensShifted = LensCamera();
    lensShifted.principalX += shift;
    lensShifted.principalY -= shift;
    const HoldoutEvaluation wide = EvaluateHoldout(LensImages(), lensShifted);

    EXPECT_LT(narrow.statistics.standardDeviation, DECIMAL(0.02) * shift);
    EXPECT_GT(wide.statistics.standardDeviation, DECIMAL(0.06) * shift);
}

// Images that cannot be evaluated are reported as such; the others still are.
TEST(EvaluateHoldoutTest, SkipsImagesWithTooFewStars) {
    const CameraParameters camera = LensCamera();
    CentroidObservations observations = LensImages();
    // A third image with only two stars.
    observations.attitudes.push_back(found::Quaternion::Identity());
    for (int i = 0; i < 2; ++i) {
        const found::Vec3 eI = found::Vec3(DECIMAL(0.1) * DECIMAL(i), DECIMAL(0.0), DECIMAL(1.0)).normalized();
        const std::optional<found::Vec2> pixel = ProjectStarToPixel(eI, found::Quaternion::Identity(), camera);
        ASSERT_TRUE(pixel.has_value());
        observations.observations.push_back(Observation{2, 100 + i, eI, *pixel});
    }

    const HoldoutEvaluation evaluation = EvaluateHoldout(observations, camera);

    EXPECT_TRUE(evaluation.attitudes[0].has_value());
    EXPECT_TRUE(evaluation.attitudes[1].has_value());
    EXPECT_FALSE(evaluation.attitudes[2].has_value());
    EXPECT_FALSE(evaluation.residuals.back().has_value());
    EXPECT_EQ(evaluation.statistics.count, observations.observations.size() - 2);
}

TEST(EvaluateHoldoutTest, SkipsImageWithStarBehindCamera) {
    const CameraParameters camera = LensCamera();
    CentroidObservations observations = LensImages();
    const found::Vec3 antiBoresight(DECIMAL(0.0), DECIMAL(0.0), DECIMAL(-1.0));
    const found::Vec3 behind = observations.attitudes[0].conjugate() * antiBoresight;
    observations.observations.push_back(Observation{0, 999, behind, found::Vec2{DECIMAL(10.0), DECIMAL(10.0)}});

    const HoldoutEvaluation evaluation = EvaluateHoldout(observations, camera);

    EXPECT_FALSE(evaluation.attitudes[0].has_value());
    EXPECT_TRUE(evaluation.attitudes[1].has_value());
}

// Three sightings of one star fix only two axes: the roll about it is free.
TEST(EvaluateHoldoutTest, SkipsImageWhoseStarsDoNotFixAttitude) {
    const CameraParameters camera = LensCamera();
    CentroidObservations observations;
    observations.attitudes.push_back(found::Quaternion::Identity());
    const found::Vec3 eI = found::Vec3(DECIMAL(0.2), DECIMAL(0.1), DECIMAL(1.0)).normalized();
    const std::optional<found::Vec2> pixel = ProjectStarToPixel(eI, found::Quaternion::Identity(), camera);
    ASSERT_TRUE(pixel.has_value());
    for (int i = 0; i < 3; ++i) {
        observations.observations.push_back(Observation{0, 1, eI, *pixel});
    }

    const HoldoutEvaluation evaluation = EvaluateHoldout(observations, camera);

    EXPECT_FALSE(evaluation.attitudes[0].has_value());
    EXPECT_EQ(evaluation.statistics.count, 0u);
}

// Too few iterations to converge gives no answer rather than a wrong one.
TEST(EvaluateHoldoutTest, ReportsNonConvergence) {
    const CameraParameters camera = LensCamera();
    const CentroidObservations observations = LensImages();

    const HoldoutEvaluation evaluation = EvaluateHoldout(observations, camera, 1);

    EXPECT_FALSE(evaluation.attitudes[0].has_value());
    EXPECT_FALSE(evaluation.attitudes[1].has_value());
    EXPECT_EQ(evaluation.statistics.count, 0u);
}

TEST(EvaluateHoldoutTest, ThrowsOnObservationWithoutAttitude) {
    const CameraParameters camera = LensCamera();
    CentroidObservations observations = LensImages();
    observations.observations.push_back(
        Observation{2, 1, found::Vec3(DECIMAL(0.0), DECIMAL(0.0), DECIMAL(1.0)), found::Vec2::Zero()});
    EXPECT_THROW(EvaluateHoldout(observations, camera), std::invalid_argument);

    observations.observations.back().imageIndex = -1;
    EXPECT_THROW(EvaluateHoldout(observations, camera), std::invalid_argument);
}

}  // namespace scope
