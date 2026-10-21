#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

#include <algorithm>
#include <array>
#include <initializer_list>
#include <limits>
#include <optional>
#include <random>
#include <stdexcept>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/attitude/attitude.hpp"
#include "scope/catalog/catalog.hpp"
#include "scope/common/types.hpp"
#include "scope/optimization/calibration-fit.hpp"
#include "scope/projection/projection.hpp"
#include "scope/synthetic/synthetic.hpp"

#include "test/scope/common/cameras.hpp"

namespace scope {

namespace {

/// Tolerances for a noise-free fit, per parameter in kCameraParameterFields
/// order: what the convergence test leaves, with a wide allowance.
constexpr std::array<decimal, kCameraParameterCount> kExactTolerance = {DECIMAL(1e-5),
                                                                        DECIMAL(1e-5),
                                                                        DECIMAL(1e-5),
                                                                        DECIMAL(1e-5),
                                                                        DECIMAL(1e-5),
                                                                        DECIMAL(1e-7),
                                                                        DECIMAL(1e-7),
                                                                        DECIMAL(1e-7),
                                                                        DECIMAL(1e-8),
                                                                        DECIMAL(1e-8)};

/// A sky of 600 stars (about 40 per frame) and six images taken by the
/// as-built camera, reported with 30" errors and a 60" misalignment.
class CalibrationFitTest : public testing::Test {
 protected:
    CalibrationFitTest()
        : random_(20261021),
          catalog_(RandomCatalog(600, DECIMAL(6.0), &random_)),
          scene_(RandomScene(AsBuiltCamera(),
                             kSensorWidth,
                             kSensorHeight,
                             6,
                             found::ArcSecToRad(DECIMAL(30.0)),
                             found::ArcSecToRad(DECIMAL(60.0)),
                             &random_)) {}

    CentroidObservations Observe(decimal noise) {
        return SimulateObservations(scene_, catalog_, kDefaultSearchMargin, noise, &random_);
    }

    /// The largest angle between a refined attitude and the truth.
    decimal WorstAttitudeError(const CalibrationResult &result) const {
        decimal worst = DECIMAL(0.0);
        for (std::size_t k = 0; k < scene_.trueAttitudes.size(); ++k) {
            worst = std::max(worst, result.attitudes[k].angularDistance(scene_.trueAttitudes[k]));
        }
        return worst;
    }

    Random random_;
    Catalog catalog_;
    SyntheticScene scene_;
};

void ExpectCameraNear(const CameraParameters &actual,
                      const CameraParameters &expected,
                      const std::array<decimal, kCameraParameterCount> &tolerance) {
    for (int p = 0; p < kCameraParameterCount; ++p) {
        EXPECT_NEAR(actual.*kCameraParameterFields[p], expected.*kCameraParameterFields[p], tolerance[p])
            << kCameraParameterNames[p];
    }
}

/// The settings that estimate every parameter except those listed.
FitSettings EstimateAllBut(std::initializer_list<int> fixed) {
    FitSettings settings;
    settings.estimate.fill(true);
    for (const int p : fixed) {
        settings.estimate[p] = false;
    }
    return settings;
}

constexpr int kFocalLengthY = 2;
constexpr int kPrincipalX = 3;
constexpr int kPrincipalY = 4;
constexpr int kK3 = 7;
constexpr int kP2 = 9;

}  // namespace

// From the datasheet prior, with exact centroids, the fit lands on the truth:
// the camera, every attitude, and the alignment correction.
TEST_F(CalibrationFitTest, RecoversNoiseFreeCalibration) {
    const CentroidObservations observations = Observe(DECIMAL(0.0));

    const CalibrationResult result = FitCalibration(observations, DatasheetCamera(), FitSettings());

    EXPECT_TRUE(result.converged);
    ExpectCameraNear(result.camera, AsBuiltCamera(), kExactTolerance);
    EXPECT_LT(WorstAttitudeError(result), DECIMAL(1e-9));
    EXPECT_LT(result.residualRms, DECIMAL(1e-6));
    EXPECT_EQ(std::count(result.inliers.begin(), result.inliers.end(), true),
              static_cast<std::ptrdiff_t>(observations.observations.size()));

    // The alignment correction is the mean of truth_k * prior_k^-1.
    std::vector<found::Quaternion> corrections;
    for (std::size_t k = 0; k < scene_.trueAttitudes.size(); ++k) {
        corrections.push_back(scene_.trueAttitudes[k] * scene_.priorAttitudes[k].conjugate());
    }
    EXPECT_LT(result.alignmentCorrection.angularDistance(AverageQuaternions(corrections)), DECIMAL(1e-9));
}

// With a 64-degree field the principal point is not interchangeable with a
// pointing error (LENS.md §6), so all ten parameters can be fitted at once.
TEST_F(CalibrationFitTest, RecoversPrincipalPointOnWideField) {
    CameraParameters truth = AsBuiltCamera();
    truth.principalX = DECIMAL(1037.0);
    truth.principalY = DECIMAL(767.25);
    scene_.camera = truth;
    const CentroidObservations observations = Observe(DECIMAL(0.0));

    const CalibrationResult result = FitCalibration(observations, DatasheetCamera(), EstimateAllBut({}));

    EXPECT_TRUE(result.converged);
    ExpectCameraNear(result.camera, truth, kExactTolerance);
    EXPECT_LT(WorstAttitudeError(result), DECIMAL(1e-9));
}

// With 0.15 px of centroid noise each parameter lands within a few of its
// reported standard deviations, and the residual RMS is the noise level.
TEST_F(CalibrationFitTest, NoisyFitIsWithinItsUncertainty) {
    const CentroidObservations observations = Observe(DECIMAL(0.15));

    const CalibrationResult result = FitCalibration(observations, DatasheetCamera(), FitSettings());

    EXPECT_TRUE(result.converged);
    for (int p = 0; p < kCameraParameterCount; ++p) {
        if (!result.estimated[p]) {
            continue;
        }
        const decimal sigma = result.uncertainty.*kCameraParameterFields[p];
        EXPECT_GT(sigma, DECIMAL(0.0)) << kCameraParameterNames[p];
        EXPECT_NEAR(
            result.camera.*kCameraParameterFields[p], AsBuiltCamera().*kCameraParameterFields[p], DECIMAL(4.0) * sigma)
            << kCameraParameterNames[p];
    }
    EXPECT_NEAR(result.residualRms, DECIMAL(0.15), DECIMAL(0.01));
}

TEST_F(CalibrationFitTest, FixedParametersKeepTheirPrior) {
    const CentroidObservations observations = Observe(DECIMAL(0.1));
    const CameraParameters prior = DatasheetCamera();
    FitSettings settings = EstimateAllBut({kFocalLengthY, kPrincipalX, kPrincipalY, kK3, kP2});
    // The fixed focal length leaves a misfit that rejection would chew on.
    settings.outlierThreshold = DECIMAL(0.0);

    const CalibrationResult result = FitCalibration(observations, prior, settings);

    EXPECT_EQ(result.estimated, settings.estimate);
    for (const int p : {kFocalLengthY, kPrincipalX, kPrincipalY, kK3, kP2}) {
        EXPECT_EQ(result.camera.*kCameraParameterFields[p], prior.*kCameraParameterFields[p]);
        EXPECT_EQ(result.uncertainty.*kCameraParameterFields[p], DECIMAL(0.0));
    }
    // The estimated focal length absorbs what it can of the fixed one's error.
    EXPECT_NE(result.camera.focalLengthX, prior.focalLengthX);
}

// Holding every camera parameter at a correct prior and refining only the
// attitudes is how held-out images are validated.
TEST_F(CalibrationFitTest, AttitudeOnlyFitRecoversAttitudes) {
    const CentroidObservations observations = Observe(DECIMAL(0.0));
    FitSettings settings;
    settings.estimate.fill(false);

    const CalibrationResult result = FitCalibration(observations, AsBuiltCamera(), settings);

    EXPECT_TRUE(result.converged);
    EXPECT_LT(WorstAttitudeError(result), DECIMAL(1e-9));
}

// Method 2 carries one correction for all images. With no per-image error it
// is exactly the misalignment.
TEST_F(CalibrationFitTest, SharedCorrectionRecoversMisalignment) {
    Random random(7);
    const SyntheticScene scene = RandomScene(
        AsBuiltCamera(), kSensorWidth, kSensorHeight, 6, DECIMAL(0.0), found::ArcSecToRad(DECIMAL(60.0)), &random);
    const CentroidObservations observations =
        SimulateObservations(scene, catalog_, kDefaultSearchMargin, DECIMAL(0.0), &random);
    FitSettings settings;
    settings.attitudeModel = AttitudeCorrectionModel::kShared;

    const CalibrationResult result = FitCalibration(observations, DatasheetCamera(), settings);

    EXPECT_TRUE(result.converged);
    ExpectCameraNear(result.camera, AsBuiltCamera(), kExactTolerance);
    EXPECT_LT(result.alignmentCorrection.angularDistance(scene.misalignment), DECIMAL(1e-9));
    for (std::size_t k = 0; k < scene.trueAttitudes.size(); ++k) {
        EXPECT_LT(result.attitudes[k].angularDistance(scene.trueAttitudes[k]), DECIMAL(1e-9));
    }
}

// Under Method 2 every image carries the one correction, including an image
// with no stars, which with no per-image error then lands on its truth.
TEST_F(CalibrationFitTest, SharedCorrectionReachesImageWithoutStars) {
    Random random(7);
    const SyntheticScene scene = RandomScene(
        AsBuiltCamera(), kSensorWidth, kSensorHeight, 6, DECIMAL(0.0), found::ArcSecToRad(DECIMAL(60.0)), &random);
    CentroidObservations observations =
        SimulateObservations(scene, catalog_, kDefaultSearchMargin, DECIMAL(0.0), &random);
    std::vector<Observation> kept;
    for (const Observation &observation : observations.observations) {
        if (observation.imageIndex != 2) {
            kept.push_back(observation);
        }
    }
    observations.observations = kept;
    FitSettings settings;
    settings.attitudeModel = AttitudeCorrectionModel::kShared;

    const CalibrationResult result = FitCalibration(observations, DatasheetCamera(), settings);

    EXPECT_TRUE(result.converged);
    EXPECT_LT(result.attitudes[2].angularDistance(scene.trueAttitudes[2]), DECIMAL(1e-9));
}

// An observation with no usable direction makes every trial's cost
// infinite. With rejection off, the fit stalls and must say so rather than
// report convergence; with rejection on, the observation is rejected and the
// rest converges.
TEST_F(CalibrationFitTest, StalledFitIsNotConverged) {
    CentroidObservations observations = Observe(DECIMAL(0.1));
    observations.observations[5].inertialDirection = found::Vec3::Constant(std::numeric_limits<decimal>::quiet_NaN());
    FitSettings keepAll;
    keepAll.outlierThreshold = DECIMAL(0.0);

    const CalibrationResult stalled = FitCalibration(observations, DatasheetCamera(), keepAll);
    const CalibrationResult rejected = FitCalibration(observations, DatasheetCamera(), FitSettings());

    EXPECT_FALSE(stalled.converged);
    EXPECT_TRUE(rejected.converged);
    EXPECT_FALSE(rejected.inliers[5]);
    EXPECT_NEAR(rejected.residualRms, DECIMAL(0.1), DECIMAL(0.01));
}

// Wrong matches (a centroid of the wrong star) are rejected and the fit is as
// good as without them; with rejection off they pull the fit away.
TEST_F(CalibrationFitTest, RejectsOutliers) {
    CentroidObservations observations = Observe(DECIMAL(0.1));
    std::vector<bool> corrupted(observations.observations.size(), false);
    std::uniform_real_distribution<decimal> angle(DECIMAL(0.0), DECIMAL(2.0) * DECIMAL_M_PI);
    for (std::size_t i = 0; i < observations.observations.size(); i += 20) {
        const decimal a = angle(random_);
        observations.observations[i].measuredPixel += DECIMAL(12.0) * found::Vec2(DECIMAL_COS(a), DECIMAL_SIN(a));
        corrupted[i] = true;
    }

    const CalibrationResult result = FitCalibration(observations, DatasheetCamera(), FitSettings());

    EXPECT_TRUE(result.converged);
    std::size_t cleanRejected = 0;
    for (std::size_t i = 0; i < corrupted.size(); ++i) {
        if (corrupted[i]) {
            EXPECT_FALSE(result.inliers[i]) << "observation " << i;
        } else if (!result.inliers[i]) {
            ++cleanRejected;
        }
    }
    EXPECT_LT(cleanRejected, corrupted.size() / 100);
    EXPECT_NEAR(result.residualRms, DECIMAL(0.1), DECIMAL(0.01));
    EXPECT_NEAR(
        result.camera.focalLengthX, AsBuiltCamera().focalLengthX, DECIMAL(4.0) * result.uncertainty.focalLengthX);

    FitSettings keepAll;
    keepAll.outlierThreshold = DECIMAL(0.0);
    const CalibrationResult contaminated = FitCalibration(observations, DatasheetCamera(), keepAll);
    EXPECT_EQ(std::count(contaminated.inliers.begin(), contaminated.inliers.end(), false), 0);
    EXPECT_GT(contaminated.residualRms, DECIMAL(1.0));
}

// An image with only two stars cannot pin its three attitude angles. It is
// left out, its attitude is the others' average correction applied to its
// prior, and the rest is fitted as usual.
TEST_F(CalibrationFitTest, LeavesOutImageWithTooFewStars) {
    CentroidObservations observations = Observe(DECIMAL(0.0));
    std::vector<Observation> kept;
    int imageZero = 0;
    for (const Observation &observation : observations.observations) {
        if (observation.imageIndex != 0 || imageZero++ < 2) {
            kept.push_back(observation);
        }
    }
    observations.observations = kept;

    const CalibrationResult result = FitCalibration(observations, DatasheetCamera(), FitSettings());

    EXPECT_TRUE(result.converged);
    EXPECT_LT(result.attitudes[0].angularDistance(result.alignmentCorrection * scene_.priorAttitudes[0]),
              DECIMAL(1e-12));
    EXPECT_FALSE(result.inliers[0]);
    EXPECT_FALSE(result.inliers[1]);
    EXPECT_TRUE(result.inliers[2]);
    ExpectCameraNear(result.camera, AsBuiltCamera(), kExactTolerance);
}

// Measured minus predicted at the fitted state, and an RMS over the inliers'
// components.
TEST_F(CalibrationFitTest, ResidualsAreMeasuredMinusPredicted) {
    const CentroidObservations observations = Observe(DECIMAL(0.2));

    const CalibrationResult result = FitCalibration(observations, DatasheetCamera(), FitSettings());

    ASSERT_EQ(result.residuals.size(), observations.observations.size());
    decimal sum = DECIMAL(0.0);
    int count = 0;
    for (std::size_t i = 0; i < observations.observations.size(); ++i) {
        const Observation &observation = observations.observations[i];
        const std::optional<found::Vec2> predicted =
            ProjectStarToPixel(observation.inertialDirection, result.attitudes[observation.imageIndex], result.camera);
        ASSERT_TRUE(predicted.has_value());
        EXPECT_NEAR(
            (result.residuals[i] - (observation.measuredPixel - *predicted)).norm(), DECIMAL(0.0), DECIMAL(1e-9));
        if (result.inliers[i]) {
            sum += result.residuals[i].squaredNorm();
            count += 2;
        }
    }
    EXPECT_NEAR(result.residualRms, DECIMAL_SQRT(sum / DECIMAL(count)), DECIMAL(1e-12));
}

// With one round, the outliers the first fit finds are not refitted without.
TEST_F(CalibrationFitTest, StopsRejectingAfterMaxRounds) {
    CentroidObservations observations = Observe(DECIMAL(0.1));
    observations.observations[3].measuredPixel += found::Vec2(DECIMAL(20.0), DECIMAL(0.0));
    FitSettings oneRound;
    oneRound.maxRejectionRounds = 1;

    const CalibrationResult result = FitCalibration(observations, DatasheetCamera(), oneRound);

    EXPECT_TRUE(result.converged);
    EXPECT_TRUE(result.inliers[3]);
}

TEST_F(CalibrationFitTest, StopsAtMaxIterations) {
    const CentroidObservations observations = Observe(DECIMAL(0.1));
    FitSettings settings;
    settings.maxIterations = 1;
    settings.outlierThreshold = DECIMAL(0.0);

    const CalibrationResult result = FitCalibration(observations, DatasheetCamera(), settings);

    EXPECT_FALSE(result.converged);
    EXPECT_EQ(result.iterations, 1);
}

// Two observations cannot determine 13 states: the prior comes back, not
// converged, with the residuals it predicts.
TEST_F(CalibrationFitTest, TooFewObservationsReturnsPrior) {
    CentroidObservations observations = Observe(DECIMAL(0.0));
    observations.observations.resize(2);

    const CalibrationResult result = FitCalibration(observations, DatasheetCamera(), FitSettings());

    EXPECT_FALSE(result.converged);
    EXPECT_EQ(result.iterations, 0);
    ExpectCameraNear(result.camera, DatasheetCamera(), {});
    EXPECT_EQ(result.attitudes.size(), observations.attitudes.size());
    EXPECT_EQ(result.residuals.size(), 2u);
    EXPECT_EQ(std::count(result.inliers.begin(), result.inliers.end(), true), 0);
    EXPECT_TRUE(std::isnan(result.uncertainty.focalLengthX));
    EXPECT_EQ(result.uncertainty.principalX, DECIMAL(0.0));
}

TEST_F(CalibrationFitTest, NoObservationsReturnsPrior) {
    CentroidObservations observations;
    observations.attitudes = scene_.priorAttitudes;

    const CalibrationResult result = FitCalibration(observations, DatasheetCamera(), FitSettings());

    EXPECT_FALSE(result.converged);
    EXPECT_TRUE(result.residuals.empty());
    EXPECT_EQ(result.alignmentCorrection.coeffs(), found::Quaternion::Identity().coeffs());
}

TEST_F(CalibrationFitTest, ThrowsOnObservationWithoutAttitude) {
    CentroidObservations observations = Observe(DECIMAL(0.0));
    observations.attitudes.pop_back();

    EXPECT_THROW(FitCalibration(observations, DatasheetCamera(), FitSettings()), std::invalid_argument);
}

}  // namespace scope
