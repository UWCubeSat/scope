#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/catalog/catalog.hpp"
#include "scope/command-line/execution/executors.hpp"
#include "scope/command-line/parsing/options.hpp"
#include "scope/common/style.hpp"
#include "scope/noise-filter/noise-filter.hpp"
#include "scope/optimization/calibration-fit.hpp"
#include "scope/optimization/optimization.hpp"
#include "scope/projection/projection.hpp"
#include "scope/providers/factory.hpp"
#include "scope/providers/stage-providers.hpp"
#include "scope/star-centroid/star-centroid.hpp"

#include "test/scope/common/test-files.hpp"

namespace scope {

namespace {

/// Attitude that turns inertial +x onto the camera boresight (+z), so the
/// fixture catalog's first star lands on the principal point.
const char *kInertialXOnBoresight = "0.7071067811865476 0 -0.7071067811865476 0\n";

/// Pinhole options (focal 100, principal at the image center), no distortion.
RecalibrationOptions CenteredOptions() {
    RecalibrationOptions options;
    options.focalLengthX = DECIMAL(100.0);
    options.focalLengthY = DECIMAL(100.0);
    options.principalX = DECIMAL(32.0);
    options.principalY = DECIMAL(32.0);
    options.catalogPath = kFixtureCatalog;
    return options;
}

}  // namespace

// The values are all different, so a swapped pair would show.
TEST(ProvideCameraParametersTest, CopiesEveryPriorParameter) {
    RecalibrationOptions options;
    options.focalLengthX = DECIMAL(1.0);
    options.alpha = DECIMAL(2.0);
    options.focalLengthY = DECIMAL(3.0);
    options.principalX = DECIMAL(4.0);
    options.principalY = DECIMAL(5.0);
    options.k1 = DECIMAL(6.0);
    options.k2 = DECIMAL(7.0);
    options.k3 = DECIMAL(8.0);
    options.p1 = DECIMAL(9.0);
    options.p2 = DECIMAL(10.0);

    const CameraParameters camera = ProvideCameraParameters(options);

    EXPECT_EQ(camera.focalLengthX, DECIMAL(1.0));
    EXPECT_EQ(camera.alpha, DECIMAL(2.0));
    EXPECT_EQ(camera.focalLengthY, DECIMAL(3.0));
    EXPECT_EQ(camera.principalX, DECIMAL(4.0));
    EXPECT_EQ(camera.principalY, DECIMAL(5.0));
    EXPECT_EQ(camera.k1, DECIMAL(6.0));
    EXPECT_EQ(camera.k2, DECIMAL(7.0));
    EXPECT_EQ(camera.k3, DECIMAL(8.0));
    EXPECT_EQ(camera.p1, DECIMAL(9.0));
    EXPECT_EQ(camera.p2, DECIMAL(10.0));
}

TEST(ProvideCameraParametersTest, ZeroFocalLengthWithStarImagesThrows) {
    std::vector<unsigned char> pixels = FlatPixels(kWidth, kHeight, 10);
    RecalibrationOptions options;
    options.focalLengthX = DECIMAL(100.0);
    options.starImages = {Image{kWidth, kHeight, 1, pixels.data()}};

    EXPECT_THROW(ProvideCameraParameters(options), std::runtime_error);
}

// The command line's default and FitSettings' own default agree: everything
// but the principal point is estimated.
TEST(ProvideFitSettingsTest, DefaultFixesPrincipalPoint) {
    const FitSettings settings = ProvideFitSettings(RecalibrationOptions());

    EXPECT_EQ(settings.estimate, FitSettings().estimate);
    EXPECT_FALSE(settings.estimate[3]);
    EXPECT_FALSE(settings.estimate[4]);
    EXPECT_EQ(settings.attitudeModel, AttitudeCorrectionModel::kPerImage);
    EXPECT_EQ(settings.outlierThreshold, FitSettings().outlierThreshold);
    EXPECT_EQ(settings.maxIterations, FitSettings().maxIterations);
}

TEST(ProvideFitSettingsTest, ParsesFixedParameters) {
    RecalibrationOptions options;
    options.fixedParameters = "k3, p2,alpha";

    const FitSettings settings = ProvideFitSettings(options);

    for (int p = 0; p < kCameraParameterCount; ++p) {
        const std::string name = kCameraParameterNames[p];
        EXPECT_EQ(settings.estimate[p], name != "k3" && name != "p2" && name != "alpha") << name;
    }
}

TEST(ProvideFitSettingsTest, NoneAndAll) {
    RecalibrationOptions options;
    options.fixedParameters = "none";
    for (const bool estimated : ProvideFitSettings(options).estimate) {
        EXPECT_TRUE(estimated);
    }
    options.fixedParameters = "all";
    for (const bool estimated : ProvideFitSettings(options).estimate) {
        EXPECT_FALSE(estimated);
    }
}

TEST(ProvideFitSettingsTest, ParsesTheRest) {
    RecalibrationOptions options;
    options.attitudeCorrection = "shared";
    options.outlierThreshold = DECIMAL(3.5);
    options.maxIterations = 12;

    const FitSettings settings = ProvideFitSettings(options);

    EXPECT_EQ(settings.attitudeModel, AttitudeCorrectionModel::kShared);
    EXPECT_EQ(settings.outlierThreshold, DECIMAL(3.5));
    EXPECT_EQ(settings.maxIterations, 12);
}

TEST(ProvideFitSettingsTest, RejectsBadValues) {
    RecalibrationOptions unknownParameter;
    unknownParameter.fixedParameters = "focal-length";
    RecalibrationOptions unknownModel;
    unknownModel.attitudeCorrection = "per-frame";
    RecalibrationOptions negativeThreshold;
    negativeThreshold.outlierThreshold = DECIMAL(-1.0);
    RecalibrationOptions noIterations;
    noIterations.maxIterations = 0;

    EXPECT_THROW(ProvideFitSettings(unknownParameter), std::runtime_error);
    EXPECT_THROW(ProvideFitSettings(unknownModel), std::runtime_error);
    EXPECT_THROW(ProvideFitSettings(negativeThreshold), std::runtime_error);
    EXPECT_THROW(ProvideFitSettings(noIterations), std::runtime_error);
}

TEST(ProvideAttitudesTest, LoadsAttitudesFile) {
    TempFile file("providers-attitudes.txt", "1 0 0 0\n0 0 0 1\n");
    RecalibrationOptions options;
    options.attitudesPath = file.Path();

    std::vector<found::Quaternion> attitudes = ProvideAttitudes(options);

    ASSERT_EQ(attitudes.size(), 2u);
    EXPECT_NEAR(attitudes[0].w(), DECIMAL(1.0), DECIMAL(1e-6));
    EXPECT_NEAR(attitudes[1].z(), DECIMAL(1.0), DECIMAL(1e-6));
}

TEST(ProvideAttitudesTest, NoStarImagesNeedNoAttitudes) {
    EXPECT_TRUE(ProvideAttitudes(RecalibrationOptions()).empty());
}

// Star images with no attitudes file is an error, not a silent identity.
TEST(ProvideAttitudesTest, ThrowsWhenStarImagesHaveNoAttitudes) {
    std::vector<unsigned char> pixels = FlatPixels(kWidth, kHeight, 10);
    RecalibrationOptions options;
    options.starImages = {Image{kWidth, kHeight, 1, pixels.data()}};

    EXPECT_THROW(ProvideAttitudes(options), std::runtime_error);
}

// The catalog star at inertial +x is only on the sensor because the attitude
// in the file turns +x onto the boresight.
TEST(FactoryTest, BuildsExecutorThatUsesCatalogAndAttitudes) {
    std::vector<unsigned char> dark = FlatPixels(kWidth, kHeight, 10);
    std::vector<unsigned char> star = dark;
    PaintStar(&star, kWidth, 32, 32);
    TempFile attitudes("providers-factory-attitudes.txt", kInertialXOnBoresight);

    // The executor takes ownership of the images, so they must be malloc'd.
    RecalibrationOptions options = CenteredOptions();
    options.darkFrames = {MallocImage(kWidth, kHeight, dark), MallocImage(kWidth, kHeight, dark)};
    options.starImages = {MallocImage(kWidth, kHeight, star)};
    options.attitudesPath = attitudes.Path();

    std::unique_ptr<PrimaryScopePipelineExecutor> executor = CreatePrimaryScopePipelineExecutor(std::move(options));
    ASSERT_NE(executor, nullptr);

    testing::internal::CaptureStdout();
    executor->ExecutePipeline();
    const std::string output = testing::internal::GetCapturedStdout();

    EXPECT_THAT(output, testing::HasSubstr("Star image 0: 1 of 1 centroids kept"));
}

TEST(FactoryTest, MissingCatalogThrows) {
    RecalibrationOptions options;
    options.catalogPath = "test/fixtures/does-not-exist.tsv";

    EXPECT_THROW(CreatePrimaryScopePipelineExecutor(std::move(options)), std::runtime_error);
}

TEST(FactoryTest, MissingAttitudesThrows) {
    // No executor is built, so these stay owned by the test.
    std::vector<unsigned char> pixels = FlatPixels(kWidth, kHeight, 10);
    RecalibrationOptions options = CenteredOptions();
    options.starImages = {Image{kWidth, kHeight, 1, pixels.data()}};

    EXPECT_THROW(CreatePrimaryScopePipelineExecutor(std::move(options)), std::runtime_error);
}

}  // namespace scope
