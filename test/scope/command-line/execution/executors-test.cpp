#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/attitude/attitude.hpp"
#include "scope/catalog/catalog.hpp"
#include "scope/command-line/execution/executors.hpp"
#include "scope/command-line/parsing/options.hpp"
#include "scope/common/style.hpp"
#include "scope/noise-filter/noise-filter.hpp"
#include "scope/optimization/calibration-fit.hpp"
#include "scope/optimization/optimization.hpp"
#include "scope/projection/projection.hpp"
#include "scope/star-centroid/star-centroid.hpp"

#include "test/scope/common/test-files.hpp"

namespace scope {

namespace {

/// An optimization stage that returns a fixed result.
class FixedResultOptimization : public OptimizationAlgorithm {
 public:
    explicit FixedResultOptimization(const CalibrationResult &result) : result_(result) {}

    CalibrationResult Run(const CentroidObservations &) override { return result_; }

 private:
    const CalibrationResult result_;
};

}  // namespace

// The sanitizer fails this test if the executor does not free the images in
// its options.
TEST(ExecutorsTest, RunsPipelineAndReleasesImages) {
    std::vector<unsigned char> dark = FlatPixels(kWidth, kHeight, 10);
    std::vector<unsigned char> star = dark;
    PaintStar(&star, kWidth, 40, 36);

    // Pinhole camera (focal 100, principal at (32, 32)), no distortion.
    CameraParameters camera;
    camera.focalLengthX = DECIMAL(100.0);
    camera.focalLengthY = DECIMAL(100.0);
    camera.principalX = DECIMAL(32.0);
    camera.principalY = DECIMAL(32.0);

    RecalibrationOptions options;
    options.darkFrames = {MallocImage(kWidth, kHeight, dark), MallocImage(kWidth, kHeight, dark)};
    options.starImages = {MallocImage(kWidth, kHeight, star)};

    // Under identity attitude this direction projects to (40, 36).
    Catalog catalog;
    catalog.push_back(CatalogStar{found::Vec3(DECIMAL(0.08), DECIMAL(0.04), DECIMAL(1.0)).normalized(), 200, 1});
    std::vector<found::Quaternion> attitudes{found::Quaternion::Identity()};

    std::unique_ptr<StarCentroidAlgorithm> starAlgorithm =
        std::make_unique<ROIFilterAlgorithm>(options, camera, std::move(catalog), std::move(attitudes));
    std::unique_ptr<OptimizationAlgorithm> optimizationAlgorithm =
        std::make_unique<LMAOptimizationAlgorithm>(camera, FitSettings());
    PrimaryScopePipelineExecutor executor(std::move(options),
                                          std::make_unique<DarkScreenFilter>(),
                                          std::move(starAlgorithm),
                                          std::move(optimizationAlgorithm));

    testing::internal::CaptureStdout();
    executor.ExecutePipeline();
    executor.OutputResults();
    const std::string output = testing::internal::GetCapturedStdout();

    EXPECT_THAT(output, testing::HasSubstr("Star image 0: 1 of 1 centroids kept"));
    // One star is too few to fit, so the prior is printed, unconverged.
    EXPECT_THAT(output, testing::HasSubstr("converged: no after 0 iterations\n"));
    EXPECT_THAT(output, testing::HasSubstr("focal-length-x: 100 +/- nan\n"));
    EXPECT_THAT(output, testing::HasSubstr("residual-rms: 0 px over 0 observations (1 not used)\n"));
}

// --output-attitudes and --output-residuals write what the fit produced: the
// attitudes in the --attitudes format, and one CSV row per observation.
TEST(ExecutorsTest, WritesAttitudesAndResiduals) {
    std::vector<unsigned char> dark = FlatPixels(kWidth, kHeight, 10);
    std::vector<unsigned char> star = dark;
    PaintStar(&star, kWidth, 40, 36);
    CameraParameters camera;
    camera.focalLengthX = DECIMAL(100.0);
    camera.focalLengthY = DECIMAL(100.0);
    camera.principalX = DECIMAL(32.0);
    camera.principalY = DECIMAL(32.0);
    TempFile attitudes("executors-refined-attitudes.txt", "");
    TempFile residuals("executors-residuals.csv", "");

    RecalibrationOptions options;
    options.darkFrames = {MallocImage(kWidth, kHeight, dark)};
    options.starImages = {MallocImage(kWidth, kHeight, star)};
    options.outputAttitudesPath = attitudes.Path();
    options.outputResidualsPath = residuals.Path();
    Catalog catalog;
    catalog.push_back(CatalogStar{found::Vec3(DECIMAL(0.08), DECIMAL(0.04), DECIMAL(1.0)).normalized(), 200, 17});
    std::unique_ptr<StarCentroidAlgorithm> starAlgorithm = std::make_unique<ROIFilterAlgorithm>(
        options, camera, std::move(catalog), std::vector<found::Quaternion>{found::Quaternion::Identity()});
    PrimaryScopePipelineExecutor executor(std::move(options),
                                          std::make_unique<DarkScreenFilter>(),
                                          std::move(starAlgorithm),
                                          std::make_unique<LMAOptimizationAlgorithm>(camera, FitSettings()));

    testing::internal::CaptureStdout();
    executor.ExecutePipeline();
    executor.OutputResults();
    testing::internal::GetCapturedStdout();

    const std::vector<found::Quaternion> written = LoadAttitudes(attitudes.Path());
    ASSERT_EQ(written.size(), 1u);
    EXPECT_NEAR(written[0].w(), DECIMAL(1.0), DECIMAL(1e-12));
    std::ifstream file(residuals.Path());
    std::string header;
    std::string row;
    std::getline(file, header);
    std::getline(file, row);
    EXPECT_EQ(header, "image,star,measured_u,measured_v,predicted_u,predicted_v,residual_u,residual_v,inlier");
    EXPECT_THAT(row, testing::StartsWith("0,17,40,36,40,36,"));
    EXPECT_THAT(row, testing::EndsWith(",0"));
}

// A directory that does not exist, and /dev/full, which opens but fails
// every write as a full disk would.
TEST(ExecutorsTest, UnwritableOutputThrows) {
    std::vector<std::string> paths = {"test/fixtures/no-such-directory/residuals.csv"};
    if (std::ifstream("/dev/full")) {
        paths.push_back("/dev/full");
    }
    for (const std::string &path : paths) {
        std::vector<unsigned char> dark = FlatPixels(kWidth, kHeight, 10);
        RecalibrationOptions options;
        options.darkFrames = {MallocImage(kWidth, kHeight, dark)};
        options.outputResidualsPath = path;
        std::unique_ptr<StarCentroidAlgorithm> starAlgorithm = std::make_unique<ROIFilterAlgorithm>(
            options, CameraParameters(), Catalog(), std::vector<found::Quaternion>());
        PrimaryScopePipelineExecutor executor(
            std::move(options),
            std::make_unique<DarkScreenFilter>(),
            std::move(starAlgorithm),
            std::make_unique<LMAOptimizationAlgorithm>(CameraParameters(), FitSettings()));

        testing::internal::CaptureStdout();
        executor.ExecutePipeline();
        EXPECT_THROW(executor.OutputResults(), std::runtime_error) << path;
        testing::internal::GetCapturedStdout();
    }
}

// The values are exactly representable, so they print the same in double and
// float builds.
TEST(ExecutorsTest, PrintsCalibrationResult) {
    CalibrationResult result;
    result.converged = true;
    result.camera.focalLengthX = DECIMAL(2543.125);
    result.camera.alpha = DECIMAL(0.5);
    result.camera.focalLengthY = DECIMAL(2541.75);
    result.camera.principalX = DECIMAL(1295.5);
    result.camera.principalY = DECIMAL(1023.5);
    result.camera.k1 = DECIMAL(-0.25);
    result.camera.k2 = DECIMAL(0.125);
    result.camera.k3 = DECIMAL(-0.0625);
    result.camera.p1 = DECIMAL(0.03125);
    result.camera.p2 = DECIMAL(-0.015625);
    result.estimated = {true, true, true, false, true, true, true, true, true, true};
    result.uncertainty.focalLengthX = DECIMAL(0.5);
    result.uncertainty.k1 = DECIMAL(0.00125);
    result.residuals = {found::Vec2(DECIMAL(0.25), DECIMAL(-0.25)),
                        found::Vec2(DECIMAL(-0.25), DECIMAL(0.25)),
                        found::Vec2(DECIMAL(9.0), DECIMAL(9.0))};
    result.inliers = {true, true, false};
    result.residualRms = DECIMAL(0.25);
    result.iterations = 7;
    // 10 arcsec about camera +z.
    result.alignmentCorrection =
        AttitudeCorrection(found::Vec3(DECIMAL(0.0), DECIMAL(0.0), found::ArcSecToRad(DECIMAL(10.0))));

    // No star images: the first two stages have nothing to find, and the fixed
    // result is what reaches the output.
    std::vector<unsigned char> dark = FlatPixels(kWidth, kHeight, 10);
    RecalibrationOptions options;
    options.darkFrames = {MallocImage(kWidth, kHeight, dark)};

    std::unique_ptr<StarCentroidAlgorithm> starAlgorithm =
        std::make_unique<ROIFilterAlgorithm>(options, CameraParameters(), Catalog(), std::vector<found::Quaternion>());
    PrimaryScopePipelineExecutor executor(std::move(options),
                                          std::make_unique<DarkScreenFilter>(),
                                          std::move(starAlgorithm),
                                          std::make_unique<FixedResultOptimization>(result));

    testing::internal::CaptureStdout();
    executor.ExecutePipeline();
    executor.OutputResults();
    const std::string output = testing::internal::GetCapturedStdout();

    EXPECT_THAT(output, testing::HasSubstr("converged: yes after 7 iterations\n"));
    EXPECT_THAT(output, testing::HasSubstr("focal-length-x: 2543.125 +/- 0.5\n"));
    EXPECT_THAT(output, testing::HasSubstr("alpha: 0.5 +/- 0\n"));
    EXPECT_THAT(output, testing::HasSubstr("focal-length-y: 2541.75 +/- 0\n"));
    EXPECT_THAT(output, testing::HasSubstr("principal-point-x: 1295.5 (fixed)\n"));
    EXPECT_THAT(output, testing::HasSubstr("principal-point-y: 1023.5 +/- 0\n"));
    EXPECT_THAT(output, testing::HasSubstr("k1: -0.25 +/- 0.00125\n"));
    EXPECT_THAT(output, testing::HasSubstr("k2: 0.125 +/- 0\n"));
    EXPECT_THAT(output, testing::HasSubstr("k3: -0.0625 +/- 0\n"));
    EXPECT_THAT(output, testing::HasSubstr("p1: 0.03125 +/- 0\n"));
    EXPECT_THAT(output, testing::HasSubstr("p2: -0.015625 +/- 0\n"));
    EXPECT_THAT(output, testing::HasSubstr("residual-rms: 0.25 px over 2 observations (1 not used)\n"));
    EXPECT_THAT(output, testing::HasSubstr("alignment-correction: 0 0 10 arcsec"));
    EXPECT_THAT(output,
                testing::HasSubstr("as-prior: --focal-length-x 2543.125 --alpha 0.5 --focal-length-y 2541.75 "
                                   "--principal-point-x 1295.5 --principal-point-y 1023.5 --k1 -0.25 --k2 0.125 "
                                   "--k3 -0.0625 --p1 0.03125 --p2 -0.015625\n"));
}

}  // namespace scope
