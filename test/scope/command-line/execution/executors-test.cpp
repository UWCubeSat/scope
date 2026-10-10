#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>
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
#include "scope/optimization/optimization.hpp"
#include "scope/projection/projection.hpp"
#include "scope/star-centroid/star-centroid.hpp"

#include "test/scope/common/test-files.hpp"

namespace scope {

namespace {

constexpr int kWidth = 64;
constexpr int kHeight = 64;

/// An optimization stage that ignores its input and returns a fixed result, so
/// the executor's output can be checked against known values.
class FixedResultOptimization : public OptimizationAlgorithm {
 public:
    explicit FixedResultOptimization(const CalibrationResult &result) : result_(result) {}

    CalibrationResult Run(const CentroidObservations &) override { return result_; }

 private:
    const CalibrationResult result_;
};

}  // namespace

// The executor runs all three stages over the images in its options, prints the
// result, and releases those images when it is destroyed (the sanitizer and
// valgrind runs fail this test if it does not).
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
    std::unique_ptr<OptimizationAlgorithm> optimizationAlgorithm = std::make_unique<LMAOptimizationAlgorithm>(options);
    PrimaryScopePipelineExecutor executor(std::move(options),
                                          std::make_unique<DarkScreenFilter>(),
                                          std::move(starAlgorithm),
                                          std::move(optimizationAlgorithm));

    testing::internal::CaptureStdout();
    executor.ExecutePipeline();
    executor.OutputResults();
    const std::string output = testing::internal::GetCapturedStdout();

    EXPECT_THAT(output, testing::HasSubstr("Star image 0: 1 of 1 centroids kept"));
    // The optimizer is a stub, so what gets printed is an empty, unconverged result.
    EXPECT_THAT(output, testing::HasSubstr("converged: no"));
    EXPECT_THAT(output, testing::HasSubstr("residual-rms: 0 px over 0 observations"));
}

// OutputResults prints what the optimization stage produced: whether it
// converged, each parameter under the name of its command-line flag, and the
// residual RMS with the number of observations behind it. The values are exactly
// representable, so they print the same in double and float builds.
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
    result.residuals = {found::Vec2(DECIMAL(0.25), DECIMAL(-0.25)), found::Vec2(DECIMAL(-0.25), DECIMAL(0.25))};
    result.residualRms = DECIMAL(0.25);

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

    EXPECT_THAT(output, testing::HasSubstr("converged: yes\n"));
    EXPECT_THAT(output, testing::HasSubstr("focal-length-x: 2543.125\n"));
    EXPECT_THAT(output, testing::HasSubstr("alpha: 0.5\n"));
    EXPECT_THAT(output, testing::HasSubstr("focal-length-y: 2541.75\n"));
    EXPECT_THAT(output, testing::HasSubstr("principal-point-x: 1295.5\n"));
    EXPECT_THAT(output, testing::HasSubstr("principal-point-y: 1023.5\n"));
    EXPECT_THAT(output, testing::HasSubstr("k1: -0.25\n"));
    EXPECT_THAT(output, testing::HasSubstr("k2: 0.125\n"));
    EXPECT_THAT(output, testing::HasSubstr("k3: -0.0625\n"));
    EXPECT_THAT(output, testing::HasSubstr("p1: 0.03125\n"));
    EXPECT_THAT(output, testing::HasSubstr("p2: -0.015625\n"));
    EXPECT_THAT(output, testing::HasSubstr("residual-rms: 0.25 px over 2 observations\n"));
}

}  // namespace scope
