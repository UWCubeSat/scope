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
#include "scope/star-centroid/star-centroid.hpp"

#include "test/scope/common/test-files.hpp"

namespace scope {

namespace {

constexpr int kWidth = 64;
constexpr int kHeight = 64;

}  // namespace

// The executor runs all three stages over the images in its options and
// releases those images when it is destroyed (the sanitizer and valgrind runs
// fail this test if it does not).
TEST(ExecutorsTest, RunsPipelineAndReleasesImages) {
    std::vector<unsigned char> dark = FlatPixels(kWidth, kHeight, 10);
    std::vector<unsigned char> star = dark;
    PaintStar(&star, kWidth, 40, 36);

    // Pinhole camera (focal 100, principal at (32, 32)), no distortion.
    RecalibrationOptions options;
    options.focalLengthX = DECIMAL(100.0);
    options.focalLengthY = DECIMAL(100.0);
    options.principalX = DECIMAL(32.0);
    options.principalY = DECIMAL(32.0);
    options.darkFrames = {MallocImage(kWidth, kHeight, dark), MallocImage(kWidth, kHeight, dark)};
    options.starImages = {MallocImage(kWidth, kHeight, star)};

    // Under identity attitude this direction projects to (40, 36).
    Catalog catalog;
    catalog.push_back(CatalogStar{found::Vec3(DECIMAL(0.08), DECIMAL(0.04), DECIMAL(1.0)).normalized(), 200, 1});
    std::vector<found::Quaternion> attitudes{found::Quaternion::Identity()};

    std::unique_ptr<StarCentroidAlgorithm> starAlgorithm =
        std::make_unique<ROIFilterAlgorithm>(options, std::move(catalog), std::move(attitudes));
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
}

}  // namespace scope
