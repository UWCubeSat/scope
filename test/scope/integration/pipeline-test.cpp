#include <gtest/gtest.h>

#include <cstdlib>

#include <memory>
#include <utility>
#include <vector>

#include "common/decimal.hpp"
#include "common/pipeline/stages.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/catalog/catalog.hpp"
#include "scope/command-line/parsing/options.hpp"
#include "scope/common/style.hpp"
#include "scope/noise-filter/noise-filter.hpp"
#include "scope/optimization/optimization.hpp"
#include "scope/projection/projection.hpp"
#include "scope/star-centroid/star-centroid.hpp"

#include "test/scope/common/test-files.hpp"

namespace scope {

TEST(PipelineIntegrationTest, RunsEndToEnd) {
    TestImage darkA(10);
    TestImage darkB(10);
    Images darkFrames = {darkA.View(), darkB.View()};

    TestImage star(10);
    star.PaintStar(40, 36);

    // Pinhole camera, no distortion; the catalog star below projects to (40, 36).
    CameraParameters camera;
    camera.focalLengthX = DECIMAL(100.0);
    camera.focalLengthY = DECIMAL(100.0);
    camera.principalX = DECIMAL(32.0);
    camera.principalY = DECIMAL(32.0);

    RecalibrationOptions options;
    options.centroidThreshold = 40;
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{found::Vec3(DECIMAL(0.08), DECIMAL(0.04), DECIMAL(1.0)).normalized(), 200, 1});

    // The pipeline deduces its types from the FunctionStage base type.
    std::unique_ptr<found::FunctionStage<Images, Image>> noiseStage = std::make_unique<DarkScreenFilter>();
    found::FunctionStage<Images, Image> *noisePtr = noiseStage.get();

    std::vector<found::Quaternion> attitudes{found::Quaternion::Identity()};
    std::unique_ptr<found::FunctionStage<Image, CentroidObservations>> starStage =
        std::make_unique<ROIFilterAlgorithm>(options, camera, catalog, std::move(attitudes));
    found::FunctionStage<Image, CentroidObservations> *starPtr = starStage.get();

    std::unique_ptr<found::FunctionStage<CentroidObservations, CalibrationResult>> optStage =
        std::make_unique<LMAOptimizationAlgorithm>(options);

    PrimaryScopePipeline pipeline;
    pipeline.AddStage(std::move(noiseStage)).AddStage(std::move(starStage)).Complete(std::move(optStage));

    CalibrationResult result = pipeline.Run(darkFrames);

    // The optimizer is a stub, so the result is empty and unconverged.
    EXPECT_FALSE(result.converged);
    EXPECT_TRUE(result.residuals.empty());

    CentroidObservations *observations = starPtr->GetProduct();
    ASSERT_NE(observations, nullptr);
    ASSERT_EQ(observations->observations.size(), 1u);
    EXPECT_NEAR(observations->observations[0].measuredPixel.x(), DECIMAL(40.0), DECIMAL(1e-3));
    EXPECT_NEAR(observations->observations[0].measuredPixel.y(), DECIMAL(36.0), DECIMAL(1e-3));

    // No stage owns the noise filter's malloc'd dark frame.
    Image *darkFrame = noisePtr->GetProduct();
    ASSERT_NE(darkFrame, nullptr);
    std::free(darkFrame->image);
}

}  // namespace scope
