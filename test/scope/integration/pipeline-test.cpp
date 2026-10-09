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
#include "scope/star-centroid/star-centroid.hpp"

namespace scope {

namespace {

constexpr int kWidth = 64;
constexpr int kHeight = 64;

/// Owns a single-channel pixel buffer and exposes it as a FOUND Image.
class TestImage {
 public:
    explicit TestImage(unsigned char background) : pixels_(static_cast<std::size_t>(kWidth) * kHeight, background) {}

    void Set(int x, int y, unsigned char value) { pixels_[static_cast<std::size_t>(y) * kWidth + x] = value; }

    Image View() { return Image{kWidth, kHeight, 1, pixels_.data()}; }

 private:
    std::vector<unsigned char> pixels_;
};

}  // namespace

// Drives the full noise-filter -> star-centroid -> optimization pipeline on
// synthetic data and verifies it runs end-to-end, the star is centroided in the
// intermediate payload, and the (stub) optimizer is reached.
TEST(PipelineIntegrationTest, RunsEndToEnd) {
    // Two constant dark frames -> median dark frame of value 10.
    TestImage darkA(10);
    TestImage darkB(10);
    Images darkFrames = {darkA.View(), darkB.View()};

    // One star image: background 10 with a symmetric blob at (40, 36).
    TestImage star(10);
    star.Set(40, 36, 210);
    star.Set(39, 36, 110);
    star.Set(41, 36, 110);
    star.Set(40, 35, 110);
    star.Set(40, 37, 110);

    // Pinhole camera (focal 100, principal at (32, 32)), no distortion. A star
    // along DirectionForPixel projects to (40, 36).
    RecalibrationOptions options;
    options.focalLengthX = DECIMAL(100.0);
    options.focalLengthY = DECIMAL(100.0);
    options.principalX = DECIMAL(32.0);
    options.principalY = DECIMAL(32.0);
    options.alpha = DECIMAL(0.0);
    options.k1 = DECIMAL(0.0);
    options.k2 = DECIMAL(0.0);
    options.k3 = DECIMAL(0.0);
    options.p1 = DECIMAL(0.0);
    options.p2 = DECIMAL(0.0);
    options.centroidThreshold = 40;
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{found::Vec3(DECIMAL(0.08), DECIMAL(0.04), DECIMAL(1.0)).normalized(), 200, 1});

    // Wrap stages in their FunctionStage base type so the pipeline can deduce the
    // input/output types, keeping raw observers to inspect/free products later.
    std::unique_ptr<found::FunctionStage<Images, Image>> noiseStage = std::make_unique<DarkScreenFilter>();
    found::FunctionStage<Images, Image> *noisePtr = noiseStage.get();

    std::vector<found::Quaternion> attitudes{found::Quaternion::Identity()};
    std::unique_ptr<found::FunctionStage<Image, CentroidObservations>> starStage =
        std::make_unique<ROIFilterAlgorithm>(options, catalog, std::move(attitudes));
    found::FunctionStage<Image, CentroidObservations> *starPtr = starStage.get();

    std::unique_ptr<found::FunctionStage<CentroidObservations, std::vector<float>>> optStage =
        std::make_unique<LMAOptimizationAlgorithm>(options);

    PrimaryScopePipeline pipeline;
    pipeline.AddStage(std::move(noiseStage)).AddStage(std::move(starStage)).Complete(std::move(optStage));

    std::vector<float> result = pipeline.Run(darkFrames);

    // The optimizer is a stub, so the final product is empty -- but reaching it
    // means the whole chain ran.
    EXPECT_TRUE(result.empty());

    // Inspect the intermediate payload: the star was centroided at (40, 36).
    CentroidObservations *observations = starPtr->GetProduct();
    ASSERT_NE(observations, nullptr);
    ASSERT_EQ(observations->observations.size(), 1u);
    EXPECT_NEAR(observations->observations[0].measured_pixel.x(), DECIMAL(40.0), DECIMAL(1e-3));
    EXPECT_NEAR(observations->observations[0].measured_pixel.y(), DECIMAL(36.0), DECIMAL(1e-3));

    // The noise filter's dark frame is a malloc'd buffer owned by no one in this
    // WIP pipeline; free it here so the test stays clean under leak detection.
    Image *darkFrame = noisePtr->GetProduct();
    ASSERT_NE(darkFrame, nullptr);
    std::free(darkFrame->image);
}

}  // namespace scope
