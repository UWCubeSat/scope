#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdlib>

#include <memory>
#include <utility>
#include <vector>

#include "common/decimal.hpp"
#include "common/pipeline/stages.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/attitude/attitude.hpp"
#include "scope/catalog/catalog.hpp"
#include "scope/command-line/parsing/options.hpp"
#include "scope/common/style.hpp"
#include "scope/noise-filter/noise-filter.hpp"
#include "scope/optimization/calibration-fit.hpp"
#include "scope/optimization/optimization.hpp"
#include "scope/projection/projection.hpp"
#include "scope/star-centroid/star-centroid.hpp"
#include "scope/synthetic/synthetic.hpp"

#include "test/scope/common/cameras.hpp"

namespace scope {

namespace {

constexpr int kBinning = 8;
constexpr int kBinnedWidth = kSensorWidth / kBinning;
constexpr int kBinnedHeight = kSensorHeight / kBinning;

/// The camera read out with 8 x 8 binning, which keeps the frames small: the
/// same field and distortion, with an eighth of the focal length in pixels.
/// A binned pixel centre u covers full-resolution centres 8u to 8u + 7, so
/// u_binned = (u - 3.5) / 8.
CameraParameters Binned(const CameraParameters &camera) {
    const decimal binning = DECIMAL(kBinning);
    const decimal offset = (binning - DECIMAL(1.0)) / DECIMAL(2.0);
    CameraParameters binned = camera;
    binned.focalLengthX /= binning;
    binned.alpha /= binning;
    binned.focalLengthY /= binning;
    binned.principalX = (camera.principalX - offset) / binning;
    binned.principalY = (camera.principalY - offset) / binning;
    return binned;
}

}  // namespace

// Rendered images through all three stages: the dark frame median, stage 2's
// window search and centroiding, and the fit. The prior is the datasheet's and
// the attitudes are reported with 20" errors and a 300" misalignment, large
// enough that an alignment correction not recovered would stand out.
TEST(SyntheticPipelineTest, RecoversCalibrationFromRenderedImages) {
    Random random(2026);
    const Catalog catalog = RandomCatalog(2500, DECIMAL(6.5), &random);
    const CameraParameters truth = Binned(AsBuiltCamera());
    const CameraParameters prior = Binned(DatasheetCamera());
    const SyntheticScene scene = RandomScene(truth,
                                             kBinnedWidth,
                                             kBinnedHeight,
                                             6,
                                             found::ArcSecToRad(DECIMAL(20.0)),
                                             found::ArcSecToRad(DECIMAL(300.0)),
                                             &random);

    const SensorModel sensor;
    const std::vector<decimal> pattern = RandomFixedPattern(kBinnedWidth, kBinnedHeight, sensor, &random);
    std::vector<std::vector<unsigned char>> darkPixels;
    std::vector<std::vector<unsigned char>> starPixels;
    Images darkFrames;
    Images starImages;
    for (int i = 0; i < 2; ++i) {
        darkPixels.push_back(RenderDarkFrame(pattern, sensor, &random));
    }
    for (const found::Quaternion &attitude : scene.trueAttitudes) {
        starPixels.push_back(
            RenderStarImage(catalog, attitude, truth, pattern, kBinnedWidth, kBinnedHeight, sensor, &random));
    }
    for (std::vector<unsigned char> &pixels : darkPixels) {
        darkFrames.push_back(Image{kBinnedWidth, kBinnedHeight, 1, pixels.data()});
    }
    for (std::vector<unsigned char> &pixels : starPixels) {
        starImages.push_back(Image{kBinnedWidth, kBinnedHeight, 1, pixels.data()});
    }

    RecalibrationOptions options;
    options.starImages = starImages;
    // The datasheet prior is at most about 1.5 binned pixels off.
    options.roiSize = 7;

    std::unique_ptr<found::FunctionStage<Images, Image>> noiseStage = std::make_unique<DarkScreenFilter>();
    found::FunctionStage<Images, Image> *noisePtr = noiseStage.get();
    std::unique_ptr<found::FunctionStage<Image, CentroidObservations>> starStage =
        std::make_unique<ROIFilterAlgorithm>(options, prior, catalog, scene.priorAttitudes);
    std::unique_ptr<found::FunctionStage<CentroidObservations, CalibrationResult>> optStage =
        std::make_unique<LMAOptimizationAlgorithm>(prior, FitSettings());
    PrimaryScopePipeline pipeline;
    pipeline.AddStage(std::move(noiseStage)).AddStage(std::move(starStage)).Complete(std::move(optStage));

    const CalibrationResult result = pipeline.Run(darkFrames);
    std::free(noisePtr->GetProduct()->image);

    EXPECT_TRUE(result.converged);
    for (int p = 0; p < kCameraParameterCount; ++p) {
        if (!result.estimated[p]) {
            continue;
        }
        const decimal sigma = result.uncertainty.*kCameraParameterFields[p];
        EXPECT_GT(sigma, DECIMAL(0.0)) << kCameraParameterNames[p];
        EXPECT_NEAR(result.camera.*kCameraParameterFields[p], truth.*kCameraParameterFields[p], DECIMAL(5.0) * sigma)
            << kCameraParameterNames[p];
    }
    EXPECT_LT(result.residualRms, DECIMAL(0.2));
    const std::ptrdiff_t used = std::count(result.inliers.begin(), result.inliers.end(), true);
    EXPECT_GT(used, 250);
    EXPECT_GT(static_cast<decimal>(used), DECIMAL(0.8) * static_cast<decimal>(result.inliers.size()));
    // At 950" per binned pixel each image's own attitude is uncertain too; over
    // twelve seeds the error of the mean correction was about 20" per axis.
    const found::Vec3 alignmentError =
        CorrectionRotationVector(result.alignmentCorrection * scene.misalignment.conjugate());
    EXPECT_LT(found::RadToArcSec(alignmentError.norm()), DECIMAL(90.0))
        << "alignment error (arcsec): " << found::RadToArcSec(alignmentError.x()) << " "
        << found::RadToArcSec(alignmentError.y()) << " " << found::RadToArcSec(alignmentError.z());
}

}  // namespace scope
