#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/catalog/catalog.hpp"
#include "scope/command-line/parsing/options.hpp"
#include "scope/common/style.hpp"
#include "scope/star-centroid/star-centroid.hpp"

namespace scope {

namespace {

constexpr int kWidth = 64;
constexpr int kHeight = 64;
const decimal kTol = DECIMAL(1e-3);

/// Owns a single-channel pixel buffer and exposes it as a FOUND Image.
class TestImage {
 public:
    explicit TestImage(unsigned char background) : pixels_(static_cast<std::size_t>(kWidth) * kHeight, background) {}

    void Set(int x, int y, unsigned char value) { pixels_[static_cast<std::size_t>(y) * kWidth + x] = value; }

    Image View() { return Image{kWidth, kHeight, 1, pixels_.data()}; }

 private:
    std::vector<unsigned char> pixels_;
};

/// Pinhole options (focal 100, principal at the image center) with no distortion.
RecalibrationOptions CenteredOptions() {
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
    return options;
}

/// Inertial direction that, under identity attitude and CenteredOptions,
/// projects to pixel (px, py).
found::Vec3 DirectionForPixel(decimal px, decimal py) {
    return found::Vec3((px - DECIMAL(32.0)) / DECIMAL(100.0), (py - DECIMAL(32.0)) / DECIMAL(100.0), DECIMAL(1.0))
        .normalized();
}

/// Paints a symmetric blob (bright center + 4 neighbors) at (x, y).
void PaintStar(TestImage *image, int x, int y) {
    image->Set(x, y, 210);
    image->Set(x - 1, y, 110);
    image->Set(x + 1, y, 110);
    image->Set(x, y - 1, 110);
    image->Set(x, y + 1, 110);
}

}  // namespace

// A catalog star that projects onto a visible blob produces one observation,
// with the catalog and attitudes forwarded.
TEST(ROIFilterAlgorithmTest, ProducesObservationForVisibleStar) {
    TestImage dark(10);
    TestImage star(10);
    PaintStar(&star, 40, 36);

    RecalibrationOptions options = CenteredOptions();
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});

    ROIFilterAlgorithm algorithm(options, catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    ASSERT_EQ(result.observations.size(), 1u);
    EXPECT_EQ(result.observations[0].image_index, 0);
    EXPECT_EQ(result.observations[0].catalog_index, 0);
    EXPECT_NEAR(result.observations[0].measured_pixel.x(), DECIMAL(40.0), kTol);
    EXPECT_NEAR(result.observations[0].measured_pixel.y(), DECIMAL(36.0), kTol);

    ASSERT_NE(result.catalog, nullptr);
    EXPECT_EQ(result.catalog->size(), 1u);
    ASSERT_EQ(result.attitudes.size(), 1u);
}

// A star whose projection falls outside the sensor margin is skipped, while a
// visible star in the same catalog still produces its observation.
TEST(ROIFilterAlgorithmTest, SkipsOutOfSensorStar) {
    TestImage dark(10);
    TestImage star(10);
    PaintStar(&star, 40, 36);

    RecalibrationOptions options = CenteredOptions();
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});
    // u = 100 * 0.5 + 32 = 82, well outside a 64-wide sensor.
    catalog.push_back(CatalogStar{found::Vec3(DECIMAL(0.5), DECIMAL(0.0), DECIMAL(1.0)).normalized(), 200, 2});

    ROIFilterAlgorithm algorithm(options, catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    ASSERT_EQ(result.observations.size(), 1u);
    EXPECT_EQ(result.observations[0].catalog_index, 0);
}

// An in-sensor star with no corresponding blob yields no observation.
TEST(ROIFilterAlgorithmTest, NoObservationWhenStarNotPresent) {
    TestImage dark(10);
    TestImage star(10);  // uniform background, nothing above threshold after subtraction

    RecalibrationOptions options = CenteredOptions();
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});

    ROIFilterAlgorithm algorithm(options, catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    EXPECT_TRUE(result.observations.empty());
    ASSERT_NE(result.catalog, nullptr);
}

// Observations from multiple star images are tagged with the right image index.
TEST(ROIFilterAlgorithmTest, TagsObservationsPerImage) {
    TestImage dark(10);
    TestImage starA(10);
    TestImage starB(10);
    PaintStar(&starA, 40, 36);
    PaintStar(&starB, 40, 36);

    RecalibrationOptions options = CenteredOptions();
    options.starImages = {starA.View(), starB.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});

    ROIFilterAlgorithm algorithm(options, catalog, {found::Quaternion::Identity(), found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    ASSERT_EQ(result.observations.size(), 2u);
    EXPECT_EQ(result.observations[0].image_index, 0);
    EXPECT_EQ(result.observations[1].image_index, 1);
}

// A catalog star fainter than the magnitude threshold is not even projected, so
// its blob is ignored while a bright star in the same field still produces its
// observation.
TEST(ROIFilterAlgorithmTest, SkipsStarFainterThanMagnitudeThreshold) {
    TestImage dark(10);
    TestImage star(10);
    PaintStar(&star, 40, 36);  // the bright star's blob
    PaintStar(&star, 24, 24);  // the faint star's blob (its own, non-overlapping ROI)

    RecalibrationOptions options = CenteredOptions();  // magnitudeThreshold defaults to 6.0
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});  // mag 2.0 -> kept
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(24.0), DECIMAL(24.0)), 700, 2});  // mag 7.0 -> skipped

    ROIFilterAlgorithm algorithm(options, catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    ASSERT_EQ(result.observations.size(), 1u);
    EXPECT_EQ(result.observations[0].catalog_index, 0);
}

// Two catalog stars projecting a pixel apart both lock onto the same blob, an
// ambiguous correspondence; both observations are dropped rather than guessed.
TEST(ROIFilterAlgorithmTest, DropsCollidingObservations) {
    TestImage dark(10);
    TestImage star(10);
    PaintStar(&star, 40, 36);  // a single blob both stars will centroid onto

    RecalibrationOptions options = CenteredOptions();
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(41.0), DECIMAL(36.0)), 200, 2});

    ROIFilterAlgorithm algorithm(options, catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    EXPECT_TRUE(result.observations.empty());
}

// Two bright stars far enough apart that their centroids do not collide are both
// kept as separate observations.
TEST(ROIFilterAlgorithmTest, KeepsWellSeparatedStars) {
    TestImage dark(10);
    TestImage star(10);
    PaintStar(&star, 40, 36);
    PaintStar(&star, 24, 24);

    RecalibrationOptions options = CenteredOptions();
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(24.0), DECIMAL(24.0)), 200, 2});

    ROIFilterAlgorithm algorithm(options, catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    EXPECT_EQ(result.observations.size(), 2u);
}

// A catalog star sitting behind the camera projects to no pixel and is skipped,
// while a visible star in the same catalog still produces its observation.
TEST(ROIFilterAlgorithmTest, SkipsStarBehindCamera) {
    TestImage dark(10);
    TestImage star(10);
    PaintStar(&star, 40, 36);

    RecalibrationOptions options = CenteredOptions();
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});
    // Anti-boresight under identity attitude: camera-frame z < 0, not imageable.
    catalog.push_back(CatalogStar{found::Vec3(DECIMAL(0.0), DECIMAL(0.0), DECIMAL(-1.0)), 200, 2});

    ROIFilterAlgorithm algorithm(options, catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    ASSERT_EQ(result.observations.size(), 1u);
    EXPECT_EQ(result.observations[0].catalog_index, 0);
}

// Mismatched attitude / star-image counts are a programming error and throw.
TEST(ROIFilterAlgorithmTest, ThrowsOnAttitudeCountMismatch) {
    TestImage dark(10);
    TestImage star(10);

    RecalibrationOptions options = CenteredOptions();
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});

    // Two attitudes for one star image.
    ROIFilterAlgorithm algorithm(options, catalog, {found::Quaternion::Identity(), found::Quaternion::Identity()});
    Image darkView = dark.View();
    EXPECT_THROW(algorithm.Run(darkView), std::runtime_error);
}

// A star image whose dimensions differ from the dark frame throws.
TEST(ROIFilterAlgorithmTest, ThrowsOnDimensionMismatch) {
    std::vector<unsigned char> darkPixels(32u * 32u, 10);
    Image darkView{32, 32, 1, darkPixels.data()};

    TestImage star(10);  // 64x64, mismatched against the 32x32 dark frame

    RecalibrationOptions options = CenteredOptions();
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});

    ROIFilterAlgorithm algorithm(options, catalog, {found::Quaternion::Identity()});
    EXPECT_THROW(algorithm.Run(darkView), std::runtime_error);
}

}  // namespace scope
