#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/catalog/catalog.hpp"
#include "scope/command-line/parsing/options.hpp"
#include "scope/common/style.hpp"
#include "scope/projection/projection.hpp"
#include "scope/star-centroid/star-centroid.hpp"

#include "test/scope/common/test-files.hpp"

namespace scope {

namespace {

const decimal kTol = DECIMAL(1e-3);

/// Pinhole camera (focal 100, principal at the image center) with no distortion.
CameraParameters CenteredCamera() {
    CameraParameters camera;
    camera.focalLengthX = DECIMAL(100.0);
    camera.focalLengthY = DECIMAL(100.0);
    camera.principalX = DECIMAL(32.0);
    camera.principalY = DECIMAL(32.0);
    return camera;
}

/// Inertial direction that, under identity attitude and CenteredCamera,
/// projects to pixel (px, py).
found::Vec3 DirectionForPixel(decimal px, decimal py) {
    return found::Vec3((px - DECIMAL(32.0)) / DECIMAL(100.0), (py - DECIMAL(32.0)) / DECIMAL(100.0), DECIMAL(1.0))
        .normalized();
}

}  // namespace

// The observation carries the star's name and inertial direction, and the
// attitudes are forwarded.
TEST(ROIFilterAlgorithmTest, ProducesObservationForVisibleStar) {
    TestImage dark(10);
    TestImage star(10);
    star.PaintStar(40, 36);

    RecalibrationOptions options;
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});

    ROIFilterAlgorithm algorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    ASSERT_EQ(result.observations.size(), 1u);
    EXPECT_EQ(result.observations[0].imageIndex, 0);
    EXPECT_EQ(result.observations[0].starName, 1);
    EXPECT_NEAR((result.observations[0].inertialDirection - catalog[0].spatial).norm(), DECIMAL(0.0), kTol);
    EXPECT_NEAR(result.observations[0].measuredPixel.x(), DECIMAL(40.0), kTol);
    EXPECT_NEAR(result.observations[0].measuredPixel.y(), DECIMAL(36.0), kTol);

    ASSERT_EQ(result.attitudes.size(), 1u);
}

// A visible star in the same catalog is still observed.
TEST(ROIFilterAlgorithmTest, SkipsOutOfSensorStar) {
    TestImage dark(10);
    TestImage star(10);
    star.PaintStar(40, 36);

    RecalibrationOptions options;
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});
    // u = 100 * 0.5 + 32 = 82, well outside a 64-wide sensor.
    catalog.push_back(CatalogStar{found::Vec3(DECIMAL(0.5), DECIMAL(0.0), DECIMAL(1.0)).normalized(), 200, 2});

    ROIFilterAlgorithm algorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    ASSERT_EQ(result.observations.size(), 1u);
    EXPECT_EQ(result.observations[0].starName, 1);
}

TEST(ROIFilterAlgorithmTest, NoObservationWhenStarNotPresent) {
    TestImage dark(10);
    TestImage star(10);  // uniform background, nothing above threshold after subtraction

    RecalibrationOptions options;
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});

    ROIFilterAlgorithm algorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    EXPECT_TRUE(result.observations.empty());
}

TEST(ROIFilterAlgorithmTest, TagsObservationsPerImage) {
    TestImage dark(10);
    TestImage starA(10);
    TestImage starB(10);
    starA.PaintStar(40, 36);
    starB.PaintStar(40, 36);

    RecalibrationOptions options;
    options.starImages = {starA.View(), starB.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});

    ROIFilterAlgorithm algorithm(
        options, CenteredCamera(), catalog, {found::Quaternion::Identity(), found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    ASSERT_EQ(result.observations.size(), 2u);
    EXPECT_EQ(result.observations[0].imageIndex, 0);
    EXPECT_EQ(result.observations[1].imageIndex, 1);
}

// The faint star's blob is ignored; a bright star in the same field is still
// observed.
TEST(ROIFilterAlgorithmTest, SkipsStarFainterThanMagnitudeThreshold) {
    TestImage dark(10);
    TestImage star(10);
    star.PaintStar(40, 36);  // the bright star's blob
    star.PaintStar(24, 24);  // the faint star's blob (its own, non-overlapping ROI)

    RecalibrationOptions options;  // magnitudeThreshold defaults to 6.0
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});  // mag 2.0 -> kept
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(24.0), DECIMAL(24.0)), 700, 2});  // mag 7.0 -> skipped

    ROIFilterAlgorithm algorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    ASSERT_EQ(result.observations.size(), 1u);
    EXPECT_EQ(result.observations[0].starName, 1);
}

// Two catalog stars a pixel apart lock onto the same blob; both are dropped.
TEST(ROIFilterAlgorithmTest, DropsCollidingObservations) {
    TestImage dark(10);
    TestImage star(10);
    star.PaintStar(40, 36);  // a single blob both stars will centroid onto

    RecalibrationOptions options;
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(41.0), DECIMAL(36.0)), 200, 2});

    ROIFilterAlgorithm algorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    EXPECT_TRUE(result.observations.empty());
}

TEST(ROIFilterAlgorithmTest, KeepsWellSeparatedStars) {
    TestImage dark(10);
    TestImage star(10);
    star.PaintStar(40, 36);
    star.PaintStar(24, 24);

    RecalibrationOptions options;
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(24.0), DECIMAL(24.0)), 200, 2});

    ROIFilterAlgorithm algorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    EXPECT_EQ(result.observations.size(), 2u);
}

// A visible star in the same catalog is still observed.
TEST(ROIFilterAlgorithmTest, SkipsStarBehindCamera) {
    TestImage dark(10);
    TestImage star(10);
    star.PaintStar(40, 36);

    RecalibrationOptions options;
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});
    // Anti-boresight under identity attitude: camera-frame z < 0, not imageable.
    catalog.push_back(CatalogStar{found::Vec3(DECIMAL(0.0), DECIMAL(0.0), DECIMAL(-1.0)), 200, 2});

    ROIFilterAlgorithm algorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    ASSERT_EQ(result.observations.size(), 1u);
    EXPECT_EQ(result.observations[0].starName, 1);
}

TEST(ROIFilterAlgorithmTest, ThrowsOnAttitudeCountMismatch) {
    TestImage dark(10);
    TestImage star(10);

    RecalibrationOptions options;
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});

    // Two attitudes for one star image.
    ROIFilterAlgorithm algorithm(
        options, CenteredCamera(), catalog, {found::Quaternion::Identity(), found::Quaternion::Identity()});
    Image darkView = dark.View();
    EXPECT_THROW(algorithm.Run(darkView), std::runtime_error);
}

TEST(ROIFilterAlgorithmTest, ThrowsOnDimensionMismatch) {
    std::vector<unsigned char> darkPixels(32u * 32u, 10);
    Image darkView{32, 32, 1, darkPixels.data()};

    TestImage star(10);  // 64x64, mismatched against the 32x32 dark frame

    RecalibrationOptions options;
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});

    ROIFilterAlgorithm algorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()});
    EXPECT_THROW(algorithm.Run(darkView), std::runtime_error);
}

// The ROI size sets how far a star may sit from its predicted pixel and still
// be found: a star 10 px off is inside the default 31 px window but outside an
// 11 px one.
TEST(ROIFilterAlgorithmTest, RoiSizeSetsSearchReach) {
    TestImage dark(10);
    TestImage star(10);
    star.PaintStar(40, 30);  // 10 px to the right of the prediction

    RecalibrationOptions options;  // roiSize defaults to 31
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(30.0), DECIMAL(30.0)), 200, 1});

    Image darkView = dark.View();
    CentroidObservations wide =
        ROIFilterAlgorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()}).Run(darkView);
    ASSERT_EQ(wide.observations.size(), 1u);
    EXPECT_NEAR(wide.observations[0].measuredPixel.x(), DECIMAL(40.0), kTol);
    EXPECT_NEAR(wide.observations[0].measuredPixel.y(), DECIMAL(30.0), kTol);

    options.roiSize = 11;
    CentroidObservations narrow =
        ROIFilterAlgorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()}).Run(darkView);
    EXPECT_TRUE(narrow.observations.empty());
}

// The border in which predictions are skipped shrinks with the ROI: a star
// predicted 10 px from the edge is skipped at the default size (16 px border)
// and observed with an 11 px ROI (6 px border).
TEST(ROIFilterAlgorithmTest, RoiSizeSetsEdgeMargin) {
    TestImage dark(10);
    TestImage star(10);
    star.PaintStar(10, 32);

    RecalibrationOptions options;
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(10.0), DECIMAL(32.0)), 200, 1});

    Image darkView = dark.View();
    CentroidObservations wide =
        ROIFilterAlgorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()}).Run(darkView);
    EXPECT_TRUE(wide.observations.empty());

    options.roiSize = 11;
    CentroidObservations narrow =
        ROIFilterAlgorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()}).Run(darkView);
    ASSERT_EQ(narrow.observations.size(), 1u);
    EXPECT_NEAR(narrow.observations[0].measuredPixel.x(), DECIMAL(10.0), kTol);
    EXPECT_NEAR(narrow.observations[0].measuredPixel.y(), DECIMAL(32.0), kTol);
}

// The ROI is centered on a pixel, so an even size reaches as far as the next
// odd one: 30 finds a star 15 px off just as 31 does, and 29 does not.
TEST(ROIFilterAlgorithmTest, EvenRoiSizeActsAsNextOdd) {
    TestImage dark(10);
    TestImage star(10);
    // One lit pixel 15 px to the right of the prediction. A blob would not do:
    // its wing would reach into the 29 px window.
    star.Set(45, 30, 210);

    RecalibrationOptions options;
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(30.0), DECIMAL(30.0)), 200, 1});

    Image darkView = dark.View();
    options.roiSize = 30;
    CentroidObservations even =
        ROIFilterAlgorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()}).Run(darkView);
    EXPECT_EQ(even.observations.size(), 1u);

    options.roiSize = 29;
    CentroidObservations odd =
        ROIFilterAlgorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()}).Run(darkView);
    EXPECT_TRUE(odd.observations.empty());
}

TEST(ROIFilterAlgorithmTest, ThrowsOnNonPositiveRoiSize) {
    TestImage dark(10);
    TestImage star(10);

    RecalibrationOptions options;
    options.starImages = {star.View()};
    options.roiSize = 0;

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});

    ROIFilterAlgorithm algorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    EXPECT_THROW(algorithm.Run(darkView), std::runtime_error);
}

}  // namespace scope
