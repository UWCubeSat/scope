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

// Two catalog stars a pixel apart share one blob, which could be either's; both
// are dropped.
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

// A star too faint to show, 8 px from a bright one, finds nothing on its side
// of the midpoint instead of taking the bright star's blob, so the bright star
// is kept.
TEST(ROIFilterAlgorithmTest, KeepsBrightStarNextToFaintOne) {
    TestImage dark(10);
    TestImage star(10);
    star.PaintStar(40, 36);

    RecalibrationOptions options;
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(32.0), DECIMAL(36.0)), 590, 2});

    ROIFilterAlgorithm algorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    ASSERT_EQ(result.observations.size(), 1u);
    EXPECT_EQ(result.observations[0].starName, 1);
    EXPECT_NEAR(result.observations[0].measuredPixel.x(), DECIMAL(40.0), kTol);
    EXPECT_NEAR(result.observations[0].measuredPixel.y(), DECIMAL(36.0), kTol);
}

// Each window holds both stars of a pair 8 px apart, and the brighter one's
// peak would win both searches; each star finds its own blob instead.
TEST(ROIFilterAlgorithmTest, KeepsBothStarsOfAClosePair) {
    TestImage dark(10);
    TestImage star(10);
    star.PaintStar(40, 36);
    star.Set(32, 36, 120);

    RecalibrationOptions options;
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(40.0), DECIMAL(36.0)), 200, 1});
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(32.0), DECIMAL(36.0)), 300, 2});

    ROIFilterAlgorithm algorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    ASSERT_EQ(result.observations.size(), 2u);
    EXPECT_EQ(result.observations[0].starName, 1);
    EXPECT_NEAR(result.observations[0].measuredPixel.x(), DECIMAL(40.0), kTol);
    EXPECT_EQ(result.observations[1].starName, 2);
    EXPECT_NEAR(result.observations[1].measuredPixel.x(), DECIMAL(32.0), kTol);
    EXPECT_NEAR(result.observations[1].measuredPixel.y(), DECIMAL(36.0), kTol);
}

namespace {

/// Every star sits 4 px left of where the prior puts it. Three are alone in
/// their 11 px windows; the other two are a faint star with no blob, predicted
/// at (26, 40), and a bright one predicted 6 px to its right but imaged at
/// (28, 40), nearer the faint star's prediction than its own.
struct OffsetScene {
    TestImage dark{10};
    TestImage star{10};
    Catalog catalog;

    explicit OffsetScene(int isolatedStars) {
        const int isolatedColumns[] = {12, 32, 52};
        for (int s = 0; s < 3; ++s) {
            const decimal column = DECIMAL(isolatedColumns[s]);
            catalog.push_back(CatalogStar{DirectionForPixel(column, DECIMAL(12.0)), 200, s + 1});
            if (s < isolatedStars) {
                star.PaintStar(isolatedColumns[s] - 4, 12);
            }
        }
        catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(26.0), DECIMAL(40.0)), 590, 4});
        catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(32.0), DECIMAL(40.0)), 200, 5});
        star.Set(28, 40, 210);
    }

    CentroidObservations Run() {
        RecalibrationOptions options;
        options.starImages = {star.View()};
        options.roiSize = 11;
        Image darkView = dark.View();
        return ROIFilterAlgorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()}).Run(darkView);
    }
};

}  // namespace

// The isolated stars measure the 4 px offset, the windows and the midpoint
// between the pair move with it, and the bright star's blob falls on its own
// side.
TEST(ROIFilterAlgorithmTest, MovesWindowsByTheMedianOffset) {
    CentroidObservations result = OffsetScene(3).Run();

    ASSERT_EQ(result.observations.size(), 4u);
    for (int s = 0; s < 3; ++s) {
        EXPECT_EQ(result.observations[s].starName, s + 1);
        EXPECT_NEAR(result.observations[s].measuredPixel.y(), DECIMAL(12.0), kTol);
    }
    EXPECT_NEAR(result.observations[0].measuredPixel.x(), DECIMAL(8.0), kTol);
    EXPECT_EQ(result.observations[3].starName, 5);
    EXPECT_NEAR(result.observations[3].measuredPixel.x(), DECIMAL(28.0), kTol);
    EXPECT_NEAR(result.observations[3].measuredPixel.y(), DECIMAL(40.0), kTol);
}

// Two isolated stars are too few to trust their offset, so the windows stay
// put and the faint star takes the bright one's blob, the mistake the offset
// prevents.
TEST(ROIFilterAlgorithmTest, NeedsThreeIsolatedStarsToMoveWindows) {
    CentroidObservations result = OffsetScene(2).Run();

    ASSERT_EQ(result.observations.size(), 3u);
    EXPECT_EQ(result.observations[2].starName, 4);
    EXPECT_NEAR(result.observations[2].measuredPixel.x(), DECIMAL(28.0), kTol);
}

// A star the prior puts just off the sensor still claims its side of the
// midpoint, so a faint star near the edge does not take its blob.
TEST(ROIFilterAlgorithmTest, StarJustOffTheSensorKeepsItsBlob) {
    TestImage dark(10);
    TestImage star(10);
    star.PaintStar(1, 30);

    RecalibrationOptions options;
    options.starImages = {star.View()};

    Catalog catalog;
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(-3.0), DECIMAL(30.0)), 200, 1});
    catalog.push_back(CatalogStar{DirectionForPixel(DECIMAL(17.0), DECIMAL(30.0)), 590, 2});

    ROIFilterAlgorithm algorithm(options, CenteredCamera(), catalog, {found::Quaternion::Identity()});
    Image darkView = dark.View();
    CentroidObservations result = algorithm.Run(darkView);

    EXPECT_TRUE(result.observations.empty());
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
