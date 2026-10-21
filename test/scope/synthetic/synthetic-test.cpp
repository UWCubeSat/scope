#include <gtest/gtest.h>

#include <stb_image/stb_image.h>

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/attitude/attitude.hpp"
#include "scope/catalog/catalog.hpp"
#include "scope/common/style.hpp"
#include "scope/common/types.hpp"
#include "scope/projection/projection.hpp"
#include "scope/star-centroid/coi.hpp"
#include "scope/synthetic/synthetic.hpp"

#include "test/scope/common/cameras.hpp"
#include "test/scope/common/test-files.hpp"

namespace scope {

namespace {

/// A pinhole camera on the 64 x 64 test image, principal point at its centre.
CameraParameters TestImageCamera() {
    CameraParameters camera;
    camera.focalLengthX = DECIMAL(100.0);
    camera.focalLengthY = DECIMAL(100.0);
    camera.principalX = DECIMAL(31.5);
    camera.principalY = DECIMAL(31.5);
    return camera;
}

/// A sensor with no noise, offsets or sky, so a render is exactly the light.
SensorModel NoiselessSensor() {
    SensorModel sensor;
    sensor.skyBackground = DECIMAL(0.0);
    sensor.blackLevel = DECIMAL(0.0);
    sensor.fixedPatternNoise = DECIMAL(0.0);
    sensor.hotPixelFraction = DECIMAL(0.0);
    sensor.readNoise = DECIMAL(0.0);
    sensor.gain = DECIMAL(1e12);
    return sensor;
}

/// One magnitude 0 star whose camera-frame direction lands on pixel (u, v)
/// under the identity attitude and TestImageCamera.
Catalog StarAt(decimal u, decimal v) {
    const CameraParameters camera = TestImageCamera();
    const found::Vec3 direction(
        (u - camera.principalX) / camera.focalLengthX, (v - camera.principalY) / camera.focalLengthY, DECIMAL(1.0));
    return {CatalogStar{direction.normalized(), 0, 1}};
}

}  // namespace

TEST(RandomAttitudeTest, IsUnitAndSpreadOverAllPointings) {
    Random random(3);
    found::Vec3 sum = found::Vec3::Zero();
    constexpr int kDraws = 2000;
    for (int i = 0; i < kDraws; ++i) {
        const found::Quaternion attitude = RandomAttitude(&random);
        EXPECT_NEAR(attitude.norm(), DECIMAL(1.0), DECIMAL(1e-12));
        sum += attitude.conjugate() * found::Vec3::UnitZ();
    }
    // The mean boresight of uniform pointings is the origin, to about 0.013.
    EXPECT_LT((sum / DECIMAL(kDraws)).norm(), DECIMAL(0.05));
}

TEST(RandomSmallRotationTest, HasTheRequestedSpread) {
    Random random(4);
    const decimal sigma = DECIMAL(1e-4);
    decimal sumOfSquares = DECIMAL(0.0);
    constexpr int kDraws = 3000;
    for (int i = 0; i < kDraws; ++i) {
        sumOfSquares += CorrectionRotationVector(RandomSmallRotation(sigma, &random)).squaredNorm();
    }
    // Three independent axes: the mean squared angle is 3 sigma^2.
    const decimal rmsAngle = DECIMAL_SQRT(sumOfSquares / DECIMAL(kDraws));
    EXPECT_NEAR(rmsAngle, sigma * DECIMAL_SQRT(DECIMAL(3.0)), DECIMAL(0.05) * sigma);
    EXPECT_EQ(RandomSmallRotation(DECIMAL(0.0), &random).coeffs(), found::Quaternion::Identity().coeffs());
}

TEST(RandomCatalogTest, FollowsTheBrightStarCatalogLaw) {
    Random random(5);
    const Catalog catalog = RandomCatalog(5000, DECIMAL(6.0), &random);

    ASSERT_EQ(catalog.size(), 5000u);
    int brighterThanFour = 0;
    for (std::size_t i = 0; i < catalog.size(); ++i) {
        EXPECT_EQ(catalog[i].name, static_cast<int>(i) + 1);
        EXPECT_NEAR(catalog[i].spatial.norm(), DECIMAL(1.0), DECIMAL(1e-12));
        EXPECT_LE(catalog[i].magnitude, 600);
        EXPECT_GE(catalog[i].magnitude, -150);
        brighterThanFour += catalog[i].magnitude < 400 ? 1 : 0;
    }
    // Two magnitudes brighter than the limit holds 10^-1 of the stars.
    EXPECT_NEAR(brighterThanFour, 500, 70);
}

// With no per-image error, the correction from each reported attitude to the
// truth is exactly the misalignment.
TEST(RandomSceneTest, ReportedAttitudesCarryTheMisalignment) {
    Random random(6);
    const SyntheticScene scene = RandomScene(
        AsBuiltCamera(), kSensorWidth, kSensorHeight, 4, DECIMAL(0.0), found::ArcSecToRad(DECIMAL(60.0)), &random);

    ASSERT_EQ(scene.trueAttitudes.size(), 4u);
    ASSERT_EQ(scene.priorAttitudes.size(), 4u);
    EXPECT_EQ(scene.width, kSensorWidth);
    EXPECT_GT(scene.misalignment.angularDistance(found::Quaternion::Identity()), DECIMAL(0.0));
    for (std::size_t k = 0; k < 4; ++k) {
        const found::Quaternion correction = scene.trueAttitudes[k] * scene.priorAttitudes[k].conjugate();
        EXPECT_LT(correction.angularDistance(scene.misalignment), DECIMAL(1e-12));
    }
}

TEST(RandomSceneTest, PerImageErrorsDifferBetweenImages) {
    Random random(7);
    const SyntheticScene scene = RandomScene(
        AsBuiltCamera(), kSensorWidth, kSensorHeight, 3, found::ArcSecToRad(DECIMAL(30.0)), DECIMAL(0.0), &random);

    EXPECT_EQ(scene.misalignment.coeffs(), found::Quaternion::Identity().coeffs());
    for (std::size_t k = 0; k < 3; ++k) {
        const decimal error = found::RadToArcSec(scene.trueAttitudes[k].angularDistance(scene.priorAttitudes[k]));
        EXPECT_GT(error, DECIMAL(0.0));
        EXPECT_LT(error, DECIMAL(300.0));
    }
}

TEST(VisibleStarsTest, ListsOnlyStarsInsideTheMargin) {
    const Catalog catalog = {
        CatalogStar{found::Vec3(DECIMAL(0.0), DECIMAL(0.0), DECIMAL(1.0)), 100, 1},
        CatalogStar{found::Vec3(DECIMAL(0.3), DECIMAL(0.0), DECIMAL(1.0)).normalized(), 100, 2},
        CatalogStar{found::Vec3(DECIMAL(0.0), DECIMAL(0.0), DECIMAL(-1.0)), 100, 3},
        CatalogStar{found::Vec3(DECIMAL(0.1), DECIMAL(-0.05), DECIMAL(1.0)).normalized(), 100, 4},
    };

    // 0.3 lands 30 px from the centre, beyond the 10 px margin of a 64 px image.
    const std::vector<VisibleStar> visible =
        VisibleStars(catalog, found::Quaternion::Identity(), TestImageCamera(), kWidth, kHeight, 10);

    ASSERT_EQ(visible.size(), 2u);
    EXPECT_EQ(visible[0].catalogIndex, 0u);
    EXPECT_NEAR(visible[0].pixel.x(), DECIMAL(31.5), DECIMAL(1e-9));
    EXPECT_EQ(visible[1].catalogIndex, 3u);
    EXPECT_NEAR(visible[1].pixel.x(), DECIMAL(41.5), DECIMAL(1e-9));
    EXPECT_NEAR(visible[1].pixel.y(), DECIMAL(26.5), DECIMAL(1e-9));
}

TEST(SimulateObservationsTest, NoiseFreeObservationsAreExactProjections) {
    Random random(8);
    const Catalog catalog = RandomCatalog(800, DECIMAL(6.0), &random);
    const SyntheticScene scene = RandomScene(AsBuiltCamera(),
                                             kSensorWidth,
                                             kSensorHeight,
                                             3,
                                             found::ArcSecToRad(DECIMAL(30.0)),
                                             found::ArcSecToRad(DECIMAL(30.0)),
                                             &random);

    const CentroidObservations observations = SimulateObservations(scene, catalog, 16, DECIMAL(0.0), &random);

    ASSERT_FALSE(observations.observations.empty());
    EXPECT_EQ(observations.attitudes.size(), 3u);
    int previousImage = 0;
    for (const Observation &observation : observations.observations) {
        EXPECT_GE(observation.imageIndex, previousImage);
        previousImage = observation.imageIndex;
        const std::optional<found::Vec2> pixel = ProjectStarToPixel(
            observation.inertialDirection, scene.trueAttitudes[observation.imageIndex], scene.camera);
        ASSERT_TRUE(pixel.has_value());
        EXPECT_EQ(observation.measuredPixel, *pixel);
        EXPECT_TRUE(InSensorWithMargin(observation.measuredPixel, kSensorWidth, kSensorHeight, 16));
        EXPECT_EQ(catalog[static_cast<std::size_t>(observation.starName) - 1].spatial, observation.inertialDirection);
    }
}

// The renderer and ExtractCentroid share the convention that an integer
// coordinate is a pixel centre: a star rendered at a non-integer position
// centroids back to that position, and its light is all there.
TEST(RenderStarImageTest, RenderedStarCentroidsToItsPixel) {
    const SensorModel sensor = NoiselessSensor();
    SensorModel bright = sensor;
    bright.zeroPoint = DECIMAL(1500.0);
    const std::vector<decimal> pattern(static_cast<std::size_t>(kWidth) * kHeight, DECIMAL(0.0));
    Random random(9);

    for (const found::Vec2 &position : {found::Vec2(DECIMAL(30.3), DECIMAL(35.7)),
                                        found::Vec2(DECIMAL(20.0), DECIMAL(40.5)),
                                        found::Vec2(DECIMAL(41.75), DECIMAL(22.25))}) {
        std::vector<unsigned char> pixels = RenderStarImage(StarAt(position.x(), position.y()),
                                                            found::Quaternion::Identity(),
                                                            TestImageCamera(),
                                                            pattern,
                                                            kWidth,
                                                            kHeight,
                                                            bright,
                                                            &random);

        decimal total = DECIMAL(0.0);
        for (const unsigned char value : pixels) {
            total += DECIMAL(value);
        }
        EXPECT_NEAR(total, DECIMAL(1500.0), DECIMAL(15.0));
        EXPECT_LT(*std::max_element(pixels.begin(), pixels.end()), 255);

        // Threshold zero and a mask wide enough for the whole star.
        const Image image{kWidth, kHeight, 1, pixels.data()};
        const std::optional<found::Vec2> centroid = ExtractCentroid(image, position, 9, 6, 0);
        ASSERT_TRUE(centroid.has_value());
        EXPECT_NEAR(centroid->x(), position.x(), DECIMAL(0.01));
        EXPECT_NEAR(centroid->y(), position.y(), DECIMAL(0.01));
    }
}

TEST(RenderStarImageTest, BrightStarsSaturate) {
    SensorModel sensor = NoiselessSensor();
    sensor.zeroPoint = DECIMAL(1e5);
    const std::vector<decimal> pattern(static_cast<std::size_t>(kWidth) * kHeight, DECIMAL(0.0));
    Random random(10);

    const std::vector<unsigned char> pixels = RenderStarImage(StarAt(DECIMAL(32.0), DECIMAL(32.0)),
                                                              found::Quaternion::Identity(),
                                                              TestImageCamera(),
                                                              pattern,
                                                              kWidth,
                                                              kHeight,
                                                              sensor,
                                                              &random);

    EXPECT_EQ(pixels[32 * kWidth + 32], 255);
    EXPECT_EQ(pixels[32 * kWidth + 33], 255);
}

// With no stars a star image is the fixed pattern plus the sky, and its noise
// has the variance the model gives: shot noise light / gain plus read noise.
TEST(RenderStarImageTest, BackgroundHasModelledLevelAndNoise) {
    SensorModel sensor;
    sensor.skyBackground = DECIMAL(100.0);
    sensor.gain = DECIMAL(4.0);
    sensor.readNoise = DECIMAL(2.0);
    Random random(11);
    const std::vector<decimal> pattern = RandomFixedPattern(kWidth, kHeight, sensor, &random);

    const std::vector<unsigned char> pixels = RenderStarImage(
        Catalog(), found::Quaternion::Identity(), TestImageCamera(), pattern, kWidth, kHeight, sensor, &random);

    decimal sum = DECIMAL(0.0);
    decimal sumOfSquares = DECIMAL(0.0);
    int count = 0;
    for (std::size_t i = 0; i < pixels.size(); ++i) {
        if (pattern[i] > sensor.blackLevel + DECIMAL(20.0)) {
            continue;  // hot pixel
        }
        const decimal deviation = DECIMAL(pixels[i]) - pattern[i] - sensor.skyBackground;
        sum += deviation;
        sumOfSquares += deviation * deviation;
        ++count;
    }
    const decimal mean = sum / DECIMAL(count);
    // 100 / 4 + 2^2, plus 1/12 for rounding to whole counts.
    const decimal expectedSigma = DECIMAL_SQRT(DECIMAL(29.0) + DECIMAL(1.0) / DECIMAL(12.0));
    EXPECT_NEAR(mean, DECIMAL(0.0), DECIMAL(0.3));
    const decimal sigma = DECIMAL_SQRT(sumOfSquares / DECIMAL(count) - mean * mean);
    EXPECT_NEAR(sigma, expectedSigma, DECIMAL(0.05) * expectedSigma);
}

TEST(RandomFixedPatternTest, HasBlackLevelSpreadAndHotPixels) {
    SensorModel sensor;
    sensor.blackLevel = DECIMAL(10.0);
    sensor.fixedPatternNoise = DECIMAL(2.0);
    sensor.hotPixelFraction = DECIMAL(0.01);
    Random random(12);

    const std::vector<decimal> pattern = RandomFixedPattern(200, 200, sensor, &random);

    ASSERT_EQ(pattern.size(), 40000u);
    decimal sum = DECIMAL(0.0);
    decimal sumOfSquares = DECIMAL(0.0);
    int hot = 0;
    for (const decimal level : pattern) {
        if (level > DECIMAL(40.0)) {
            ++hot;
            continue;
        }
        sum += level;
        sumOfSquares += level * level;
    }
    const decimal count = DECIMAL(40000 - hot);
    const decimal mean = sum / count;
    EXPECT_NEAR(hot, 400, 80);
    EXPECT_NEAR(mean, DECIMAL(10.0), DECIMAL(0.05));
    EXPECT_NEAR(DECIMAL_SQRT(sumOfSquares / count - mean * mean), DECIMAL(2.0), DECIMAL(0.05));
}

TEST(RenderDarkFrameTest, IsTheFixedPatternRounded) {
    SensorModel sensor = NoiselessSensor();
    const std::vector<decimal> pattern = {DECIMAL(3.4), DECIMAL(3.6), DECIMAL(-2.0), DECIMAL(300.0), DECIMAL(17.0)};
    Random random(13);

    const std::vector<unsigned char> pixels = RenderDarkFrame(pattern, sensor, &random);

    EXPECT_EQ(pixels, (std::vector<unsigned char>{3, 4, 0, 255, 17}));
}

TEST(WritePgmTest, RoundTripsThroughStbImage) {
    const std::vector<unsigned char> pixels = {0, 1, 2, 3, 4, 250, 251, 252, 253, 254, 9, 99, 199, 255, 128};
    TempFile file("synthetic-round-trip.pgm", "");

    WritePgm(file.Path(), 5, 3, pixels);

    int width = 0;
    int height = 0;
    int channels = 0;
    unsigned char *loaded = stbi_load(file.Path().c_str(), &width, &height, &channels, 0);
    ASSERT_NE(loaded, nullptr);
    EXPECT_EQ(width, 5);
    EXPECT_EQ(height, 3);
    EXPECT_EQ(channels, 1);
    EXPECT_EQ(std::vector<unsigned char>(loaded, loaded + pixels.size()), pixels);
    stbi_image_free(loaded);
}

TEST(WritePgmTest, UnwritablePathThrows) {
    EXPECT_THROW(WritePgm("test/fixtures/no-such-directory/image.pgm", 1, 1, {1}), std::runtime_error);
}

// /dev/full opens but fails every write, as a full disk would.
TEST(WritePgmTest, FailedWriteThrows) {
    if (!std::ifstream("/dev/full")) {
        GTEST_SKIP() << "needs /dev/full";
    }
    EXPECT_THROW(WritePgm("/dev/full", 2, 1, {1, 2}), std::runtime_error);
}

}  // namespace scope
