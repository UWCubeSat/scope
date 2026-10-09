#include <gtest/gtest.h>

#include <optional>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/common/style.hpp"
#include "scope/star-centroid/coi.hpp"

namespace scope {

namespace {

constexpr int kWidth = 64;
constexpr int kHeight = 64;
constexpr int kRoiSize = 31;
constexpr int kRecenterRadius = 3;
const decimal kTol = DECIMAL(1e-6);

/// Owns a single-channel pixel buffer and exposes it as a FOUND Image.
class TestImage {
 public:
    TestImage() : pixels_(static_cast<std::size_t>(kWidth) * kHeight, 0) {}

    void Set(int x, int y, unsigned char value) { pixels_[static_cast<std::size_t>(y) * kWidth + x] = value; }

    Image View() { return Image{kWidth, kHeight, 1, pixels_.data()}; }

 private:
    std::vector<unsigned char> pixels_;
};

}  // namespace

// A symmetric blob centered on the expected pixel yields that pixel back.
TEST(ExtractCentroidTest, CenteredBlob) {
    TestImage image;
    image.Set(40, 30, 200);
    image.Set(39, 30, 100);
    image.Set(41, 30, 100);
    image.Set(40, 29, 100);
    image.Set(40, 31, 100);
    Image view = image.View();

    std::optional<found::Vec2> centroid =
        ExtractCentroid(view, found::Vec2(DECIMAL(40.0), DECIMAL(30.0)), kRoiSize, kRecenterRadius, 40);

    ASSERT_TRUE(centroid.has_value());
    EXPECT_NEAR(centroid->x(), DECIMAL(40.0), kTol);
    EXPECT_NEAR(centroid->y(), DECIMAL(30.0), kTol);
}

// The brightest pixel drives recentering, so a star offset from the expected
// pixel (but still in the ROI) is centroided at its true location.
TEST(ExtractCentroidTest, OffCenterStarInRoi) {
    TestImage image;
    image.Set(45, 33, 200);
    Image view = image.View();

    std::optional<found::Vec2> centroid =
        ExtractCentroid(view, found::Vec2(DECIMAL(40.0), DECIMAL(30.0)), kRoiSize, kRecenterRadius, 40);

    ASSERT_TRUE(centroid.has_value());
    EXPECT_NEAR(centroid->x(), DECIMAL(45.0), kTol);
    EXPECT_NEAR(centroid->y(), DECIMAL(33.0), kTol);
}

// A single bright pixel is its own centroid.
TEST(ExtractCentroidTest, SingleBrightPixel) {
    TestImage image;
    image.Set(40, 30, 200);
    Image view = image.View();

    std::optional<found::Vec2> centroid =
        ExtractCentroid(view, found::Vec2(DECIMAL(40.0), DECIMAL(30.0)), kRoiSize, kRecenterRadius, 40);

    ASSERT_TRUE(centroid.has_value());
    EXPECT_NEAR(centroid->x(), DECIMAL(40.0), kTol);
    EXPECT_NEAR(centroid->y(), DECIMAL(30.0), kTol);
}

// A symmetric saturated 3x3 cluster centroids at its geometric center even
// though every pixel ties for brightest.
TEST(ExtractCentroidTest, SaturatedCluster) {
    TestImage image;
    for (int y = 29; y <= 31; ++y) {
        for (int x = 39; x <= 41; ++x) {
            image.Set(x, y, 255);
        }
    }
    Image view = image.View();

    std::optional<found::Vec2> centroid =
        ExtractCentroid(view, found::Vec2(DECIMAL(40.0), DECIMAL(30.0)), kRoiSize, kRecenterRadius, 40);

    ASSERT_TRUE(centroid.has_value());
    EXPECT_NEAR(centroid->x(), DECIMAL(40.0), kTol);
    EXPECT_NEAR(centroid->y(), DECIMAL(30.0), kTol);
}

// A star near the edge of the ROI window is still found (recenter handles it).
TEST(ExtractCentroidTest, StarAtRoiEdge) {
    TestImage image;
    // Expected pixel (40,30); ROI spans x in [25,55], so 54 is near the edge.
    image.Set(54, 30, 200);
    Image view = image.View();

    std::optional<found::Vec2> centroid =
        ExtractCentroid(view, found::Vec2(DECIMAL(40.0), DECIMAL(30.0)), kRoiSize, kRecenterRadius, 40);

    ASSERT_TRUE(centroid.has_value());
    EXPECT_NEAR(centroid->x(), DECIMAL(54.0), kTol);
    EXPECT_NEAR(centroid->y(), DECIMAL(30.0), kTol);
}

// No pixel above the threshold means no centroid.
TEST(ExtractCentroidTest, ReturnsNulloptWhenNothingAboveThreshold) {
    TestImage image;
    image.Set(40, 30, 30);  // below threshold 40
    Image view = image.View();

    std::optional<found::Vec2> centroid =
        ExtractCentroid(view, found::Vec2(DECIMAL(40.0), DECIMAL(30.0)), kRoiSize, kRecenterRadius, 40);

    EXPECT_FALSE(centroid.has_value());
}

// The threshold is strict: a pixel exactly at the threshold does not qualify,
// but one intensity above it does.
TEST(ExtractCentroidTest, ThresholdIsStrict) {
    TestImage atThreshold;
    atThreshold.Set(40, 30, 40);
    Image atView = atThreshold.View();
    EXPECT_FALSE(
        ExtractCentroid(atView, found::Vec2(DECIMAL(40.0), DECIMAL(30.0)), kRoiSize, kRecenterRadius, 40).has_value());

    TestImage aboveThreshold;
    aboveThreshold.Set(40, 30, 41);
    Image aboveView = aboveThreshold.View();
    std::optional<found::Vec2> centroid =
        ExtractCentroid(aboveView, found::Vec2(DECIMAL(40.0), DECIMAL(30.0)), kRoiSize, kRecenterRadius, 40);
    ASSERT_TRUE(centroid.has_value());
    EXPECT_NEAR(centroid->x(), DECIMAL(40.0), kTol);
    EXPECT_NEAR(centroid->y(), DECIMAL(30.0), kTol);
}

// An expected pixel whose ROI lies entirely outside the image yields no centroid.
TEST(ExtractCentroidTest, ReturnsNulloptWhenRoiOutsideImage) {
    TestImage image;
    image.Set(40, 30, 200);
    Image view = image.View();

    std::optional<found::Vec2> centroid =
        ExtractCentroid(view, found::Vec2(DECIMAL(-100.0), DECIMAL(-100.0)), kRoiSize, kRecenterRadius, 40);

    EXPECT_FALSE(centroid.has_value());
}

}  // namespace scope
