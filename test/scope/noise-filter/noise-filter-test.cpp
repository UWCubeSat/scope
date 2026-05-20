#include <gtest/gtest.h>

#include <array>
#include <cstdlib>
#include <stdexcept>

#include "scope/noise-filter/noise-filter.hpp"

namespace scope {

TEST(DarkScreenFilterTest, ComputesPerPixelMedian) {
    DarkScreenFilter filter;

    std::array<unsigned char, 4> imageA{10, 20, 30, 40};
    std::array<unsigned char, 4> imageB{50, 10, 20, 30};
    std::array<unsigned char, 4> imageC{20, 30, 10, 50};

    Image a{2, 2, 1, imageA.data()};
    Image b{2, 2, 1, imageB.data()};
    Image c{2, 2, 1, imageC.data()};

    Image dark = filter.Run({a, b, c});

    ASSERT_EQ(dark.width, 2);
    ASSERT_EQ(dark.height, 2);
    ASSERT_EQ(dark.channels, 1);
    EXPECT_EQ(dark.image[0], 20);
    EXPECT_EQ(dark.image[1], 20);
    EXPECT_EQ(dark.image[2], 20);
    EXPECT_EQ(dark.image[3], 40);

    std::free(dark.image);
}

TEST(DarkScreenFilterTest, UsesLowerMedianWhenInputCountIsEven) {
    DarkScreenFilter filter;

    std::array<unsigned char, 1> imageA{10};
    std::array<unsigned char, 1> imageB{20};
    std::array<unsigned char, 1> imageC{30};
    std::array<unsigned char, 1> imageD{200};

    Image a{1, 1, 1, imageA.data()};
    Image b{1, 1, 1, imageB.data()};
    Image c{1, 1, 1, imageC.data()};
    Image d{1, 1, 1, imageD.data()};

    Image dark = filter.Run({a, b, c, d});

    EXPECT_EQ(dark.image[0], 20);

    std::free(dark.image);
}

TEST(DarkScreenFilterTest, ThrowsOnEmptyImageSet) {
    DarkScreenFilter filter;

    EXPECT_THROW(filter.Run({}), std::invalid_argument);
}

TEST(DarkScreenFilterTest, ThrowsOnMismatchedDimensions) {
    DarkScreenFilter filter;

    std::array<unsigned char, 4> imageA{1, 2, 3, 4};
    std::array<unsigned char, 3> imageB{5, 6, 7};

    Image a{2, 2, 1, imageA.data()};
    Image b{3, 1, 1, imageB.data()};

    EXPECT_THROW(filter.Run({a, b}), std::runtime_error);
}

TEST(DarkScreenFilterTest, ThrowsOnMismatchedChannels) {
    DarkScreenFilter filter;

    std::array<unsigned char, 2> imageA{1, 2};
    std::array<unsigned char, 4> imageB{1, 2, 3, 4};

    Image a{1, 2, 1, imageA.data()};
    Image b{1, 2, 2, imageB.data()};

    EXPECT_THROW(filter.Run({a, b}), std::runtime_error);
}

TEST(DarkScreenFilterTest, ThrowsOnNullReferenceImage) {
    DarkScreenFilter filter;

    Image a{2, 2, 1, nullptr};

    EXPECT_THROW(filter.Run({a}), std::runtime_error);
}

TEST(DarkScreenFilterTest, ThrowsOnNullSubsequentImage) {
    DarkScreenFilter filter;

    std::array<unsigned char, 4> imageA{1, 2, 3, 4};
    Image a{2, 2, 1, imageA.data()};
    Image b{2, 2, 1, nullptr};

    EXPECT_THROW(filter.Run({a, b}), std::runtime_error);
}

TEST(DarkScreenFilterTest, ThrowsOnNonPositiveDimensions) {
    DarkScreenFilter filter;

    std::array<unsigned char, 1> data{1};
    Image zeroWidth{0, 2, 1, data.data()};
    Image zeroHeight{2, 0, 1, data.data()};
    Image zeroChannels{2, 2, 0, data.data()};

    EXPECT_THROW(filter.Run({zeroWidth}), std::runtime_error);
    EXPECT_THROW(filter.Run({zeroHeight}), std::runtime_error);
    EXPECT_THROW(filter.Run({zeroChannels}), std::runtime_error);
}

TEST(DarkScreenFilterTest, SingleImageReturnsCopy) {
    DarkScreenFilter filter;

    std::array<unsigned char, 4> data{7, 42, 0, 255};
    Image a{2, 2, 1, data.data()};

    Image dark = filter.Run({a});

    ASSERT_EQ(dark.width, 2);
    ASSERT_EQ(dark.height, 2);
    ASSERT_EQ(dark.channels, 1);
    ASSERT_NE(dark.image, data.data());
    EXPECT_EQ(dark.image[0], 7);
    EXPECT_EQ(dark.image[1], 42);
    EXPECT_EQ(dark.image[2], 0);
    EXPECT_EQ(dark.image[3], 255);

    std::free(dark.image);
}

TEST(DarkScreenFilterTest, MultiChannelMediansChannelsIndependently) {
    DarkScreenFilter filter;

    // 1x2 image, 3 channels, layout per pixel: [R, G, B].
    std::array<unsigned char, 6> imageA{10, 100, 200, 50, 60, 70};
    std::array<unsigned char, 6> imageB{20, 110, 180, 40, 80, 90};
    std::array<unsigned char, 6> imageC{30, 90, 220, 60, 70, 80};

    Image a{1, 2, 3, imageA.data()};
    Image b{1, 2, 3, imageB.data()};
    Image c{1, 2, 3, imageC.data()};

    Image dark = filter.Run({a, b, c});

    ASSERT_EQ(dark.channels, 3);
    // Pixel 0: R={10,20,30}->20, G={100,110,90}->100, B={200,180,220}->200.
    EXPECT_EQ(dark.image[0], 20);
    EXPECT_EQ(dark.image[1], 100);
    EXPECT_EQ(dark.image[2], 200);
    // Pixel 1: R={50,40,60}->50, G={60,80,70}->70, B={70,90,80}->80.
    EXPECT_EQ(dark.image[3], 50);
    EXPECT_EQ(dark.image[4], 70);
    EXPECT_EQ(dark.image[5], 80);

    std::free(dark.image);
}

}  // namespace scope
