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

}  // namespace scope
