#include <gtest/gtest.h>

#include <stb_image/stb_image.h>

#include <stdexcept>
#include <string>

#include "scope/common/style.hpp"
#include "scope/providers/converters.hpp"

#include "test/scope/common/test-files.hpp"

namespace scope {

namespace {

/// Releases images loaded by strtoimages.
void FreeImages(const Images &images) {
    for (const Image &image : images) {
        stbi_image_free(image.image);
    }
}

}  // namespace

// A single path loads that one image.
TEST(ConvertersTest, StrToImagesSinglePath) {
    TempFile file("converters-single.pgm", EncodePgm(4, 3, FlatPixels(4, 3, 7)));

    Images images = strtoimages(file.Path());

    ASSERT_EQ(images.size(), 1u);
    EXPECT_EQ(images[0].width, 4);
    EXPECT_EQ(images[0].height, 3);
    EXPECT_EQ(images[0].channels, 1);
    EXPECT_EQ(images[0].image[0], 7);
    FreeImages(images);
}

// A comma-separated list loads every image, in order.
TEST(ConvertersTest, StrToImagesCommaSeparated) {
    TempFile first("converters-comma-a.pgm", EncodePgm(4, 3, FlatPixels(4, 3, 1)));
    TempFile second("converters-comma-b.pgm", EncodePgm(4, 3, FlatPixels(4, 3, 2)));
    TempFile third("converters-comma-c.pgm", EncodePgm(4, 3, FlatPixels(4, 3, 3)));

    Images images = strtoimages(first.Path() + "," + second.Path() + "," + third.Path());

    ASSERT_EQ(images.size(), 3u);
    EXPECT_EQ(images[0].image[0], 1);
    EXPECT_EQ(images[1].image[0], 2);
    EXPECT_EQ(images[2].image[0], 3);
    FreeImages(images);
}

// A space-separated list loads every image, in order.
TEST(ConvertersTest, StrToImagesSpaceSeparated) {
    TempFile first("converters-space-a.pgm", EncodePgm(4, 3, FlatPixels(4, 3, 1)));
    TempFile second("converters-space-b.pgm", EncodePgm(4, 3, FlatPixels(4, 3, 2)));

    Images images = strtoimages(first.Path() + " " + second.Path());

    ASSERT_EQ(images.size(), 2u);
    EXPECT_EQ(images[0].image[0], 1);
    EXPECT_EQ(images[1].image[0], 2);
    FreeImages(images);
}

// Empty entries (a trailing or doubled delimiter) are skipped, not loaded.
TEST(ConvertersTest, StrToImagesSkipsEmptyEntries) {
    TempFile first("converters-empty-a.pgm", EncodePgm(4, 3, FlatPixels(4, 3, 1)));
    TempFile second("converters-empty-b.pgm", EncodePgm(4, 3, FlatPixels(4, 3, 2)));

    Images images = strtoimages(first.Path() + ",," + second.Path() + ",");

    ASSERT_EQ(images.size(), 2u);
    EXPECT_EQ(images[0].image[0], 1);
    EXPECT_EQ(images[1].image[0], 2);
    FreeImages(images);
}

// An empty string names no images.
TEST(ConvertersTest, StrToImagesEmptyString) {
    EXPECT_TRUE(strtoimages("").empty());
}

// A path that is not an image throws rather than yielding a partial list.
TEST(ConvertersTest, StrToImagesMissingFileThrows) {
    EXPECT_THROW(strtoimages("test/fixtures/does-not-exist.png"), std::runtime_error);
}

}  // namespace scope
