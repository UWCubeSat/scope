#include <gtest/gtest.h>

#include <getopt.h>
#include <stb_image/stb_image.h>

#include <cstdlib>

#include <string>

#include "common/decimal.hpp"

#include "scope/command-line/parsing/options.hpp"
#include "scope/command-line/parsing/parser.hpp"

#include "test/scope/common/test-files.hpp"

namespace scope {

class ParserTest : public testing::Test {
 protected:
    /// getopt keeps its position in a global; rewind it for the next parse.
    void TearDown() override { optind = 2; }
};

// The values are exactly representable, so they compare equal in both double
// and float builds.
TEST_F(ParserTest, ParsesEveryFlag) {
    TempFile darkA("parser-dark-a.pgm", EncodePgm(4, 3, FlatPixels(4, 3, 10)));
    TempFile darkB("parser-dark-b.pgm", EncodePgm(4, 3, FlatPixels(4, 3, 12)));
    TempFile star("parser-star.pgm", EncodePgm(4, 3, FlatPixels(4, 3, 90)));
    const std::string darkList = darkA.Path() + "," + darkB.Path();

    const char *argv[] = {"scope",
                          "recalibrate",
                          "--focal-length-x",
                          "3478.5",
                          "--focal-length-y",
                          "3480.25",
                          "--principal-point-x",
                          "1032.5",
                          "--principal-point-y",
                          "772.75",
                          "--alpha",
                          "0.125",
                          "--k1",
                          "-0.25",
                          "--k2",
                          "0.0625",
                          "--k3",
                          "-0.5",
                          "--p1",
                          "0.375",
                          "--p2",
                          "-0.75",
                          "--dark-frames",
                          darkList.c_str(),
                          "--star-images",
                          star.Path().c_str(),
                          "--attitudes",
                          "attitudes.txt",
                          "--catalog-path",
                          "catalog.tsv",
                          "--centroid-threshold",
                          "55",
                          "--magnitude-threshold",
                          "4.5",
                          "--roi-size",
                          "51"};
    const int argc = static_cast<int>(sizeof(argv) / sizeof(argv[0]));

    RecalibrationOptions options = ParseRecalibrationOptions(argc, const_cast<char **>(argv));

    EXPECT_EQ(options.focalLengthX, DECIMAL(3478.5));
    EXPECT_EQ(options.focalLengthY, DECIMAL(3480.25));
    EXPECT_EQ(options.principalX, DECIMAL(1032.5));
    EXPECT_EQ(options.principalY, DECIMAL(772.75));
    EXPECT_EQ(options.alpha, DECIMAL(0.125));
    EXPECT_EQ(options.k1, DECIMAL(-0.25));
    EXPECT_EQ(options.k2, DECIMAL(0.0625));
    EXPECT_EQ(options.k3, DECIMAL(-0.5));
    EXPECT_EQ(options.p1, DECIMAL(0.375));
    EXPECT_EQ(options.p2, DECIMAL(-0.75));
    EXPECT_EQ(options.attitudesPath, "attitudes.txt");
    EXPECT_EQ(options.catalogPath, "catalog.tsv");
    EXPECT_EQ(options.centroidThreshold, 55);
    EXPECT_EQ(options.magnitudeThreshold, DECIMAL(4.5));
    EXPECT_EQ(options.roiSize, 51);

    ASSERT_EQ(options.darkFrames.size(), 2u);
    EXPECT_EQ(options.darkFrames[0].image[0], 10);
    EXPECT_EQ(options.darkFrames[1].image[0], 12);
    ASSERT_EQ(options.starImages.size(), 1u);
    EXPECT_EQ(options.starImages[0].width, 4);
    EXPECT_EQ(options.starImages[0].height, 3);
    EXPECT_EQ(options.starImages[0].image[0], 90);

    stbi_image_free(options.darkFrames[0].image);
    stbi_image_free(options.darkFrames[1].image);
    stbi_image_free(options.starImages[0].image);
}

TEST_F(ParserTest, UnknownFlagExits) {
    const char *argv[] = {"scope", "recalibrate", "--input-images", "a.png"};

    ASSERT_EXIT(ParseRecalibrationOptions(4, const_cast<char **>(argv)), testing::ExitedWithCode(EXIT_FAILURE), "");
}

TEST_F(ParserTest, MissingValueExits) {
    const char *argv[] = {"scope", "recalibrate", "--roi-size"};

    ASSERT_EXIT(ParseRecalibrationOptions(3, const_cast<char **>(argv)), testing::ExitedWithCode(EXIT_FAILURE), "");
}

}  // namespace scope
