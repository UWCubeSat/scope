#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <getopt.h>

#include <cstdlib>

#include <string>
#include <vector>

#include "scope/command-line/scope-main.hpp"

#include "test/scope/common/test-files.hpp"

namespace scope {

class ScopeMainTest : public testing::Test {
 protected:
    /// getopt keeps its position in a global; rewind it for the next parse.
    void TearDown() override { optind = 2; }
};

// Running with no arguments reports the problem instead of reading past argv.
TEST_F(ScopeMainTest, NoArgumentsFails) {
    const char *argv[] = {"scope", nullptr};

    testing::internal::CaptureStderr();
    const int status = main(1, const_cast<char **>(argv));
    const std::string errors = testing::internal::GetCapturedStderr();

    EXPECT_EQ(status, EXIT_FAILURE);
    EXPECT_THAT(errors, testing::HasSubstr("No command provided"));
}

// --help lists the flags and succeeds without running anything.
TEST_F(ScopeMainTest, HelpListsFlags) {
    const char *argv[] = {"scope", "--help"};

    testing::internal::CaptureStdout();
    const int status = main(2, const_cast<char **>(argv));
    const std::string output = testing::internal::GetCapturedStdout();

    EXPECT_EQ(status, EXIT_SUCCESS);
    EXPECT_THAT(output, testing::HasSubstr("--dark-frames"));
    EXPECT_THAT(output, testing::HasSubstr("--star-images"));
    EXPECT_THAT(output, testing::HasSubstr("--attitudes"));
    EXPECT_THAT(output, testing::HasSubstr("--roi-size"));
}

TEST_F(ScopeMainTest, ShortHelpListsFlags) {
    const char *argv[] = {"scope", "-h"};

    testing::internal::CaptureStdout();
    const int status = main(2, const_cast<char **>(argv));
    const std::string output = testing::internal::GetCapturedStdout();

    EXPECT_EQ(status, EXIT_SUCCESS);
    EXPECT_THAT(output, testing::HasSubstr("--star-images"));
}

// A missing catalog is reported and returns a failure code; it does not abort.
TEST_F(ScopeMainTest, MissingCatalogFailsCleanly) {
    const char *argv[] = {"scope", "recalibrate", "--catalog-path", "test/fixtures/does-not-exist.tsv"};

    testing::internal::CaptureStderr();
    const int status = main(4, const_cast<char **>(argv));
    const std::string errors = testing::internal::GetCapturedStderr();

    EXPECT_EQ(status, EXIT_FAILURE);
    EXPECT_THAT(errors, testing::HasSubstr("failed to open catalog"));
}

// The attitude turns inertial +x onto the boresight, so the fixture catalog's
// star at (ra = 0, dec = 0) lands on the principal point, where the blob is.
TEST_F(ScopeMainTest, RunsEndToEndFromFiles) {
    std::vector<unsigned char> dark = FlatPixels(kWidth, kHeight, 10);
    std::vector<unsigned char> star = dark;
    PaintStar(&star, kWidth, 32, 32);

    TempFile darkA("main-dark-a.pgm", EncodePgm(kWidth, kHeight, dark));
    TempFile darkB("main-dark-b.pgm", EncodePgm(kWidth, kHeight, dark));
    TempFile starImage("main-star.pgm", EncodePgm(kWidth, kHeight, star));
    TempFile attitudes("main-attitudes.txt", "0.7071067811865476 0 -0.7071067811865476 0\n");
    const std::string darkList = darkA.Path() + "," + darkB.Path();

    const char *argv[] = {"scope",
                          "recalibrate",
                          "--focal-length-x",
                          "100",
                          "--focal-length-y",
                          "100",
                          "--principal-point-x",
                          "32",
                          "--principal-point-y",
                          "32",
                          "--dark-frames",
                          darkList.c_str(),
                          "--star-images",
                          starImage.Path().c_str(),
                          "--attitudes",
                          attitudes.Path().c_str(),
                          "--catalog-path",
                          kFixtureCatalog};
    const int argc = static_cast<int>(sizeof(argv) / sizeof(argv[0]));

    testing::internal::CaptureStdout();
    const int status = main(argc, const_cast<char **>(argv));
    const std::string output = testing::internal::GetCapturedStdout();

    EXPECT_EQ(status, EXIT_SUCCESS);
    EXPECT_THAT(output, testing::HasSubstr("Star image 0: 1 of 1 centroids kept"));
    EXPECT_THAT(output, testing::HasSubstr("Calibration result:"));
}

}  // namespace scope
