#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <getopt.h>
#include <stb_image/stb_image.h>

#include <cstddef>
#include <cstdlib>

#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/attitude/attitude.hpp"
#include "scope/command-line/parsing/options.hpp"
#include "scope/command-line/parsing/parser.hpp"
#include "scope/synthetic/synthetic-cli.hpp"

#include "test/scope/common/test-files.hpp"

namespace scope {

namespace {

std::string ReadFile(const std::string &path) {
    std::ifstream file(path);
    std::stringstream contents;
    contents << file.rdbuf();
    return contents.str();
}

int CountOccurrences(const std::string &text, const std::string &pattern) {
    int count = 0;
    for (std::size_t at = text.find(pattern); at != std::string::npos; at = text.find(pattern, at + 1)) {
        ++count;
    }
    return count;
}

/// A tiny data set: two 64 x 48 star images and one dark frame, from the
/// fixture catalog.
SynthesisOptions SmallDataSet(const std::string &directory) {
    SynthesisOptions options;
    options.outputDir = directory;
    options.catalogPath = kFixtureCatalog;
    options.starImageCount = 2;
    options.darkFrameCount = 1;
    options.width = 64;
    options.height = 48;
    options.focalLengthX = DECIMAL(40.0);
    options.focalLengthY = DECIMAL(40.0);
    options.principalX = DECIMAL(31.5);
    options.principalY = DECIMAL(23.5);
    options.k1 = DECIMAL(0.0);
    options.k2 = DECIMAL(0.0);
    return options;
}

}  // namespace

class SynthCliTest : public testing::Test {
 protected:
    SynthCliTest() : directory_(testing::TempDir() + "scope-synth-test") {}

    void TearDown() override {
        std::filesystem::remove_all(directory_);
        // scope's parser expects getopt to start at its second argument.
        optind = 2;
    }

    const std::string directory_;
};

// Unlike scope, scope-synth has no command word, so the flags start at argv[1].
TEST_F(SynthCliTest, ParsesFlags) {
    const char *argv[] = {"scope-synth",
                          "--output-dir",
                          "out",
                          "--star-images",
                          "3",
                          "--k1",
                          "-0.125",
                          "--psf-sigma",
                          "1.5",
                          "--seed",
                          "9",
                          "--misalignment",
                          "0"};
    const int argc = static_cast<int>(sizeof(argv) / sizeof(argv[0]));

    const SynthesisOptions options = ParseSynthesisOptions(argc, const_cast<char **>(argv));

    EXPECT_EQ(options.outputDir, "out");
    EXPECT_EQ(options.starImageCount, 3);
    EXPECT_EQ(options.k1, DECIMAL(-0.125));
    EXPECT_EQ(options.psfSigma, DECIMAL(1.5));
    EXPECT_EQ(options.seed, 9);
    EXPECT_EQ(options.misalignment, DECIMAL(0.0));
    // Unset flags keep their defaults.
    EXPECT_EQ(options.width, 2064);
    EXPECT_EQ(options.darkFrameCount, 5);
}

TEST_F(SynthCliTest, RejectsUnknownFlagAndStrayArgument) {
    const char *unknown[] = {"scope-synth", "--output-dir", "out", "--bogus", "1"};
    const char *missingValue[] = {"scope-synth", "--seed"};
    const char *stray[] = {"scope-synth", "--output-dir", "out", "extra"};

    EXPECT_THROW(ParseSynthesisOptions(5, const_cast<char **>(unknown)), std::runtime_error);
    EXPECT_THROW(ParseSynthesisOptions(2, const_cast<char **>(missingValue)), std::runtime_error);
    EXPECT_THROW(ParseSynthesisOptions(4, const_cast<char **>(stray)), std::runtime_error);
}

// The data set is what scope reads: its arguments file parses with scope's
// own parser and loads every image.
TEST_F(SynthCliTest, WritesDataSetThatScopeLoads) {
    const std::string summary = WriteSyntheticDataSet(SmallDataSet(directory_));

    EXPECT_THAT(summary, testing::HasSubstr("Wrote 1 dark frames, 2 star images"));
    EXPECT_EQ(LoadAttitudes(directory_ + "/attitudes.txt").size(), 2u);

    const std::string truth = ReadFile(directory_ + "/truth.txt");
    EXPECT_THAT(truth, testing::HasSubstr("focal-length-x: 40\n"));
    EXPECT_THAT(truth, testing::HasSubstr("principal-point-y: 23.5\n"));
    EXPECT_THAT(truth, testing::HasSubstr("misalignment: "));
    EXPECT_EQ(CountOccurrences(truth, "true-attitude: "), 2);

    std::istringstream arguments(ReadFile(directory_ + "/scope-args.txt"));
    std::vector<std::string> words = {"scope", "run"};
    std::string word;
    while (arguments >> word) {
        words.push_back(word);
    }
    std::vector<char *> argv;
    for (std::string &w : words) {
        argv.push_back(w.data());
    }
    optind = 2;
    const RecalibrationOptions options = ParseRecalibrationOptions(static_cast<int>(argv.size()), argv.data());

    ASSERT_EQ(options.darkFrames.size(), 1u);
    ASSERT_EQ(options.starImages.size(), 2u);
    EXPECT_EQ(options.starImages[1].width, 64);
    EXPECT_EQ(options.starImages[1].height, 48);
    EXPECT_EQ(options.attitudesPath, directory_ + "/attitudes.txt");
    EXPECT_EQ(options.catalogPath, kFixtureCatalog);
    stbi_image_free(options.darkFrames[0].image);
    stbi_image_free(options.starImages[0].image);
    stbi_image_free(options.starImages[1].image);
}

TEST_F(SynthCliTest, SameSeedGivesSameDataSet) {
    WriteSyntheticDataSet(SmallDataSet(directory_ + "/a"));
    WriteSyntheticDataSet(SmallDataSet(directory_ + "/b"));

    EXPECT_EQ(ReadFile(directory_ + "/a/star-001.pgm"), ReadFile(directory_ + "/b/star-001.pgm"));
    EXPECT_EQ(ReadFile(directory_ + "/a/attitudes.txt"), ReadFile(directory_ + "/b/attitudes.txt"));
}

// A directory where truth.txt should go makes that write fail.
TEST_F(SynthCliTest, FailedWriteThrows) {
    std::filesystem::create_directories(directory_ + "/truth.txt");

    EXPECT_THROW(WriteSyntheticDataSet(SmallDataSet(directory_)), std::runtime_error);
}

TEST_F(SynthCliTest, RejectsBadOptions) {
    SynthesisOptions noDirectory = SmallDataSet("");
    SynthesisOptions noImages = SmallDataSet(directory_);
    noImages.starImageCount = 0;
    SynthesisOptions noWidth = SmallDataSet(directory_);
    noWidth.width = 0;
    SynthesisOptions flatPsf = SmallDataSet(directory_);
    flatPsf.psfSigma = DECIMAL(0.0);
    SynthesisOptions noCatalog = SmallDataSet(directory_);
    noCatalog.catalogPath = "test/fixtures/does-not-exist.tsv";

    EXPECT_THROW(WriteSyntheticDataSet(noDirectory), std::runtime_error);
    EXPECT_THROW(WriteSyntheticDataSet(noImages), std::runtime_error);
    EXPECT_THROW(WriteSyntheticDataSet(noWidth), std::runtime_error);
    EXPECT_THROW(WriteSyntheticDataSet(flatPsf), std::runtime_error);
    EXPECT_THROW(WriteSyntheticDataSet(noCatalog), std::runtime_error);
}

TEST_F(SynthCliTest, MainPrintsHelp) {
    const char *argv[] = {"scope-synth", "--help"};

    testing::internal::CaptureStdout();
    const int status = SynthMain(2, const_cast<char **>(argv));
    const std::string output = testing::internal::GetCapturedStdout();

    EXPECT_EQ(status, EXIT_SUCCESS);
    EXPECT_THAT(output, testing::HasSubstr("--output-dir"));
    EXPECT_THAT(output, testing::HasSubstr("--zero-point"));
}

TEST_F(SynthCliTest, MainWritesDataSetAndReportsErrors) {
    const std::string directoryFlag = directory_;
    const char *good[] = {"scope-synth",
                          "--output-dir",
                          directoryFlag.c_str(),
                          "--catalog-path",
                          kFixtureCatalog,
                          "--width",
                          "32",
                          "--height",
                          "24",
                          "--star-images",
                          "1",
                          "--dark-frames",
                          "1"};
    const char *bad[] = {"scope-synth", "--bogus"};

    testing::internal::CaptureStdout();
    const int goodStatus = SynthMain(13, const_cast<char **>(good));
    const std::string output = testing::internal::GetCapturedStdout();
    testing::internal::CaptureStderr();
    const int badStatus = SynthMain(2, const_cast<char **>(bad));
    const std::string errors = testing::internal::GetCapturedStderr();

    EXPECT_EQ(goodStatus, EXIT_SUCCESS);
    EXPECT_THAT(output, testing::HasSubstr("star image 0: "));
    EXPECT_TRUE(std::filesystem::exists(directory_ + "/star-000.pgm"));
    EXPECT_EQ(badStatus, EXIT_FAILURE);
    EXPECT_THAT(errors, testing::HasSubstr("Error: unknown flag or missing value: --bogus"));
}

}  // namespace scope
