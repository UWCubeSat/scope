#include <gtest/gtest.h>

#include <fstream>
#include <stdexcept>
#include <string>

#include "scope/common/files.hpp"

#include "test/scope/common/test-files.hpp"

namespace scope {

TEST(WriteFileTest, ReplacesContents) {
    TempFile file("write-file.txt", "old contents that are longer");

    WriteFile(file.Path(), "new");

    EXPECT_EQ(ReadFile(file.Path()), "new");
}

TEST(WriteFileTest, MissingDirectoryThrows) {
    EXPECT_THROW(WriteFile(std::string(kMissingDirectory) + "file.txt", "x"), std::runtime_error);
}

// /dev/full opens but fails every write, as a full disk would.
TEST(WriteFileTest, FailedWriteThrows) {
    if (!std::ifstream("/dev/full")) {
        GTEST_SKIP() << "needs /dev/full";
    }
    EXPECT_THROW(WriteFile("/dev/full", "x"), std::runtime_error);
}

}  // namespace scope
