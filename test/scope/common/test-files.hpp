#ifndef TEST_SCOPE_COMMON_TEST_FILES_HPP_
#define TEST_SCOPE_COMMON_TEST_FILES_HPP_

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fstream>
#include <string>
#include <vector>

#include "scope/common/style.hpp"

namespace scope {

/// Size of the synthetic test images.
constexpr int kWidth = 64;
constexpr int kHeight = 64;

/// The small fixture catalog, relative to the repository root. Its first star
/// sits at (ra = 0, dec = 0), i.e. along inertial +x.
constexpr const char *kFixtureCatalog = "test/fixtures/bright-star-catalog-test.tsv";

/// A file under gtest's temp directory, deleted when it goes out of scope.
class TempFile {
 public:
    /// Writes contents to a new file called name in the temp directory.
    TempFile(const std::string &name, const std::string &contents) : path_(testing::TempDir() + name) {
        std::ofstream(path_, std::ios::binary) << contents;
    }

    ~TempFile() { std::remove(path_.c_str()); }

    TempFile(const TempFile &) = delete;
    TempFile &operator=(const TempFile &) = delete;

    /// The full path of the file.
    const std::string &Path() const { return path_; }

 private:
    std::string path_;
};

/// Encodes a single-channel 8-bit image as a binary PGM, a format stb_image
/// (and so the CLI's image flags) can load.
inline std::string EncodePgm(int width, int height, const std::vector<unsigned char> &pixels) {
    std::string encoded = "P5\n" + std::to_string(width) + " " + std::to_string(height) + "\n255\n";
    encoded.append(pixels.begin(), pixels.end());
    return encoded;
}

/// Row-major pixels of a width x height image filled with one value.
inline std::vector<unsigned char> FlatPixels(int width, int height, unsigned char value) {
    return std::vector<unsigned char>(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), value);
}

/// Paints a symmetric blob (bright center + 4 neighbors) at (x, y).
inline void PaintStar(std::vector<unsigned char> *pixels, int width, int x, int y) {
    const auto at = [&](int px, int py) -> unsigned char & {
        return (*pixels)[static_cast<std::size_t>(py) * static_cast<std::size_t>(width) + static_cast<std::size_t>(px)];
    };
    at(x, y) = 210;
    at(x - 1, y) = 110;
    at(x + 1, y) = 110;
    at(x, y - 1) = 110;
    at(x, y + 1) = 110;
}

/// Owns a kWidth x kHeight single-channel pixel buffer and exposes it as an Image.
class TestImage {
 public:
    explicit TestImage(unsigned char background = 0) : pixels_(FlatPixels(kWidth, kHeight, background)) {}

    void Set(int x, int y, unsigned char value) { pixels_[static_cast<std::size_t>(y) * kWidth + x] = value; }

    /// Paints a symmetric blob (bright center + 4 neighbors) at (x, y).
    void PaintStar(int x, int y) { scope::PaintStar(&pixels_, kWidth, x, y); }

    Image View() { return Image{kWidth, kHeight, 1, pixels_.data()}; }

 private:
    std::vector<unsigned char> pixels_;
};

/// Copies pixels into a malloc'd single-channel Image, the way stb_image hands
/// images to the CLI. For code that takes ownership of its images and frees them.
inline Image MallocImage(int width, int height, const std::vector<unsigned char> &pixels) {
    unsigned char *buffer = static_cast<unsigned char *>(std::malloc(pixels.size()));
    std::memcpy(buffer, pixels.data(), pixels.size());
    return Image{width, height, 1, buffer};
}

}  // namespace scope

#endif  // TEST_SCOPE_COMMON_TEST_FILES_HPP_
