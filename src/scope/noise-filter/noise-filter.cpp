#include "scope/noise-filter/noise-filter.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <stdexcept>
#include <vector>

namespace scope {

namespace {

/// Validates that input images are non-empty, non-null, and shape-compatible.
///
/// @param images Input images to validate.
/// @throws std::invalid_argument if the image collection is empty.
/// @throws std::runtime_error if any image is null, invalid, or mismatched.
void ValidateInput(const Images& images) {
    if (images.empty()) {
        throw std::invalid_argument(
            "DarkScreenFilter requires at least one image");
    }

    const Image& reference = images.front();
    if (reference.image == nullptr) {
        throw std::runtime_error("Input image pointer is null");
    }
    if (reference.width <= 0 || reference.height <= 0 ||
        reference.channels <= 0) {
        throw std::runtime_error(
            "Input image dimensions/channels must be positive");
    }

    for (size_t i = 1; i < images.size(); ++i) {
        const Image& image = images[i];
        if (image.image == nullptr) {
            throw std::runtime_error("Input image pointer is null");
        }
        if (image.width != reference.width ||
            image.height != reference.height ||
            image.channels != reference.channels) {
            throw std::runtime_error(
                "All images must share width, height, and channels");
        }
    }
}

}  // namespace

///// DarkScreenFilter /////

Image DarkScreenFilter::Run(const Images& images) {
    ValidateInput(images);

    const Image& reference = images.front();
    const size_t valueCount = static_cast<size_t>(reference.width) *
                              static_cast<size_t>(reference.height) *
                              static_cast<size_t>(reference.channels);

    unsigned char* buffer =
        static_cast<unsigned char*>(std::malloc(valueCount));
    if (buffer == nullptr) {
        throw std::bad_alloc();
    }

    std::vector<unsigned char> samples(images.size());
    const size_t medianIndex = (images.size() - 1) / 2;

    for (size_t i = 0; i < valueCount; ++i) {
        for (size_t j = 0; j < images.size(); ++j) {
            samples[j] = images[j].image[i];
        }

        std::nth_element(
            samples.begin(),
            samples.begin() + static_cast<std::ptrdiff_t>(medianIndex),
            samples.end());
        buffer[i] = samples[medianIndex];
    }

    return Image{reference.width, reference.height, reference.channels, buffer};
}

}  // namespace scope
