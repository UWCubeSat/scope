#include "scope/star-centroid/star-centroid.hpp"

#include <cstddef>
#include <cstdlib>

#include <optional>
#include <stdexcept>
#include <vector>

#include "common/spatial/attitude-utils.hpp"

#include "scope/projection/projection.hpp"
#include "scope/star-centroid/coi.hpp"

namespace scope {

namespace {

/// ROI side length, in pixels (Orion paper §"Image Processing and Star Centroiding").
constexpr int kRoiSize = 31;
/// Mask radius about the brightest pixel, in pixels (paper, same section).
constexpr int kRecenterRadius = 3;
/// Keep this clear of every edge so a full ROI never overruns the image.
constexpr int kSensorMargin = kRoiSize / 2 + 1;

/// Per-pixel saturating subtraction of the dark frame from a star image, clamped
/// to zero. Returns a newly malloc'd buffer the caller must std::free.
///
/// @param star The star-field image.
/// @param dark The dark frame to remove.
/// @return The dark-subtracted pixel buffer (same value count as the inputs).
/// @throws std::runtime_error if allocation fails.
unsigned char *DarkSubtract(const Image &star, const Image &dark) {
    const std::size_t valueCount = static_cast<std::size_t>(star.width) * static_cast<std::size_t>(star.height) *
                                   static_cast<std::size_t>(star.channels);
    unsigned char *out = static_cast<unsigned char *>(std::malloc(valueCount));
    // GCOVR_EXCL_START — malloc failure on a CubeSat-sized image is unrecoverable and not unit-testable.
    if (out == nullptr) {
        throw std::runtime_error("ROIFilterAlgorithm: failed to allocate dark-subtracted buffer");
    }
    // GCOVR_EXCL_STOP
    for (std::size_t i = 0; i < valueCount; ++i) {
        out[i] = star.image[i] > dark.image[i] ? static_cast<unsigned char>(star.image[i] - dark.image[i]) : 0;
    }
    return out;
}

}  // namespace

CentroidObservations ROIFilterAlgorithm::Run(const Image &darkFrame) {
    if (attitudes_.size() != options_.starImages.size()) {
        throw std::runtime_error("ROIFilterAlgorithm: attitude count does not match star image count");
    }

    CentroidObservations result;
    result.attitudes = attitudes_;
    result.catalog = &catalog_;

    const unsigned char threshold = static_cast<unsigned char>(options_.centroidThreshold);

    for (std::size_t i = 0; i < options_.starImages.size(); ++i) {
        const Image &star = options_.starImages[i];
        if (star.width != darkFrame.width || star.height != darkFrame.height || star.channels != darkFrame.channels) {
            throw std::runtime_error("ROIFilterAlgorithm: star image dimensions do not match the dark frame");
        }

        unsigned char *subtracted = DarkSubtract(star, darkFrame);
        const Image darkSubtracted{star.width, star.height, star.channels, subtracted};

        for (std::size_t j = 0; j < catalog_.size(); ++j) {
            const found::Vec2 expected = ProjectStarToPixel(catalog_[j].spatial, attitudes_[i], options_);
            if (!InSensorWithMargin(expected, star.width, star.height, kSensorMargin)) {
                continue;
            }

            const std::optional<found::Vec2> centroid =
                ExtractCentroid(darkSubtracted, expected, kRoiSize, kRecenterRadius, threshold);
            if (centroid.has_value()) {
                result.observations.push_back(Observation{static_cast<int>(i), static_cast<int>(j), *centroid});
            }
        }

        std::free(subtracted);
    }

    return result;
}

}  // namespace scope
