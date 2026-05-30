#include "scope/star-centroid/star-centroid.hpp"

#include <cstddef>
#include <cstdlib>

#include <optional>
#include <stdexcept>
#include <vector>

#include "common/decimal.hpp"
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
/// Two centroids closer than this (pixels) are treated as the same blob -- a
/// collision the a-priori matcher cannot disambiguate. Sized to the mask radius.
constexpr decimal kCentroidMatchTolerance = DECIMAL(3.0);

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
    // Faintest magnitude worth projecting, in the catalog's (mag * 100) integer
    // units. Stars dimmer than this are unlikely to centroid and only crowd the
    // field with collision candidates, so they are skipped outright.
    const int magnitudeLimit = static_cast<int>(DECIMAL_ROUND(options_.magnitudeThreshold * DECIMAL(100.0)));

    for (std::size_t i = 0; i < options_.starImages.size(); ++i) {
        const Image &star = options_.starImages[i];
        if (star.width != darkFrame.width || star.height != darkFrame.height || star.channels != darkFrame.channels) {
            throw std::runtime_error("ROIFilterAlgorithm: star image dimensions do not match the dark frame");
        }

        unsigned char *subtracted = DarkSubtract(star, darkFrame);
        const Image darkSubtracted{star.width, star.height, star.channels, subtracted};

        // Gather this image's centroids first, then prune ambiguous ones below.
        std::vector<Observation> candidates;
        for (std::size_t j = 0; j < catalog_.size(); ++j) {
            if (catalog_[j].magnitude > magnitudeLimit) {
                continue;
            }

            const std::optional<found::Vec2> expected =
                ProjectStarToPixel(catalog_[j].spatial, attitudes_[i], options_);
            if (!expected.has_value() || !InSensorWithMargin(*expected, star.width, star.height, kSensorMargin)) {
                continue;
            }

            const std::optional<found::Vec2> centroid =
                ExtractCentroid(darkSubtracted, *expected, kRoiSize, kRecenterRadius, threshold);
            if (centroid.has_value()) {
                candidates.push_back(Observation{static_cast<int>(i), static_cast<int>(j), *centroid});
            }
        }

        std::free(subtracted);

        // Two catalog stars projecting close enough to share a blob yield an
        // ambiguous correspondence (same measured pixel, different catalog star)
        // that would feed the optimizer an outlier. Rather than guess which star
        // the blob belongs to, drop every candidate that collides with another in
        // this image.
        for (std::size_t a = 0; a < candidates.size(); ++a) {
            bool ambiguous = false;
            for (std::size_t b = 0; b < candidates.size(); ++b) {
                if (a == b) {
                    continue;
                }
                if ((candidates[a].measured_pixel - candidates[b].measured_pixel).norm() < kCentroidMatchTolerance) {
                    ambiguous = true;
                    break;
                }
            }
            if (!ambiguous) {
                result.observations.push_back(candidates[a]);
            }
        }
    }

    return result;
}

}  // namespace scope
