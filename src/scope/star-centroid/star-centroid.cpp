#include "scope/star-centroid/star-centroid.hpp"

#include <cstddef>
#include <cstdlib>

#include <optional>
#include <stdexcept>
#include <vector>

#include "common/decimal.hpp"
#include "common/logging.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/projection/projection.hpp"
#include "scope/star-centroid/coi.hpp"

namespace scope {

namespace {

/// Mask radius about the brightest pixel, in pixels (Orion paper §"Image
/// Processing and Star Centroiding").
constexpr int kRecenterRadius = 3;
/// Two centroids closer than this (pixels) are treated as the same blob. Sized
/// to the mask radius.
constexpr decimal kCentroidMatchTolerance = DECIMAL(3.0);

/// Subtracts the dark frame from a star image, clamping at zero. The caller
/// must std::free the returned buffer.
unsigned char *DarkSubtract(const Image &star, const Image &dark) {
    const std::size_t valueCount = static_cast<std::size_t>(star.width) * star.height * star.channels;
    unsigned char *out = static_cast<unsigned char *>(std::malloc(valueCount));
    // GCOVR_EXCL_START: malloc failure is not unit-testable.
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
    if (options_.roiSize < 1) {
        throw std::runtime_error("ROIFilterAlgorithm: ROI size must be at least 1 pixel");
    }
    // Wide enough that a full ROI never overruns the image.
    const int sensorMargin = options_.roiSize / 2 + 1;

    CentroidObservations result;
    result.attitudes = attitudes_;

    const unsigned char threshold = static_cast<unsigned char>(options_.centroidThreshold);
    // In the catalog's units (magnitude * 100). Fainter stars rarely centroid
    // and only add collision candidates.
    const int magnitudeLimit = static_cast<int>(DECIMAL_ROUND(options_.magnitudeThreshold * DECIMAL(100.0)));

    for (std::size_t i = 0; i < options_.starImages.size(); ++i) {
        const Image &star = options_.starImages[i];
        if (star.width != darkFrame.width || star.height != darkFrame.height || star.channels != darkFrame.channels) {
            throw std::runtime_error("ROIFilterAlgorithm: star image dimensions do not match the dark frame");
        }

        unsigned char *subtracted = DarkSubtract(star, darkFrame);
        const Image darkSubtracted{star.width, star.height, star.channels, subtracted};

        std::vector<Observation> candidates;
        for (std::size_t j = 0; j < catalog_.size(); ++j) {
            if (catalog_[j].magnitude > magnitudeLimit) {
                continue;
            }

            const std::optional<found::Vec2> expected = ProjectStarToPixel(catalog_[j].spatial, attitudes_[i], camera_);
            if (!expected.has_value() || !InSensorWithMargin(*expected, star.width, star.height, sensorMargin)) {
                continue;
            }

            const std::optional<found::Vec2> centroid =
                ExtractCentroid(darkSubtracted, *expected, options_.roiSize, kRecenterRadius, threshold);
            if (centroid.has_value()) {
                candidates.push_back(
                    Observation{static_cast<int>(i), catalog_[j].name, catalog_[j].spatial, *centroid});
            }
        }

        std::free(subtracted);

        // Two catalog stars sharing a blob would hand the optimizer an outlier.
        // Rather than guess which star the blob belongs to, drop both.
        [[maybe_unused]] const std::size_t observationsBefore = result.observations.size();
        for (std::size_t a = 0; a < candidates.size(); ++a) {
            bool ambiguous = false;
            for (std::size_t b = 0; b < candidates.size() && !ambiguous; ++b) {
                ambiguous = a != b && (candidates[a].measuredPixel - candidates[b].measuredPixel).norm() <
                                          kCentroidMatchTolerance;
            }
            if (!ambiguous) {
                result.observations.push_back(candidates[a]);
            }
        }
        LOG_INFO("Star image " << i << ": " << result.observations.size() - observationsBefore << " of "
                               << candidates.size() << " centroids kept");
    }

    return result;
}

}  // namespace scope
