#include "scope/star-centroid/star-centroid.hpp"

#include <algorithm>
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
/// The median offset is applied only when at least this many isolated stars
/// measure it, so that one wrong match cannot move it far.
constexpr std::size_t kMinOffsetStars = 3;

/// A catalog star and the pixel the prior puts it at.
struct Prediction {
    std::size_t catalogIndex;
    found::Vec2 pixel;
};

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

/// For each prediction, the indices of the others whose windows overlap its
/// own: those within roiSize of it in both u and v.
std::vector<std::vector<std::size_t>> FindNeighbours(const std::vector<Prediction> &predictions, int roiSize) {
    const decimal reach = DECIMAL(roiSize);
    std::vector<std::vector<std::size_t>> neighbours(predictions.size());
    for (std::size_t a = 0; a < predictions.size(); ++a) {
        for (std::size_t b = a + 1; b < predictions.size(); ++b) {
            const found::Vec2 apart = predictions[b].pixel - predictions[a].pixel;
            if (DECIMAL_ABS(apart.x()) <= reach && DECIMAL_ABS(apart.y()) <= reach) {
                neighbours[a].push_back(b);
                neighbours[b].push_back(a);
            }
        }
    }
    return neighbours;
}

/// The median of the values (the lower one for an even count).
decimal Median(std::vector<decimal> values) {
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>((values.size() - 1) / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

/// The median offset of the centroids from their predictions over the stars
/// with no neighbour, which have their windows to themselves; zero if fewer
/// than kMinOffsetStars of them centroid. Mostly the attitude error.
found::Vec2 MedianOffset(const Image &darkSubtracted,
                         const std::vector<Prediction> &predictions,
                         const std::vector<std::vector<std::size_t>> &neighbours,
                         int roiSize,
                         int sensorMargin,
                         unsigned char threshold) {
    std::vector<decimal> offsetsU;
    std::vector<decimal> offsetsV;
    for (std::size_t k = 0; k < predictions.size(); ++k) {
        const found::Vec2 &expected = predictions[k].pixel;
        if (!neighbours[k].empty() ||
            !InSensorWithMargin(expected, darkSubtracted.width, darkSubtracted.height, sensorMargin)) {
            continue;
        }
        const std::optional<found::Vec2> centroid =
            ExtractCentroid(darkSubtracted, expected, roiSize, kRecenterRadius, threshold);
        if (centroid.has_value()) {
            offsetsU.push_back(centroid->x() - expected.x());
            offsetsV.push_back(centroid->y() - expected.y());
        }
    }
    if (offsetsU.size() < kMinOffsetStars) {
        return found::Vec2(DECIMAL(0.0), DECIMAL(0.0));
    }
    return found::Vec2(Median(offsetsU), Median(offsetsV));
}

/// Appends to observations each candidate with no other candidate's centroid
/// within kCentroidMatchTolerance. A blob about halfway between two
/// predictions could be either star's, so both are dropped.
void KeepUnambiguous(const std::vector<Observation> &candidates, std::vector<Observation> *observations) {
    for (std::size_t a = 0; a < candidates.size(); ++a) {
        bool ambiguous = false;
        for (std::size_t b = 0; b < candidates.size() && !ambiguous; ++b) {
            ambiguous =
                a != b && (candidates[a].measuredPixel - candidates[b].measuredPixel).norm() < kCentroidMatchTolerance;
        }
        if (!ambiguous) {
            observations->push_back(candidates[a]);
        }
    }
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

        // Stars bright enough to search for, on the sensor or within a window
        // of its edge, where they still claim their side of a neighbour's.
        std::vector<Prediction> predictions;
        for (std::size_t j = 0; j < catalog_.size(); ++j) {
            if (catalog_[j].magnitude > magnitudeLimit) {
                continue;
            }
            const std::optional<found::Vec2> pixel = ProjectStarToPixel(catalog_[j].spatial, attitudes_[i], camera_);
            if (pixel.has_value() && InSensorWithMargin(*pixel, star.width, star.height, -options_.roiSize)) {
                predictions.push_back(Prediction{j, *pixel});
            }
        }
        const std::vector<std::vector<std::size_t>> neighbours = FindNeighbours(predictions, options_.roiSize);

        const found::Vec2 offset =
            MedianOffset(darkSubtracted, predictions, neighbours, options_.roiSize, sensorMargin, threshold);

        // Searching only its own side of each neighbour keeps a star too faint
        // to centroid from taking a brighter neighbour's blob.
        std::vector<Observation> candidates;
        for (std::size_t k = 0; k < predictions.size(); ++k) {
            const found::Vec2 expected = predictions[k].pixel + offset;
            if (!InSensorWithMargin(expected, star.width, star.height, sensorMargin)) {
                continue;
            }
            std::vector<found::Vec2> nearby;
            for (std::size_t n : neighbours[k]) {
                nearby.push_back(predictions[n].pixel + offset);
            }
            const std::optional<found::Vec2> centroid =
                ExtractCentroid(darkSubtracted, expected, options_.roiSize, kRecenterRadius, threshold, nearby);
            if (centroid.has_value()) {
                const CatalogStar &catalogStar = catalog_[predictions[k].catalogIndex];
                candidates.push_back(
                    Observation{static_cast<int>(i), catalogStar.name, catalogStar.spatial, *centroid});
            }
        }

        std::free(subtracted);

        [[maybe_unused]] const std::size_t observationsBefore = result.observations.size();
        KeepUnambiguous(candidates, &result.observations);
        LOG_INFO("Star image " << i << ": " << result.observations.size() - observationsBefore << " of "
                               << candidates.size() << " centroids kept, windows moved by (" << offset.x() << ", "
                               << offset.y() << ") px");
    }

    return result;
}

}  // namespace scope
