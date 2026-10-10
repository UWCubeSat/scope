#include "scope/star-centroid/coi.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

namespace scope {

std::optional<found::Vec2> ExtractCentroid(const Image &darkSubtracted,
                                           const found::Vec2 &expectedPixel,
                                           int roiSize,
                                           int recenterRadius,
                                           unsigned char threshold) {
    const int width = darkSubtracted.width;
    const int height = darkSubtracted.height;
    const int channels = darkSubtracted.channels;
    const int thr = threshold;

    // First-channel intensity at (x, y).
    const auto intensity = [&](int x, int y) -> int {
        return darkSubtracted.image[(static_cast<std::size_t>(y) * width + x) * channels];
    };

    const int cx = static_cast<int>(DECIMAL_ROUND(expectedPixel.x()));
    const int cy = static_cast<int>(DECIMAL_ROUND(expectedPixel.y()));
    const int half = roiSize / 2;
    const int x0 = std::max(0, cx - half);
    const int x1 = std::min(width - 1, cx + half);
    const int y0 = std::max(0, cy - half);
    const int y1 = std::min(height - 1, cy + half);

    // Brightest pixel in the ROI.
    int bx = -1;
    int by = -1;
    int brightest = -1;
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            const int val = intensity(x, y);
            if (val > brightest) {
                brightest = val;
                bx = x;
                by = y;
            }
        }
    }

    // The ROI was clamped away entirely, or nothing in it clears the threshold.
    if (bx < 0 || brightest <= thr) {
        return std::nullopt;
    }

    // Intensity-weighted center of the masked pixels around the brightest one.
    const int radius2 = recenterRadius * recenterRadius;
    const int mx0 = std::max(0, bx - recenterRadius);
    const int mx1 = std::min(width - 1, bx + recenterRadius);
    const int my0 = std::max(0, by - recenterRadius);
    const int my1 = std::min(height - 1, by + recenterRadius);

    decimal sumU = DECIMAL(0.0);
    decimal sumV = DECIMAL(0.0);
    int64_t iTot = 0;
    for (int y = my0; y <= my1; ++y) {
        for (int x = mx0; x <= mx1; ++x) {
            const int dx = x - bx;
            const int dy = y - by;
            if (dx * dx + dy * dy > radius2) {
                continue;
            }
            const int val = intensity(x, y);
            if (val <= thr) {
                continue;
            }
            iTot += val;
            sumU += DECIMAL(val) * DECIMAL(x);
            sumV += DECIMAL(val) * DECIMAL(y);
        }
    }

    // The brightest pixel is always in the mask, so iTot > 0.
    return found::Vec2{sumU / DECIMAL(iTot), sumV / DECIMAL(iTot)};
}

}  // namespace scope
