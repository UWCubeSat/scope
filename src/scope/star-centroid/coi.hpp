/**
 * @file coi.hpp
 * @brief Center-of-intensity star centroiding kernel.
 *
 * Implements the per-star centroid extraction described in the Orion paper
 * (Christian et al. 2016, §"Image Processing and Star Centroiding", Eqs. 8-9):
 * draw a ROI around the expected location, find the brightest pixel, recenter,
 * mask the pixels within a small radius above a threshold, and take the
 * intensity-weighted center of those masked pixels.
 */

#ifndef SRC_SCOPE_STAR_CENTROID_COI_HPP_
#define SRC_SCOPE_STAR_CENTROID_COI_HPP_

#include <optional>

#include "common/spatial/attitude-utils.hpp"

#include "scope/common/style.hpp"

namespace scope {

/**
 * Extracts a single star centroid via center of intensity (paper Eqs. 8-9).
 *
 * Steps:
 *   1. Define a roi_size x roi_size ROI centered on expected_pixel (clamped to
 *      the image bounds).
 *   2. Find the brightest pixel inside the ROI.
 *   3. Recenter on that brightest pixel.
 *   4. Build the mask of pixels within recenter_radius of the recentered point
 *      whose (dark-subtracted) intensity exceeds the threshold.
 *   5. Return the intensity-weighted center of the masked pixels.
 *
 * @param darkSubtracted Single-channel, dark-frame-subtracted image. If the
 *        image has multiple channels, only the first channel is used.
 * @param expectedPixel The expected star location [u, v] (column, row).
 * @param roiSize The ROI side length, in pixels (paper uses 31).
 * @param recenterRadius The mask radius about the brightest pixel (paper uses 3).
 * @param threshold Minimum intensity for a pixel to join the mask.
 *
 * @return The measured centroid [u', v'] (column, row), or std::nullopt if no
 *         pixel in the ROI exceeds the threshold.
 *
 * @pre The caller has verified expected_pixel is in-sensor with adequate margin.
 */
std::optional<found::Vec2> ExtractCentroid(const Image &darkSubtracted,
                                           const found::Vec2 &expectedPixel,
                                           int roiSize,
                                           int recenterRadius,
                                           unsigned char threshold);

}  // namespace scope

#endif  // SRC_SCOPE_STAR_CENTROID_COI_HPP_
