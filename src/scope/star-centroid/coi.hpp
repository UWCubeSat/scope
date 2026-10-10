/**
 * @file coi.hpp
 * @brief Center-of-intensity star centroiding kernel.
 *
 * From the Orion paper (Christian et al. 2016, §"Image Processing and Star
 * Centroiding", Eqs. 8-9).
 */

#ifndef SRC_SCOPE_STAR_CENTROID_COI_HPP_
#define SRC_SCOPE_STAR_CENTROID_COI_HPP_

#include <optional>

#include "common/spatial/attitude-utils.hpp"

#include "scope/common/style.hpp"

namespace scope {

/**
 * Extracts one star centroid by center of intensity (paper Eqs. 8-9): find the
 * brightest pixel in a ROI around the expected pixel, then take the
 * intensity-weighted center of the pixels within recenterRadius of it that
 * exceed the threshold.
 *
 * An integer coordinate is the center of a pixel, as in FOUND: a star lit on
 * the single pixel in column 40, row 30 centroids to exactly (40, 30). LOST's
 * centroider adds 0.5, so subtract 0.5 from a LOST centroid before comparing.
 *
 * @param darkSubtracted Dark-subtracted image. Only the first channel is used.
 * @param expectedPixel The expected star location [u, v] (column, row).
 * @param roiSize The ROI side length, in pixels (paper uses 31). The ROI spans
 *        roiSize / 2 pixels either side of the rounded expected pixel, clamped
 *        to the image, so an even value acts as the next odd one.
 * @param recenterRadius The mask radius about the brightest pixel (paper uses 3).
 * @param threshold Intensity a pixel must exceed to join the mask.
 *
 * @return The measured centroid [u', v'] (column, row), or std::nullopt if no
 *         pixel in the ROI exceeds the threshold.
 */
std::optional<found::Vec2> ExtractCentroid(const Image &darkSubtracted,
                                           const found::Vec2 &expectedPixel,
                                           int roiSize,
                                           int recenterRadius,
                                           unsigned char threshold);

}  // namespace scope

#endif  // SRC_SCOPE_STAR_CENTROID_COI_HPP_
