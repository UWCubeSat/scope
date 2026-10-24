/**
 * @file catalog.hpp
 * @brief Bright star catalog types and loader for SCOPE calibration.
 */

#ifndef SRC_SCOPE_CATALOG_CATALOG_HPP_
#define SRC_SCOPE_CATALOG_CATALOG_HPP_

#include <string>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

namespace scope {

/**
 * A single catalog star, shaped like LOST's CatalogStar.
 */
struct CatalogStar {
    /// Unit line of sight in the inertial (equatorial J2000) frame.
    found::Vec3 spatial;
    /// Apparent magnitude times 100, as in LOST.
    int magnitude;
    /// The HR number.
    int name;
};

/// A collection of catalog stars.
using Catalog = std::vector<CatalogStar>;

/// The epoch of the positions in the Yale Bright Star Catalog, as a Julian year.
constexpr decimal kBscEpoch = DECIMAL(2000.0);

/**
 * Converts a catalog position to a unit line of sight, moved along the star's
 * proper motion from the catalog epoch. Axes follow LOST: (ra = 0, dec = 0)
 * maps to +x and the north celestial pole to +z.
 *
 * The motion is applied along the tangent plane at the catalog position and
 * the result renormalized. Over a few decades the error from ignoring the
 * sky's curvature is well under 0.01 arcseconds for every star in the
 * catalog.
 *
 * @param ra Right ascension at the catalog epoch, in radians.
 * @param dec Declination at the catalog epoch, in radians.
 * @param pmRaCosDec Proper motion in right ascension times cos(dec), in
 *                   arcseconds per year, as V/50's pmRA is given.
 * @param pmDec Proper motion in declination, in arcseconds per year.
 * @param years Time from the catalog epoch to the target epoch, in Julian years.
 *
 * @return The unit line of sight at the target epoch, in the catalog's frame.
 */
found::Vec3 PropagateProperMotion(decimal ra, decimal dec, decimal pmRaCosDec, decimal pmDec, decimal years);

/**
 * Loads the Yale Bright Star Catalog (Vizier V/50) from the file written by
 * download-bsc.sh: one star per line,
 * `RA(deg)|Dec(deg)|HR|Multiple|Vmag|pmRA|pmDE`, with the proper motions in
 * arcseconds per year and pmRA already multiplied by cos(dec).
 *
 * The two proper-motion columns are optional, so a catalog downloaded before
 * they were added still loads; a star without them (or with them blank) is
 * left where the catalog puts it. Blank lines and lines starting with `#` are
 * skipped.
 *
 * @param path Path to the catalog file.
 * @param epoch The Julian year to move the stars to, such as 2026.8 for
 *              October 2026. The default, kBscEpoch, leaves them as listed.
 *
 * @return The parsed catalog.
 *
 * @throws std::runtime_error if the file cannot be opened, a line is not a
 *         catalog row, or no stars parse.
 */
Catalog LoadBsc(const std::string &path, decimal epoch = kBscEpoch);

}  // namespace scope

#endif  // SRC_SCOPE_CATALOG_CATALOG_HPP_
