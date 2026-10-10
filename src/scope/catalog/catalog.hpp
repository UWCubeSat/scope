/**
 * @file catalog.hpp
 * @brief Bright star catalog types and loader for SCOPE calibration.
 */

#ifndef SRC_SCOPE_CATALOG_CATALOG_HPP_
#define SRC_SCOPE_CATALOG_CATALOG_HPP_

#include <string>
#include <vector>

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

/**
 * Loads the Yale Bright Star Catalog (Vizier V/50) from the file written by
 * download-bsc.sh: one star per line, `RA(deg)|Dec(deg)|HR|Multiple|Vmag`.
 *
 * @param path Path to the catalog file.
 *
 * @return The parsed catalog.
 *
 * @throws std::runtime_error if the file cannot be opened or no stars parse.
 */
Catalog LoadBsc(const std::string &path);

}  // namespace scope

#endif  // SRC_SCOPE_CATALOG_CATALOG_HPP_
