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
 * A single catalog star.
 *
 * The data shape mirrors LOST's CatalogStar so an eventual LOST integration is a
 * thin adapter, but the type is SCOPE-owned and backed by FOUND's Vec3.
 */
struct CatalogStar {
    /// Unit line-of-sight vector in the inertial (equatorial J2000) frame.
    /// This is the e_I used by the calibration measurement model.
    found::Vec3 spatial;
    /// Apparent magnitude stored as (true magnitude * 100), matching LOST's convention.
    int magnitude;
    /// Stable identifier (the HR number); survives catalog filtering.
    int name;
};

/// A collection of catalog stars.
using Catalog = std::vector<CatalogStar>;

/**
 * Loads the Yale Bright Star Catalog (BSC, Vizier V/50) from a pre-filtered TSV.
 *
 * The TSV is the output of download-bsc.sh: one star per line, pipe-separated as
 * `RA(deg)|Dec(deg)|HR|Multiple|Vmag`, where Vmag is written as `<high>.<low>`.
 *
 * @param path Path to the catalog TSV.
 *
 * @return The parsed catalog. Each star's spatial vector is a unit vector built
 *         from its J2000 right ascension and declination.
 *
 * @throws std::runtime_error if the file cannot be opened or no stars parse.
 */
Catalog LoadBsc(const std::string &path);

}  // namespace scope

#endif  // SRC_SCOPE_CATALOG_CATALOG_HPP_
