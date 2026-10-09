#include "scope/catalog/catalog.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <stdexcept>
#include <string>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

namespace scope {

namespace {

/// Converts a right ascension and declination (radians) to a unit vector in the
/// equatorial frame, matching LOST's SphericalToSpatial convention: a star at
/// (ra = 0, dec = 0) maps to (1, 0, 0).
///
/// @param ra Right ascension, in radians.
/// @param dec Declination, in radians.
/// @return The corresponding unit line-of-sight vector.
found::Vec3 SphericalToSpatial(decimal ra, decimal dec) {
    return found::Vec3{
        DECIMAL_COS(ra) * DECIMAL_COS(dec),
        DECIMAL_SIN(ra) * DECIMAL_COS(dec),
        DECIMAL_SIN(dec),
    };
}

}  // namespace

Catalog LoadBsc(const std::string &path) {
    FILE *file = std::fopen(path.c_str(), "r");
    if (file == nullptr) {
        throw std::runtime_error("LoadBsc: failed to open catalog '" + path + "': " + std::strerror(errno));
    }

    Catalog catalog;
    // Fields per line, pipe-separated: RA(deg), Dec(deg), HR name, single-char
    // Multiple flag (discarded), and Vmag.
    double raDeg;
    double decDeg;
    int name;
    char multiple;
    double vmag;

    // Read into double regardless of decimal width so the scanf format never
    // disagrees with the storage type, then narrow to decimal during conversion.
    while (std::fscanf(file, "%lf|%lf|%d|%c|%lf", &raDeg, &decDeg, &name, &multiple, &vmag) == 5) {
        const decimal ra = found::DegToRad(DECIMAL(raDeg));
        const decimal dec = found::DegToRad(DECIMAL(decDeg));
        // Store apparent magnitude scaled by 100, matching LOST's integer
        // convention. Read Vmag as a single float and round so the sign survives
        // for stars in (-1, 0) (e.g. -0.74); parsing the integer part on its own
        // would read "-0" as 0 and silently drop the minus.
        const int magnitude = static_cast<int>(std::lround(vmag * 100.0));
        catalog.push_back(CatalogStar{SphericalToSpatial(ra, dec), magnitude, name});
    }

    std::fclose(file);

    if (catalog.empty()) {
        throw std::runtime_error("LoadBsc: no stars parsed from catalog '" + path + "'");
    }

    return catalog;
}

}  // namespace scope
