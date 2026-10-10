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

/// Unit vector for a right ascension and declination in radians, as in LOST:
/// (ra = 0, dec = 0) maps to (1, 0, 0).
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
    double raDeg;
    double decDeg;
    int name;
    char multiple;  // discarded
    double vmag;

    // Scan as double whatever the width of decimal, then narrow.
    while (std::fscanf(file, "%lf|%lf|%d|%c|%lf", &raDeg, &decDeg, &name, &multiple, &vmag) == 5) {
        const decimal ra = found::DegToRad(DECIMAL(raDeg));
        const decimal dec = found::DegToRad(DECIMAL(decDeg));
        // Vmag is read whole and rounded so the sign survives for stars in
        // (-1, 0): parsing the integer part alone reads "-0" as 0.
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
