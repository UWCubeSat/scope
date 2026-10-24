#include "scope/catalog/catalog.hpp"

#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

namespace scope {

namespace {

/// Whether a line holds nothing to parse: only whitespace, or a `#` comment.
bool IsBlankOrComment(const std::string &line) {
    const std::size_t first = line.find_first_not_of(" \t\r\n");
    return first == std::string::npos || line[first] == '#';
}

}  // namespace

found::Vec3 PropagateProperMotion(decimal ra, decimal dec, decimal pmRaCosDec, decimal pmDec, decimal years) {
    const decimal sinRa = DECIMAL_SIN(ra);
    const decimal cosRa = DECIMAL_COS(ra);
    const decimal sinDec = DECIMAL_SIN(dec);
    const decimal cosDec = DECIMAL_COS(dec);

    const found::Vec3 position{cosDec * cosRa, cosDec * sinRa, sinDec};
    // Unit vectors toward increasing right ascension and declination. Both are
    // defined at the poles too, where right ascension still fixes the axes.
    const found::Vec3 east{-sinRa, cosRa, DECIMAL(0.0)};
    const found::Vec3 north{-sinDec * cosRa, -sinDec * sinRa, cosDec};

    const found::Vec3 motion = found::ArcSecToRad(pmRaCosDec) * east + found::ArcSecToRad(pmDec) * north;
    return (position + years * motion).normalized();
}

Catalog LoadBsc(const std::string &path, decimal epoch) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("LoadBsc: failed to open catalog '" + path + "': " + std::strerror(errno));
    }

    const decimal years = epoch - kBscEpoch;

    Catalog catalog;
    std::string line;
    int lineNumber = 0;
    while (std::getline(file, line)) {
        ++lineNumber;
        if (IsBlankOrComment(line)) {
            continue;
        }

        double raDeg;
        double decDeg;
        int name;
        char multiple;  // discarded
        double vmag;
        double pmRaCosDec;
        double pmDec;

        // Scan as double whatever the width of decimal, then narrow. Blank
        // proper-motion fields stop the scan early, as missing ones do.
        const int fields = std::sscanf(line.c_str(), "%lf|%lf|%d|%c|%lf|%lf|%lf", &raDeg, &decDeg, &name, &multiple,
                                       &vmag, &pmRaCosDec, &pmDec);
        if (fields < 5) {
            throw std::runtime_error("LoadBsc: line " + std::to_string(lineNumber) + " of catalog '" + path +
                                     "' is not a catalog row");
        }
        if (fields < 7) {
            pmRaCosDec = 0.0;
            pmDec = 0.0;
        }

        const found::Vec3 spatial = PropagateProperMotion(found::DegToRad(DECIMAL(raDeg)),
                                                          found::DegToRad(DECIMAL(decDeg)),
                                                          DECIMAL(pmRaCosDec),
                                                          DECIMAL(pmDec),
                                                          years);

        // Vmag is read whole and rounded so the sign survives for stars in
        // (-1, 0): parsing the integer part alone reads "-0" as 0.
        const int magnitude = static_cast<int>(std::lround(vmag * 100.0));
        catalog.push_back(CatalogStar{spatial, magnitude, name});
    }

    if (catalog.empty()) {
        throw std::runtime_error("LoadBsc: no stars parsed from catalog '" + path + "'");
    }

    return catalog;
}

}  // namespace scope
