#include "scope/attitude/attitude.hpp"

#include <algorithm>
#include <cstddef>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

namespace scope {

std::vector<found::Quaternion> LoadAttitudes(const std::string &path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("LoadAttitudes: failed to open attitudes file '" + path + "'");
    }

    std::vector<found::Quaternion> attitudes;
    std::string line;
    int lineNumber = 0;
    while (std::getline(file, line)) {
        ++lineNumber;

        const std::size_t comment = line.find('#');
        if (comment != std::string::npos) {
            line.erase(comment);
        }
        std::replace(line.begin(), line.end(), ',', ' ');
        if (line.find_first_not_of(" \t\r") == std::string::npos) {
            continue;
        }

        // Parse as double whatever the width of decimal, then narrow.
        std::istringstream fields(line);
        double w, x, y, z;
        std::string extra;
        if (!(fields >> w >> x >> y >> z) || (fields >> extra)) {
            throw std::runtime_error("LoadAttitudes: expected 'w x y z' on line " + std::to_string(lineNumber) +
                                     " of '" + path + "'");
        }

        found::Quaternion attitude(DECIMAL(w), DECIMAL(x), DECIMAL(y), DECIMAL(z));
        if (!(attitude.norm() > DECIMAL(0.0))) {
            throw std::runtime_error("LoadAttitudes: zero-length quaternion on line " + std::to_string(lineNumber) +
                                     " of '" + path + "'");
        }
        attitude.normalize();
        attitudes.push_back(attitude);
    }

    return attitudes;
}

}  // namespace scope
