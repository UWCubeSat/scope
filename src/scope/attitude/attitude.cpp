#include "scope/attitude/attitude.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cstddef>

#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/common/files.hpp"

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

void WriteAttitudes(const std::string &path,
                    const std::vector<found::Quaternion> &attitudes,
                    const std::string &header) {
    std::ostringstream text;
    std::istringstream headerLines(header);
    std::string line;
    while (std::getline(headerLines, line)) {
        text << "# " << line << "\n";
    }

    // Enough digits that the attitudes read back exactly.
    text << std::setprecision(std::numeric_limits<decimal>::max_digits10);
    for (const found::Quaternion &attitude : attitudes) {
        text << attitude.w() << " " << attitude.x() << " " << attitude.y() << " " << attitude.z() << "\n";
    }
    WriteFile(path, text.str());
}

found::Quaternion AttitudeCorrection(const found::Vec3 &dtheta) {
    const decimal angle = dtheta.norm();
    if (angle == DECIMAL(0.0)) {
        return found::Quaternion::Identity();
    }
    // Eigen's AngleAxis(a, n) is exp(a [n x]), so exp(-[dtheta x]) is a
    // rotation by -|dtheta| about dtheta.
    return found::Quaternion(found::AngleAxis(-angle, dtheta / angle));
}

found::Vec3 CorrectionRotationVector(const found::Quaternion &correction) {
    const found::AngleAxis angleAxis(correction);
    return -angleAxis.angle() * angleAxis.axis();
}

found::Quaternion AverageQuaternions(const std::vector<found::Quaternion> &quaternions) {
    if (quaternions.empty()) {
        throw std::invalid_argument("AverageQuaternions: no quaternions to average");
    }

    Eigen::Matrix<decimal, 4, 4> sum = Eigen::Matrix<decimal, 4, 4>::Zero();
    for (const found::Quaternion &q : quaternions) {
        const Eigen::Matrix<decimal, 4, 1> v(q.w(), q.x(), q.y(), q.z());
        sum += v * v.transpose();
    }

    // sum is positive semi-definite, so its first singular vector is the
    // eigenvector of the largest eigenvalue. (SelfAdjointEigenSolver trips a
    // false maybe-uninitialized warning in GCC 13 at -O2.)
    const Eigen::JacobiSVD<Eigen::Matrix<decimal, 4, 4>> svd(sum, Eigen::ComputeFullU);
    Eigen::Matrix<decimal, 4, 1> mean = svd.matrixU().col(0);
    // Either sign is the same rotation; a non-negative real part reads better.
    if (mean(0) < DECIMAL(0.0)) {
        mean = -mean;
    }
    return found::Quaternion(mean(0), mean(1), mean(2), mean(3)).normalized();
}

}  // namespace scope
