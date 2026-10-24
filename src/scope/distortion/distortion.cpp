#include "scope/distortion/distortion.hpp"

#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/projection/projection.hpp"

namespace scope {

namespace {

/// Significant digits per value in the CSV, as in the calibration printout.
constexpr int kCsvPrecision = 10;

/// Newton's method stops once a step is smaller than this, relative to the
/// iterate. Convergence is quadratic, so the remaining error is far smaller.
const decimal kUndistortStepTolerance = DECIMAL_SQRT(std::numeric_limits<decimal>::epsilon());

/// The grid coordinates along one axis: 0, spacing, 2 * spacing, ... and
/// always size - 1.
std::vector<int> GridCoordinates(int size, int spacing) {
    std::vector<int> coordinates;
    for (int c = 0; c < size; c += spacing) {
        coordinates.push_back(c);
    }
    if (coordinates.back() != size - 1) {
        coordinates.push_back(size - 1);
    }
    return coordinates;
}

/// The undistorted image-plane point [x, y] a raw pixel observes, or
/// std::nullopt if BrownUndistort fails.
std::optional<found::Vec2> PixelToUndistortedImagePlane(const found::Vec2 &pixel, const CameraParameters &camera) {
    return BrownUndistort(PixelToImagePlane(pixel, camera), camera);
}

}  // namespace

found::Mat2 BrownDistortJacobian(const found::Vec2 &ideal, const CameraParameters &camera) {
    const decimal x = ideal.x();
    const decimal y = ideal.y();
    const decimal r2 = x * x + y * y;
    const decimal r4 = r2 * r2;
    const decimal r6 = r4 * r2;

    // Eq. 37: gradients of r^2, r^4 and r^6.
    const found::Vec2 dr2{DECIMAL(2.0) * x, DECIMAL(2.0) * y};
    const found::Vec2 dr4 = DECIMAL(2.0) * r2 * dr2;
    const found::Vec2 dr6 = DECIMAL(3.0) * r4 * dr2;

    const decimal radial = DECIMAL(1.0) + camera.k1 * r2 + camera.k2 * r4 + camera.k3 * r6;
    const found::Vec2 dRadial = camera.k1 * dr2 + camera.k2 * dr4 + camera.k3 * dr6;

    // Eq. 36, term by term.
    found::Mat2 jacobian = radial * found::Mat2::Identity() + ideal * dRadial.transpose();

    found::Mat2 decentering;
    decentering << DECIMAL(2.0) * camera.p1 * y + DECIMAL(4.0) * camera.p2 * x, DECIMAL(2.0) * camera.p1 * x,
        DECIMAL(2.0) * camera.p2 * y, DECIMAL(4.0) * camera.p1 * y + DECIMAL(2.0) * camera.p2 * x;
    jacobian += decentering;

    const found::Vec2 p{camera.p2, camera.p1};
    jacobian += p * dr2.transpose();

    return jacobian;
}

std::optional<found::Vec2> BrownUndistort(const found::Vec2 &distorted,
                                          const CameraParameters &camera,
                                          int maxIterations) {
    found::Vec2 ideal = distorted;
    for (int i = 0; i < maxIterations; ++i) {
        const found::Mat2 jacobian = BrownDistortJacobian(ideal, camera);
        // A non-positive determinant means the polynomial has folded back (or
        // is about to), where the inverse is not unique.
        if (jacobian.determinant() <= DECIMAL(0.0)) {
            return std::nullopt;
        }

        const found::Vec2 error =
            BrownDistort(ideal, camera.k1, camera.k2, camera.k3, camera.p1, camera.p2) - distorted;
        const found::Vec2 step = jacobian.inverse() * error;
        ideal -= step;

        if (step.norm() <= kUndistortStepTolerance * (DECIMAL(1.0) + ideal.norm())) {
            return ideal;
        }
    }
    return std::nullopt;
}

found::Vec2 PixelToImagePlane(const found::Vec2 &pixel, const CameraParameters &camera) {
    if (camera.focalLengthX == DECIMAL(0.0) || camera.focalLengthY == DECIMAL(0.0)) {
        throw std::invalid_argument("PixelToImagePlane: focal length must be nonzero");
    }
    // Eq. 7, solved from the bottom row up.
    const decimal y = (pixel.y() - camera.principalY) / camera.focalLengthY;
    const decimal x = (pixel.x() - camera.principalX - camera.alpha * y) / camera.focalLengthX;
    return found::Vec2{x, y};
}

std::optional<found::Vec3> PixelToLineOfSight(const found::Vec2 &pixel, const CameraParameters &camera) {
    const std::optional<found::Vec2> ideal = PixelToUndistortedImagePlane(pixel, camera);
    if (!ideal.has_value()) {
        return std::nullopt;
    }
    // Eq. 1 with the image plane at z = 1.
    return found::Vec3(ideal->x(), ideal->y(), DECIMAL(1.0)).normalized();
}

std::optional<found::Vec2> UndistortPixel(const found::Vec2 &pixel, const CameraParameters &camera) {
    const std::optional<found::Vec2> ideal = PixelToUndistortedImagePlane(pixel, camera);
    if (!ideal.has_value()) {
        return std::nullopt;
    }
    // Eq. 7 applied to the undistorted point.
    return found::Vec2{camera.focalLengthX * ideal->x() + camera.alpha * ideal->y() + camera.principalX,
                       camera.focalLengthY * ideal->y() + camera.principalY};
}

std::vector<DistortionSample> ComputeDistortionMap(const CameraParameters &camera,
                                                   int width,
                                                   int height,
                                                   int spacing) {
    if (width < 1 || height < 1) {
        throw std::invalid_argument("ComputeDistortionMap: image dimensions must be at least 1 pixel");
    }
    if (spacing < 1) {
        throw std::invalid_argument("ComputeDistortionMap: grid spacing must be at least 1 pixel");
    }

    const std::vector<int> columns = GridCoordinates(width, spacing);
    const std::vector<int> rows = GridCoordinates(height, spacing);

    std::vector<DistortionSample> samples;
    samples.reserve(columns.size() * rows.size());
    for (const int row : rows) {
        for (const int column : columns) {
            DistortionSample sample{found::Vec2{DECIMAL(column), DECIMAL(row)}, std::nullopt};
            if (const std::optional<found::Vec2> undistorted = UndistortPixel(sample.pixel, camera)) {
                sample.correction = *undistorted - sample.pixel;
            }
            samples.push_back(sample);
        }
    }
    return samples;
}

DistortionSummary SummarizeDistortionMap(const std::vector<DistortionSample> &samples) {
    DistortionSummary summary;
    decimal sumOfSquares = DECIMAL(0.0);
    for (const DistortionSample &sample : samples) {
        if (!sample.correction.has_value()) {
            ++summary.invalidCount;
            continue;
        }
        ++summary.validCount;
        const decimal magnitude = sample.correction->norm();
        if (magnitude > summary.maxMagnitude) {
            summary.maxMagnitude = magnitude;
        }
        sumOfSquares += magnitude * magnitude;
    }
    if (summary.validCount > 0) {
        summary.rmsMagnitude = DECIMAL_SQRT(sumOfSquares / DECIMAL(summary.validCount));
    }
    return summary;
}

std::string FormatDistortionMapCsv(const std::vector<DistortionSample> &samples) {
    std::ostringstream csv;
    csv << std::setprecision(kCsvPrecision);
    csv << "u,v,du,dv\n";
    for (const DistortionSample &sample : samples) {
        csv << sample.pixel.x() << "," << sample.pixel.y() << ",";
        if (sample.correction.has_value()) {
            csv << sample.correction->x() << "," << sample.correction->y();
        } else {
            csv << ",";
        }
        csv << "\n";
    }
    return csv.str();
}

}  // namespace scope
