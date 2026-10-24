#include "scope/evaluation/evaluation.hpp"

#include <Eigen/LU>

#include <cstddef>

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/common/types.hpp"
#include "scope/projection/projection.hpp"

namespace scope {

namespace {

/// Step for the central-difference attitude derivatives, in radians (about
/// 0.2 arcseconds).
constexpr decimal kAttitudeStep = DECIMAL(1e-6);
/// The refinement has converged once a correction is smaller than this, in
/// radians.
constexpr decimal kAttitudeTolerance = DECIMAL(1e-10);
/// Pivot threshold, relative to the largest pivot, below which the normal
/// matrix is treated as singular.
constexpr decimal kSingularThreshold = DECIMAL(1e-9);

/// One image's refined attitude and its residuals there.
struct RefinedImage {
    /// The refined attitude.
    found::Quaternion attitude;
    /// Measured minus predicted pixel for each of the image's stars.
    std::vector<found::Vec2> residuals;
};

/// The attitude correction of the paper's Eq. 11, exp(-[dtheta x]): to first
/// order it moves a camera-frame direction e_C to e_C + e_C x dtheta. dtheta
/// must be nonzero.
found::Quaternion AttitudeCorrectionFor(const found::Vec3 &dtheta) {
    const decimal angle = dtheta.norm();
    return found::Quaternion(found::AngleAxis(-angle, dtheta / angle));
}

/// Refines one image's attitude by Gauss-Newton with the camera fixed. The
/// derivatives with respect to the attitude correction are central
/// differences of ProjectStarToPixel. Returns std::nullopt if a star projects
/// to no pixel, the stars do not fix the attitude, or the refinement does not
/// converge.
std::optional<RefinedImage> RefineAttitude(const std::vector<const Observation *> &stars,
                                           const found::Quaternion &prior,
                                           const CameraParameters &camera,
                                           int maxIterations) {
    found::Quaternion attitude = prior;
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        found::Mat3 normal = found::Mat3::Zero();
        found::Vec3 gradient = found::Vec3::Zero();
        std::vector<found::Vec2> residuals;
        residuals.reserve(stars.size());

        for (const Observation *star : stars) {
            const std::optional<found::Vec2> predicted =
                ProjectStarToPixel(star->inertialDirection, attitude, camera);

            Eigen::Matrix<decimal, 2, 3> jacobian;
            for (int axis = 0; axis < 3; ++axis) {
                found::Vec3 dtheta = found::Vec3::Zero();
                dtheta(axis) = kAttitudeStep;
                const std::optional<found::Vec2> plus =
                    ProjectStarToPixel(star->inertialDirection, AttitudeCorrectionFor(dtheta) * attitude, camera);
                const std::optional<found::Vec2> minus =
                    ProjectStarToPixel(star->inertialDirection, AttitudeCorrectionFor(-dtheta) * attitude, camera);
                if (!predicted.has_value() || !plus.has_value() || !minus.has_value()) {
                    return std::nullopt;
                }
                jacobian.col(axis) = (*plus - *minus) / (DECIMAL(2.0) * kAttitudeStep);
            }

            const found::Vec2 residual = star->measuredPixel - *predicted;
            normal += jacobian.transpose() * jacobian;
            gradient += jacobian.transpose() * residual;
            residuals.push_back(residual);
        }

        Eigen::FullPivLU<found::Mat3> solver(normal);
        solver.setThreshold(kSingularThreshold);
        if (!solver.isInvertible()) {
            return std::nullopt;
        }
        const found::Vec3 step = solver.solve(gradient);

        // The residuals are those at the current attitude, so once the step
        // is negligible the current attitude is the answer.
        if (step.norm() < kAttitudeTolerance) {
            return RefinedImage{attitude, residuals};
        }
        attitude = (AttitudeCorrectionFor(step) * attitude).normalized();
    }
    return std::nullopt;
}

}  // namespace

ResidualStatistics ComputeResidualStatistics(const std::vector<found::Vec2> &residuals) {
    ResidualStatistics statistics;
    statistics.count = residuals.size();
    if (residuals.empty()) {
        return statistics;
    }

    const decimal componentCount = DECIMAL(2.0) * DECIMAL(residuals.size());
    decimal sum = DECIMAL(0.0);
    for (const found::Vec2 &residual : residuals) {
        sum += residual.x() + residual.y();
    }
    statistics.mean = sum / componentCount;

    // Central moments, both components pooled.
    decimal m2 = DECIMAL(0.0);
    decimal m3 = DECIMAL(0.0);
    decimal m4 = DECIMAL(0.0);
    for (const found::Vec2 &residual : residuals) {
        for (const decimal component : {residual.x(), residual.y()}) {
            const decimal d = component - statistics.mean;
            const decimal d2 = d * d;
            m2 += d2;
            m3 += d2 * d;
            m4 += d2 * d2;
        }
    }
    m2 /= componentCount;
    m3 /= componentCount;
    m4 /= componentCount;

    statistics.standardDeviation = DECIMAL_SQRT(m2);
    if (m2 > DECIMAL(0.0)) {
        statistics.skewness = m3 / (m2 * statistics.standardDeviation);
        statistics.kurtosis = m4 / (m2 * m2);
    }
    return statistics;
}

HoldoutEvaluation EvaluateHoldout(const CentroidObservations &observations,
                                  const CameraParameters &camera,
                                  int maxIterations) {
    const std::vector<Observation> &all = observations.observations;
    const std::size_t imageCount = observations.attitudes.size();

    std::vector<std::vector<std::size_t>> starsByImage(imageCount);
    for (std::size_t i = 0; i < all.size(); ++i) {
        const int image = all[i].imageIndex;
        if (image < 0 || static_cast<std::size_t>(image) >= imageCount) {
            throw std::invalid_argument("EvaluateHoldout: observation " + std::to_string(i) + " refers to image " +
                                        std::to_string(image) + ", which has no attitude");
        }
        starsByImage[static_cast<std::size_t>(image)].push_back(i);
    }

    HoldoutEvaluation evaluation;
    evaluation.residuals.assign(all.size(), std::nullopt);
    evaluation.attitudes.assign(imageCount, std::nullopt);
    std::vector<found::Vec2> evaluated;

    for (std::size_t image = 0; image < imageCount; ++image) {
        const std::vector<std::size_t> &indices = starsByImage[image];
        if (indices.size() < kMinStarsPerHoldoutImage) {
            continue;
        }

        std::vector<const Observation *> stars;
        stars.reserve(indices.size());
        for (const std::size_t i : indices) {
            stars.push_back(&all[i]);
        }

        const std::optional<RefinedImage> refined =
            RefineAttitude(stars, observations.attitudes[image], camera, maxIterations);
        if (!refined.has_value()) {
            continue;
        }

        evaluation.attitudes[image] = refined->attitude;
        for (std::size_t j = 0; j < indices.size(); ++j) {
            evaluation.residuals[indices[j]] = refined->residuals[j];
            evaluated.push_back(refined->residuals[j]);
        }
    }

    evaluation.statistics = ComputeResidualStatistics(evaluated);
    return evaluation;
}

}  // namespace scope
