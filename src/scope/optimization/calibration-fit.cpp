#include "scope/optimization/calibration-fit.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <array>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "common/decimal.hpp"
#include "common/logging.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/attitude/attitude.hpp"
#include "scope/optimization/measurement-model.hpp"
#include "scope/projection/projection.hpp"

namespace scope {

namespace {

/// Inliers an image needs to pin its three attitude angles.
constexpr int kMinObservationsPerImage = 3;
/// Starting damping. The scaled normal matrix has a unit diagonal, so this is
/// 0.001 of it, as in the paper.
constexpr double kInitialDamping = 1e-3;
/// How much the damping grows after a rejected step and shrinks after an
/// accepted one.
constexpr double kDampingFactor = 10.0;
/// Floor on the damping, so a bad step late in the fit takes only a few tries
/// to back off from.
constexpr double kMinDamping = 1e-12;
/// Past this with no step accepted, the fit has stalled.
constexpr double kMaxDamping = 1e20;
/// Convergence limits (see RunLevenbergMarquardt): RMS step in pixels, and
/// fraction of the cost (MINPACK's default ftol).
constexpr double kStepTolerance = 1e-8;
const double kCostTolerance = std::sqrt(static_cast<double>(std::numeric_limits<decimal>::epsilon()));
/// sqrt(2 ln 2): the median of a Rayleigh distribution, i.e. of the 2-D
/// residual distance under Gaussian noise, in units of the per-axis sigma.
constexpr double kRayleighMedian = 1.1774100225154747;
/// Residuals under this many pixels are never outliers, so noise-free data
/// isn't clipped at round-off.
constexpr double kMinOutlierDistance = 0.1;

/// The quantities the fit refines.
struct FitState {
    CameraParameters camera;
    std::vector<found::Quaternion> attitudes;
};

/// Which column of the normal equations each fitted value gets.
class StateLayout {
 public:
    StateLayout(const FitSettings &settings, const std::vector<bool> &imageInFit)
        : attitudeIndex_(imageInFit.size(), -1) {
        int next = 0;
        for (int p = 0; p < kCameraParameterCount; ++p) {
            cameraIndex_[p] = settings.estimate[p] ? next++ : -1;
        }

        const bool shared = settings.attitudeModel == AttitudeCorrectionModel::kShared;
        const int sharedIndex = next;
        for (std::size_t k = 0; k < imageInFit.size(); ++k) {
            if (!imageInFit[k]) {
                continue;
            }
            if (shared) {
                attitudeIndex_[k] = sharedIndex;
                next = sharedIndex + 3;
            } else {
                attitudeIndex_[k] = next;
                next += 3;
            }
        }
        size_ = next;
    }

    /// The number of states.
    int Size() const { return size_; }
    /// The state index of camera parameter p, or -1 if it is fixed.
    int CameraIndex(int p) const { return cameraIndex_[p]; }
    /// The index of the first of image k's three attitude states, or -1 if
    /// the image is not in the fit.
    int AttitudeIndex(std::size_t k) const { return attitudeIndex_[k]; }

 private:
    std::array<int, kCameraParameterCount> cameraIndex_;
    std::vector<int> attitudeIndex_;
    int size_ = 0;
};

/// Measured minus predicted pixel for each observation; NaN where the star is
/// at or behind the image plane.
std::vector<found::Vec2> ComputeResiduals(const std::vector<Observation> &observations, const FitState &state) {
    std::vector<found::Vec2> residuals;
    residuals.reserve(observations.size());
    for (const Observation &observation : observations) {
        const std::optional<found::Vec2> predicted =
            ProjectStarToPixel(observation.inertialDirection, state.attitudes[observation.imageIndex], state.camera);
        residuals.push_back(predicted.has_value() ? found::Vec2(observation.measuredPixel - *predicted)
                                                  : found::Vec2::Constant(std::numeric_limits<decimal>::quiet_NaN()));
    }
    return residuals;
}

/// Sum of squared residuals over the used observations; infinite if any of
/// them has no prediction.
double SumOfSquares(const std::vector<found::Vec2> &residuals, const std::vector<bool> &used) {
    double sum = 0.0;
    for (std::size_t i = 0; i < residuals.size(); ++i) {
        if (used[i]) {
            sum += static_cast<double>(residuals[i].squaredNorm());
        }
    }
    return std::isfinite(sum) ? sum : std::numeric_limits<double>::infinity();
}

/// Builds J^T J and J^T r over the used observations. These stay in double
/// even in a float build, because their columns differ in scale by orders of
/// magnitude (k3 multiplies r^6, focal length multiplies 1).
void BuildNormalEquations(const std::vector<Observation> &observations,
                          const std::vector<bool> &used,
                          const std::vector<found::Vec2> &residuals,
                          const FitState &state,
                          const StateLayout &layout,
                          Eigen::MatrixXd *normal,
                          Eigen::VectorXd *gradient) {
    normal->setZero(layout.Size(), layout.Size());
    gradient->setZero(layout.Size());

    // Each observation only touches the camera columns and its own image's
    // three attitude columns, so gather those and accumulate by hand. This
    // loop is most of the fit's runtime.
    constexpr int kMaxColumns = kCameraParameterCount + 3;
    std::array<int, kMaxColumns> columns;
    std::array<std::array<double, 2>, kMaxColumns> derivatives;

    for (std::size_t i = 0; i < observations.size(); ++i) {
        if (!used[i]) {
            continue;
        }
        const Observation &observation = observations[i];
        const found::Vec3 eC = state.attitudes[observation.imageIndex] * observation.inertialDirection;
        const MeasurementJacobian jacobian = ComputeMeasurementJacobian(eC, state.camera);

        int count = 0;
        const auto gather = [&](int column, decimal du, decimal dv) {
            columns[count] = column;
            derivatives[count] = {static_cast<double>(du), static_cast<double>(dv)};
            ++count;
        };
        for (int p = 0; p < kCameraParameterCount; ++p) {
            if (layout.CameraIndex(p) >= 0) {
                gather(layout.CameraIndex(p), jacobian.camera(0, p), jacobian.camera(1, p));
            }
        }
        const int attitudeIndex = layout.AttitudeIndex(observation.imageIndex);
        for (int a = 0; a < 3; ++a) {
            gather(attitudeIndex + a, jacobian.attitude(0, a), jacobian.attitude(1, a));
        }

        // The columns increase, so b >= a fills the upper triangle.
        const double du = static_cast<double>(residuals[i].x());
        const double dv = static_cast<double>(residuals[i].y());
        for (int a = 0; a < count; ++a) {
            (*gradient)(columns[a]) += derivatives[a][0] * du + derivatives[a][1] * dv;
            for (int b = a; b < count; ++b) {
                (*normal)(columns[a], columns[b]) +=
                    derivatives[a][0] * derivatives[b][0] + derivatives[a][1] * derivatives[b][1];
            }
        }
    }
    normal->triangularView<Eigen::StrictlyLower>() = normal->transpose().eval();
}

/// Jacobi scaling: 1 / sqrt(N_ii), so that S N S has a unit diagonal. A state
/// no observation depends on gets 1, and its step comes out zero.
Eigen::VectorXd JacobiScale(const Eigen::MatrixXd &normal) {
    Eigen::VectorXd scale(normal.rows());
    for (Eigen::Index i = 0; i < normal.rows(); ++i) {
        scale(i) = normal(i, i) > 0.0 ? 1.0 / std::sqrt(normal(i, i)) : 1.0;
    }
    return scale;
}

/// Adds a step to the state. Attitude steps are small rotations applied on top
/// of the current attitudes.
FitState ApplyStep(const FitState &state, const Eigen::VectorXd &step, const StateLayout &layout) {
    FitState next = state;
    for (int p = 0; p < kCameraParameterCount; ++p) {
        if (layout.CameraIndex(p) >= 0) {
            next.camera.*kCameraParameterFields[p] += static_cast<decimal>(step(layout.CameraIndex(p)));
        }
    }
    for (std::size_t k = 0; k < state.attitudes.size(); ++k) {
        const int index = layout.AttitudeIndex(k);
        if (index >= 0) {
            const found::Vec3 dtheta = step.segment<3>(index).cast<decimal>();
            next.attitudes[k] = (AttitudeCorrection(dtheta) * state.attitudes[k]).normalized();
        }
    }
    return next;
}

std::size_t CountUsed(const std::vector<bool> &used) {
    return static_cast<std::size_t>(std::count(used.begin(), used.end(), true));
}

struct LevenbergMarquardtOutcome {
    FitState state;
    int iterations = 0;
    bool converged = false;
};

/// Levenberg-Marquardt over the used observations.
LevenbergMarquardtOutcome RunLevenbergMarquardt(const std::vector<Observation> &observations,
                                                const std::vector<bool> &used,
                                                const StateLayout &layout,
                                                const FitState &start,
                                                int maxIterations) {
    LevenbergMarquardtOutcome outcome{start};
    std::vector<found::Vec2> residuals = ComputeResiduals(observations, outcome.state);
    double cost = SumOfSquares(residuals, used);
    const double stepLimit = kStepTolerance * std::sqrt(2.0 * static_cast<double>(CountUsed(used)));
    double damping = kInitialDamping;

    Eigen::MatrixXd normal;
    Eigen::VectorXd gradient;
    while (outcome.iterations < maxIterations) {
        BuildNormalEquations(observations, used, residuals, outcome.state, layout, &normal, &gradient);
        const Eigen::VectorXd scale = JacobiScale(normal);
        const Eigen::MatrixXd scaledNormal = scale.asDiagonal() * normal * scale.asDiagonal();
        const Eigen::VectorXd scaledGradient = scale.cwiseProduct(gradient);

        // Raise the damping until a step lowers the cost. The paper stops on a
        // residual threshold that real data never reaches, so instead we stop
        // when the first (least damped) step is negligible. Only the first one
        // is checked, since more damping always shrinks the step.
        bool firstStep = true;
        while (true) {
            Eigen::MatrixXd damped = scaledNormal;
            damped.diagonal().array() += damping;
            const Eigen::VectorXd scaledStep = damped.ldlt().solve(scaledGradient);

            if (firstStep) {
                // The scaled step is how far it moves the residuals, so
                // stepLimit is in pixels. predictedReduction is how much the
                // cost would drop if the model were linear.
                const double predictedReduction = scaledStep.dot(2.0 * scaledGradient - scaledNormal * scaledStep);
                if (scaledStep.lpNorm<Eigen::Infinity>() < stepLimit ||
                    (std::isfinite(cost) && predictedReduction <= kCostTolerance * cost)) {
                    outcome.converged = true;
                    return outcome;
                }
                firstStep = false;
            }

            const FitState trial = ApplyStep(outcome.state, scale.cwiseProduct(scaledStep), layout);
            std::vector<found::Vec2> trialResiduals = ComputeResiduals(observations, trial);
            const double trialCost = SumOfSquares(trialResiduals, used);
            if (trialCost < cost) {
                outcome.state = trial;
                residuals = std::move(trialResiduals);
                cost = trialCost;
                damping = std::max(damping / kDampingFactor, kMinDamping);
                ++outcome.iterations;
                break;
            }

            damping *= kDampingFactor;
            if (damping > kMaxDamping) {
                return outcome;
            }
        }
    }
    return outcome;
}

/// One-sigma uncertainty of each camera parameter, from the inverse normal
/// matrix scaled by the residual variance.
CameraParameters ParameterUncertainty(const std::vector<Observation> &observations,
                                      const std::vector<bool> &used,
                                      const FitState &state,
                                      const StateLayout &layout) {
    CameraParameters uncertainty;
    const decimal undetermined = std::numeric_limits<decimal>::quiet_NaN();

    const std::vector<found::Vec2> residuals = ComputeResiduals(observations, state);
    Eigen::MatrixXd normal;
    Eigen::VectorXd gradient;
    BuildNormalEquations(observations, used, residuals, state, layout, &normal, &gradient);

    const double degreesOfFreedom = 2.0 * static_cast<double>(CountUsed(used)) - layout.Size();
    const Eigen::VectorXd scale = JacobiScale(normal);
    const Eigen::LDLT<Eigen::MatrixXd> factor(scale.asDiagonal() * normal * scale.asDiagonal());
    // Singular if some parameter isn't constrained by any observation.
    const bool invertible = factor.info() == Eigen::Success && factor.isPositive() &&
                            factor.rcond() > std::numeric_limits<double>::epsilon();
    const double variance = SumOfSquares(residuals, used) / degreesOfFreedom;

    for (int p = 0; p < kCameraParameterCount; ++p) {
        const int index = layout.CameraIndex(p);
        if (index < 0) {
            continue;
        }
        if (!invertible || degreesOfFreedom <= 0.0) {
            uncertainty.*kCameraParameterFields[p] = undetermined;
            continue;
        }
        const Eigen::VectorXd column = factor.solve(Eigen::VectorXd::Unit(layout.Size(), index));
        const double sigma = scale(index) * std::sqrt(variance * column(index));
        uncertainty.*kCameraParameterFields[p] = static_cast<decimal>(sigma);
    }
    return uncertainty;
}

/// The robust per-axis standard deviation of the used residuals, from their
/// median distance.
double RobustSigma(const std::vector<found::Vec2> &residuals, const std::vector<bool> &used) {
    std::vector<double> distances;
    for (std::size_t i = 0; i < residuals.size(); ++i) {
        if (used[i]) {
            distances.push_back(static_cast<double>(residuals[i].norm()));
        }
    }
    const auto middle = distances.begin() + static_cast<std::ptrdiff_t>(distances.size() / 2);
    std::nth_element(distances.begin(), middle, distances.end());
    return *middle / kRayleighMedian;
}

/// Flags the observations within threshold robust sigmas of their prediction.
/// Already-rejected ones are checked again, so one thrown out early can come
/// back.
std::vector<bool> FindInliers(const std::vector<found::Vec2> &residuals,
                              const std::vector<bool> &used,
                              double threshold) {
    const double limit = std::max(threshold * RobustSigma(residuals, used), kMinOutlierDistance);
    std::vector<bool> inliers(residuals.size());
    for (std::size_t i = 0; i < residuals.size(); ++i) {
        inliers[i] = static_cast<double>(residuals[i].norm()) <= limit;
    }
    return inliers;
}

/// RMS of the used residuals' u and v components pooled together.
double Rms(const std::vector<found::Vec2> &residuals, const std::vector<bool> &used) {
    return std::sqrt(SumOfSquares(residuals, used) / (2.0 * static_cast<double>(CountUsed(used))));
}

/// Mean of the per-image corrections refined * prior^-1 over the images in
/// the fit.
found::Quaternion MeanCorrection(const FitState &state,
                                 const std::vector<found::Quaternion> &priors,
                                 const StateLayout &layout) {
    std::vector<found::Quaternion> corrections;
    for (std::size_t k = 0; k < priors.size(); ++k) {
        if (layout.AttitudeIndex(k) >= 0) {
            corrections.push_back(state.attitudes[k] * priors[k].conjugate());
        }
    }
    return AverageQuaternions(corrections);
}

}  // namespace

CalibrationResult FitCalibration(const CentroidObservations &observations,
                                 const CameraParameters &prior,
                                 const FitSettings &settings) {
    const std::vector<Observation> &measured = observations.observations;
    const std::vector<found::Quaternion> &priorAttitudes = observations.attitudes;
    for (const Observation &observation : measured) {
        if (observation.imageIndex < 0 || static_cast<std::size_t>(observation.imageIndex) >= priorAttitudes.size()) {
            throw std::invalid_argument("FitCalibration: an observation refers to an image with no attitude");
        }
    }

    const int minObservations =
        settings.attitudeModel == AttitudeCorrectionModel::kPerImage ? kMinObservationsPerImage : 1;

    FitState state{prior, priorAttitudes};
    std::vector<bool> inliers(measured.size(), true);
    // What the last completed round used. Images left out of it get its
    // average correction.
    std::vector<bool> used(measured.size(), false);
    std::optional<StateLayout> layout;
    found::Quaternion meanCorrection = found::Quaternion::Identity();

    CalibrationResult result;
    result.estimated = settings.estimate;
    int round = 0;
    for (; round < settings.maxRejectionRounds; ++round) {
        std::vector<int> inlierCount(priorAttitudes.size(), 0);
        for (std::size_t i = 0; i < measured.size(); ++i) {
            inlierCount[measured[i].imageIndex] += inliers[i];
        }
        std::vector<bool> imageInFit(priorAttitudes.size());
        for (std::size_t k = 0; k < priorAttitudes.size(); ++k) {
            imageInFit[k] = inlierCount[k] >= minObservations;
        }
        std::vector<bool> roundUsed(measured.size());
        for (std::size_t i = 0; i < measured.size(); ++i) {
            roundUsed[i] = inliers[i] && imageInFit[measured[i].imageIndex];
        }

        const StateLayout roundLayout(settings, imageInFit);
        const std::size_t usedCount = CountUsed(roundUsed);
        const bool anyImage = std::find(imageInFit.begin(), imageInFit.end(), true) != imageInFit.end();
        if (!anyImage || 2 * usedCount < static_cast<std::size_t>(roundLayout.Size())) {
            LOG_WARN("Optimization: too few observations to fit (" << usedCount << " usable for " << roundLayout.Size()
                                                                   << " states)");
            result.converged = false;
            break;
        }

        const LevenbergMarquardtOutcome outcome =
            RunLevenbergMarquardt(measured, roundUsed, roundLayout, state, settings.maxIterations);
        state = outcome.state;
        meanCorrection = MeanCorrection(state, priorAttitudes, roundLayout);
        for (std::size_t k = 0; k < priorAttitudes.size(); ++k) {
            if (!imageInFit[k]) {
                state.attitudes[k] = meanCorrection * priorAttitudes[k];
            }
        }
        used = roundUsed;
        layout = roundLayout;
        result.iterations += outcome.iterations;
        result.converged = outcome.converged;

        const std::vector<found::Vec2> residuals = ComputeResiduals(measured, state);
        LOG_INFO("Optimization round " << round << ": " << usedCount << " observations, " << roundLayout.Size()
                                       << " states, " << outcome.iterations << " steps, residual RMS "
                                       << Rms(residuals, used) << " px"
                                       << (outcome.converged ? "" : " (not converged)"));
        if (settings.outlierThreshold == DECIMAL(0.0)) {
            break;
        }

        std::vector<bool> nextInliers = FindInliers(residuals, used, static_cast<double>(settings.outlierThreshold));
        if (nextInliers == inliers) {
            break;
        }
        inliers = std::move(nextInliers);
    }
    if (round == settings.maxRejectionRounds) {
        LOG_WARN("Optimization: the inlier set was still changing after " << settings.maxRejectionRounds << " rounds");
    }

    result.camera = state.camera;
    result.attitudes = state.attitudes;
    result.residuals = ComputeResiduals(measured, state);
    result.inliers = used;
    if (!layout.has_value()) {
        for (int p = 0; p < kCameraParameterCount; ++p) {
            if (settings.estimate[p]) {
                result.uncertainty.*kCameraParameterFields[p] = std::numeric_limits<decimal>::quiet_NaN();
            }
        }
        return result;
    }

    result.residualRms = static_cast<decimal>(Rms(result.residuals, used));
    result.uncertainty = ParameterUncertainty(measured, used, state, *layout);
    result.alignmentCorrection = meanCorrection;
    return result;
}

}  // namespace scope
