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

// The normal equations are formed and solved in double whatever decimal is:
// their columns differ in scale by many orders of magnitude (r^6 against 1
// for k3 against focal length), which single precision cannot hold.

namespace scope {

namespace {

/// Inliers an image needs to pin its three attitude angles.
constexpr int kMinObservationsPerImage = 3;
/// Paper step 4. With the Jacobi scaling used here diag(N) is the identity,
/// so step 4's 0.001 * mean(diag N) and Eq. 26's lambda * diag(N) agree.
constexpr double kInitialDamping = 1e-3;
/// Paper step 6.
constexpr double kDampingFactor = 10.0;
/// Keeps a poor Gauss-Newton step late in the fit only a few tries from being
/// backed off.
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
/// No residual under this is an outlier, so noise-free data is not clipped at
/// round-off. In pixels.
constexpr double kMinOutlierDistance = 0.1;

/// The quantities the fit refines.
struct FitState {
    CameraParameters camera;
    std::vector<found::Quaternion> attitudes;
};

/// Where each estimated quantity sits in the state vector beta (paper Eq. 16
/// or Eq. 17).
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

/// The cost J(beta) of paper Eq. 20 over the used observations; infinite if
/// any of them has no prediction.
double SumOfSquares(const std::vector<found::Vec2> &residuals, const std::vector<bool> &used) {
    double sum = 0.0;
    for (std::size_t i = 0; i < residuals.size(); ++i) {
        if (used[i]) {
            sum += static_cast<double>(residuals[i].squaredNorm());
        }
    }
    return std::isfinite(sum) ? sum : std::numeric_limits<double>::infinity();
}

/// Accumulates the normal matrix N = J^T J (Eq. 23) and J^T (y - h) (the
/// right-hand side of Eq. 26) over the used observations, one observation's
/// rows at a time.
void BuildNormalEquations(const std::vector<Observation> &observations,
                          const std::vector<bool> &used,
                          const std::vector<found::Vec2> &residuals,
                          const FitState &state,
                          const StateLayout &layout,
                          Eigen::MatrixXd *normal,
                          Eigen::VectorXd *gradient) {
    normal->setZero(layout.Size(), layout.Size());
    gradient->setZero(layout.Size());

    // An observation touches at most every camera parameter and its own
    // image's three attitude states (the block structure of Eqs. 27-28), so
    // its two rows of J are gathered into those columns first. The products
    // are written out by hand because this loop is most of the fit's time.
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

/// gamma = beta + delta (paper step 6), with each attitude correction applied
/// multiplicatively (Eq. 11).
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

/// The paper's Levenberg-Marquardt loop (steps 3-7) over the used observations.
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

        // Steps 5 and 6: raise the damping until a step reduces the cost. The
        // paper stops on a residual tolerance (step 7) that real data never
        // reaches, so instead the fit has converged when the first, least
        // damped step is negligible. Later steps are not tested, since more
        // damping shrinks a step whether or not the fit is at a minimum.
        bool firstStep = true;
        while (true) {
            Eigen::MatrixXd damped = scaledNormal;
            damped.diagonal().array() += damping;
            const Eigen::VectorXd scaledStep = damped.ldlt().solve(scaledGradient);

            if (firstStep) {
                // A scaled step ||J_i|| delta_i is its change to the residuals,
                // so stepLimit is in pixels. predictedReduction is
                // ||r||^2 - ||r - J delta||^2 under the linearized model.
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

/// One standard deviation of each camera parameter from the covariance
/// sigma^2 N^-1 at the solution, with sigma^2 the residual variance.
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
    // A singular N (a parameter no observation constrains) has no inverse.
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

/// The observations within threshold robust standard deviations of their
/// prediction. Every observation is judged, used or not, so one rejected under
/// a poor early fit can come back.
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
    // The observations and layout of the last fit that ran, and the average
    // correction of its images, which images left out of it get instead.
    std::vector<bool> used(measured.size(), false);
    std::optional<StateLayout> layout;
    found::Quaternion meanCorrection = found::Quaternion::Identity();
    bool settled = false;

    CalibrationResult result;
    result.estimated = settings.estimate;
    for (int round = 0; round < settings.maxRejectionRounds; ++round) {
        std::vector<int> inlierCount(priorAttitudes.size(), 0);
        for (std::size_t i = 0; i < measured.size(); ++i) {
            inlierCount[measured[i].imageIndex] += inliers[i] ? 1 : 0;
        }
        std::vector<bool> imageInFit(priorAttitudes.size());
        FitState start = state;
        for (std::size_t k = 0; k < priorAttitudes.size(); ++k) {
            imageInFit[k] = inlierCount[k] >= minObservations;
            if (!imageInFit[k]) {
                start.attitudes[k] = meanCorrection * priorAttitudes[k];
            }
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
            settled = true;
            break;
        }

        const LevenbergMarquardtOutcome outcome =
            RunLevenbergMarquardt(measured, roundUsed, roundLayout, start, settings.maxIterations);
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
            settled = true;
            break;
        }

        std::vector<bool> nextInliers = FindInliers(residuals, used, static_cast<double>(settings.outlierThreshold));
        if (nextInliers == inliers) {
            settled = true;
            break;
        }
        inliers = std::move(nextInliers);
    }
    if (!settled) {
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
