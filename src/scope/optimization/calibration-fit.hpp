/**
 * @file calibration-fit.hpp
 * @brief The Levenberg-Marquardt calibration fit (Orion paper, Christian et
 * al. 2016, §"Estimation of Camera Calibration Parameters").
 */

#ifndef SRC_SCOPE_OPTIMIZATION_CALIBRATION_FIT_HPP_
#define SRC_SCOPE_OPTIMIZATION_CALIBRATION_FIT_HPP_

#include <array>

#include "common/decimal.hpp"

#include "scope/common/types.hpp"
#include "scope/projection/projection.hpp"

namespace scope {

/// How the fit corrects the prior attitudes.
enum class AttitudeCorrectionModel {
    /// One correction per image, 10 + 3m states (paper Eq. 16, "Method 1").
    kPerImage,
    /// One correction shared by every image, 13 states (Eq. 17, "Method 2").
    kShared,
};

/// Settings for FitCalibration.
struct FitSettings {
    /// Which camera parameters to estimate, in kCameraParameterFields order.
    /// The others stay at their prior values. The default holds the principal
    /// point fixed, as the paper recommends (§"Observability Considerations").
    std::array<bool, kCameraParameterCount> estimate = {true, true, true, false, false, true, true, true, true, true};
    /// How the prior attitudes are corrected.
    AttitudeCorrectionModel attitudeModel = AttitudeCorrectionModel::kPerImage;
    /// The most Levenberg-Marquardt steps in one fit.
    int maxIterations = 100;
    /// Observations whose residual is more than this many robust standard
    /// deviations from their prediction are rejected and the fit is repeated.
    /// Zero disables rejection.
    decimal outlierThreshold = DECIMAL(4.0);
    /// The most fits in a row; rejection stops when the inlier set stops
    /// changing or this many fits have run.
    int maxRejectionRounds = 10;
};

/**
 * Fits the camera parameters and attitude corrections to star centroids.
 *
 * Minimizes the sum of squared differences between the measured centroids and
 * ProjectStarToPixel's predictions (paper Eq. 20) with the paper's
 * Levenberg-Marquardt variant (Eq. 26 and its steps 1-7) and analytic
 * Jacobians (Eqs. 27-37). The attitude of image k is refined as
 * AttitudeCorrection(dtheta_k) * prior_k (Eq. 11).
 *
 * Two additions to the paper, both from the project's plan: outliers are
 * rejected by repeating the fit without them, and each parameter's
 * uncertainty is estimated from the covariance of the fit.
 *
 * An image is left out of the fit if fewer than three of its observations
 * are inliers (fewer than one under the shared model). Its attitude is then
 * the average correction of the images in the fit applied to its prior.
 *
 * If no image is left, or there are fewer residuals than states, before any
 * fit has run, the result holds the prior and is not converged. If outlier
 * rejection leaves too few later on, the result is the last fit that ran,
 * marked not converged. Otherwise converged means the last fit stopped on its
 * convergence test, not on maxIterations or a stall.
 *
 * @param observations The star-centroid stage's output.
 * @param prior The prior camera parameters, where the fit starts.
 * @param settings Which parameters to estimate and how.
 *
 * @return The fitted calibration.
 *
 * @throws std::invalid_argument if an observation refers to an image with no
 *         attitude, or the settings are out of range (maxIterations or
 *         maxRejectionRounds below 1, outlierThreshold negative).
 */
CalibrationResult FitCalibration(const CentroidObservations &observations,
                                 const CameraParameters &prior,
                                 const FitSettings &settings);

}  // namespace scope

#endif  // SRC_SCOPE_OPTIMIZATION_CALIBRATION_FIT_HPP_
