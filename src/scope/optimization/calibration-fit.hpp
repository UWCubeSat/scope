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
    /// Which camera parameters to estimate, in kCameraParameterFields order;
    /// the rest stay at the prior. The principal point is fixed by default,
    /// as the paper recommends.
    std::array<bool, kCameraParameterCount> estimate = {true, true, true, false, false, true, true, true, true, true};
    /// How the prior attitudes are corrected.
    AttitudeCorrectionModel attitudeModel = AttitudeCorrectionModel::kPerImage;
    /// The most Levenberg-Marquardt steps in one fit.
    int maxIterations = 100;
    /// Observations more than this many robust standard deviations from their
    /// prediction are rejected and the fit repeated. Zero disables rejection.
    decimal outlierThreshold = DECIMAL(4.0);
    /// The most fits; rejection stops sooner if the inlier set stops changing.
    int maxRejectionRounds = 10;
};

/**
 * Fits the camera parameters and attitude corrections to star centroids with
 * the paper's Levenberg-Marquardt method (Eqs. 20-37), refining the attitude
 * of image k as AttitudeCorrection(dtheta_k) * prior_k (Eq. 11). Outliers are
 * rejected by refitting without them, which the paper does not do.
 *
 * An image with fewer than three inliers (one under the shared model) is left
 * out, and gets the mean correction of the others applied to its prior.
 *
 * The result is not converged if the last fit hit maxIterations or stalled,
 * or if too few observations were left to fit; with none fitted at all, it
 * holds the prior.
 *
 * @param observations The star-centroid stage's output.
 * @param prior The prior camera parameters, where the fit starts.
 * @param settings Which parameters to estimate and how.
 *
 * @return The fitted calibration.
 *
 * @throws std::invalid_argument if an observation refers to an image with no
 *         attitude.
 */
CalibrationResult FitCalibration(const CentroidObservations &observations,
                                 const CameraParameters &prior,
                                 const FitSettings &settings);

}  // namespace scope

#endif  // SRC_SCOPE_OPTIMIZATION_CALIBRATION_FIT_HPP_
