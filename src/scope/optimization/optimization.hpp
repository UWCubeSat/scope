/**
 * @file optimization.hpp
 * @brief Parameter-fit optimization algorithms for SCOPE calibration.
 */

#ifndef SRC_SCOPE_OPTIMIZATION_OPTIMIZATION_HPP_
#define SRC_SCOPE_OPTIMIZATION_OPTIMIZATION_HPP_

#include "common/pipeline/stages.hpp"

#include "scope/common/style.hpp"
#include "scope/optimization/calibration-fit.hpp"
#include "scope/projection/projection.hpp"

namespace scope {

/**
 * Fits the camera intrinsics and distortion parameters to star centroids.
 */
class OptimizationAlgorithm : public found::FunctionStage<CentroidObservations, CalibrationResult> {};

/**
 * Levenberg-Marquardt parameter optimizer (Orion paper §"Estimation of Camera
 * Calibration Parameters"); a pipeline wrapper around FitCalibration.
 */
class LMAOptimizationAlgorithm : public OptimizationAlgorithm {
 public:
    /**
     * @param prior The prior camera parameters, where the fit starts.
     * @param settings Which parameters to estimate and how.
     */
    LMAOptimizationAlgorithm(const CameraParameters &prior, const FitSettings &settings)
        : prior_(prior), settings_(settings) {}

    /**
     * Fits camera parameters to the star centroids.
     *
     * @param observations The star-centroid stage's output.
     *
     * @return The fitted calibration (see FitCalibration).
     */
    CalibrationResult Run(const CentroidObservations &observations) override;

 private:
    /// The prior camera parameters.
    const CameraParameters prior_;
    /// Which parameters to estimate and how.
    const FitSettings settings_;
};

}  // namespace scope

#endif  // SRC_SCOPE_OPTIMIZATION_OPTIMIZATION_HPP_
