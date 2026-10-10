/**
 * @file optimization.hpp
 * @brief Parameter-fit optimization algorithms for SCOPE calibration.
 */

#ifndef SRC_SCOPE_OPTIMIZATION_OPTIMIZATION_HPP_
#define SRC_SCOPE_OPTIMIZATION_OPTIMIZATION_HPP_

#include "common/pipeline/stages.hpp"

#include "scope/command-line/parsing/options.hpp"
#include "scope/common/style.hpp"

namespace scope {

/**
 * Fits the camera intrinsics and distortion parameters to star centroids.
 */
class OptimizationAlgorithm : public found::FunctionStage<CentroidObservations, CalibrationResult> {};

/**
 * Levenberg-Marquardt parameter optimizer. Stub pending the real algorithm.
 */
class LMAOptimizationAlgorithm : public OptimizationAlgorithm {
 public:
    /**
     * @param options Parsed recalibration options (currently unused).
     */
    explicit LMAOptimizationAlgorithm([[maybe_unused]] const RecalibrationOptions &options) {}

    /**
     * Fits camera parameters to the star centroids.
     *
     * @param observations The star-centroid stage's output.
     *
     * @return The fitted calibration; from the stub, an empty, unconverged result.
     */
    CalibrationResult Run(const CentroidObservations &observations) override;
};

}  // namespace scope

#endif  // SRC_SCOPE_OPTIMIZATION_OPTIMIZATION_HPP_
