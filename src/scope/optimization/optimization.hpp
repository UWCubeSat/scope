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
 * Produces updated camera intrinsic + distortion parameters from star centroids.
 */
class OptimizationAlgorithm : public found::FunctionStage<CentroidObservations, CalibrationResult> {
 public:
    OptimizationAlgorithm() = default;
    virtual ~OptimizationAlgorithm() {}
};

/**
 * Levenberg-Marquardt parameter optimizer. Stub pending the real algorithm.
 */
class LMAOptimizationAlgorithm : public OptimizationAlgorithm {
 public:
    LMAOptimizationAlgorithm() = default;

    /**
     * Constructs a new LMAOptimizationAlgorithm.
     *
     * @param options Parsed recalibration options (currently unused).
     */
    explicit LMAOptimizationAlgorithm([[maybe_unused]] const RecalibrationOptions &options) {}

    ~LMAOptimizationAlgorithm() override = default;

    /**
     * Fits camera parameters to the given star centroids.
     *
     * @param observations Star measurements + prior attitudes from the
     *                     star-centroid stage.
     *
     * @return The fitted calibration. The stub returns an empty, unconverged
     *         result.
     */
    CalibrationResult Run(const CentroidObservations &observations) override;
};

}  // namespace scope

#endif  // SRC_SCOPE_OPTIMIZATION_OPTIMIZATION_HPP_
