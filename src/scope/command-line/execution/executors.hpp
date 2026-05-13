/**
 * @file executors.hpp
 * @brief Pipeline executor for the primary SCOPE calibration chain.
 */

#ifndef SRC_SCOPE_COMMAND_LINE_EXECUTION_EXECUTORS_HPP_
#define SRC_SCOPE_COMMAND_LINE_EXECUTION_EXECUTORS_HPP_

#include <memory>

#include "scope/command-line/parsing/options.hpp"
#include "scope/common/style.hpp"
#include "scope/noise-filter/noise-filter.hpp"
#include "scope/optimization/optimization.hpp"
#include "scope/star-centroid/star-centroid.hpp"

#include "command-line/execution/executors.hpp"

namespace scope {

/**
 * Owns and runs the primary SCOPE calibration pipeline
 * (noise filter -> star centroid -> optimization).
 */
class PrimaryScopePipelineExecutor : public found::PipelineExecutor {
 public:
    /**
     * Constructs a PrimaryScopePipelineExecutor and assembles its pipeline.
     *
     * @param options Parsed recalibration options consumed by the run.
     * @param noiseFilterAlgorithm Stage that reduces raw frames to one image.
     * @param starCentroidAlgorithm Stage that extracts star centroids.
     * @param optimizationAlgorithm Stage that fits camera parameters.
     */
    explicit PrimaryScopePipelineExecutor(RecalibrationOptions &&options,
                                          std::unique_ptr<NoiseFilterAlgorithm> noiseFilterAlgorithm,
                                          std::unique_ptr<StarCentroidAlgorithm> starCentroidAlgorithm,
                                          std::unique_ptr<OptimizationAlgorithm> optimizationAlgorithm);

    /// Runs the assembled pipeline end-to-end.
    void ExecutePipeline() override;
    /// Prints the calibrated parameters produced by the pipeline.
    void OutputResults() override;

 private:
    /// The options driving this pipeline run.
    const RecalibrationOptions options_;
    /// The pipeline assembled for SCOPE operation.
    PrimaryScopePipeline pipeline_;
};

}  // namespace scope

#endif  // SRC_SCOPE_COMMAND_LINE_EXECUTION_EXECUTORS_HPP_
