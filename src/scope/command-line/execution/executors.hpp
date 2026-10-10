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
 * Owns and runs the calibration pipeline
 * (noise filter -> star centroid -> optimization).
 */
class PrimaryScopePipelineExecutor : public found::PipelineExecutor {
 public:
    /**
     * @param options Parsed recalibration options. The executor takes ownership
     *                of the image buffers in darkFrames and starImages, which
     *                must be malloc'd.
     * @param noiseFilterAlgorithm Stage that reduces raw frames to one image.
     * @param starCentroidAlgorithm Stage that extracts star centroids.
     * @param optimizationAlgorithm Stage that fits camera parameters.
     */
    explicit PrimaryScopePipelineExecutor(RecalibrationOptions &&options,
                                          std::unique_ptr<NoiseFilterAlgorithm> noiseFilterAlgorithm,
                                          std::unique_ptr<StarCentroidAlgorithm> starCentroidAlgorithm,
                                          std::unique_ptr<OptimizationAlgorithm> optimizationAlgorithm);

    /// Frees the dark frames and star images held in the options.
    ~PrimaryScopePipelineExecutor() override;

    /// Runs the pipeline end to end.
    void ExecutePipeline() override;
    /**
     * Prints whether the fit converged, the ten camera parameters and the
     * residual RMS to standard output.
     *
     * @pre ExecutePipeline has run.
     */
    void OutputResults() override;

 private:
    /// The options for this run, including the input images.
    const RecalibrationOptions options_;
    /// The three-stage pipeline.
    PrimaryScopePipeline pipeline_;
    /// The noise-filter stage (not owned), kept so its malloc'd dark frame can
    /// be freed after the run.
    found::FunctionStage<Images, Image> *noiseStage_ = nullptr;
};

}  // namespace scope

#endif  // SRC_SCOPE_COMMAND_LINE_EXECUTION_EXECUTORS_HPP_
