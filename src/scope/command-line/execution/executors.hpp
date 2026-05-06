#ifndef SRC_SCOPE_COMMAND_LINE_EXECUTION_EXECUTORS_HPP_
#define SRC_SCOPE_COMMAND_LINE_EXECUTION_EXECUTORS_HPP_

#include <memory>

#include "scope/command-line/parsing/options.hpp"
#include "scope/common/style.hpp"
#include "scope/algorithms-placeholder/algorithms-placeholder.hpp"

#include "command-line/execution/executors.hpp"

namespace scope {

class PrimaryScopePipelineExecutor : public found::PipelineExecutor {
    explicit PrimaryScopePipelineExecutor(RecalibrationOptions &&options,
                                          std::unique_ptr<NoiseFilterAlgorithm> noiseFilterAlgorithm,
                                          std::unique_ptr<StarCentroidAlgorithm> starCentroidAlgorithm,
                                          std::unique_ptr<OptimizationAlgorithm> optimizationAlgorithm);

    void ExecutePipeline() override;
    void OutputResults() override;

  private:
    // The options being used by the pipeline
    const RecalibrationOptions options_;
    // The primary pipeline for SCOPE operation
    PrimaryScopePipeline pipeline_;
};

}  // namespace scope

#endif  // SRC_SCOPE_COMMAND_LINE_EXECUTION_EXECUTORS_HPP_
