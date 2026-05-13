#include "scope/command-line/execution/executors.hpp"

#include <cstring>

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "scope/common/style.hpp"
#include "common/logging.hpp"
#include "common/time/time.hpp"

namespace scope {

PrimaryScopePipelineExecutor::PrimaryScopePipelineExecutor(
    RecalibrationOptions &&options, std::unique_ptr<NoiseFilterAlgorithm> noiseFilterAlgorithm,
    std::unique_ptr<StarCentroidAlgorithm> starCentroidAlgorithm,
    std::unique_ptr<OptimizationAlgorithm> optimizationAlgorithm) :
    options_(std::move(options)) {
    // TODO: change inputs + outputs of stages to actual values we will use
    std::unique_ptr<found::FunctionStage<Images, Image>> noiseFilterStage(std::move(noiseFilterAlgorithm));
    std::unique_ptr<found::FunctionStage<Image, std::vector<float>>> starCentroidStage(
        std::move(starCentroidAlgorithm));
    std::unique_ptr<found::FunctionStage<std::vector<float>, std::vector<float>>> optimizationStage(
        std::move(optimizationAlgorithm));
    this->pipeline_.AddStage(std::move(noiseFilterStage))
        .AddStage(std::move(starCentroidStage))
        .Complete(std::move(optimizationStage));
}

void PrimaryScopePipelineExecutor::ExecutePipeline() {
    // Results are stored within the pipeline, can also be accessed
    // via this function call
    this->pipeline_.Run(this->options_.images);
}

// Will output the values found by this recalibration
void PrimaryScopePipelineExecutor::OutputResults() {
    // std::vector<float> *&output = this->pipeline_.GetProduct();
    // TODO: something with output
    std::cout << "Nothing is implemented :(" << std::endl;
}

}  // namespace scope
