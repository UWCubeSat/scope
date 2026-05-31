#include "scope/command-line/execution/executors.hpp"

#include <cstdlib>
#include <cstring>

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "scope/common/style.hpp"
#include "common/logging.hpp"
#include "common/time/time.hpp"

namespace scope {

PrimaryScopePipelineExecutor::PrimaryScopePipelineExecutor(RecalibrationOptions &&options,
                                                           std::unique_ptr<NoiseFilterAlgorithm> noiseFilterAlgorithm,
                                                           std::unique_ptr<StarCentroidAlgorithm> starCentroidAlgorithm,
                                                           std::unique_ptr<OptimizationAlgorithm> optimizationAlgorithm)
    : options_(std::move(options)) {
    std::unique_ptr<found::FunctionStage<Images, Image>> noiseFilterStage(std::move(noiseFilterAlgorithm));
    this->noiseStage_ = noiseFilterStage.get();
    std::unique_ptr<found::FunctionStage<Image, CentroidObservations>> starCentroidStage(
        std::move(starCentroidAlgorithm));
    std::unique_ptr<found::FunctionStage<CentroidObservations, std::vector<float>>> optimizationStage(
        std::move(optimizationAlgorithm));
    this->pipeline_.AddStage(std::move(noiseFilterStage))
        .AddStage(std::move(starCentroidStage))
        .Complete(std::move(optimizationStage));
}

void PrimaryScopePipelineExecutor::ExecutePipeline() {
    this->pipeline_.Run(this->options_.darkFrames);
    // The noise-filter stage's dark frame is a malloc'd buffer that no pipeline
    // stage owns; release it now that every downstream consumer has run.
    Image *darkFrame = this->noiseStage_->GetProduct();
    if (darkFrame != nullptr) {
        std::free(darkFrame->image);
    }
}

void PrimaryScopePipelineExecutor::OutputResults() {
    // std::vector<float> *&output = this->pipeline_.GetProduct();
    // TODO: something with output
    std::cout << "Nothing is implemented :(" << std::endl;
}

}  // namespace scope
