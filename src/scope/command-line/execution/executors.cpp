#include "scope/command-line/execution/executors.hpp"

#include <stb_image/stb_image.h>

#include <cstdlib>
#include <cstring>

#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <utility>

#include "scope/common/style.hpp"
#include "common/logging.hpp"
#include "common/time/time.hpp"

namespace scope {

namespace {

/// Significant digits printed for each result value: enough that a printed
/// calibration can be passed back in as the next run's prior without a loss
/// that matters at the sub-pixel level.
constexpr int kOutputPrecision = 10;

}  // namespace

PrimaryScopePipelineExecutor::PrimaryScopePipelineExecutor(RecalibrationOptions &&options,
                                                           std::unique_ptr<NoiseFilterAlgorithm> noiseFilterAlgorithm,
                                                           std::unique_ptr<StarCentroidAlgorithm> starCentroidAlgorithm,
                                                           std::unique_ptr<OptimizationAlgorithm> optimizationAlgorithm)
    : options_(std::move(options)) {
    std::unique_ptr<found::FunctionStage<Images, Image>> noiseFilterStage(std::move(noiseFilterAlgorithm));
    this->noiseStage_ = noiseFilterStage.get();
    std::unique_ptr<found::FunctionStage<Image, CentroidObservations>> starCentroidStage(
        std::move(starCentroidAlgorithm));
    std::unique_ptr<found::FunctionStage<CentroidObservations, CalibrationResult>> optimizationStage(
        std::move(optimizationAlgorithm));
    this->pipeline_.AddStage(std::move(noiseFilterStage))
        .AddStage(std::move(starCentroidStage))
        .Complete(std::move(optimizationStage));
}

PrimaryScopePipelineExecutor::~PrimaryScopePipelineExecutor() {
    for (const Image &image : this->options_.darkFrames) {
        stbi_image_free(image.image);
    }
    for (const Image &image : this->options_.starImages) {
        stbi_image_free(image.image);
    }
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
    const CalibrationResult &result = *this->pipeline_.GetProduct();
    const CameraParameters &camera = result.camera;

    // Each parameter is labelled with the command-line flag that takes it.
    std::ostringstream text;
    text << std::setprecision(kOutputPrecision);
    text << "Calibration result:\n";
    text << "    converged: " << (result.converged ? "yes" : "no") << "\n";
    text << "    focal-length-x: " << camera.focalLengthX << "\n";
    text << "    alpha: " << camera.alpha << "\n";
    text << "    focal-length-y: " << camera.focalLengthY << "\n";
    text << "    principal-point-x: " << camera.principalX << "\n";
    text << "    principal-point-y: " << camera.principalY << "\n";
    text << "    k1: " << camera.k1 << "\n";
    text << "    k2: " << camera.k2 << "\n";
    text << "    k3: " << camera.k3 << "\n";
    text << "    p1: " << camera.p1 << "\n";
    text << "    p2: " << camera.p2 << "\n";
    text << "    residual-rms: " << result.residualRms << " px over " << result.residuals.size() << " observations\n";
    std::cout << text.str() << std::flush;
}

}  // namespace scope
