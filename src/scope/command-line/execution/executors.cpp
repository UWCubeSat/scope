#include "scope/command-line/execution/executors.hpp"

#include <iostream>

namespace scope {

CalibrationPipelineExecutor::CalibrationPipelineExecutor(
    CalibrationOptions&& options,
    std::unique_ptr<CalibrationAlgorithm> calibrationAlgorithm)
    : options_(std::move(options)),
      calibrationAlgorithm_(std::move(calibrationAlgorithm)) {}

void CalibrationPipelineExecutor::ExecutePipeline() {
    calibrationAlgorithm_->Calibrate(options_);
    executed_ = true;
}

void CalibrationPipelineExecutor::OutputResults() {
    if (!executed_) {
        std::cout << "Calibration pipeline has not been executed." << std::endl;
        return;
    }

    std::cout << "Calibration command executed." << std::endl;
    std::cout << "  image-list: " << options_.imageListPath << std::endl;
    std::cout << "  attitudes: " << options_.attitudesPath << std::endl;
    if (!options_.darkFramePath.empty()) {
        std::cout << "  dark-frame: " << options_.darkFramePath << std::endl;
    }
    if (!options_.outputFile.empty()) {
        std::cout << "  output-file: " << options_.outputFile << std::endl;
    }
    std::cout << "  dry-run: " << (options_.dryRun ? "true" : "false")
              << std::endl;
}

}  // namespace scope
