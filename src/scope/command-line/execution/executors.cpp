#include "scope/command-line/execution/executors.hpp"

#include <stb_image/stb_image.h>

#include <algorithm>
#include <cstddef>
#include <cstdlib>

#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/attitude/attitude.hpp"
#include "scope/common/files.hpp"
#include "scope/common/style.hpp"
#include "scope/common/types.hpp"
#include "scope/projection/projection.hpp"

namespace scope {

namespace {

/// Significant digits printed per value: enough to pass a printed calibration
/// back in as the next prior without sub-pixel loss.
constexpr int kOutputPrecision = 10;
/// Significant digits printed for an uncertainty or an angle.
constexpr int kShortPrecision = 3;

/// The value to kShortPrecision significant digits.
std::string FormatShort(decimal value) {
    std::ostringstream text;
    text << std::setprecision(kShortPrecision) << value;
    return text.str();
}

/// Writes one CSV row per observation: where it was measured, where the fit
/// predicts it, and whether the fit used it.
void WriteResiduals(const std::string &path,
                    const std::vector<Observation> &observations,
                    const CalibrationResult &result) {
    std::ostringstream text;
    text << std::setprecision(kOutputPrecision);
    text << "image,star,measured_u,measured_v,predicted_u,predicted_v,residual_u,residual_v,inlier\n";
    for (std::size_t i = 0; i < observations.size(); ++i) {
        const Observation &observation = observations[i];
        const found::Vec2 &residual = result.residuals[i];
        const found::Vec2 predicted = observation.measuredPixel - residual;
        text << observation.imageIndex << "," << observation.starName << "," << observation.measuredPixel.x() << ","
             << observation.measuredPixel.y() << "," << predicted.x() << "," << predicted.y() << "," << residual.x()
             << "," << residual.y() << "," << (result.inliers[i] ? 1 : 0) << "\n";
    }
    WriteFile(path, text.str());
}

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
    this->starStage_ = starCentroidStage.get();
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
    // No stage owns the noise filter's malloc'd dark frame.
    Image *darkFrame = this->noiseStage_->GetProduct();
    if (darkFrame != nullptr) {
        std::free(darkFrame->image);
    }
}

void PrimaryScopePipelineExecutor::OutputResults() {
    const CalibrationResult &result = *this->pipeline_.GetProduct();

    // Each parameter is labelled with its command-line flag.
    std::ostringstream text;
    text << std::setprecision(kOutputPrecision);
    text << "Calibration result:\n";
    text << "    converged: " << (result.converged ? "yes" : "no") << " after " << result.iterations << " iterations\n";
    for (int p = 0; p < kCameraParameterCount; ++p) {
        text << "    " << kCameraParameterNames[p] << ": " << result.camera.*kCameraParameterFields[p];
        if (result.estimated[p]) {
            text << " +/- " << FormatShort(result.uncertainty.*kCameraParameterFields[p]) << "\n";
        } else {
            text << " (fixed)\n";
        }
    }

    const std::size_t inliers =
        static_cast<std::size_t>(std::count(result.inliers.begin(), result.inliers.end(), true));
    text << "    residual-rms: " << result.residualRms << " px over " << inliers << " observations";
    if (inliers < result.residuals.size()) {
        text << " (" << result.residuals.size() - inliers << " not used)";
    }
    text << "\n";

    const found::Vec3 alignment = CorrectionRotationVector(result.alignmentCorrection);
    text << "    alignment-correction: " << FormatShort(found::RadToArcSec(alignment.x())) << " "
         << FormatShort(found::RadToArcSec(alignment.y())) << " " << FormatShort(found::RadToArcSec(alignment.z()))
         << " arcsec (rotation vector, camera frame)\n";

    // The fitted parameters as flags, to centroid again with a better prior.
    text << "    as-prior:";
    for (int p = 0; p < kCameraParameterCount; ++p) {
        text << " --" << kCameraParameterNames[p] << " " << result.camera.*kCameraParameterFields[p];
    }
    text << "\n";
    std::cout << text.str() << std::flush;

    if (!this->options_.outputAttitudesPath.empty()) {
        WriteAttitudes(this->options_.outputAttitudesPath,
                       result.attitudes,
                       "Refined attitudes from SCOPE, one per star image: w x y z, inertial to camera.");
    }
    if (!this->options_.outputResidualsPath.empty()) {
        WriteResiduals(this->options_.outputResidualsPath, this->starStage_->GetProduct()->observations, result);
    }
}

}  // namespace scope
