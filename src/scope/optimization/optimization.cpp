#include "scope/optimization/optimization.hpp"

#include "scope/optimization/calibration-fit.hpp"

namespace scope {

CalibrationResult LMAOptimizationAlgorithm::Run(const CentroidObservations &observations) {
    return FitCalibration(observations, prior_, settings_);
}

}  // namespace scope
