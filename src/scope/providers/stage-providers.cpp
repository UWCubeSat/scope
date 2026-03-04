#include "scope/providers/stage-providers.hpp"

#include <memory>

namespace scope {

class NoOpCalibrationAlgorithm : public CalibrationAlgorithm {
 public:
    void Calibrate(const CalibrationOptions& options) override {
        (void)options;
    }
};

std::unique_ptr<CalibrationAlgorithm> ProvideCalibrationAlgorithm(
    [[maybe_unused]] CalibrationOptions&& options) {
    return std::make_unique<NoOpCalibrationAlgorithm>();
}

}  // namespace scope
