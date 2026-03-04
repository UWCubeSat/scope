#ifndef SRC_SCOPE_PROVIDERS_FACTORY_HPP_
#define SRC_SCOPE_PROVIDERS_FACTORY_HPP_

#include <memory>
#include <utility>

#include "scope/command-line/execution/executors.hpp"
#include "scope/providers/stage-providers.hpp"

namespace scope {

inline std::unique_ptr<CalibrationPipelineExecutor>
CreateCalibrationPipelineExecutor(CalibrationOptions&& options) {
    CalibrationOptions providerOptions = options;
    return std::make_unique<CalibrationPipelineExecutor>(
        std::move(options),
        ProvideCalibrationAlgorithm(std::move(providerOptions)));
}

}  // namespace scope

#endif  // SRC_SCOPE_PROVIDERS_FACTORY_HPP_
