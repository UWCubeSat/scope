#ifndef SRC_SCOPE_PROVIDERS_STAGE_PROVIDERS_HPP_
#define SRC_SCOPE_PROVIDERS_STAGE_PROVIDERS_HPP_

#include <memory>

#include "scope/command-line/execution/executors.hpp"

namespace scope {

std::unique_ptr<CalibrationAlgorithm>
ProvideCalibrationAlgorithm(CalibrationOptions &&options);

} // namespace scope

#endif // SRC_SCOPE_PROVIDERS_STAGE_PROVIDERS_HPP_
