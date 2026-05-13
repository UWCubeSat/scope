#ifndef SRC_SCOPE_PROVIDERS_STAGE_PROVIDERS_HPP_
#define SRC_SCOPE_PROVIDERS_STAGE_PROVIDERS_HPP_

#include <memory>

#include "scope/command-line/execution/executors.hpp"
#include "scope/noise-filter/noise-filter.hpp"
#include "scope/optimization/optimization.hpp"
#include "scope/star-centroid/star-centroid.hpp"

namespace scope {

/**
 * Provides a NoiseFilterAlgorithm
 *
 * @param options The options to derive the noise filter algorithm from
 *
 * @return std::unique_ptr<NoiseFilterAlgorithm> The noise filter algorithm
 */
inline std::unique_ptr<NoiseFilterAlgorithm> ProvideNoiseFilterAlgorithm(
    [[maybe_unused]] const RecalibrationOptions &&options) {
    return std::make_unique<DarkScreenFilter>();
}

/**
 * Provides a StarCentroidAlgorithm
 *
 * @param options The options to derive the star centroid algorithm from
 *
 * @return std::unique_ptr<StarCentroidAlgorithm> The star centroid algorithm
 */
inline std::unique_ptr<StarCentroidAlgorithm> ProvideStarCentroidAlgorithm(const RecalibrationOptions &&options) {
    return std::make_unique<ROIFilterAlgorithm>(options);
}

/**
 * Provides an OptimizationAlgorithm
 *
 * @param options The options to derive the optimization algorithm from
 *
 * @return std::unique_ptr<OptimizationAlgorithm> The optimization algorithm
 */
inline std::unique_ptr<OptimizationAlgorithm> ProvideOptimizationAlgorithm(const RecalibrationOptions &&options) {
    return std::make_unique<LMAOptimizationAlgorithm>(options);
}

}  // namespace scope

#endif  // SRC_SCOPE_PROVIDERS_STAGE_PROVIDERS_HPP_
