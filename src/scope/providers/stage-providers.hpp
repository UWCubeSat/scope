#ifndef SRC_SCOPE_PROVIDERS_STAGE_PROVIDERS_HPP_
#define SRC_SCOPE_PROVIDERS_STAGE_PROVIDERS_HPP_

#include <memory>
#include <utility>
#include <vector>

#include "common/spatial/attitude-utils.hpp"

#include "scope/catalog/catalog.hpp"
#include "scope/command-line/execution/executors.hpp"
#include "scope/noise-filter/noise-filter.hpp"
#include "scope/optimization/optimization.hpp"
#include "scope/star-centroid/star-centroid.hpp"

namespace scope {

/**
 * Selects a NoiseFilterAlgorithm implementation for the run.
 *
 * @param options Parsed recalibration options.
 *
 * @return The chosen NoiseFilterAlgorithm.
 */
inline std::unique_ptr<NoiseFilterAlgorithm> ProvideNoiseFilterAlgorithm(
    [[maybe_unused]] const RecalibrationOptions &options) {
    return std::make_unique<DarkScreenFilter>();
}

/**
 * Selects a StarCentroidAlgorithm implementation for the run.
 *
 * @param options Parsed recalibration options.
 * @param catalog The star catalog (moved into the stage).
 * @param attitudes One prior attitude per star image.
 *
 * @return The chosen StarCentroidAlgorithm.
 */
inline std::unique_ptr<StarCentroidAlgorithm> ProvideStarCentroidAlgorithm(const RecalibrationOptions &options,
                                                                           Catalog catalog,
                                                                           std::vector<found::Quaternion> attitudes) {
    return std::make_unique<ROIFilterAlgorithm>(options, std::move(catalog), std::move(attitudes));
}

/**
 * Selects an OptimizationAlgorithm implementation for the run.
 *
 * @param options Parsed recalibration options.
 *
 * @return The chosen OptimizationAlgorithm.
 */
inline std::unique_ptr<OptimizationAlgorithm> ProvideOptimizationAlgorithm(const RecalibrationOptions &options) {
    return std::make_unique<LMAOptimizationAlgorithm>(options);
}

}  // namespace scope

#endif  // SRC_SCOPE_PROVIDERS_STAGE_PROVIDERS_HPP_
