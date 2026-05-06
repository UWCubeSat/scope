#ifndef SRC_SCOPE_PROVIDERS_FACTORY_HPP_
#define SRC_SCOPE_PROVIDERS_FACTORY_HPP_

#include <memory>
#include <utility>

#include "scope/command-line/execution/executors.hpp"
#include "scope/providers/stage-providers.hpp"

namespace scope {

/**
 * Creates a PrimaryScopePipelineExecutor
 * 
 * @param options The options to create the pipeline from
 *                (refer to src/scope/command-line/parsing/options.hpp)
 * 
 * @return A pointer to a PrimaryScopePipelineExecutor
 */
inline std::unique_ptr<PrimaryScopePipelineExecutor> CreatePrimaryScopePipelineExecutor(RecalibrationOptions &&options) {
    std::unique_ptr<NoiseFilterAlgorithm> noiseAlg = ProvideNoiseFilterAlgorithm(
                                                       std::forward<const RecalibrationOptions&&>(options));
    std::unique_ptr<StarCentroidAlgorithm> starAlg = ProvideStarCentroidAlgorithm(
                                                       std::forward<const RecalibrationOptions&&>(options));
    std::unique_ptr<OptimizationAlgorithm> optAlg = ProvideOptimizationAlgorithm(
                                                       std::forward<const RecalibrationOptions&&>(options));

    return std::make_unique<PrimaryScopePipelineExecutor>(
        std::move(options),
        std::move(noiseAlg),
        std::move(starAlg),
        std::move(optAlg));
}

}  // namespace scope

#endif  // SRC_SCOPE_PROVIDERS_FACTORY_HPP_
