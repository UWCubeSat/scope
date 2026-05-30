#ifndef SRC_SCOPE_PROVIDERS_FACTORY_HPP_
#define SRC_SCOPE_PROVIDERS_FACTORY_HPP_

#include <memory>
#include <utility>
#include <vector>

#include "common/spatial/attitude-utils.hpp"

#include "scope/catalog/catalog.hpp"
#include "scope/command-line/execution/executors.hpp"
#include "scope/providers/stage-providers.hpp"

namespace scope {

/**
 * Assembles a PrimaryScopePipelineExecutor from parsed options.
 *
 * @param options Parsed recalibration options
 *                (see src/scope/command-line/parsing/options.hpp).
 *
 * @return The assembled PrimaryScopePipelineExecutor.
 */
inline std::unique_ptr<PrimaryScopePipelineExecutor> CreatePrimaryScopePipelineExecutor(
    RecalibrationOptions &&options) {
    // Load the catalog once; the star-centroid stage takes ownership of it.
    Catalog catalog = LoadBsc(options.catalogPath);

    // TODO(lost-integration): replace identity attitudes with per-image quaternions
    // queried from LOST. Until then, integration tests must supply explicit attitudes.
    std::vector<found::Quaternion> attitudes(options.starImages.size(), found::Quaternion::Identity());

    std::unique_ptr<NoiseFilterAlgorithm> noiseAlg = ProvideNoiseFilterAlgorithm(options);
    std::unique_ptr<StarCentroidAlgorithm> starAlg =
        ProvideStarCentroidAlgorithm(options, std::move(catalog), std::move(attitudes));
    std::unique_ptr<OptimizationAlgorithm> optAlg = ProvideOptimizationAlgorithm(options);

    return std::make_unique<PrimaryScopePipelineExecutor>(
        std::move(options), std::move(noiseAlg), std::move(starAlg), std::move(optAlg));
}

}  // namespace scope

#endif  // SRC_SCOPE_PROVIDERS_FACTORY_HPP_
