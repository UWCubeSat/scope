#ifndef SRC_SCOPE_PROVIDERS_STAGE_PROVIDERS_HPP_
#define SRC_SCOPE_PROVIDERS_STAGE_PROVIDERS_HPP_

#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "common/spatial/attitude-utils.hpp"

#include "scope/attitude/attitude.hpp"
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
 * Supplies the prior attitude for each star image.
 *
 * This is the one place that knows where attitudes come from, so a different
 * source (a star tracker, a plate solver) is swapped in here. Whatever the
 * source, the attitudes returned must already be in SCOPE's camera frame:
 * e_C = attitude * e_I with the boresight on +z. An attitude from LOST has to
 * go through LostAttitudeToScopeFrame (src/scope/projection/projection.hpp)
 * first; skipping that yields plausible-but-wrong pixels.
 *
 * @param options Parsed recalibration options.
 *
 * @return One attitude per star image, in star-image order.
 *
 * @throws std::runtime_error if there are star images but no attitudes file,
 *         or the attitudes file cannot be read.
 */
inline std::vector<found::Quaternion> ProvideAttitudes(const RecalibrationOptions &options) {
    if (options.attitudesPath.empty()) {
        if (!options.starImages.empty()) {
            throw std::runtime_error("No attitudes given: --attitudes needs one attitude per star image");
        }
        return {};
    }
    return LoadAttitudes(options.attitudesPath);
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
