/// @file optimization.hpp
/// @brief Parameter-fit optimization algorithms for SCOPE calibration.
///
/// This header defines the optimization pipeline stage: its abstract
/// base and concrete implementations.

#ifndef SRC_SCOPE_OPTIMIZATION_OPTIMIZATION_HPP_
#define SRC_SCOPE_OPTIMIZATION_OPTIMIZATION_HPP_

#include <vector>

#include "common/pipeline/stages.hpp"

#include "scope/command-line/parsing/options.hpp"

namespace scope {

/// Pipeline stage that produces updated camera parameters from centroids.
class OptimizationAlgorithm : public found::FunctionStage<std::vector<float>, std::vector<float>> {
 public:
    OptimizationAlgorithm() = default;
    virtual ~OptimizationAlgorithm() {}
};

/// Levenberg-Marquardt parameter optimizer. Stub implementation pending
/// the real algorithm.
class LMAOptimizationAlgorithm : public OptimizationAlgorithm {
 public:
    LMAOptimizationAlgorithm() = default;
    explicit LMAOptimizationAlgorithm([[maybe_unused]] const RecalibrationOptions &options) {}
    ~LMAOptimizationAlgorithm() override = default;

    std::vector<float> Run(const std::vector<float> &stars) override;
};

}  // namespace scope

#endif  // SRC_SCOPE_OPTIMIZATION_OPTIMIZATION_HPP_
