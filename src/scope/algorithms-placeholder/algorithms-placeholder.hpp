#ifndef SRC_SCOPE_ALGORITHMS_PLACEHOLDER_ALGORITHMS_PLACEHOLDER_HPP_
#define SRC_SCOPE_ALGORITHMS_PLACEHOLDER_ALGORITHMS_PLACEHOLDER_HPP_

#include "common/pipeline/stages.hpp"
#include "common/style.hpp"
#include "scope/command-line/parsing/options.hpp"
#include "scope/noise-filter/noise-filter.hpp"

namespace scope {

class StarCentroidAlgorithm
    : public found::FunctionStage<found::Image, std::vector<float>> {
  public:
    // Constructs this
    StarCentroidAlgorithm() = default;
    // Destroys this
    virtual ~StarCentroidAlgorithm() {}
};

class ROIFilterAlgorithm : public StarCentroidAlgorithm {
  public:
    // Constructs this
    ROIFilterAlgorithm() = default;
    // Constructs this with options
    explicit ROIFilterAlgorithm(
        [[maybe_unused]] const RecalibrationOptions &options) {}
    // Destroys this
    virtual ~ROIFilterAlgorithm() {}

    // Runs this, in dummy, returns empty vector
    std::vector<float> Run(const found::Image &darkScreen) override;
};

class OptimizationAlgorithm
    : public found::FunctionStage<std::vector<float>, std::vector<float>> {
  public:
    // Constructs this
    OptimizationAlgorithm() = default;
    // Destroys this
    virtual ~OptimizationAlgorithm() {}
};

class LMAOptimizationAlgorithm : public OptimizationAlgorithm {
  public:
    // Constructs this
    LMAOptimizationAlgorithm() = default;
    // Constructs this with options
    explicit LMAOptimizationAlgorithm(
        [[maybe_unused]] const RecalibrationOptions &options) {}
    // Destroys this
    virtual ~LMAOptimizationAlgorithm() {}

    // Runs this, in dummy, returns empty vector
    std::vector<float> Run(const std::vector<float> &stars) override;
};

} // namespace scope

#endif // SRC_SCOPE_ALGORITHMS_PLACEHOLDER_ALGORITHMS_PLACEHOLDER_HPP_
