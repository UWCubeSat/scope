#ifndef SRC_SCOPE_ALGORITHMS_PLACEHOLDER_ALGORITHMS_PLACEHOLDER_HPP_
#define SRC_SCOPE_ALGORITHMS_PLACEHOLDER_ALGORITHMS_PLACEHOLDER_HPP_

#include "common/pipeline/stages.hpp"
#include "common/style.hpp"

namespace scope {

class NoiseFilterAlgorithm : public found::FunctionStage<Images, found::Image> {
 public:
    // Constructs this
    NoiseFilterAlgorithm() = default;
    // Destroys this
    virtual ~NoiseFilterAlgorithm() {}
};

class DarkScreenFilter : public NoiseFilterAlgorithm {
 public:
    // Constructs this
    DarkScreenFilter() = default;
    // Destroys this
    virtual ~DarkScreenFilter() {}

    // Runs this, in dummy, returns an empty image
    found::Image Run(const Images& images) override;
};

class StarCentroidAlgorithm : public found::FunctionStage<found::Image, std::vector<float>> {
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
    // Destroys this
    virtual ~ROIFilterAlgorithm() {}

    // Runs this, in dummy, returns empty vector
    std::vector<float> Run(const found::Image& darkScreen) override;
};

class OptimizationAlgorithm : public found::FunctionStage<std::vector<float>, std::vector<float>> {
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
    // Destroys this
    virtual ~LMAOptimizationAlgorithm() {}

    // Runs this, in dummy, returns empty vector
    std::vector<float> Run(const std::vector<float>& stars) override;
};

}  // namespace scope

#endif  //SRC_SCOPE_ALGORITHMS_PLACEHOLDER_ALGORITHMS_PLACEHOLDER_HPP_