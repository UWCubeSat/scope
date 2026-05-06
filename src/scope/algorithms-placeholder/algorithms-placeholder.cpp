#include "scope/algorithms-placeholder/algorithms-placeholder.hpp"

namespace scope {

found::Image DarkScreenFilter::Run([[maybe_unused]] const Images &images) {
    return found::Image();
}

std::vector<float>
ROIFilterAlgorithm::Run([[maybe_unused]] const found::Image &darkScreen) {
    return std::vector<float>();
}

std::vector<float> LMAOptimizationAlgorithm::Run(
    [[maybe_unused]] const std::vector<float> &stars) {
    return std::vector<float>();
}

} // namespace scope
