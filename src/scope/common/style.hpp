#ifndef SRC_SCOPE_COMMON_STYLE_HPP_
#define SRC_SCOPE_COMMON_STYLE_HPP_

#include "common/pipeline/pipelines.hpp"

#include "common/style.hpp"

#include <vector>

namespace scope {

// Images passed in by the user
typedef std::vector<found::Image> Images;

// Number of (maximum) stages for each pipeline
constexpr size_t recalibration_size = 3;

/// Pipeline for Recalibration
typedef found::SequentialPipeline<std::vector<found::Image>, std::vector<float>, recalibration_size> PrimaryScopePipeline;

}  // namespace scope

#endif  //SRC_SCOPE_COMMON_STYLE_HPP_