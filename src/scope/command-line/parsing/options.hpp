#ifndef SRC_SCOPE_COMMAND_LINE_PARSING_OPTIONS_HPP_
#define SRC_SCOPE_COMMAND_LINE_PARSING_OPTIONS_HPP_

// Arguments to SCOPE_CLI_OPTION:
// 1. String used as the command line option.
// 2. Type of the option value.
// 3. Property name
// 4. Default value
// 5. Code to convert optarg into the value.
// 6. The default value if the flag is specified
// 7. The macro to use to assign a value to the variable
// 8. Documentation

#include <string>

#include "common/decimal.hpp"
#include "scope/common/style.hpp"
#include "scope/providers/converters.hpp"

// NOLINTBEGIN

#define RECALIBRATE \
SCOPE_CLI_OPTION("focal-length-x", decimal, focalLengthX, 0, found::strtodecimal(optarg), kNoDefaultArgument, REQ_ASSIGN, "Focal length of camera, x parameter")  \
SCOPE_CLI_OPTION("focal-length-y", decimal, focalLengthY, 0, found::strtodecimal(optarg), kNoDefaultArgument, REQ_ASSIGN, "Focal length of camera, y paramter")  \
SCOPE_CLI_OPTION("principal-point-x", decimal, principalX, 0, found::strtodecimal(optarg), kNoDefaultArgument, OPT_ASSIGN, "Principal point of image, x parameter")  \
SCOPE_CLI_OPTION("principal-point-y", decimal, principalY, 0, found::strtodecimal(optarg), kNoDefaultArgument, OPT_ASSIGN, "Principal point of image, y parameter")  \
SCOPE_CLI_OPTION("alpha", decimal, alpha, 0, found::strtodecimal(optarg), kNoDefaultArgument, REQ_ASSIGN, "Check Artemis I paper for more info")  \
SCOPE_CLI_OPTION("k1", decimal, k1, 0, found::strtodecimal(optarg), kNoDefaultArgument, REQ_ASSIGN, "First radial distortion coefficient")  \
SCOPE_CLI_OPTION("k2", decimal, k2, 0, found::strtodecimal(optarg), kNoDefaultArgument, REQ_ASSIGN, "Second radial distortion coefficient")  \
SCOPE_CLI_OPTION("k3", decimal, k3, 0, found::strtodecimal(optarg), kNoDefaultArgument, REQ_ASSIGN, "Third radial distortion coefficient")  \
SCOPE_CLI_OPTION("p1", decimal, p1, 0, found::strtodecimal(optarg), kNoDefaultArgument, REQ_ASSIGN, "First tangential distortion coefficient")  \
SCOPE_CLI_OPTION("p2", decimal, p2, 0, found::strtodecimal(optarg), kNoDefaultArgument, REQ_ASSIGN, "Second tangential distortion coefficient")  \
SCOPE_CLI_OPTION("input-images", scope::Images, images, {}, scope::strtoimages(optarg), kNoDefaultArgument, REQ_ASSIGN, "Images used to calculate paramters (list of comma or space seperated file paths)")  \

// NOLINTEND

/**
 * Options object, contains all data requested from user to run all algorithms
 */
class RecalibrationOptions {
 public:
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, defaultArg,  \
                         ASSIGN, doc)                                          \
    type prop = defaultVal;
    RECALIBRATE
#undef SCOPE_CLI_OPTION
};

#endif  // SRC_SCOPE_COMMAND_LINE_PARSING_OPTIONS_HPP_
