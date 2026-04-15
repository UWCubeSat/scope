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
#include "common/spatial/attitude-utils.hpp"
#include "providers/converters.hpp"

// NOLINTBEGIN

#define CALIBRATION \
    SCOPE_CLI_OPTION("input-images", std::string, inputDir, "", optarg, kNoDefaultArgument, REQ_ASSIGN, "Images used to calculate paramters (directory path)")  \
    SCOPE_CLI_OPTION("focal-length-x", decimal, focalLengthX, 0, found::strtodecimal(optarg), kNoDefaultArgument, REQ_ASSIGN, "Focal length of camera, x parameter")  \
    SCOPE_CLI_OPTION("focal-length-y", decimal, focalLengthY, 0, found::strtodecimal(optarg), kNoDefaultArgument, REQ_ASSIGN, "Focal length of camera, y paramter")  \
    SCOPE_CLI_OPTION("principal-point-x", decimal, principalX, 0, found::strtodecimal(optarg), kNoDefaultArgument, OPT_ASSIGN, "Principal point of image, x parameter")  \
    SCOPE_CLI_OPTION("principal-point-y", decimal, principalX, 0, found::strtodecimal(optarg), kNoDefaultArgument, OPT_ASSIGN, "Principal point of image, y parameter")  \
    SCOPE_CLI_OPTION("alpha", decimal, alpha, 0, optarg, kNoDefaultArgument, REQ_ASSIGN, "Check Artemis I paper for more info")  \
    SCOPE_CLI_OPTION("k1", decimal, k1, 0, found::strtodecimal(optarg), kNoDefaultArgument, REQ_ASSIGN, "First radial distortion coefficient")  \
    SCOPE_CLI_OPTION("k2", decimal, k2, 0, found::strtodecimal(optarg), kNoDefaultArgument, REQ_ASSIGN, "Second radial distortion coefficient")  \
    SCOPE_CLI_OPTION("k3", decimal, k3, 0, found::strtodecimal(optarg), kNoDefaultArgument, REQ_ASSIGN, "Third radial distortion coefficient")  \
    SCOPE_CLI_OPTION("p1", decimal, p1, 0, found::strtodecimal(optarg), kNoDefaultArgument, REQ_ASSIGN, "First tangential distortion coefficient")  \
    SCOPE_CLI_OPTION("p2", decimal, p2, 0, found::strtodecimal(optarg), kNoDefaultArgument, REQ_ASSIGN, "Second tangential distortion coefficient")

// NOLINTEND

namespace scope {

inline bool strtobool(const char *value) {
    if (value == nullptr)
        return true;

    std::string token(value);
    if (token == "1" || token == "true" || token == "TRUE" || token == "on" ||
        token == "ON") {
        return true;
    }
    if (token == "0" || token == "false" || token == "FALSE" ||
        token == "off" || token == "OFF") {
        return false;
    }

    throw std::invalid_argument("Expected boolean value, got: " + token);
}

class CalibrationOptions {
 public:
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, defaultArg,  \
                         ASSIGN, doc)                                          \
    type prop = defaultVal;
    CALIBRATION
#undef SCOPE_CLI_OPTION
};

}  // namespace scope

#endif  // SRC_SCOPE_COMMAND_LINE_PARSING_OPTIONS_HPP_
