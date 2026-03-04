#ifndef SRC_SCOPE_COMMAND_LINE_PARSING_OPTIONS_HPP_
#define SRC_SCOPE_COMMAND_LINE_PARSING_OPTIONS_HPP_

#include <string>
#include <stdexcept>

// NOLINTBEGIN

#define CALIBRATION                                                            \
    SCOPE_CLI_OPTION("image-list", std::string, imageListPath, "", optarg,     \
                     kNoDefaultArgument, REQ_ASSIGN,                           \
                     "Path to input image list")                               \
    SCOPE_CLI_OPTION("attitudes", std::string, attitudesPath, "", optarg,      \
                     kNoDefaultArgument, REQ_ASSIGN,                           \
                     "Path to LOST attitude input")                            \
    SCOPE_CLI_OPTION("output-file", std::string, outputFile, "", optarg,       \
                     kNoDefaultArgument, REQ_ASSIGN,                           \
                     "Path for calibrated parameter output")                   \
    SCOPE_CLI_OPTION("dark-frame", std::string, darkFramePath, "", optarg,     \
                     kNoDefaultArgument, REQ_ASSIGN,                           \
                     "Optional path to precomputed dark frame")                \
    SCOPE_CLI_OPTION("dry-run", bool, dryRun, false, scope::strtobool(optarg), \
                     true, OPT_ASSIGN,                                         \
                     "Parse and validate inputs without full processing")

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
