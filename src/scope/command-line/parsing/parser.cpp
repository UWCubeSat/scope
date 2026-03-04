#include "scope/command-line/parsing/parser.hpp"

#include <getopt.h>

#include <stdexcept>

#include "scope/command-line/parsing/options.hpp"

int optind = 2;

#define OPTIONAL_OPTARG()                                                      \
    ((optarg == NULL && optind < argc && argv[optind][0] != '-')               \
         ? static_cast<bool>(optarg = argv[optind++])                          \
         : (optarg != NULL))

#define REQ_ASSIGN(options, prop, value, default) options.prop = (value);

#define OPT_ASSIGN(options, prop, value, default)                              \
    if (OPTIONAL_OPTARG()) {                                                   \
        options.prop = value;                                                  \
    } else {                                                                   \
        options.prop = default;                                                \
    }

namespace scope {

const int kNoDefaultArgument = 0;

CalibrationOptions ParseCalibrationOptions(int argc, char **argv) {
    optind = 2;

    enum class ClientOption {
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, defaultArg,  \
                         ASSIGN, doc)                                          \
    prop,
        CALIBRATION
#undef SCOPE_CLI_OPTION
    };

    static option long_options[] = {
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, defaultArg,  \
                         ASSIGN, doc)                                          \
    {name,                                                                     \
     defaultArg == kNoDefaultArgument ? required_argument : optional_argument, \
     0, static_cast<int>(ClientOption::prop)},
        CALIBRATION
#undef SCOPE_CLI_OPTION
        {0}};

    CalibrationOptions options;
    int index;
    int option;

    while ((option = getopt_long(argc, argv, "", long_options, &index)) != -1) {
        switch (option) {
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, defaultArg,  \
                         ASSIGN, doc)                                          \
    case static_cast<int>(ClientOption::prop):                                 \
        ASSIGN(options, prop, converter, defaultArg)                           \
        break;
            CALIBRATION
#undef SCOPE_CLI_OPTION
        default:
            throw std::invalid_argument("Illegal flag detected. " HELP_MSG);
        }
    }

    if (options.imageListPath.empty()) {
        throw std::invalid_argument("Missing required flag --image-list");
    }
    if (options.attitudesPath.empty()) {
        throw std::invalid_argument("Missing required flag --attitudes");
    }

    return options;
}

}  // namespace scope
