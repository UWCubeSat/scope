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

/// For macro processing
const char kNoDefaultArgument = 0;

RecalibrationOptions ParseRecalibrationOptions(int argc, char **argv) {
    // Define an enum for each valid flag (command-line entry), which maps it
    // from the name to an integer
    enum class ClientOption {
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, defaultArg,  \
                         ASSIGN, doc)                                          \
    prop,
        RECALIBRATE
#undef SCOPE_CLI_OPTION
    };

    // Define an array of options, which defines the traits pertaining to each
    // expected command-line entry
    static option long_options[] = {
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, defaultArg,  \
                         ASSIGN, doc)                                          \
    {name,                                                                     \
     defaultArg == kNoDefaultArgument ? required_argument : optional_argument, \
     0, static_cast<int>(ClientOption::prop)},
        RECALIBRATE
#undef SCOPE_CLI_OPTION
        {0}};

    // Define our result, and iterator helpers
    RecalibrationOptions options;
    int index;
    int option;

    // Iterates through the list of command-line tokens and figures out
    // what data to assign to which field in options. Note that the
    // SCOPE_CLI_OPTION defines the conversion already between any
    // particular parameter (as a string) to its actual type
    while ((option = getopt_long(argc, argv, "", long_options, &index)) != -1) {
        switch (option) {
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, defaultArg,  \
                         ASSIGN, doc)                                          \
    case static_cast<int>(ClientOption::prop):                                 \
        ASSIGN(options, prop, converter, defaultArg)                           \
        break;
            RECALIBRATE
#undef SCOPE_CLI_OPTION
        default:
            LOG_ERROR("Illegal flag detected. " << HELP_MSG);
            exit(EXIT_FAILURE);
            break;
        }
    }

    return options;
}

} // namespace scope
