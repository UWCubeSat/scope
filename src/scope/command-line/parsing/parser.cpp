#include "scope/command-line/parsing/parser.hpp"

#include <getopt.h>

#include <cstdlib>
#include <stdexcept>
#include <string>

#include "scope/command-line/parsing/options.hpp"

int optind = 2;

namespace scope {

RecalibrationOptions ParseRecalibrationOptions(int argc, char **argv) {
    // Each block below re-expands RECALIBRATE (options.hpp) into a piece of the
    // getopt wiring.
    enum class ClientOption {
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, doc) prop,
        RECALIBRATE
#undef SCOPE_CLI_OPTION
    };

    static option long_options[] = {
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, doc) \
    {name, required_argument, 0, static_cast<int>(ClientOption::prop)},
        RECALIBRATE
#undef SCOPE_CLI_OPTION
        {0}};

    RecalibrationOptions options;
    int index;
    int option;

    while ((option = getopt_long(argc, argv, "", long_options, &index)) != -1) {
        switch (option) {
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, doc) \
    case static_cast<int>(ClientOption::prop):                         \
        options.prop = (converter);                                    \
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

}  // namespace scope
