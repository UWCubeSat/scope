#include <cstdlib>

#include <iostream>
#include <string>

#include "scope/command-line/scope-main.hpp"

#include "scope/command-line/parsing/options.hpp"
#include "scope/command-line/parsing/parser.hpp"
#include "scope/providers/factory.hpp"

namespace scope {

namespace {

void PrintHelp() {
    std::cout << "Usage: ./scope <option> [[flag value]...] [[flag=value]...]"
              << std::endl;
    std::cout << std::endl;
    std::cout << "Current capabilities:" << std::endl;
    std::cout << "  1. calibration - command-path scaffold for camera "
                 "parameter estimation"
              << std::endl;
    std::cout << std::endl;
    std::cout << "==================== Calibration Flags ===================="
              << std::endl;
    std::cout << std::endl;
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, defaultArg,  \
                         ASSIGN, doc)                                          \
    std::cout << "    --" << name << std::endl;                                \
    std::cout << "        " << doc << std::endl;
    CALIBRATION
#undef SCOPE_CLI_OPTION
}

}  // namespace

int main(int argc, char **argv) {
    if (argc == 1) {
        std::cerr << "No command provided. " << HELP_MSG << std::endl;
        return EXIT_FAILURE;
    }

    std::string command(argv[1]);

    if (command == "--help" || command == "-h") {
        PrintHelp();
        return EXIT_SUCCESS;
    }

    if (command == "calibration") {
        try {
            auto executor = CreateCalibrationPipelineExecutor(
                ParseCalibrationOptions(argc, argv));
            executor->ExecutePipeline();
            executor->OutputResults();
            return EXIT_SUCCESS;
        } catch (const std::exception &exception) {
            std::cerr << exception.what() << std::endl;
            return EXIT_FAILURE;
        }
    }

    std::cerr << "Unrecognized command: " << command << ". " << HELP_MSG
              << std::endl;
    return EXIT_FAILURE;
}

}  // namespace scope
