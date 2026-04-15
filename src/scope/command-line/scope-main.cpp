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
    std::cout << "\tCalculates camera intrisic and distortion paramters."
              << std::endl;
    std::cout << std::endl;
    std::cout << "==================== Calibration Flags ===================="
              << std::endl;
    std::cout << std::endl;
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, defaultArg,  ASSIGN, doc)  \
    std::cout << "\t--" << name << std::endl;  \
    std::cout << "\t\t" << doc << std::endl;
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

    std::unique_ptr<PipelineExecutor> executor;
    executor = CreateCalibrationPipelineExecutor(
                       ParseCalibrationOptions(argc, argv));
}

}  // namespace scope
