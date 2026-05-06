#ifndef SRC_SCOPE_COMMAND_LINE_PARSING_PARSER_HPP_
#define SRC_SCOPE_COMMAND_LINE_PARSING_PARSER_HPP_

#include "scope/command-line/parsing/options.hpp"

namespace scope {

#define HELP_MSG "Use ./scope --help or ./scope -h for help"

/**
 * Parses the recalibration options from the command line to
 * run the recalibration algorithm
 * 
 * @param argc The number of command-line arguments
 * @param argv The command line arguments
 * 
 * @return CalibrationOptions The options for the calibration
 * algorithm as extracted on the command line
 */
RecalibrationOptions ParseRecalibrationOptions(int argc, char **argv);

}  // namespace scope

#endif  // SRC_SCOPE_COMMAND_LINE_PARSING_PARSER_HPP_
