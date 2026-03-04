#ifndef SRC_SCOPE_COMMAND_LINE_PARSING_PARSER_HPP_
#define SRC_SCOPE_COMMAND_LINE_PARSING_PARSER_HPP_

#include "scope/command-line/parsing/options.hpp"

namespace scope {

#define HELP_MSG "Use ./scope --help or ./scope -h for help"

CalibrationOptions ParseCalibrationOptions(int argc, char** argv);

}  // namespace scope

#endif  // SRC_SCOPE_COMMAND_LINE_PARSING_PARSER_HPP_
