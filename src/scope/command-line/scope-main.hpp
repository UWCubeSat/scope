#ifndef SRC_SCOPE_COMMAND_LINE_SCOPE_MAIN_HPP_
#define SRC_SCOPE_COMMAND_LINE_SCOPE_MAIN_HPP_

namespace scope {

/**
 * Program entry point. Parses command-line options and runs the
 * recalibration pipeline.
 *
 * @param argc The number of command-line arguments.
 * @param argv The command-line arguments.
 *
 * @return 0 on success, non-zero on failure.
 */
int main(int argc, char **argv);

}  // namespace scope

#endif  // SRC_SCOPE_COMMAND_LINE_SCOPE_MAIN_HPP_
