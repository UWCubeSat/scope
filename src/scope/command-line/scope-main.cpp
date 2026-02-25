#include <cstdlib>

#include "scope/command-line/scope-main.hpp"

namespace scope {

int main(int argc, char **argv) {
  // Remove compile-time warnings about unused parameters
  (void)argc;
  (void)argv;

  return EXIT_SUCCESS;
}

}  // namespace scope
