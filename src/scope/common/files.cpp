#include "scope/common/files.hpp"

#include <fstream>
#include <stdexcept>
#include <string>

namespace scope {

void WriteFile(const std::string &path, const std::string &contents) {
    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("WriteFile: failed to open '" + path + "' for writing");
    }
    file << contents;
    // Closing flushes, which is where a full disk shows up.
    file.close();
    if (file.fail()) {
        throw std::runtime_error("WriteFile: failed to write '" + path + "'");
    }
}

}  // namespace scope
