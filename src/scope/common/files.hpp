/**
 * @file files.hpp
 * @brief Writing output files.
 */

#ifndef SRC_SCOPE_COMMON_FILES_HPP_
#define SRC_SCOPE_COMMON_FILES_HPP_

#include <string>

namespace scope {

/**
 * Writes contents to a file, replacing anything already there.
 *
 * @param path Path to the file to write.
 * @param contents The bytes to write.
 *
 * @throws std::runtime_error if the file cannot be opened or written.
 */
void WriteFile(const std::string &path, const std::string &contents);

}  // namespace scope

#endif  // SRC_SCOPE_COMMON_FILES_HPP_
