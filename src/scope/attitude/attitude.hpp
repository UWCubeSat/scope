/**
 * @file attitude.hpp
 * @brief Loader for the per-image prior attitudes.
 */

#ifndef SRC_SCOPE_ATTITUDE_ATTITUDE_HPP_
#define SRC_SCOPE_ATTITUDE_ATTITUDE_HPP_

#include <string>
#include <vector>

#include "common/spatial/attitude-utils.hpp"

namespace scope {

/**
 * Loads one prior attitude per star image from a text file.
 *
 * One quaternion per line, `w x y z`, separated by spaces, tabs or commas.
 * Blank lines are skipped and `#` starts a comment. The k-th attitude belongs
 * to the k-th star image.
 *
 * Each quaternion must rotate an inertial (J2000) direction into SCOPE's
 * camera frame (boresight on +z), as ProjectStarToPixel expects. Whatever
 * writes the file does that conversion; for LOST it is LostAttitudeToScopeFrame.
 *
 * @param path Path to the attitudes file.
 *
 * @return The attitudes in file order, normalized.
 *
 * @throws std::runtime_error if the file cannot be opened, or a line is not
 *         exactly four numbers or is the zero quaternion.
 */
std::vector<found::Quaternion> LoadAttitudes(const std::string &path);

}  // namespace scope

#endif  // SRC_SCOPE_ATTITUDE_ATTITUDE_HPP_
