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
 * Each attitude is a quaternion on its own line, written as four numbers
 * `w x y z` (real part first) separated by spaces, tabs or commas. Blank lines
 * are skipped and `#` starts a comment that runs to the end of the line. The
 * k-th attitude in the file belongs to the k-th star image.
 *
 * The quaternion must rotate an inertial (equatorial J2000) direction into
 * SCOPE's camera frame, e_C = q * e_I, with the boresight on +z. That is what
 * ProjectStarToPixel consumes, so the file is deliberately ignorant of where the
 * attitudes came from: whatever writes it (a plate solver, a star tracker) is
 * responsible for converting into this frame. For an attitude from LOST that
 * conversion is LostAttitudeToScopeFrame (src/scope/projection/projection.hpp).
 *
 * @param path Path to the attitudes file.
 *
 * @return The attitudes in file order, each normalized to unit length.
 *
 * @throws std::runtime_error if the file cannot be opened, or if a line is not
 *         exactly four numbers or has zero length.
 */
std::vector<found::Quaternion> LoadAttitudes(const std::string &path);

}  // namespace scope

#endif  // SRC_SCOPE_ATTITUDE_ATTITUDE_HPP_
