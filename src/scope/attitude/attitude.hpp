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

/**
 * Writes attitudes in the format LoadAttitudes reads, one `w x y z` line each.
 *
 * @param path Path to the file to write.
 * @param attitudes The attitudes, in star-image order.
 * @param header Comment written at the top of the file, one `#` line per line of text. May be empty.
 *
 * @throws std::runtime_error if the file cannot be written.
 */
void WriteAttitudes(const std::string &path,
                    const std::vector<found::Quaternion> &attitudes,
                    const std::string &header);

/**
 * The attitude correction for a small rotation vector dtheta (paper Eq. 11
 * and 38). To first order it is I - [dtheta x], so a camera-frame direction
 * becomes e_C' = e_C + e_C x dtheta, and a corrected attitude is
 * correction * attitude. It is computed exactly, as exp(-[dtheta x]), so the
 * result is a rotation however large dtheta is.
 *
 * @param dtheta The rotation vector, in radians.
 *
 * @return The correction as a unit quaternion.
 */
found::Quaternion AttitudeCorrection(const found::Vec3 &dtheta);

/**
 * The rotation vector of an attitude correction; the inverse of
 * AttitudeCorrection.
 *
 * @param correction A unit quaternion.
 *
 * @return dtheta, in radians, with norm at most pi.
 */
found::Vec3 CorrectionRotationVector(const found::Quaternion &correction);

/**
 * Averages rotations by Markley et al.'s method (2007, the paper's ref. 13):
 * the unit eigenvector of sum(q q^T) with the largest eigenvalue. Each
 * quaternion's sign is irrelevant, since q and -q give the same q q^T.
 *
 * @param quaternions The rotations to average, as unit quaternions.
 *
 * @return The average rotation.
 *
 * @throws std::invalid_argument if quaternions is empty.
 */
found::Quaternion AverageQuaternions(const std::vector<found::Quaternion> &quaternions);

}  // namespace scope

#endif  // SRC_SCOPE_ATTITUDE_ATTITUDE_HPP_
