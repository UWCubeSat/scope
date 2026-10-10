#ifndef SRC_SCOPE_COMMAND_LINE_PARSING_OPTIONS_HPP_
#define SRC_SCOPE_COMMAND_LINE_PARSING_OPTIONS_HPP_

// Arguments to SCOPE_CLI_OPTION:
// 1. Flag name on the command line.
// 2. Type of the option value.
// 3. Property name.
// 4. Default value.
// 5. Code to convert optarg into the value.
// 6. Help text.

#include <string>

#include "common/decimal.hpp"
#include "scope/common/style.hpp"
#include "scope/providers/converters.hpp"

// NOLINTBEGIN

#define RECALIBRATE \
SCOPE_CLI_OPTION("focal-length-x", decimal, focalLengthX, 0, found::strtodecimal(optarg), "Focal length of camera, x parameter")  \
SCOPE_CLI_OPTION("focal-length-y", decimal, focalLengthY, 0, found::strtodecimal(optarg), "Focal length of camera, y parameter")  \
SCOPE_CLI_OPTION("principal-point-x", decimal, principalX, 0, found::strtodecimal(optarg), "Principal point of image, x parameter")  \
SCOPE_CLI_OPTION("principal-point-y", decimal, principalY, 0, found::strtodecimal(optarg), "Principal point of image, y parameter")  \
SCOPE_CLI_OPTION("alpha", decimal, alpha, 0, found::strtodecimal(optarg), "Check Artemis I paper for more info")  \
SCOPE_CLI_OPTION("k1", decimal, k1, 0, found::strtodecimal(optarg), "First radial distortion coefficient")  \
SCOPE_CLI_OPTION("k2", decimal, k2, 0, found::strtodecimal(optarg), "Second radial distortion coefficient")  \
SCOPE_CLI_OPTION("k3", decimal, k3, 0, found::strtodecimal(optarg), "Third radial distortion coefficient")  \
SCOPE_CLI_OPTION("p1", decimal, p1, 0, found::strtodecimal(optarg), "First tangential distortion coefficient")  \
SCOPE_CLI_OPTION("p2", decimal, p2, 0, found::strtodecimal(optarg), "Second tangential distortion coefficient")  \
SCOPE_CLI_OPTION("dark-frames", scope::Images, darkFrames, {}, scope::strtoimages(optarg), "Dark calibration frames used to estimate fixed-pattern noise (list of comma or space separated file paths)")  \
SCOPE_CLI_OPTION("star-images", scope::Images, starImages, {}, scope::strtoimages(optarg), "Star-field images to centroid (list of comma or space separated file paths)")  \
SCOPE_CLI_OPTION("attitudes", std::string, attitudesPath, std::string(""), std::string(optarg), "Path to a text file with one prior attitude per star image, in the same order: a quaternion 'w x y z' per line that rotates inertial (J2000) directions into SCOPE's +z-boresight camera frame")  \
SCOPE_CLI_OPTION("catalog-path", std::string, catalogPath, std::string("./bright-star-catalog.tsv"), std::string(optarg), "Path to the bright star catalog TSV (see download-bsc.sh)")  \
SCOPE_CLI_OPTION("centroid-threshold", int, centroidThreshold, 40, std::atoi(optarg), "Minimum dark-subtracted intensity for a pixel to join a centroid mask (paper does not specify; 40 is a starting value)")  \
SCOPE_CLI_OPTION("magnitude-threshold", decimal, magnitudeThreshold, 6, found::strtodecimal(optarg), "Faintest apparent magnitude to project from the catalog; fainter stars are skipped to keep the field sparse (default 6.0)")  \
SCOPE_CLI_OPTION("roi-size", int, roiSize, 31, std::atoi(optarg), "Side length in pixels of the square window searched around each star's predicted position (default 31, the paper's value); the window is centered on a pixel, so an even value acts as the next odd one")  \

// NOLINTEND

/** Parsed CLI options driving a recalibration run. */
class RecalibrationOptions {
 public:
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, doc) type prop = defaultVal;
    RECALIBRATE
#undef SCOPE_CLI_OPTION
};

#endif  // SRC_SCOPE_COMMAND_LINE_PARSING_OPTIONS_HPP_
