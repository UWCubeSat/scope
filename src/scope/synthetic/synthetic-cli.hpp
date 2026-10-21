/**
 * @file synthetic-cli.hpp
 * @brief Command line of scope-synth, which writes a synthetic data set for
 * the scope command: dark frames, star images, the attitudes file and the
 * truth they were made from.
 */

#ifndef SRC_SCOPE_SYNTHETIC_SYNTHETIC_CLI_HPP_
#define SRC_SCOPE_SYNTHETIC_SYNTHETIC_CLI_HPP_

#include <cstdlib>

#include <string>

#include "common/decimal.hpp"
#include "providers/converters.hpp"

// Arguments to SCOPE_CLI_OPTION, as for RECALIBRATE (options.hpp): flag name,
// type, property name, default value, converter from optarg, help text.

// NOLINTBEGIN

#define SYNTHESIZE \
SCOPE_CLI_OPTION("output-dir", std::string, outputDir, std::string(""), std::string(optarg), "Directory to write the data set to; created if missing (required)")  \
SCOPE_CLI_OPTION("catalog-path", std::string, catalogPath, std::string("./bright-star-catalog.tsv"), std::string(optarg), "Path to the bright star catalog TSV the stars are drawn from (see download-bsc.sh)")  \
SCOPE_CLI_OPTION("star-images", int, starImageCount, 15, std::atoi(optarg), "Number of star images, each at a random attitude (default 15, the paper's number)")  \
SCOPE_CLI_OPTION("dark-frames", int, darkFrameCount, 5, std::atoi(optarg), "Number of lens-capped dark frames (default 5)")  \
SCOPE_CLI_OPTION("seed", int, seed, 1, std::atoi(optarg), "Random seed; the same seed and flags give the same data set (default 1)")  \
SCOPE_CLI_OPTION("width", int, width, 2064, std::atoi(optarg), "Sensor width in pixels (default 2064, the IMX252)")  \
SCOPE_CLI_OPTION("height", int, height, 1544, std::atoi(optarg), "Sensor height in pixels (default 1544)")  \
SCOPE_CLI_OPTION("focal-length-x", decimal, focalLengthX, 1745.0, found::strtodecimal(optarg), "True focal length along u, in pixels (default 1745.0, 0.3% longer than the 6 mm lens's nominal 1739.13)")  \
SCOPE_CLI_OPTION("alpha", decimal, alpha, 0, found::strtodecimal(optarg), "True skew, in pixels (default 0)")  \
SCOPE_CLI_OPTION("focal-length-y", decimal, focalLengthY, 1744.2, found::strtodecimal(optarg), "True focal length along v, in pixels (default 1744.2)")  \
SCOPE_CLI_OPTION("principal-point-x", decimal, principalX, 1031.5, found::strtodecimal(optarg), "True principal point column (default 1031.5, the centre of a 2064-pixel row)")  \
SCOPE_CLI_OPTION("principal-point-y", decimal, principalY, 771.5, found::strtodecimal(optarg), "True principal point row (default 771.5)")  \
SCOPE_CLI_OPTION("k1", decimal, k1, -0.152, found::strtodecimal(optarg), "True first radial distortion coefficient (default -0.152; the datasheet fit is -0.157)")  \
SCOPE_CLI_OPTION("k2", decimal, k2, 0.077, found::strtodecimal(optarg), "True second radial distortion coefficient (default 0.077; the datasheet fit is 0.082)")  \
SCOPE_CLI_OPTION("k3", decimal, k3, 0, found::strtodecimal(optarg), "True third radial distortion coefficient (default 0)")  \
SCOPE_CLI_OPTION("p1", decimal, p1, 1e-4, found::strtodecimal(optarg), "True first decentering coefficient (default 1e-4)")  \
SCOPE_CLI_OPTION("p2", decimal, p2, -5e-5, found::strtodecimal(optarg), "True second decentering coefficient (default -5e-5)")  \
SCOPE_CLI_OPTION("attitude-noise", decimal, attitudeNoise, 20, found::strtodecimal(optarg), "Per-axis standard deviation of each reported attitude's own error, in arcseconds (default 20)")  \
SCOPE_CLI_OPTION("misalignment", decimal, misalignment, 60, found::strtodecimal(optarg), "Per-axis standard deviation of the error common to every reported attitude (a camera-to-star-tracker misalignment), drawn once, in arcseconds (default 60)")  \
SCOPE_CLI_OPTION("magnitude-limit", decimal, magnitudeLimit, 7, found::strtodecimal(optarg), "Faintest catalog magnitude rendered (default 7, the whole catalog)")  \
SCOPE_CLI_OPTION("psf-sigma", decimal, psfSigma, 1.0, found::strtodecimal(optarg), "Standard deviation of the Gaussian star image (a defocused star), in pixels (default 1.0)")  \
SCOPE_CLI_OPTION("zero-point", decimal, zeroPoint, 50000, found::strtodecimal(optarg), "Total counts from a magnitude 0 star (default 50000, which puts the default --centroid-threshold near magnitude 5.5 and saturates stars brighter than about 3.5)")  \
SCOPE_CLI_OPTION("sky-background", decimal, skyBackground, 2, found::strtodecimal(optarg), "Sky level in star images, in counts (default 2)")  \
SCOPE_CLI_OPTION("black-level", decimal, blackLevel, 8, found::strtodecimal(optarg), "Mean of the fixed pattern, in counts (default 8)")  \
SCOPE_CLI_OPTION("fixed-pattern-noise", decimal, fixedPatternNoise, 1, found::strtodecimal(optarg), "Pixel-to-pixel standard deviation of the fixed pattern, in counts (default 1)")  \
SCOPE_CLI_OPTION("hot-pixel-fraction", decimal, hotPixelFraction, 1e-4, found::strtodecimal(optarg), "Fraction of pixels that are hot, 50 to 200 counts above the rest (default 1e-4)")  \
SCOPE_CLI_OPTION("read-noise", decimal, readNoise, 1, found::strtodecimal(optarg), "Read noise per pixel and frame, in counts (default 1)")  \
SCOPE_CLI_OPTION("gain", decimal, gain, 4, found::strtodecimal(optarg), "Electrons per count, which sets the shot noise (default 4)")  \

// NOLINTEND

namespace scope {

/** Parsed scope-synth options. */
class SynthesisOptions {
 public:
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, doc) type prop = defaultVal;
    SYNTHESIZE
#undef SCOPE_CLI_OPTION
};

/**
 * Parses scope-synth's flags. Unlike scope, there is no command word: the
 * flags start at argv[1].
 *
 * @param argc The number of command-line arguments.
 * @param argv The command-line arguments.
 *
 * @return The parsed options.
 *
 * @throws std::runtime_error on an unknown flag or a flag with no value.
 */
SynthesisOptions ParseSynthesisOptions(int argc, char **argv);

/**
 * Writes the data set the options describe into options.outputDir:
 * dark-NNN.pgm and star-NNN.pgm, attitudes.txt (the reported attitudes, for
 * --attitudes), truth.txt (the true camera parameters, misalignment and
 * attitudes) and scope-args.txt (the scope flags that load the data set).
 *
 * @param options The parsed options.
 *
 * @return A short summary of what was written.
 *
 * @throws std::runtime_error if an option is out of range, the catalog cannot
 *         be read or a file cannot be written.
 */
std::string WriteSyntheticDataSet(const SynthesisOptions &options);

/**
 * scope-synth's entry point.
 *
 * @param argc The number of command-line arguments.
 * @param argv The command-line arguments.
 *
 * @return 0 on success, non-zero on failure.
 */
int SynthMain(int argc, char **argv);

}  // namespace scope

#endif  // SRC_SCOPE_SYNTHETIC_SYNTHETIC_CLI_HPP_
