#include "scope/synthetic/synthetic-cli.hpp"

#include <getopt.h>

#include <cstdlib>

#include <exception>
#include <filesystem>  // NOLINT(build/c++17)
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/attitude/attitude.hpp"
#include "scope/catalog/catalog.hpp"
#include "scope/common/files.hpp"
#include "scope/projection/projection.hpp"
#include "scope/synthetic/synthetic.hpp"

namespace scope {

namespace {

/// Significant digits in truth.txt, well past what any fit resolves.
constexpr int kTruthPrecision = 12;

void PrintHelp() {
    std::cout << "Usage: ./scope-synth --output-dir <dir> [[flag value]...]" << std::endl;
    std::cout << std::endl;
    std::cout << "\tWrites dark frames, star images, an attitudes file and the truth they were made from," << std::endl;
    std::cout << "\tfor testing scope end to end. scope-args.txt in the output directory holds the scope" << std::endl;
    std::cout << "\tflags that load the data set; add the prior camera parameters to them." << std::endl;
    std::cout << std::endl;
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, doc) \
    std::cout << "\t--" << name << std::endl;                          \
    std::cout << "\t\t" << doc << std::endl;
    SYNTHESIZE
#undef SCOPE_CLI_OPTION
}

/// Joins paths with commas, the list format --dark-frames and --star-images take.
std::string JoinPaths(const std::vector<std::string> &paths) {
    std::string joined;
    for (const std::string &path : paths) {
        joined += (joined.empty() ? "" : ",") + path;
    }
    return joined;
}

/// The file name of the index-th file with a prefix: prefix-007.pgm.
std::string NumberedName(const std::string &prefix, int index) {
    std::ostringstream name;
    name << prefix << "-" << std::setw(3) << std::setfill('0') << index << ".pgm";
    return name.str();
}

}  // namespace

SynthesisOptions ParseSynthesisOptions(int argc, char **argv) {
    enum class SynthOption {
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, doc) prop,
        SYNTHESIZE
#undef SCOPE_CLI_OPTION
    };

    static option long_options[] = {
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, doc) \
    {name, required_argument, 0, static_cast<int>(SynthOption::prop)},
        SYNTHESIZE
#undef SCOPE_CLI_OPTION
        {0}};

    SynthesisOptions options;
    int index;
    int option;
    // getopt keeps its position in a global, which scope's parser starts at 2.
    optind = 1;
    opterr = 0;
    while ((option = getopt_long(argc, argv, "", long_options, &index)) != -1) {
        switch (option) {
#define SCOPE_CLI_OPTION(name, type, prop, defaultVal, converter, doc) \
    case static_cast<int>(SynthOption::prop):                          \
        options.prop = (converter);                                    \
        break;
            SYNTHESIZE
#undef SCOPE_CLI_OPTION
            default:
                throw std::runtime_error("unknown flag or missing value: " + std::string(argv[optind - 1]));
        }
    }
    if (optind < argc) {
        throw std::runtime_error("unexpected argument: " + std::string(argv[optind]));
    }

    return options;
}

std::string WriteSyntheticDataSet(const SynthesisOptions &options) {
    if (options.outputDir.empty()) {
        throw std::runtime_error("--output-dir is required");
    }
    if (options.starImageCount < 1 || options.darkFrameCount < 1) {
        throw std::runtime_error("--star-images and --dark-frames must be at least 1");
    }
    if (options.width < 1 || options.height < 1) {
        throw std::runtime_error("--width and --height must be at least 1");
    }
    if (!(options.psfSigma > DECIMAL(0.0)) || !(options.gain > DECIMAL(0.0))) {
        throw std::runtime_error("--psf-sigma and --gain must be positive");
    }

    CameraParameters camera;
    camera.focalLengthX = options.focalLengthX;
    camera.alpha = options.alpha;
    camera.focalLengthY = options.focalLengthY;
    camera.principalX = options.principalX;
    camera.principalY = options.principalY;
    camera.k1 = options.k1;
    camera.k2 = options.k2;
    camera.k3 = options.k3;
    camera.p1 = options.p1;
    camera.p2 = options.p2;

    SensorModel sensor;
    sensor.psfSigma = options.psfSigma;
    sensor.zeroPoint = options.zeroPoint;
    sensor.skyBackground = options.skyBackground;
    sensor.blackLevel = options.blackLevel;
    sensor.fixedPatternNoise = options.fixedPatternNoise;
    sensor.hotPixelFraction = options.hotPixelFraction;
    sensor.readNoise = options.readNoise;
    sensor.gain = options.gain;

    Catalog catalog;
    const int magnitudeLimit = static_cast<int>(DECIMAL_ROUND(options.magnitudeLimit * DECIMAL(100.0)));
    for (const CatalogStar &star : LoadBsc(options.catalogPath)) {
        if (star.magnitude <= magnitudeLimit) {
            catalog.push_back(star);
        }
    }

    std::filesystem::create_directories(options.outputDir);
    const std::filesystem::path directory(options.outputDir);

    Random random(static_cast<Random::result_type>(options.seed));
    const SyntheticScene scene = RandomScene(camera,
                                             options.width,
                                             options.height,
                                             options.starImageCount,
                                             found::ArcSecToRad(options.attitudeNoise),
                                             found::ArcSecToRad(options.misalignment),
                                             &random);
    const std::vector<decimal> fixedPattern = RandomFixedPattern(options.width, options.height, sensor, &random);

    std::ostringstream summary;
    std::vector<std::string> darkPaths;
    for (int i = 0; i < options.darkFrameCount; ++i) {
        darkPaths.push_back((directory / NumberedName("dark", i)).string());
        WritePgm(darkPaths.back(), options.width, options.height, RenderDarkFrame(fixedPattern, sensor, &random));
    }
    std::vector<std::string> starPaths;
    for (int k = 0; k < options.starImageCount; ++k) {
        starPaths.push_back((directory / NumberedName("star", k)).string());
        WritePgm(
            starPaths.back(),
            options.width,
            options.height,
            RenderStarImage(
                catalog, scene.trueAttitudes[k], camera, fixedPattern, options.width, options.height, sensor, &random));
        summary << "star image " << k << ": "
                << VisibleStars(catalog, scene.trueAttitudes[k], camera, options.width, options.height, 0).size()
                << " catalog stars on the sensor\n";
    }

    const std::string attitudesPath = (directory / "attitudes.txt").string();
    WriteAttitudes(attitudesPath,
                   scene.priorAttitudes,
                   "Reported attitudes from scope-synth (truth with errors), one per star image:\n"
                   "w x y z, inertial (J2000) to camera. The truth is in truth.txt.");

    std::ostringstream truth;
    truth << std::setprecision(kTruthPrecision);
    truth << "# The truth behind a scope-synth data set (seed " << options.seed << ").\n";
    for (int p = 0; p < kCameraParameterCount; ++p) {
        truth << kCameraParameterNames[p] << ": " << camera.*kCameraParameterFields[p] << "\n";
    }
    const found::Vec3 misalignment = CorrectionRotationVector(scene.misalignment);
    truth << "# The correction refined * reported^-1 that a fit should find, averaged over images, as a\n"
          << "# rotation vector in arcseconds (what scope prints as alignment-correction).\n";
    truth << "misalignment: " << found::RadToArcSec(misalignment.x()) << " " << found::RadToArcSec(misalignment.y())
          << " " << found::RadToArcSec(misalignment.z()) << "\n";
    truth << "# The true attitude of each star image: w x y z.\n";
    for (const found::Quaternion &attitude : scene.trueAttitudes) {
        truth << "true-attitude: " << attitude.w() << " " << attitude.x() << " " << attitude.y() << " " << attitude.z()
              << "\n";
    }
    WriteFile((directory / "truth.txt").string(), truth.str());

    const std::string arguments = "--dark-frames " + JoinPaths(darkPaths) + " --star-images " + JoinPaths(starPaths) +
                                  " --attitudes " + attitudesPath + " --catalog-path " + options.catalogPath;
    WriteFile((directory / "scope-args.txt").string(), arguments + "\n");

    summary << "Wrote " << options.darkFrameCount << " dark frames, " << options.starImageCount
            << " star images, attitudes.txt, truth.txt and scope-args.txt to " << options.outputDir << "\n";
    return summary.str();
}

int SynthMain(int argc, char **argv) {
    if (argc >= 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
        PrintHelp();
        return EXIT_SUCCESS;
    }
    try {
        std::cout << WriteSyntheticDataSet(ParseSynthesisOptions(argc, argv)) << std::flush;
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

}  // namespace scope
