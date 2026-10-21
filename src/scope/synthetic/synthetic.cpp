#include "scope/synthetic/synthetic.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <optional>
#include <random>
#include <string>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/attitude/attitude.hpp"
#include "scope/common/files.hpp"
#include "scope/projection/projection.hpp"

namespace scope {

namespace {

/// The brightest magnitude RandomCatalog draws, about Sirius's.
constexpr decimal kBrightestMagnitude = DECIMAL(-1.5);
/// The point spread function is rendered out to this many sigma.
constexpr decimal kPsfExtent = DECIMAL(5.0);
/// Hot pixels sit this many counts above the rest of the fixed pattern.
constexpr decimal kHotPixelMin = DECIMAL(50.0);
constexpr decimal kHotPixelMax = DECIMAL(200.0);

/// A unit vector drawn uniformly over the sphere.
found::Vec3 RandomDirection(Random *random) {
    std::normal_distribution<decimal> normal;
    return found::Vec3(normal(*random), normal(*random), normal(*random)).normalized();
}

/// The standard normal cumulative distribution function.
decimal NormalCdf(decimal t) {
    return DECIMAL(0.5) * DECIMAL(std::erfc(-t / DECIMAL_M_SQRT2));
}

/// Rounds and clips a level to an 8-bit pixel value.
unsigned char ToPixel(decimal level) {
    return static_cast<unsigned char>(std::clamp(DECIMAL_ROUND(level), DECIMAL(0.0), DECIMAL(255.0)));
}

}  // namespace

found::Quaternion RandomAttitude(Random *random) {
    // A normalized 4-D Gaussian is uniform on the unit 3-sphere, which is a
    // uniform rotation.
    std::normal_distribution<decimal> normal;
    return found::Quaternion(normal(*random), normal(*random), normal(*random), normal(*random)).normalized();
}

found::Quaternion RandomSmallRotation(decimal sigma, Random *random) {
    std::normal_distribution<decimal> normal;
    const found::Vec3 dtheta(normal(*random), normal(*random), normal(*random));
    return AttitudeCorrection(sigma * dtheta);
}

Catalog RandomCatalog(int count, decimal faintestMagnitude, Random *random) {
    // Inverting N(< m) proportional to 10^(0.5 m): m = faintest + 2 log10(u).
    std::uniform_real_distribution<decimal> uniform(DECIMAL(0.0), DECIMAL(1.0));
    Catalog catalog;
    for (int i = 0; i < count; ++i) {
        const found::Vec3 direction = RandomDirection(random);
        const decimal u = DECIMAL(1.0) - uniform(*random);  // in (0, 1]
        const decimal magnitude =
            std::max(faintestMagnitude + DECIMAL(2.0) * DECIMAL(std::log10(u)), kBrightestMagnitude);
        catalog.push_back(CatalogStar{direction, static_cast<int>(DECIMAL_ROUND(magnitude * DECIMAL(100.0))), i + 1});
    }
    return catalog;
}

SyntheticScene RandomScene(const CameraParameters &camera,
                           int width,
                           int height,
                           int imageCount,
                           decimal attitudeNoise,
                           decimal misalignment,
                           Random *random) {
    SyntheticScene scene;
    scene.camera = camera;
    scene.width = width;
    scene.height = height;
    scene.misalignment = RandomSmallRotation(misalignment, random);
    for (int k = 0; k < imageCount; ++k) {
        const found::Quaternion truth = RandomAttitude(random);
        const found::Quaternion error = RandomSmallRotation(attitudeNoise, random);
        scene.trueAttitudes.push_back(truth);
        scene.priorAttitudes.push_back((error * scene.misalignment.conjugate() * truth).normalized());
    }
    return scene;
}

std::vector<VisibleStar> VisibleStars(const Catalog &catalog,
                                      const found::Quaternion &attitude,
                                      const CameraParameters &camera,
                                      int width,
                                      int height,
                                      int margin) {
    std::vector<VisibleStar> visible;
    for (std::size_t i = 0; i < catalog.size(); ++i) {
        const std::optional<found::Vec2> pixel = ProjectStarToPixel(catalog[i].spatial, attitude, camera);
        if (pixel.has_value() && InSensorWithMargin(*pixel, width, height, margin)) {
            visible.push_back(VisibleStar{i, *pixel});
        }
    }
    return visible;
}

CentroidObservations SimulateObservations(
    const SyntheticScene &scene, const Catalog &catalog, int margin, decimal centroidNoise, Random *random) {
    std::normal_distribution<decimal> normal;
    CentroidObservations observations;
    observations.attitudes = scene.priorAttitudes;
    for (std::size_t k = 0; k < scene.trueAttitudes.size(); ++k) {
        for (const VisibleStar &star :
             VisibleStars(catalog, scene.trueAttitudes[k], scene.camera, scene.width, scene.height, margin)) {
            const found::Vec2 measured = star.pixel + centroidNoise * found::Vec2(normal(*random), normal(*random));
            observations.observations.push_back(Observation{
                static_cast<int>(k), catalog[star.catalogIndex].name, catalog[star.catalogIndex].spatial, measured});
        }
    }
    return observations;
}

std::vector<decimal> RandomFixedPattern(int width, int height, const SensorModel &sensor, Random *random) {
    std::normal_distribution<decimal> normal;
    std::uniform_real_distribution<decimal> uniform(DECIMAL(0.0), DECIMAL(1.0));
    std::uniform_real_distribution<decimal> hot(kHotPixelMin, kHotPixelMax);
    std::vector<decimal> pattern(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
    for (decimal &level : pattern) {
        level = sensor.blackLevel + sensor.fixedPatternNoise * normal(*random);
        if (uniform(*random) < sensor.hotPixelFraction) {
            level += hot(*random);
        }
    }
    return pattern;
}

std::vector<unsigned char> RenderStarImage(const Catalog &catalog,
                                           const found::Quaternion &attitude,
                                           const CameraParameters &camera,
                                           const std::vector<decimal> &fixedPattern,
                                           int width,
                                           int height,
                                           const SensorModel &sensor,
                                           Random *random) {
    const std::size_t pixelCount = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    std::vector<decimal> light(pixelCount, sensor.skyBackground);

    // A star just off the sensor still lights its edge.
    const int radius = static_cast<int>(DECIMAL_CEIL(kPsfExtent * sensor.psfSigma));
    const int stampSize = 2 * radius + 1;
    std::vector<decimal> weightU(static_cast<std::size_t>(stampSize));
    std::vector<decimal> weightV(static_cast<std::size_t>(stampSize));
    for (const VisibleStar &star : VisibleStars(catalog, attitude, camera, width, height, -radius)) {
        const decimal u = star.pixel.x();
        const decimal v = star.pixel.y();
        const int centerU = static_cast<int>(DECIMAL_ROUND(u));
        const int centerV = static_cast<int>(DECIMAL_ROUND(v));

        // The Gaussian separates, so each pixel's share of the light is the
        // product of its column's and its row's.
        for (int s = 0; s < stampSize; ++s) {
            const decimal column = DECIMAL(centerU - radius + s);
            const decimal row = DECIMAL(centerV - radius + s);
            weightU[s] = NormalCdf((column + DECIMAL(0.5) - u) / sensor.psfSigma) -
                         NormalCdf((column - DECIMAL(0.5) - u) / sensor.psfSigma);
            weightV[s] = NormalCdf((row + DECIMAL(0.5) - v) / sensor.psfSigma) -
                         NormalCdf((row - DECIMAL(0.5) - v) / sensor.psfSigma);
        }

        const decimal magnitude = DECIMAL(catalog[star.catalogIndex].magnitude) / DECIMAL(100.0);
        const decimal flux = sensor.zeroPoint * DECIMAL_POW(DECIMAL(10.0), DECIMAL(-0.4) * magnitude);
        for (int sv = 0; sv < stampSize; ++sv) {
            const int y = centerV - radius + sv;
            if (y < 0 || y >= height) {
                continue;
            }
            for (int su = 0; su < stampSize; ++su) {
                const int x = centerU - radius + su;
                if (x < 0 || x >= width) {
                    continue;
                }
                light[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x)] +=
                    flux * weightU[su] * weightV[sv];
            }
        }
    }

    // Shot noise in electrons is Poisson, which at these levels is close to
    // Gaussian with variance (light * gain) e^2, i.e. light / gain counts^2.
    std::normal_distribution<decimal> normal;
    const decimal readVariance = sensor.readNoise * sensor.readNoise;
    std::vector<unsigned char> pixels(pixelCount);
    for (std::size_t i = 0; i < pixelCount; ++i) {
        const decimal sigma = DECIMAL_SQRT(std::max(light[i], DECIMAL(0.0)) / sensor.gain + readVariance);
        pixels[i] = ToPixel(fixedPattern[i] + light[i] + sigma * normal(*random));
    }
    return pixels;
}

std::vector<unsigned char> RenderDarkFrame(const std::vector<decimal> &fixedPattern,
                                           const SensorModel &sensor,
                                           Random *random) {
    std::normal_distribution<decimal> normal;
    std::vector<unsigned char> pixels(fixedPattern.size());
    for (std::size_t i = 0; i < fixedPattern.size(); ++i) {
        pixels[i] = ToPixel(fixedPattern[i] + sensor.readNoise * normal(*random));
    }
    return pixels;
}

void WritePgm(const std::string &path, int width, int height, const std::vector<unsigned char> &pixels) {
    const std::string header = "P5\n" + std::to_string(width) + " " + std::to_string(height) + "\n255\n";
    WriteFile(path, header + std::string(pixels.begin(), pixels.end()));
}

}  // namespace scope
