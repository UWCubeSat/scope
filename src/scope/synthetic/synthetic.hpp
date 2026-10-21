/**
 * @file synthetic.hpp
 * @brief Synthetic star fields from a known camera and known attitudes, for
 * testing the pipeline end to end.
 *
 * Everything here goes through ProjectStarToPixel, so it shares the camera
 * model with stages 2 and 3. What it checks independently is the rest: the
 * dark frame, the pixel convention, centroiding, matching, and the fit.
 */

#ifndef SRC_SCOPE_SYNTHETIC_SYNTHETIC_HPP_
#define SRC_SCOPE_SYNTHETIC_SYNTHETIC_HPP_

#include <cstddef>

#include <random>
#include <string>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/catalog/catalog.hpp"
#include "scope/common/types.hpp"
#include "scope/projection/projection.hpp"

namespace scope {

/// The random number generator for synthetic data.
using Random = std::mt19937_64;

/**
 * Draws a rotation uniformly over all rotations.
 *
 * @param random The generator to draw from.
 *
 * @return A unit quaternion.
 */
found::Quaternion RandomAttitude(Random *random);

/**
 * Draws a small rotation as AttitudeCorrection(dtheta), with each component
 * of dtheta drawn from N(0, sigma^2).
 *
 * @param sigma Standard deviation per axis, in radians.
 * @param random The generator to draw from.
 *
 * @return A unit quaternion.
 */
found::Quaternion RandomSmallRotation(decimal sigma, Random *random);

/**
 * Draws a catalog of stars spread uniformly over the sky. Magnitudes follow
 * the Bright Star Catalog's rough law, the number of stars brighter than m
 * growing by a factor of 10^0.5 per magnitude, up to faintestMagnitude.
 *
 * @param count Number of stars.
 * @param faintestMagnitude The faintest magnitude drawn.
 * @param random The generator to draw from.
 *
 * @return The catalog, with HR numbers 1 to count.
 */
Catalog RandomCatalog(int count, decimal faintestMagnitude, Random *random);

/**
 * A set of star images: the camera that took them, where each was really
 * pointing, and the attitudes a star tracker or plate solver reported.
 *
 * The reported attitude is prior_k = E_k * misalignment^-1 * truth_k, with
 * E_k a per-image error, so the correction a fit should find for image k,
 * truth_k * prior_k^-1 = misalignment * E_k^-1, averages to the misalignment.
 */
struct SyntheticScene {
    /// The true camera parameters.
    CameraParameters camera;
    /// Sensor width, in pixels.
    int width = 0;
    /// Sensor height, in pixels.
    int height = 0;
    /// The true attitude of each image (inertial to camera).
    std::vector<found::Quaternion> trueAttitudes;
    /// The attitude reported for each image.
    std::vector<found::Quaternion> priorAttitudes;
    /// The rotation common to every image's attitude error.
    found::Quaternion misalignment = found::Quaternion::Identity();
};

/**
 * Draws a scene of images at uniformly random attitudes.
 *
 * @param camera The true camera parameters.
 * @param width,height Sensor size, in pixels.
 * @param imageCount Number of images.
 * @param attitudeNoise Per-axis standard deviation of each image's attitude
 *                      error E_k, in radians.
 * @param misalignment Per-axis standard deviation of the misalignment, drawn
 *                     once for the scene, in radians.
 * @param random The generator to draw from.
 *
 * @return The scene.
 */
SyntheticScene RandomScene(const CameraParameters &camera,
                           int width,
                           int height,
                           int imageCount,
                           decimal attitudeNoise,
                           decimal misalignment,
                           Random *random);

/// A catalog star and the pixel it lands on.
struct VisibleStar {
    /// Index into the catalog.
    std::size_t catalogIndex;
    /// The exact (distorted) pixel [u, v].
    found::Vec2 pixel;
};

/**
 * Lists the catalog stars that land on the sensor, less a margin.
 *
 * @param catalog The stars.
 * @param attitude The attitude to project with.
 * @param camera The camera to project with.
 * @param width,height Sensor size, in pixels.
 * @param margin Pixels to keep clear of every edge (see InSensorWithMargin).
 *
 * @return The visible stars, in catalog order.
 */
std::vector<VisibleStar> VisibleStars(const Catalog &catalog,
                                      const found::Quaternion &attitude,
                                      const CameraParameters &camera,
                                      int width,
                                      int height,
                                      int margin);

/**
 * Simulates stage 2's output for a scene: every catalog star that lands on
 * the sensor (less the margin) under an image's true attitude, measured at
 * its exact pixel plus N(0, centroidNoise^2) per axis, and the scene's prior
 * attitudes. This is the setting of the paper's Monte Carlo.
 *
 * @param scene The scene.
 * @param catalog The stars.
 * @param margin Pixels to keep clear of every edge.
 * @param centroidNoise Per-axis standard deviation of each centroid, in pixels.
 * @param random The generator to draw from.
 *
 * @return The observations, image by image, with the prior attitudes.
 */
CentroidObservations SimulateObservations(const SyntheticScene &scene,
                                          const Catalog &catalog,
                                          int margin,
                                          decimal centroidNoise,
                                          Random *random);

/**
 * How the synthetic sensor turns starlight into 8-bit pixel values. Levels
 * are in counts (8-bit digital numbers).
 */
struct SensorModel {
    /// Standard deviation of the Gaussian point spread function, in pixels.
    decimal psfSigma = DECIMAL(1.0);
    /// Total counts from a magnitude 0 star; magnitude m gives
    /// zeroPoint * 10^(-0.4 m). The default puts the detection limit for
    /// --centroid-threshold 40 near magnitude 5.5 and saturates stars
    /// brighter than about 3.5.
    decimal zeroPoint = DECIMAL(50000.0);
    /// Sky level, in star images only.
    decimal skyBackground = DECIMAL(2.0);
    /// The mean of the fixed pattern.
    decimal blackLevel = DECIMAL(8.0);
    /// Pixel-to-pixel standard deviation of the fixed pattern.
    decimal fixedPatternNoise = DECIMAL(1.0);
    /// Fraction of pixels that are hot, 50 to 200 counts above the rest.
    decimal hotPixelFraction = DECIMAL(1e-4);
    /// Read noise per pixel and frame.
    decimal readNoise = DECIMAL(1.0);
    /// Electrons per count, which sets the shot noise.
    decimal gain = DECIMAL(4.0);
};

/**
 * Draws the fixed pattern of one sensor: the black level, a per-pixel offset
 * and hot pixels. Every frame from the sensor shares it.
 *
 * @param width,height Sensor size, in pixels.
 * @param sensor The sensor model.
 * @param random The generator to draw from.
 *
 * @return The pattern, row-major, in counts.
 */
std::vector<decimal> RandomFixedPattern(int width, int height, const SensorModel &sensor, Random *random);

/**
 * Renders one 8-bit star image: each catalog star's light, spread by the
 * point spread function and integrated over each pixel (an integer
 * coordinate is a pixel centre, as in ExtractCentroid), on top of the sky and
 * the fixed pattern, with shot and read noise, rounded and clipped to 0-255.
 *
 * @param catalog The stars.
 * @param attitude The image's true attitude.
 * @param camera The true camera parameters.
 * @param fixedPattern The sensor's fixed pattern (RandomFixedPattern).
 * @param width,height Sensor size, in pixels.
 * @param sensor The sensor model.
 * @param random The generator to draw from.
 *
 * @return The pixels, row-major.
 */
std::vector<unsigned char> RenderStarImage(const Catalog &catalog,
                                           const found::Quaternion &attitude,
                                           const CameraParameters &camera,
                                           const std::vector<decimal> &fixedPattern,
                                           int width,
                                           int height,
                                           const SensorModel &sensor,
                                           Random *random);

/**
 * Renders one 8-bit dark frame, as taken with the lens capped: the fixed
 * pattern with read noise.
 *
 * @param fixedPattern The sensor's fixed pattern.
 * @param sensor The sensor model.
 * @param random The generator to draw from.
 *
 * @return The pixels, row-major.
 */
std::vector<unsigned char> RenderDarkFrame(const std::vector<decimal> &fixedPattern,
                                           const SensorModel &sensor,
                                           Random *random);

/**
 * Writes a single-channel 8-bit image as a binary PGM, which stb_image (and
 * so --dark-frames and --star-images) reads.
 *
 * @param path Path to the file to write.
 * @param width,height Image size, in pixels.
 * @param pixels The pixels, row-major.
 *
 * @throws std::runtime_error if the file cannot be written.
 */
void WritePgm(const std::string &path, int width, int height, const std::vector<unsigned char> &pixels);

}  // namespace scope

#endif  // SRC_SCOPE_SYNTHETIC_SYNTHETIC_HPP_
