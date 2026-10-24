/**
 * @file evaluation.hpp
 * @brief Checking a calibration against star images it was not fitted to.
 *
 * The Orion paper fits its calibration to 15 images and judges it by the star
 * residuals in the remaining 344 (Christian et al. 2016, §"Tests with a Real
 * Camera", Fig. 7). Each held-out image still has an attitude error of its
 * own, so its attitude is refined first, with the camera held fixed; what is
 * left over is the calibration's error.
 */

#ifndef SRC_SCOPE_EVALUATION_EVALUATION_HPP_
#define SRC_SCOPE_EVALUATION_EVALUATION_HPP_

#include <cstddef>

#include <optional>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/common/types.hpp"
#include "scope/projection/projection.hpp"

namespace scope {

/// Fewest stars an image needs to be evaluated. Two stars already fix an
/// attitude, but leave a single degree of freedom in four residuals, which
/// would make the residuals look smaller than the calibration's error.
constexpr std::size_t kMinStarsPerHoldoutImage = 3;

/// Default iteration limit for refining one image's attitude. Gauss-Newton
/// converges in a few steps from any attitude a star tracker would give.
constexpr int kDefaultHoldoutIterations = 20;

/**
 * Summary statistics of residuals, pooling the u and v components as the
 * paper's Fig. 7 does.
 */
struct ResidualStatistics {
    /// Number of residuals (stars); twice as many components are pooled.
    std::size_t count = 0;
    /// Mean of the components, in pixels.
    decimal mean = DECIMAL(0.0);
    /// Population standard deviation of the components, in pixels.
    decimal standardDeviation = DECIMAL(0.0);
    /// Skewness of the components (zero for a symmetric distribution).
    decimal skewness = DECIMAL(0.0);
    /// Kurtosis of the components, not in excess (3 for a normal distribution).
    decimal kurtosis = DECIMAL(0.0);
};

/**
 * Computes the statistics of a set of residuals.
 *
 * @param residuals Residuals [du, dv], in pixels.
 *
 * @return The statistics; all zeros if residuals is empty, and zero skewness
 *         and kurtosis if every component is the same.
 */
ResidualStatistics ComputeResidualStatistics(const std::vector<found::Vec2> &residuals);

/**
 * The result of evaluating a calibration on held-out images.
 */
struct HoldoutEvaluation {
    /// Measured minus predicted pixel [du, dv] for each observation, in input
    /// order; std::nullopt for observations in an image that was not evaluated.
    std::vector<std::optional<found::Vec2>> residuals;
    /// The refined attitude of each image, in the prior attitudes' order and
    /// frame; std::nullopt for an image that was not evaluated.
    std::vector<std::optional<found::Quaternion>> attitudes;
    /// Statistics of the residuals of every evaluated image together.
    ResidualStatistics statistics;
};

/**
 * Evaluates a calibration on star images it was not fitted to.
 *
 * Each image's prior attitude is refined by Gauss-Newton, with the camera
 * fixed, to minimize that image's residuals. The attitude is corrected as in
 * the paper's Eq. 11, by exp(-[dtheta x]) * attitude.
 *
 * An image is not evaluated if it has fewer than kMinStarsPerHoldoutImage
 * stars, if its stars do not fix all three axes of its attitude, if one of its
 * stars projects to no pixel (at or behind the image plane), or if the
 * refinement does not converge within maxIterations.
 *
 * @param observations Star centroids and prior attitudes for the held-out
 *                     images, as the star-centroid stage produces them.
 * @param camera The calibration to evaluate.
 * @param maxIterations The most Gauss-Newton steps per image.
 *
 * @return The residuals, refined attitudes and statistics.
 *
 * @throws std::invalid_argument if an observation refers to an image with no
 *         attitude.
 */
HoldoutEvaluation EvaluateHoldout(const CentroidObservations &observations,
                                  const CameraParameters &camera,
                                  int maxIterations = kDefaultHoldoutIterations);

}  // namespace scope

#endif  // SRC_SCOPE_EVALUATION_EVALUATION_HPP_
