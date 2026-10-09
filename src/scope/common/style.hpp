#ifndef SRC_SCOPE_COMMON_STYLE_HPP_
#define SRC_SCOPE_COMMON_STYLE_HPP_

#include <vector>

#include "common/pipeline/pipelines.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "common/style.hpp"

#include "scope/catalog/catalog.hpp"

namespace scope {

/** Single image from FOUND. */
typedef found::Image Image;

/** Images passed in by the user. */
typedef std::vector<found::Image> Images;

/**
 * A single measured star: a catalog star paired with where it was actually
 * observed (the raw, distorted pixel) in one of the star images.
 */
struct Observation {
    /// Index of the star image this observation came from.
    int image_index;
    /// Index into the run's catalog. Stable for the duration of one run.
    int catalog_index;
    /// The measured (distorted) centroid in pixel coordinates [u', v'].
    found::Vec2 measured_pixel;
};

/**
 * The star-centroid stage's output: the measurements, the per-image prior
 * attitudes (forwarded to LMA), and a non-owning view of the catalog.
 *
 * The catalog pointer is owned by the star-centroid stage and stays valid for
 * the whole pipeline run; it is null on a default-constructed value.
 */
struct CentroidObservations {
    /// All star measurements gathered across every star image.
    std::vector<Observation> observations;
    /// One prior attitude per star image, forwarded for the optimization stage.
    std::vector<found::Quaternion> attitudes;
    /// Non-owning pointer to the catalog the observations index into.
    const Catalog *catalog = nullptr;
};

/** Number of (maximum) stages for each pipeline. */
constexpr size_t recalibration_size = 3;

/** Pipeline for Recalibration. */
typedef found::SequentialPipeline<std::vector<found::Image>, std::vector<float>, recalibration_size>
    PrimaryScopePipeline;

}  // namespace scope

#endif  // SRC_SCOPE_COMMON_STYLE_HPP_
