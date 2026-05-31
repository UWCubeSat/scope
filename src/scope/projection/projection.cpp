#include "scope/projection/projection.hpp"

#include <optional>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

namespace scope {

found::Vec2 BrownDistort(const found::Vec2 &ideal, decimal k1, decimal k2, decimal k3, decimal p1, decimal p2) {
    const decimal x = ideal.x();
    const decimal y = ideal.y();
    const decimal r2 = x * x + y * y;
    const decimal r4 = r2 * r2;
    const decimal r6 = r4 * r2;

    // Radial term: (1 + k1 r^2 + k2 r^4 + k3 r^6).
    const decimal radial = DECIMAL(1.0) + k1 * r2 + k2 * r4 + k3 * r6;

    // Decentering (tangential) term.
    const decimal dx = DECIMAL(2.0) * p1 * x * y + p2 * (r2 + DECIMAL(2.0) * x * x);
    const decimal dy = p1 * (r2 + DECIMAL(2.0) * y * y) + DECIMAL(2.0) * p2 * x * y;

    return found::Vec2{radial * x + dx, radial * y + dy};
}

found::Quaternion LostAttitudeToScopeFrame(const found::Quaternion &lostAttitude) {
    // Fixed rotation taking LOST's x-boresight camera frame into SCOPE's
    // z-boresight frame: (x, y, z)_lost -> (-y, -z, x)_scope. Derived in the
    // header from LOST's forward model; det = +1 (a proper rotation). Composing on
    // the left leaves the downstream e_C = attitude * e_I machinery unchanged.
    found::Mat3 framePermutation;
    framePermutation << DECIMAL(0.0), DECIMAL(-1.0), DECIMAL(0.0),
                        DECIMAL(0.0), DECIMAL(0.0), DECIMAL(-1.0),
                        DECIMAL(1.0), DECIMAL(0.0), DECIMAL(0.0);
    return found::Quaternion(framePermutation) * lostAttitude;
}

std::optional<found::Vec2> ProjectStarToPixel(const found::Vec3 &eI,
                                              const found::Quaternion &attitude,
                                              const RecalibrationOptions &options) {
    // Rotate the inertial line of sight into the camera frame (z is the boresight).
    const found::Vec3 eC = (attitude * eI).normalized();

    // Reject stars at or behind the image plane (z <= 0): they cannot be imaged,
    // and dividing by a non-positive z would fold a mirror image of the rear
    // hemisphere onto the sensor -- e.g. a star on the anti-boresight maps
    // straight to the principal point. InSensorWithMargin alone cannot catch this
    // because the bogus pixel can land well inside the frame.
    if (eC.z() <= DECIMAL(0.0)) {
        return std::nullopt;
    }

    // Pinhole projection onto the normalized image plane (paper Eq. 1, with the
    // focal length absorbed into the intrinsics below).
    const found::Vec2 ideal{eC.x() / eC.z(), eC.y() / eC.z()};

    // Brown distortion (paper Eq. 6).
    const found::Vec2 distorted = BrownDistort(ideal, options.k1, options.k2, options.k3, options.p1, options.p2);

    // Camera intrinsics (paper Eq. 7): [u'; v'] = [d_x, alpha, u_p; 0, d_y, v_p] [x'; y'; 1].
    const decimal u = options.focalLengthX * distorted.x() + options.alpha * distorted.y() + options.principalX;
    const decimal v = options.focalLengthY * distorted.y() + options.principalY;

    return found::Vec2{u, v};
}

bool InSensorWithMargin(const found::Vec2 &pixel, int width, int height, int margin) {
    const decimal lo = DECIMAL(margin);
    return pixel.x() >= lo && pixel.x() < DECIMAL(width - margin) && pixel.y() >= lo &&
           pixel.y() < DECIMAL(height - margin);
}

}  // namespace scope
