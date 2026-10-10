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

    const decimal radial = DECIMAL(1.0) + k1 * r2 + k2 * r4 + k3 * r6;

    // Decentering (tangential) terms.
    const decimal dx = DECIMAL(2.0) * p1 * x * y + p2 * (r2 + DECIMAL(2.0) * x * x);
    const decimal dy = p1 * (r2 + DECIMAL(2.0) * y * y) + DECIMAL(2.0) * p2 * x * y;

    return found::Vec2{radial * x + dx, radial * y + dy};
}

found::Quaternion LostAttitudeToScopeFrame(const found::Quaternion &lostAttitude) {
    // (x, y, z)_lost -> (-y, -z, x)_scope; see the header.
    found::Mat3 framePermutation;
    framePermutation << DECIMAL(0.0), DECIMAL(-1.0), DECIMAL(0.0),
                        DECIMAL(0.0), DECIMAL(0.0), DECIMAL(-1.0),
                        DECIMAL(1.0), DECIMAL(0.0), DECIMAL(0.0);
    return found::Quaternion(framePermutation) * lostAttitude;
}

std::optional<found::Vec2> ProjectStarToPixel(const found::Vec3 &eI,
                                              const found::Quaternion &attitude,
                                              const CameraParameters &camera) {
    const found::Vec3 eC = (attitude * eI).normalized();

    // Dividing by z <= 0 would mirror the rear hemisphere onto the sensor (the
    // anti-boresight lands on the principal point), which InSensorWithMargin
    // cannot catch.
    if (eC.z() <= DECIMAL(0.0)) {
        return std::nullopt;
    }

    const found::Vec2 ideal{eC.x() / eC.z(), eC.y() / eC.z()};
    const found::Vec2 distorted = BrownDistort(ideal, camera.k1, camera.k2, camera.k3, camera.p1, camera.p2);

    // Paper Eq. 7: [u'; v'] = [d_x, alpha, u_p; 0, d_y, v_p] [x'; y'; 1].
    const decimal u = camera.focalLengthX * distorted.x() + camera.alpha * distorted.y() + camera.principalX;
    const decimal v = camera.focalLengthY * distorted.y() + camera.principalY;

    return found::Vec2{u, v};
}

bool InSensorWithMargin(const found::Vec2 &pixel, int width, int height, int margin) {
    const decimal lo = DECIMAL(margin);
    return pixel.x() >= lo && pixel.x() < DECIMAL(width - margin) && pixel.y() >= lo &&
           pixel.y() < DECIMAL(height - margin);
}

}  // namespace scope
