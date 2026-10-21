#include "scope/projection/projection.hpp"

#include <cmath>

#include <limits>
#include <optional>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

namespace scope {

namespace {

/// Whether radial distortion maps ideal radius to distorted radius one-to-one
/// out to r^2, i.e. whether r * (1 + k1 r^2 + k2 r^4 + k3 r^6) increases on
/// [0, r]. Past the first turning point the polynomial folds back, and stars
/// from far outside the field would land on the sensor.
bool RadialDistortionIsMonotonic(decimal r2, decimal k1, decimal k2, decimal k3) {
    // The derivative with respect to r, as a cubic in s = r^2. It is 1 at
    // s = 0, so it stays positive up to r2 iff it is positive at r2 and at
    // every interior minimum.
    const auto slope = [&](decimal s) {
        return DECIMAL(1.0) + DECIMAL(3.0) * k1 * s + DECIMAL(5.0) * k2 * s * s + DECIMAL(7.0) * k3 * s * s * s;
    };
    if (slope(r2) <= DECIMAL(0.0)) {
        return false;
    }

    // Turning points of the slope: a s^2 + b s + c = 0, solved in the form
    // that stays accurate when a is small.
    const decimal a = DECIMAL(21.0) * k3;
    const decimal b = DECIMAL(10.0) * k2;
    const decimal c = DECIMAL(3.0) * k1;
    const decimal discriminant = b * b - DECIMAL(4.0) * a * c;
    if (discriminant < DECIMAL(0.0)) {
        return true;
    }
    const decimal q = DECIMAL(-0.5) * (b + std::copysign(DECIMAL_SQRT(discriminant), b));
    for (const decimal s : {a != DECIMAL(0.0) ? q / a : DECIMAL(-1.0), q != DECIMAL(0.0) ? c / q : DECIMAL(-1.0)}) {
        if (s > DECIMAL(0.0) && s < r2 && slope(s) <= DECIMAL(0.0)) {
            return false;
        }
    }
    return true;
}

}  // namespace

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

decimal DistortionFoldRadiusSquared(const CameraParameters &camera) {
    // tan(89.99 degrees)^2: nothing farther off axis is worth a fold.
    constexpr decimal kFarthest = DECIMAL(3.3e7);
    if (RadialDistortionIsMonotonic(kFarthest, camera.k1, camera.k2, camera.k3)) {
        return std::numeric_limits<decimal>::infinity();
    }
    // Monotonic up to r^2 is true below the fold and false past it, so bisect.
    decimal below = DECIMAL(0.0);
    decimal above = kFarthest;
    for (int i = 0; i < 200 && above - below > std::numeric_limits<decimal>::epsilon() * above; ++i) {
        const decimal middle = DECIMAL(0.5) * (below + above);
        if (RadialDistortionIsMonotonic(middle, camera.k1, camera.k2, camera.k3)) {
            below = middle;
        } else {
            above = middle;
        }
    }
    return above;
}

std::optional<found::Vec2> ProjectStarToPixel(const found::Vec3 &eI,
                                              const found::Quaternion &attitude,
                                              const CameraParameters &camera) {
    // A rotation keeps eI's length, and x/z, y/z do not depend on it anyway.
    const found::Vec3 eC = attitude * eI;

    // Dividing by z <= 0 would mirror the rear hemisphere onto the sensor (the
    // anti-boresight lands on the principal point), which InSensorWithMargin
    // cannot catch.
    if (eC.z() <= DECIMAL(0.0)) {
        return std::nullopt;
    }

    const found::Vec2 ideal{eC.x() / eC.z(), eC.y() / eC.z()};
    if (!RadialDistortionIsMonotonic(ideal.squaredNorm(), camera.k1, camera.k2, camera.k3)) {
        return std::nullopt;
    }
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
