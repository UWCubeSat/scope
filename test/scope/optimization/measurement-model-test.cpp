#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <vector>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

#include "scope/attitude/attitude.hpp"
#include "scope/optimization/measurement-model.hpp"
#include "scope/projection/projection.hpp"

#include "test/scope/common/cameras.hpp"

namespace scope {

namespace {

/// Camera-frame directions across the 6 mm lens's field, out to the corners.
std::vector<found::Vec3> FieldDirections() {
    std::vector<found::Vec3> directions;
    for (const decimal x : {DECIMAL(-0.7), DECIMAL(-0.3), DECIMAL(0.0), DECIMAL(0.35), DECIMAL(0.72)}) {
        for (const decimal y : {DECIMAL(-0.5), DECIMAL(0.0), DECIMAL(0.45)}) {
            directions.push_back(found::Vec3(x, y, DECIMAL(1.0)).normalized());
        }
    }
    return directions;
}

/// An attitude that is not the identity, so the frames cannot be confused.
found::Quaternion TiltedAttitude() {
    const found::Vec3 axis = found::Vec3(DECIMAL(1.0), DECIMAL(-2.0), DECIMAL(0.5)).normalized();
    return found::Quaternion(found::AngleAxis(DECIMAL(0.8), axis));
}

found::Vec2 Project(const found::Vec3 &eI, const found::Quaternion &attitude, const CameraParameters &camera) {
    const std::optional<found::Vec2> pixel = ProjectStarToPixel(eI, attitude, camera);
    EXPECT_TRUE(pixel.has_value());
    return pixel.value_or(found::Vec2::Zero());
}

/// Agreement to a part in 10^6 of the larger of 1 and the derivative.
void ExpectDerivative(decimal analytic, decimal numeric) {
    EXPECT_NEAR(analytic, numeric, DECIMAL(1e-6) * std::max(DECIMAL(1.0), DECIMAL_ABS(numeric)));
}

}  // namespace

// The paper checked its partials "via forward finite differencing"; this
// checks each camera column against a central difference of the forward model.
TEST(MeasurementJacobianTest, CameraColumnsMatchFiniteDifferences) {
    const CameraParameters camera = AsBuiltCamera();
    const found::Quaternion attitude = TiltedAttitude();

    for (const found::Vec3 &eC : FieldDirections()) {
        const found::Vec3 eI = attitude.conjugate() * eC;
        const MeasurementJacobian jacobian = ComputeMeasurementJacobian(eC, camera);

        for (int p = 0; p < kCameraParameterCount; ++p) {
            const decimal step = DECIMAL(1e-6) * std::max(DECIMAL(1.0), DECIMAL_ABS(camera.*kCameraParameterFields[p]));
            CameraParameters plus = camera;
            CameraParameters minus = camera;
            plus.*kCameraParameterFields[p] += step;
            minus.*kCameraParameterFields[p] -= step;
            const found::Vec2 numeric =
                (Project(eI, attitude, plus) - Project(eI, attitude, minus)) / (DECIMAL(2.0) * step);

            SCOPED_TRACE(kCameraParameterNames[p]);
            ExpectDerivative(jacobian.camera(0, p), numeric.x());
            ExpectDerivative(jacobian.camera(1, p), numeric.y());
        }
    }
}

// The attitude columns are for a correction applied as
// AttitudeCorrection(dtheta) * attitude.
TEST(MeasurementJacobianTest, AttitudeColumnsMatchFiniteDifferences) {
    const CameraParameters camera = AsBuiltCamera();
    const found::Quaternion attitude = TiltedAttitude();
    const decimal step = DECIMAL(1e-6);

    for (const found::Vec3 &eC : FieldDirections()) {
        const found::Vec3 eI = attitude.conjugate() * eC;
        const MeasurementJacobian jacobian = ComputeMeasurementJacobian(eC, camera);

        for (int a = 0; a < 3; ++a) {
            const found::Vec3 dtheta = step * found::Vec3::Unit(a);
            const found::Quaternion plus = AttitudeCorrection(dtheta) * attitude;
            const found::Quaternion minus = AttitudeCorrection(-dtheta) * attitude;
            const found::Vec2 numeric =
                (Project(eI, plus, camera) - Project(eI, minus, camera)) / (DECIMAL(2.0) * step);

            ExpectDerivative(jacobian.attitude(0, a), numeric.x());
            ExpectDerivative(jacobian.attitude(1, a), numeric.y());
        }
    }
}

// Without distortion or skew the attitude columns reduce to the paper's own
// first-order expansion, Eqs. 45 and 46, which pins the sign convention of
// dtheta to the paper's.
TEST(MeasurementJacobianTest, PinholeAttitudeColumnsMatchPaperEq45And46) {
    CameraParameters camera;
    camera.focalLengthX = DECIMAL(1745.0);
    camera.focalLengthY = DECIMAL(1744.2);
    camera.principalX = DECIMAL(1031.5);
    camera.principalY = DECIMAL(771.5);

    for (const found::Vec3 &e : FieldDirections()) {
        const MeasurementJacobian jacobian = ComputeMeasurementJacobian(e, camera);
        const decimal e1 = e.x() / e.z();
        const decimal e2 = e.y() / e.z();
        const decimal dx = camera.focalLengthX;
        const decimal dy = camera.focalLengthY;

        // Eq. 45: du = [-d_x dth2] + d_x [e1 e2 dth1 - e1^2 dth2] + d_x e2 dth3.
        EXPECT_NEAR(jacobian.attitude(0, 0), dx * e1 * e2, DECIMAL(1e-9) * dx);
        EXPECT_NEAR(jacobian.attitude(0, 1), -dx - dx * e1 * e1, DECIMAL(1e-9) * dx);
        EXPECT_NEAR(jacobian.attitude(0, 2), dx * e2, DECIMAL(1e-9) * dx);
        // Eq. 46: dv = [d_y dth1] + d_y [e2^2 dth1 - e1 e2 dth2] - d_y e1 dth3.
        EXPECT_NEAR(jacobian.attitude(1, 0), dy + dy * e2 * e2, DECIMAL(1e-9) * dy);
        EXPECT_NEAR(jacobian.attitude(1, 1), -dy * e1 * e2, DECIMAL(1e-9) * dy);
        EXPECT_NEAR(jacobian.attitude(1, 2), -dy * e1, DECIMAL(1e-9) * dy);
    }
}

// Eq. 29 is linear in the intrinsics: u' = d_x x' + alpha y' + u_p and
// v' = d_y y' + v_p, so the principal point columns are exactly unit vectors.
TEST(MeasurementJacobianTest, PrincipalPointColumnsAreUnitVectors) {
    const MeasurementJacobian jacobian = ComputeMeasurementJacobian(FieldDirections()[4], AsBuiltCamera());

    EXPECT_EQ(jacobian.camera(0, 3), DECIMAL(1.0));
    EXPECT_EQ(jacobian.camera(1, 3), DECIMAL(0.0));
    EXPECT_EQ(jacobian.camera(0, 4), DECIMAL(0.0));
    EXPECT_EQ(jacobian.camera(1, 4), DECIMAL(1.0));
}

}  // namespace scope
