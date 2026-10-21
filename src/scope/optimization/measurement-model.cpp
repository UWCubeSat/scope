#include "scope/optimization/measurement-model.hpp"

#include <Eigen/Core>

#include "common/decimal.hpp"
#include "common/spatial/attitude-utils.hpp"

namespace scope {

MeasurementJacobian ComputeMeasurementJacobian(const found::Vec3 &eC, const CameraParameters &camera) {
    const decimal X = eC.x();
    const decimal Y = eC.y();
    const decimal Z = eC.z();
    const decimal k1 = camera.k1;
    const decimal k2 = camera.k2;
    const decimal k3 = camera.k3;
    const decimal p1 = camera.p1;
    const decimal p2 = camera.p2;

    // Ideal and distorted image-plane points (Eqs. 1 and 6).
    const decimal x = X / Z;
    const decimal y = Y / Z;
    const decimal r2 = x * x + y * y;
    const decimal r4 = r2 * r2;
    const decimal r6 = r4 * r2;
    const decimal radial = DECIMAL(1.0) + k1 * r2 + k2 * r4 + k3 * r6;
    const decimal xd = radial * x + DECIMAL(2.0) * p1 * x * y + p2 * (r2 + DECIMAL(2.0) * x * x);
    const decimal yd = radial * y + p1 * (r2 + DECIMAL(2.0) * y * y) + DECIMAL(2.0) * p2 * x * y;

    MeasurementJacobian jacobian;

    // Eq. 29: du'/dk.
    jacobian.camera.leftCols<5>() << xd, yd, DECIMAL(0.0), DECIMAL(1.0), DECIMAL(0.0),
                                     DECIMAL(0.0), DECIMAL(0.0), yd, DECIMAL(0.0), DECIMAL(1.0);

    // Eq. 32: du'/dx'.
    found::Mat2 dUdXd;
    dUdXd << camera.focalLengthX, camera.alpha,
             DECIMAL(0.0), camera.focalLengthY;

    // Eq. 33: dx'/dxi.
    Eigen::Matrix<decimal, 2, 5> dXdDxi;
    dXdDxi << x * r2, x * r4, x * r6, DECIMAL(2.0) * x * y, r2 + DECIMAL(2.0) * x * x,
              y * r2, y * r4, y * r6, r2 + DECIMAL(2.0) * y * y, DECIMAL(2.0) * x * y;

    // Eq. 30.
    jacobian.camera.rightCols<5>() = dUdXd * dXdDxi;

    // Eq. 37.
    const Eigen::Matrix<decimal, 1, 2> dR2(DECIMAL(2.0) * x, DECIMAL(2.0) * y);
    const Eigen::Matrix<decimal, 1, 2> dR4 = DECIMAL(2.0) * r2 * dR2;
    const Eigen::Matrix<decimal, 1, 2> dR6 = DECIMAL(3.0) * r4 * dR2;

    // Eq. 36: dx'/dx.
    const found::Vec2 ideal(x, y);
    found::Mat2 decentering;
    decentering << DECIMAL(2.0) * p1 * y + DECIMAL(4.0) * p2 * x, DECIMAL(2.0) * p1 * x,
                   DECIMAL(2.0) * p2 * y, DECIMAL(4.0) * p1 * y + DECIMAL(2.0) * p2 * x;
    const found::Mat2 dXdDx = radial * found::Mat2::Identity() + ideal * (k1 * dR2 + k2 * dR4 + k3 * dR6) +
                              decentering + found::Vec2(p2, p1) * dR2;

    // Eq. 34: dx/de.
    Eigen::Matrix<decimal, 2, 3> dXdE;
    dXdE << DECIMAL(1.0) / Z, DECIMAL(0.0), -X / (Z * Z),
            DECIMAL(0.0), DECIMAL(1.0) / Z, -Y / (Z * Z);

    // Eq. 35: de/d(dtheta) = [e_C x].
    found::Mat3 dEdTheta;
    dEdTheta << DECIMAL(0.0), -Z, Y,
                Z, DECIMAL(0.0), -X,
                -Y, X, DECIMAL(0.0);

    // Eq. 31.
    jacobian.attitude = dUdXd * dXdDx * dXdE * dEdTheta;

    return jacobian;
}

}  // namespace scope
