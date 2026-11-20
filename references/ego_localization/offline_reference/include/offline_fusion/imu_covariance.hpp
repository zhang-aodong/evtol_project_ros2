#pragma once

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <cmath>
#include <stdexcept>
#include "offline_fusion/math.hpp"

namespace offline_fusion {
using ImuMatrix15 = Eigen::Matrix<double, 15, 15>;

inline Eigen::Matrix3d imu_skew(const Vec3& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z, v.y, v.z, 0, -v.x, -v.y, v.x, 0;
    return m;
}
inline Eigen::Matrix3d imu_rotation(const Quat& q) {
    Eigen::Matrix3d r;
    const Vec3 x = rotate(q, {1,0,0}), y = rotate(q, {0,1,0}), z = rotate(q, {0,0,1});
    r << x.x,y.x,z.x,x.y,y.y,z.y,x.z,y.z,z.z;
    return r;
}
inline ImuMatrix15 imu_transition(const ImuMatrix15& f, double dt) {
    const ImuMatrix15 a = f * dt;
    // Fourth-order exponential, used only with integration steps <= 5 ms.
    const ImuMatrix15 a2 = a*a;
    return ImuMatrix15::Identity() + a + 0.5*a2 + (a2*a)/6.0 + (a2*a2)/24.0;
}
inline ImuMatrix15 imu_error_dynamics(const Quat& q, const Vec3& accel, const Vec3& gyro) {
    ImuMatrix15 f = ImuMatrix15::Zero();
    const Eigen::Matrix3d r = imu_rotation(q);
    // Ordering: position, velocity, right attitude error, accel bias, gyro bias.
    f.block<3,3>(0,3).setIdentity();
    f.block<3,3>(3,6) = -r * imu_skew(accel);
    f.block<3,3>(3,9) = -r;
    f.block<3,3>(6,6) = -imu_skew(gyro);
    f.block<3,3>(6,12) = -Eigen::Matrix3d::Identity();
    return f;
}
inline void propagate_imu_covariance(ImuMatrix15& covariance, const ImuMatrix15& f,
                                      const ImuMatrix15& noise, double dt) {
    const ImuMatrix15 phi = imu_transition(f, dt);
    ImuMatrix15 discrete_noise = ImuMatrix15::Zero();
    // Positive quadrature of integral exp(F*t) Q exp(F*t)^T; unlike one
    // endpoint noise injection this retains position/velocity covariance.
    const double x[3] = {0.5-std::sqrt(15.0)/10, 0.5, 0.5+std::sqrt(15.0)/10};
    const double w[3] = {5.0/18, 4.0/9, 5.0/18};
    for (int i=0; i<3; ++i) {
        const ImuMatrix15 e = imu_transition(f, x[i]*dt);
        discrete_noise.noalias() += (dt*w[i]) * e * noise * e.transpose();
    }
    covariance = (phi * covariance * phi.transpose() + discrete_noise).eval();
    covariance = (0.5*(covariance + covariance.transpose())).eval();
}
inline ImuMatrix15 imu_sqrt_information(const ImuMatrix15& covariance) {
    if (!covariance.allFinite() || (covariance.diagonal().array() <= 0).any())
        throw std::runtime_error("Invalid IMU covariance diagonal");
    const Eigen::Matrix<double,15,1> inverse_std = covariance.diagonal().array().sqrt().inverse();
    const ImuMatrix15 correlation = inverse_std.asDiagonal() * covariance * inverse_std.asDiagonal();
    Eigen::LLT<ImuMatrix15> llt(correlation);
    if (llt.info() != Eigen::Success) throw std::runtime_error("IMU covariance is not positive definite");
    // W P W^T = I. Normalize first to avoid a numerical floor on short dt.
    return llt.matrixL().solve(ImuMatrix15::Identity()) * inverse_std.asDiagonal();
}
} // namespace offline_fusion
