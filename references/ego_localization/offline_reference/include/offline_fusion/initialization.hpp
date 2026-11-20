#pragma once

#include <cstddef>

#include "offline_fusion/fusion.hpp"
#include "offline_fusion/types.hpp"

namespace offline_fusion {

struct SensorInitialization {
    double reference_time = 0.0;
    Quat q_body_to_ned;
    Vec3 accel_bias;
    Vec3 gyro_bias;
    std::size_t gravity_samples = 0;
    double heading_time = 0.0;
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
};

SensorInitialization initialize_from_imu_and_gnss(const Dataset& dataset,
                                                   const FusionOptions& options);

Vec3 interpolate_gnss_position(const std::vector<GnssPositionFix>& fixes, double t);
Vec3 interpolate_gnss_velocity(const std::vector<GnssPositionFix>& fixes, double t);

Quat propagate_imu_orientation(const std::vector<ImuSample>& imu,
                               double start_time,
                               double end_time,
                               const Quat& start_orientation,
                               const Vec3& gyro_bias);

}  // namespace offline_fusion
