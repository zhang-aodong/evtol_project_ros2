#include "offline_fusion/initialization.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <vector>

#include "offline_fusion/math.hpp"

namespace offline_fusion {
namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
}

double wrap_angle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

Quat quaternion_from_rpy(double roll, double pitch, double yaw) {
    const double cr = std::cos(0.5 * roll);
    const double sr = std::sin(0.5 * roll);
    const double cp = std::cos(0.5 * pitch);
    const double sp = std::sin(0.5 * pitch);
    const double cy = std::cos(0.5 * yaw);
    const double sy = std::sin(0.5 * yaw);
    return quat_normalized({
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr,
    });
}

Vec3 quaternion_to_rpy(const Quat& input) {
    const Quat q = quat_normalized(input);
    const double roll = std::atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    const double pitch = std::asin(clamp(
        2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
    const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return {roll, pitch, yaw};
}

template <typename Sample, typename ValueAccessor>
Vec3 interpolate_vector(const std::vector<Sample>& samples,
                        double t,
                        ValueAccessor accessor) {
    if (samples.empty()) {
        throw std::runtime_error("Cannot interpolate an empty sensor vector");
    }
    if (t <= samples.front().t) {
        return accessor(samples.front());
    }
    if (t >= samples.back().t) {
        return accessor(samples.back());
    }
    const auto upper = std::lower_bound(
        samples.begin(), samples.end(), t,
        [](const Sample& sample, double query_time) { return sample.t < query_time; });
    const auto& b = *upper;
    const auto& a = *(upper - 1);
    const double alpha = (t - a.t) / (b.t - a.t);
    return lerp(accessor(a), accessor(b), alpha);
}

}  // namespace

Quat propagate_imu_orientation(const std::vector<ImuSample>& imu,
                               double start_time,
                               double end_time,
                               const Quat& start_orientation,
                               const Vec3& gyro_bias) {
    if (end_time < start_time) {
        throw std::runtime_error("Backward IMU attitude propagation is not supported");
    }
    Quat orientation = start_orientation;
    for (std::size_t i = 1; i < imu.size(); ++i) {
        const auto& previous = imu[i - 1];
        const auto& current = imu[i];
        if (current.t <= start_time || previous.t >= end_time) {
            continue;
        }
        const double interval_start = std::max(previous.t, start_time);
        const double interval_end = std::min(current.t, end_time);
        const double dt = interval_end - interval_start;
        if (dt <= 0.0 || dt > 0.1) {
            continue;
        }
        const Vec3 angular_rate = previous.gyro - gyro_bias;
        orientation = quat_normalized(
            quat_multiply(orientation, quat_from_delta_angle(angular_rate * dt)));
    }
    return orientation;
}

SensorInitialization initialize_from_imu_and_gnss(const Dataset& dataset,
                                                   const FusionOptions& options) {
    if (dataset.imu.size() < 2) {
        throw std::runtime_error("IMU initialization requires at least two samples");
    }
    if (dataset.gnss_position_fixes.empty()) {
        throw std::runtime_error("Heading initialization requires SPP-like GNSS positions");
    }

    SensorInitialization result;
    result.reference_time = dataset.imu.front().t;
    Vec3 accel_sum;
    Vec3 gyro_sum;
    for (const auto& sample : dataset.imu) {
        if (sample.t > result.reference_time + options.initialization_gravity_window_s) {
            break;
        }
        const double acceleration_error = std::abs(norm(sample.accel) - options.gravity_mps2);
        if (acceleration_error > options.initialization_gravity_tolerance_mps2 ||
            norm(sample.gyro) > options.initialization_max_gyro_rps) {
            continue;
        }
        accel_sum = accel_sum + sample.accel;
        gyro_sum = gyro_sum + sample.gyro;
        ++result.gravity_samples;
    }
    if (result.gravity_samples < 10) {
        throw std::runtime_error(
            "Insufficient gravity-consistent IMU samples for roll/pitch initialization");
    }

    const Vec3 mean_accel = accel_sum / static_cast<double>(result.gravity_samples);
    result.gyro_bias = gyro_sum / static_cast<double>(result.gravity_samples);
    const double roll = std::atan2(-mean_accel.y, -mean_accel.z);
    const double pitch = std::atan2(
        mean_accel.x,
        std::sqrt(mean_accel.y * mean_accel.y + mean_accel.z * mean_accel.z));
    const Quat tilt_orientation = quaternion_from_rpy(roll, pitch, 0.0);

    double heading_sin_sum = 0.0;
    double heading_cos_sum = 0.0;
    double heading_time_sum = 0.0;
    double heading_weight_sum = 0.0;
    double first_heading_time = -1.0;
    for (const auto& fix : dataset.gnss_position_fixes) {
        if (!fix.valid) {
            continue;
        }
        const double horizontal_speed =
            std::hypot(fix.velocity_ned.x, fix.velocity_ned.y);
        if (horizontal_speed < options.initialization_heading_speed_mps) {
            continue;
        }
        if (first_heading_time < 0.0) {
            first_heading_time = fix.t;
        }
        if (fix.t > first_heading_time + 5.0) {
            break;
        }
        const double heading = std::atan2(fix.velocity_ned.y, fix.velocity_ned.x);
        heading_sin_sum += horizontal_speed * std::sin(heading);
        heading_cos_sum += horizontal_speed * std::cos(heading);
        heading_time_sum += horizontal_speed * fix.t;
        heading_weight_sum += horizontal_speed;
    }
    if (heading_weight_sum <= 0.0) {
        throw std::runtime_error(
            "No SPP-like GNSS velocity sample met the heading speed threshold");
    }

    const double measured_heading = std::atan2(heading_sin_sum, heading_cos_sum);
    result.heading_time = heading_time_sum / heading_weight_sum;
    const Quat zero_heading_at_measurement = propagate_imu_orientation(
        dataset.imu,
        result.reference_time,
        result.heading_time,
        tilt_orientation,
        result.gyro_bias);
    const Vec3 predicted_forward = rotate(zero_heading_at_measurement, {1.0, 0.0, 0.0});
    const double predicted_heading = std::atan2(predicted_forward.y, predicted_forward.x);
    const double yaw_correction = wrap_angle(measured_heading - predicted_heading);
    const Quat yaw_rotation = quaternion_from_rpy(0.0, 0.0, yaw_correction);
    result.q_body_to_ned = quat_normalized(
        quat_multiply(yaw_rotation, tilt_orientation));

    const Vec3 expected_specific_force_body =
        rotate_inverse(result.q_body_to_ned, {0.0, 0.0, -options.gravity_mps2});
    result.accel_bias = mean_accel - expected_specific_force_body;
    const Vec3 rpy = quaternion_to_rpy(result.q_body_to_ned);
    result.roll_rad = rpy.x;
    result.pitch_rad = rpy.y;
    result.yaw_rad = rpy.z;
    return result;
}

Vec3 interpolate_gnss_position(const std::vector<GnssPositionFix>& fixes, double t) {
    return interpolate_vector(
        fixes, t, [](const GnssPositionFix& fix) { return fix.position_ned; });
}

Vec3 interpolate_gnss_velocity(const std::vector<GnssPositionFix>& fixes, double t) {
    std::vector<GnssPositionFix> valid_fixes;
    valid_fixes.reserve(fixes.size());
    std::copy_if(fixes.begin(), fixes.end(), std::back_inserter(valid_fixes),
                 [](const GnssPositionFix& fix) { return fix.valid; });
    return interpolate_vector(
        valid_fixes, t, [](const GnssPositionFix& fix) { return fix.velocity_ned; });
}

}  // namespace offline_fusion
