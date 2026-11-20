#include "offline_fusion/fusion.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "offline_fusion/dataset.hpp"
#include "offline_fusion/initialization.hpp"
#include "offline_fusion/math.hpp"
#include "offline_fusion/imu_covariance.hpp"
#ifdef OFFLINE_FUSION_HAS_LIDAR
#include "offline_fusion/lidar.hpp"
#endif
#ifdef OFFLINE_FUSION_HAS_OPENCV
#include "offline_fusion/vision.hpp"
#endif

namespace offline_fusion {
namespace {

using Matrix3 = std::array<std::array<double, 3>, 3>;

struct GraphNode {
    double t = 0.0;
    Vec3 position;
    Vec3 velocity;
    Quat orientation;
    Vec3 accel_bias;
    Vec3 gyro_bias;
    Vec3 gnss_position;
    Vec3 gnss_position_std{3.0, 3.0, 3.0};
    bool has_gnss = false;
    std::size_t fiveg_epoch_index = std::numeric_limits<std::size_t>::max();
    bool has_fiveg = false;
    bool is_visual_keyframe = false;
    bool is_lidar_keyframe = false;
};

struct ImuPreintegration {
    ImuMatrix15 covariance = ImuMatrix15::Zero();
    ImuMatrix15 sqrt_information = ImuMatrix15::Identity();
    Vec3 linearized_accel_bias;
    Vec3 linearized_gyro_bias;
    bool propagated_covariance = false;
    Vec3 delta_p;
    Vec3 delta_v;
    Quat delta_q;
    Matrix3 jacobian_p_ba{};
    Matrix3 jacobian_p_bg{};
    Matrix3 jacobian_v_ba{};
    Matrix3 jacobian_v_bg{};
    Matrix3 jacobian_q_bg{};
    double dt = 0.0;
    std::size_t samples = 0;
};

bool in_range(double t, double start, double end) {
    return t >= start && t <= end;
}

double component(const Vec3& value, int dim) {
    if (dim == 0) {
        return value.x;
    }
    if (dim == 1) {
        return value.y;
    }
    return value.z;
}

void set_component(Vec3* value, int dim, double component_value) {
    if (dim == 0) {
        value->x = component_value;
    } else if (dim == 1) {
        value->y = component_value;
    } else {
        value->z = component_value;
    }
}

std::vector<GraphNode> build_graph_nodes(const Dataset& dataset,
                                         double start_time,
                                         double end_time,
                                         const FusionOptions& options,
                                         const SensorInitialization* sensor_initialization) {
    std::vector<GraphNode> nodes;
    for (const auto& fix : dataset.gnss_position_fixes) {
        if (!in_range(fix.t, start_time, end_time)) {
            continue;
        }
        GraphNode node;
        node.t = fix.t;
        node.gnss_position = fix.position_ned;
        auto bounded_sigma = [&options](double sigma) {
            if (!std::isfinite(sigma) || sigma <= 0.0) {
                return options.gnss_position_std_m;
            }
            return std::max(0.15, std::min(options.gnss_position_std_m,
                                           sigma * options.gnss_position_noise_scale));
        };
        node.gnss_position_std = {
            bounded_sigma(fix.position_std_ned.x),
            bounded_sigma(fix.position_std_ned.y),
            bounded_sigma(fix.position_std_ned.z),
        };
        node.has_gnss = true;
        nodes.push_back(node);
    }

    if (options.use_fiveg) {
        for (std::size_t i = 0; i < dataset.fiveg_epochs.size(); ++i) {
            if (!in_range(dataset.fiveg_epochs[i].t, start_time, end_time)) {
                continue;
            }
            GraphNode node;
            node.t = dataset.fiveg_epochs[i].t;
            node.fiveg_epoch_index = i;
            node.has_fiveg = true;
            nodes.push_back(node);
        }
    }

#ifdef OFFLINE_FUSION_HAS_OPENCV
    const std::vector<double> visual_keyframe_times =
        select_visual_keyframe_times(dataset, start_time, end_time, options);
    for (double t : visual_keyframe_times) {
        GraphNode node;
        node.t = t;
        node.is_visual_keyframe = true;
        nodes.push_back(node);
    }
#else
    (void)options;
#endif

#ifdef OFFLINE_FUSION_HAS_LIDAR
    const std::vector<double> lidar_keyframe_times =
        select_lidar_keyframe_times(dataset, start_time, end_time, options);
    for (double t : lidar_keyframe_times) {
        GraphNode node;
        node.t = t;
        node.is_lidar_keyframe = true;
        nodes.push_back(node);
    }
#endif

    if (options.extra_state_interval_s > 0.0 && !nodes.empty()) {
        const double first = nodes.front().t;
        double last = first;
        for (const auto& node : nodes) last = std::max(last, node.t);
        for (double t = first + options.extra_state_interval_s; t < last; t += options.extra_state_interval_s) {
            GraphNode extra;
            extra.t = t;
            nodes.push_back(extra);
        }
    }

    std::sort(nodes.begin(), nodes.end(), [](const GraphNode& a, const GraphNode& b) {
        return a.t < b.t;
    });

    std::vector<GraphNode> merged_nodes;
    constexpr double kMergeTolerance = 1e-6;
    for (const auto& node : nodes) {
        if (!merged_nodes.empty() && std::abs(merged_nodes.back().t - node.t) <= kMergeTolerance) {
            auto& merged = merged_nodes.back();
            merged.t = 0.5 * (merged.t + node.t);
            if (node.has_gnss) {
                merged.gnss_position = node.gnss_position;
                merged.gnss_position_std = node.gnss_position_std;
                merged.has_gnss = true;
            }
            if (node.has_fiveg) {
                merged.fiveg_epoch_index = node.fiveg_epoch_index;
                merged.has_fiveg = true;
            }
            merged.is_visual_keyframe = merged.is_visual_keyframe || node.is_visual_keyframe;
            merged.is_lidar_keyframe = merged.is_lidar_keyframe || node.is_lidar_keyframe;
            continue;
        }
        merged_nodes.push_back(node);
    }

    nodes = std::move(merged_nodes);
    for (auto& node : nodes) {
        if (sensor_initialization != nullptr) {
            node.position = interpolate_gnss_position(dataset.gnss_position_fixes, node.t);
            node.velocity = interpolate_gnss_velocity(dataset.gnss_position_fixes, node.t);
            node.orientation = propagate_imu_orientation(
                dataset.imu,
                sensor_initialization->reference_time,
                node.t,
                sensor_initialization->q_body_to_ned,
                sensor_initialization->gyro_bias);
            node.accel_bias = sensor_initialization->accel_bias;
            node.gyro_bias = sensor_initialization->gyro_bias;
        } else {
            node.position = interpolate_groundtruth_position(dataset.groundtruth, node.t);
            node.velocity = estimate_groundtruth_velocity(dataset.groundtruth, node.t);
            node.orientation = nearest_groundtruth_orientation(dataset.groundtruth, node.t);
        }
    }

    if (nodes.size() < 2) {
        throw std::runtime_error("Need at least two state nodes in the selected time range");
    }
    return nodes;
}

ImuPreintegration integrate_imu_interval(const Dataset& dataset,
                                         double start_time,
                                         double end_time,
                                         const Vec3& accel_bias,
                                         const Vec3& gyro_bias,
                                         const FusionOptions* covariance_options = nullptr,
                                         bool compute_covariance = false) {
    ImuPreintegration result;
    result.delta_q = {};
    ImuMatrix15 noise = ImuMatrix15::Zero();
    if (covariance_options && compute_covariance) {
        const double densities[4] = {covariance_options->imu_accel_noise_density,
            covariance_options->imu_gyro_noise_density, covariance_options->imu_accel_bias_density,
            covariance_options->imu_gyro_bias_density};
        for (int block = 0; block < 4; ++block) {
            if (!std::isfinite(densities[block]) || densities[block] <= 0)
                throw std::runtime_error("IMU noise densities must be finite and positive");
            noise.block<3,3>(3 + 3*block, 3 + 3*block).diagonal().setConstant(densities[block]*densities[block]);
        }
    }

    for (std::size_t i = 1; i < dataset.imu.size(); ++i) {
        const auto& previous = dataset.imu[i - 1];
        const auto& current = dataset.imu[i];
        if (current.t <= start_time || previous.t >= end_time) {
            continue;
        }

        const double interval_start = std::max(previous.t, start_time);
        const double interval_end = std::min(current.t, end_time);
        const double dt = interval_end - interval_start;
        if (dt <= 0.0) continue;
        if (current.t - previous.t > 0.1) {
            if (covariance_options) throw std::runtime_error("IMU gap exceeds 0.1 s; cannot silently truncate integration");
            if (dt > 0.1) continue;
        }

        const Vec3 unbiased_accel = previous.accel - accel_bias;
        const Vec3 unbiased_gyro = previous.gyro - gyro_bias;
        if (covariance_options) {
            const int steps = std::max(1, static_cast<int>(std::ceil(dt / 0.005)));
            const double h = dt / steps;
            for (int step = 0; step < steps; ++step) {
                const Quat start_q = result.delta_q;
                const Quat middle_q = quat_multiply(start_q, quat_from_delta_angle(unbiased_gyro * (0.5*h)));
                if (compute_covariance) propagate_imu_covariance(result.covariance,
                    imu_error_dynamics(middle_q, unbiased_accel, unbiased_gyro), noise, h);
                const double x[3] = {0.5-std::sqrt(15.0)/10, 0.5, 0.5+std::sqrt(15.0)/10};
                const double weights[3] = {5.0/18, 4.0/9, 5.0/18};
                Vec3 dv{}, dp{};
                for (int k=0; k<3; ++k) {
                    const Vec3 a = rotate(quat_multiply(start_q, quat_from_delta_angle(unbiased_gyro*(x[k]*h))), unbiased_accel);
                    dv = dv + a*(weights[k]*h);
                    dp = dp + a*(weights[k]*(1-x[k])*h*h);
                }
                result.delta_p = result.delta_p + result.delta_v*h + dp;
                result.delta_v = result.delta_v + dv;
                result.delta_q = quat_normalized(quat_multiply(start_q, quat_from_delta_angle(unbiased_gyro*h)));
            }
            result.dt += dt;
            ++result.samples;
            continue;
        }
        const Vec3 accel_in_start_body = rotate(result.delta_q, unbiased_accel);

        result.delta_p =
            result.delta_p + result.delta_v * dt + accel_in_start_body * (0.5 * dt * dt);
        result.delta_v = result.delta_v + accel_in_start_body * dt;
        result.delta_q = quat_normalized(
            quat_multiply(result.delta_q, quat_from_delta_angle(unbiased_gyro * dt)));
        result.dt += dt;
        ++result.samples;
    }
    if (covariance_options && std::abs(result.dt - (end_time-start_time)) > 1e-7)
        throw std::runtime_error("IMU does not cover the complete state interval");
    return result;
}

ImuPreintegration preintegrate_with_bias_jacobians(const Dataset& dataset,
                                                   double start_time,
                                                   double end_time,
                                                   const FusionOptions* options = nullptr,
                                                   const Vec3& reference_accel = {},
                                                   const Vec3& reference_gyro = {}) {
    const Vec3 base_accel = options ? reference_accel : Vec3{};
    const Vec3 base_gyro = options ? reference_gyro : Vec3{};
    ImuPreintegration result =
        integrate_imu_interval(dataset, start_time, end_time, base_accel, base_gyro, options, true);
    result.linearized_accel_bias = base_accel;
    result.linearized_gyro_bias = base_gyro;
    result.propagated_covariance = options != nullptr;
    if (options) result.sqrt_information = imu_sqrt_information(result.covariance);
    constexpr double kBiasStep = 1e-4;

    for (int dim = 0; dim < 3; ++dim) {
        Vec3 accel_bias = base_accel;
        set_component(&accel_bias, dim, component(base_accel, dim) + kBiasStep);
        const ImuPreintegration accel_perturbed =
            integrate_imu_interval(dataset, start_time, end_time, accel_bias, base_gyro, options);
        for (int row = 0; row < 3; ++row) {
            result.jacobian_p_ba[row][dim] =
                (component(accel_perturbed.delta_p, row) - component(result.delta_p, row)) /
                kBiasStep;
            result.jacobian_v_ba[row][dim] =
                (component(accel_perturbed.delta_v, row) - component(result.delta_v, row)) /
                kBiasStep;
        }

        Vec3 gyro_bias = base_gyro;
        set_component(&gyro_bias, dim, component(base_gyro, dim) + kBiasStep);
        const ImuPreintegration gyro_perturbed =
            integrate_imu_interval(dataset, start_time, end_time, base_accel, gyro_bias, options);
        const Vec3 delta_rotation = quat_to_delta_angle(
            quat_multiply(quat_conjugate(result.delta_q), gyro_perturbed.delta_q));
        for (int row = 0; row < 3; ++row) {
            result.jacobian_p_bg[row][dim] =
                (component(gyro_perturbed.delta_p, row) - component(result.delta_p, row)) /
                kBiasStep;
            result.jacobian_v_bg[row][dim] =
                (component(gyro_perturbed.delta_v, row) - component(result.delta_v, row)) /
                kBiasStep;
            result.jacobian_q_bg[row][dim] = component(delta_rotation, row) / kBiasStep;
        }
    }
    return result;
}

template <typename T>
void quaternion_conjugate(const T* quaternion, T* conjugate) {
    conjugate[0] = quaternion[0];
    conjugate[1] = -quaternion[1];
    conjugate[2] = -quaternion[2];
    conjugate[3] = -quaternion[3];
}

template <typename T>
void rotate_inverse(const T* quaternion, const T* point, T* rotated) {
    T inverse[4];
    quaternion_conjugate(quaternion, inverse);
    ceres::QuaternionRotatePoint(inverse, point, rotated);
}

template <typename T>
void add_matrix_bias_correction(const Vec3& base,
                                const Matrix3& jacobian_a,
                                const T* bias_a,
                                const Matrix3& jacobian_b,
                                const T* bias_b,
                                T* corrected) {
    const double base_values[3] = {base.x, base.y, base.z};
    for (int row = 0; row < 3; ++row) {
        corrected[row] = T(base_values[row]);
        for (int col = 0; col < 3; ++col) {
            corrected[row] += T(jacobian_a[row][col]) * bias_a[col] +
                              T(jacobian_b[row][col]) * bias_b[col];
        }
    }
}

struct VectorPriorCost {
    Vec3 measurement;
    double inverse_sigma = 1.0;

    template <typename T>
    bool operator()(const T* value, T* residuals) const {
        residuals[0] = (value[0] - T(measurement.x)) * T(inverse_sigma);
        residuals[1] = (value[1] - T(measurement.y)) * T(inverse_sigma);
        residuals[2] = (value[2] - T(measurement.z)) * T(inverse_sigma);
        return true;
    }
};

struct AxisWeightedPositionCost {
    Vec3 measurement;
    Vec3 inverse_sigma{1.0, 1.0, 1.0};

    template <typename T>
    bool operator()(const T* position, T* residuals) const {
        residuals[0] = (position[0] - T(measurement.x)) * T(inverse_sigma.x);
        residuals[1] = (position[1] - T(measurement.y)) * T(inverse_sigma.y);
        residuals[2] = (position[2] - T(measurement.z)) * T(inverse_sigma.z);
        return true;
    }
};

struct FiveGRangeCost {
    Vec3 gnb_position_ned;
    double measured_range_m = 0.0;
    double inverse_sigma = 1.0;

    template <typename T>
    bool operator()(const T* position, T* residual) const {
        const T dn = position[0] - T(gnb_position_ned.x);
        const T de = position[1] - T(gnb_position_ned.y);
        const T dd = position[2] - T(gnb_position_ned.z);
        const T predicted_range = ceres::sqrt(dn * dn + de * de + dd * dd);
        residual[0] =
            (predicted_range - T(measured_range_m)) * T(inverse_sigma);
        return true;
    }
};

struct QuaternionPriorCost {
    Quat measurement;
    double inverse_sigma = 1.0;

    template <typename T>
    bool operator()(const T* quaternion, T* residuals) const {
        const T prior_inverse[4] = {
            T(measurement.w), T(-measurement.x), T(-measurement.y), T(-measurement.z)};
        T error[4];
        ceres::QuaternionProduct(prior_inverse, quaternion, error);
        ceres::QuaternionToAngleAxis(error, residuals);
        residuals[0] *= T(inverse_sigma);
        residuals[1] *= T(inverse_sigma);
        residuals[2] *= T(inverse_sigma);
        return true;
    }
};

struct ImuPreintegrationCost {
    ImuPreintegration preintegration;
    double gravity_mps2 = 9.793;
    double inverse_position_sigma = 1.0;
    double inverse_velocity_sigma = 1.0;
    double inverse_attitude_sigma = 1.0;
    double inverse_accel_bias_sigma = 1.0;
    double inverse_gyro_bias_sigma = 1.0;

    template <typename T>
    bool operator()(const T* position_i,
                    const T* velocity_i,
                    const T* quaternion_i,
                    const T* accel_bias_i,
                    const T* gyro_bias_i,
                    const T* position_j,
                    const T* velocity_j,
                    const T* quaternion_j,
                    const T* accel_bias_j,
                    const T* gyro_bias_j,
                    T* residuals) const {
        const T dt = T(preintegration.dt);
        const T gravity[3] = {T(0.0), T(0.0), T(gravity_mps2)};

        T world_delta_p[3];
        T world_delta_v[3];
        for (int dim = 0; dim < 3; ++dim) {
            world_delta_p[dim] = position_j[dim] - position_i[dim] -
                                 velocity_i[dim] * dt - T(0.5) * gravity[dim] * dt * dt;
            world_delta_v[dim] = velocity_j[dim] - velocity_i[dim] - gravity[dim] * dt;
        }

        T local_delta_p[3];
        T local_delta_v[3];
        rotate_inverse(quaternion_i, world_delta_p, local_delta_p);
        rotate_inverse(quaternion_i, world_delta_v, local_delta_v);

        const T delta_ba[3] = {accel_bias_i[0]-T(preintegration.linearized_accel_bias.x),
            accel_bias_i[1]-T(preintegration.linearized_accel_bias.y), accel_bias_i[2]-T(preintegration.linearized_accel_bias.z)};
        const T delta_bg[3] = {gyro_bias_i[0]-T(preintegration.linearized_gyro_bias.x),
            gyro_bias_i[1]-T(preintegration.linearized_gyro_bias.y), gyro_bias_i[2]-T(preintegration.linearized_gyro_bias.z)};
        T corrected_delta_p[3];
        T corrected_delta_v[3];
        add_matrix_bias_correction(preintegration.delta_p,
                                   preintegration.jacobian_p_ba,
                                   delta_ba,
                                   preintegration.jacobian_p_bg,
                                   delta_bg,
                                   corrected_delta_p);
        add_matrix_bias_correction(preintegration.delta_v,
                                   preintegration.jacobian_v_ba,
                                   delta_ba,
                                   preintegration.jacobian_v_bg,
                                   delta_bg,
                                   corrected_delta_v);

        for (int dim = 0; dim < 3; ++dim) {
            residuals[dim] =
                (local_delta_p[dim] - corrected_delta_p[dim]) * T(inverse_position_sigma);
            residuals[3 + dim] =
                (local_delta_v[dim] - corrected_delta_v[dim]) * T(inverse_velocity_sigma);
        }

        T gyro_correction[3] = {T(0.0), T(0.0), T(0.0)};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                gyro_correction[row] +=
                    T(preintegration.jacobian_q_bg[row][col]) * delta_bg[col];
            }
        }
        T delta_q_bias[4];
        ceres::AngleAxisToQuaternion(gyro_correction, delta_q_bias);
        const T base_delta_q[4] = {
            T(preintegration.delta_q.w),
            T(preintegration.delta_q.x),
            T(preintegration.delta_q.y),
            T(preintegration.delta_q.z),
        };
        T corrected_delta_q[4];
        ceres::QuaternionProduct(base_delta_q, delta_q_bias, corrected_delta_q);

        T quaternion_i_inverse[4];
        quaternion_conjugate(quaternion_i, quaternion_i_inverse);
        T relative_q[4];
        ceres::QuaternionProduct(quaternion_i_inverse, quaternion_j, relative_q);
        T corrected_delta_q_inverse[4];
        quaternion_conjugate(corrected_delta_q, corrected_delta_q_inverse);
        T attitude_error_q[4];
        ceres::QuaternionProduct(corrected_delta_q_inverse, relative_q, attitude_error_q);
        T attitude_error[3];
        ceres::QuaternionToAngleAxis(attitude_error_q, attitude_error);
        for (int dim = 0; dim < 3; ++dim) {
            residuals[6 + dim] = attitude_error[dim] * T(inverse_attitude_sigma);
            residuals[9 + dim] =
                (accel_bias_j[dim] - accel_bias_i[dim]) * T(inverse_accel_bias_sigma);
            residuals[12 + dim] =
                (gyro_bias_j[dim] - gyro_bias_i[dim]) * T(inverse_gyro_bias_sigma);
        }
        if (preintegration.propagated_covariance) {
            T raw[15];
            for (int i=0; i<15; ++i) raw[i] = residuals[i];
            for (int i=0; i<15; ++i) {
                residuals[i] = T(0);
                for (int j=0; j<15; ++j) residuals[i] += T(preintegration.sqrt_information(i,j)) * raw[j];
            }
        }
        return true;
    }
};

struct ReprojectionCost {
    double u_px = 0.0;
    double v_px = 0.0;
    double fx = 1.0;
    double fy = 1.0;
    double cx = 0.0;
    double cy = 0.0;
    Vec3 t_body_camera;
    double camera_pitch_rad = 0.0;
    double inverse_sigma = 1.0;

    template <typename T>
    bool operator()(const T* position_body_in_world,
                    const T* quaternion_body_to_world,
                    const T* point_world,
                    T* residuals) const {
        T delta_world[3] = {
            point_world[0] - position_body_in_world[0],
            point_world[1] - position_body_in_world[1],
            point_world[2] - position_body_in_world[2],
        };
        T point_body[3];
        rotate_inverse(quaternion_body_to_world, delta_world, point_body);
        point_body[0] -= T(t_body_camera.x);
        point_body[1] -= T(t_body_camera.y);
        point_body[2] -= T(t_body_camera.z);

        const T point_camera[3] = {
            point_body[1],
            T(std::sin(camera_pitch_rad)) * point_body[0] + T(std::cos(camera_pitch_rad)) * point_body[2],
            T(std::cos(camera_pitch_rad)) * point_body[0] - T(std::sin(camera_pitch_rad)) * point_body[2],
        };
        const T inv_depth = T(1.0) / point_camera[2];
        const T projected_u = T(fx) * point_camera[0] * inv_depth + T(cx);
        const T projected_v = T(fy) * point_camera[1] * inv_depth + T(cy);
        residuals[0] = (projected_u - T(u_px)) * T(inverse_sigma);
        residuals[1] = (projected_v - T(v_px)) * T(inverse_sigma);
        return true;
    }
};

struct LidarRelativePoseCost {
    Vec3 translation_source_in_target_body;
    Quat rotation_source_to_target_body;
    double inverse_translation_sigma = 1.0;
    double inverse_rotation_sigma = 1.0;

    template <typename T>
    bool operator()(const T* position_source,
                    const T* quaternion_source,
                    const T* position_target,
                    const T* quaternion_target,
                    T* residuals) const {
        const T world_translation[3] = {
            position_source[0] - position_target[0],
            position_source[1] - position_target[1],
            position_source[2] - position_target[2],
        };
        T predicted_translation[3];
        rotate_inverse(quaternion_target, world_translation, predicted_translation);
        residuals[0] =
            (predicted_translation[0] - T(translation_source_in_target_body.x)) *
            T(inverse_translation_sigma);
        residuals[1] =
            (predicted_translation[1] - T(translation_source_in_target_body.y)) *
            T(inverse_translation_sigma);
        residuals[2] =
            (predicted_translation[2] - T(translation_source_in_target_body.z)) *
            T(inverse_translation_sigma);

        T target_inverse[4];
        quaternion_conjugate(quaternion_target, target_inverse);
        T predicted_rotation[4];
        ceres::QuaternionProduct(target_inverse, quaternion_source, predicted_rotation);
        const T measurement_inverse[4] = {
            T(rotation_source_to_target_body.w),
            T(-rotation_source_to_target_body.x),
            T(-rotation_source_to_target_body.y),
            T(-rotation_source_to_target_body.z),
        };
        T rotation_error[4];
        ceres::QuaternionProduct(measurement_inverse, predicted_rotation, rotation_error);
        ceres::QuaternionToAngleAxis(rotation_error, residuals + 3);
        residuals[3] *= T(inverse_rotation_sigma);
        residuals[4] *= T(inverse_rotation_sigma);
        residuals[5] *= T(inverse_rotation_sigma);
        return true;
    }
};

template <typename CostType>
ceres::CostFunction* make_vector_cost(const Vec3& measurement, double sigma) {
    return new ceres::AutoDiffCostFunction<CostType, 3, 3>(
        new CostType{measurement, 1.0 / sigma});
}

std::array<double, 3> to_array(const Vec3& value) {
    return {value.x, value.y, value.z};
}

std::array<double, 4> to_array(const Quat& value) {
    return {value.w, value.x, value.y, value.z};
}

Vec3 to_vec3(const std::array<double, 3>& value) {
    return {value[0], value[1], value[2]};
}

Quat to_quat(const std::array<double, 4>& value) {
    return quat_normalized({value[0], value[1], value[2], value[3]});
}

std::string termination_name(ceres::TerminationType termination) {
    switch (termination) {
        case ceres::CONVERGENCE:
            return "CONVERGENCE";
        case ceres::NO_CONVERGENCE:
            return "NO_CONVERGENCE";
        case ceres::FAILURE:
            return "FAILURE";
        case ceres::USER_SUCCESS:
            return "USER_SUCCESS";
        case ceres::USER_FAILURE:
            return "USER_FAILURE";
        default:
            return "UNKNOWN";
    }
}

}  // namespace

std::vector<TrajectoryState> run_ceres_gins(const Dataset& dataset,
                                            const FusionOptions& options,
                                            FusionSummary* summary) {
#ifndef OFFLINE_FUSION_HAS_OPENCV
    if (options.use_visual) {
        throw std::runtime_error(
            "Visual factors requested, but this binary was built without OpenCV. "
            "Rebuild with OFFLINE_FUSION_USE_OPENCV=ON or run with --disable-visual.");
    }
#endif
    if (options.use_visual && dataset.camera_frames.empty()) {
        throw std::runtime_error("Visual factors requested, but no camera frames were loaded");
    }
#ifndef OFFLINE_FUSION_HAS_LIDAR
    if (options.use_lidar) {
        throw std::runtime_error(
            "LiDAR factors requested, but this binary was built without OpenCV FLANN.");
    }
#endif
    if (options.use_lidar && dataset.lidar_frames.empty()) {
        throw std::runtime_error("LiDAR factors requested, but no LiDAR frames were loaded");
    }
    if (options.use_fiveg && dataset.fiveg_epochs.empty()) {
        throw std::runtime_error("5G-A factors requested, but no 5G-A epochs were loaded");
    }
    if (options.gnss_position_std_m <= 0.0 ||
        options.fiveg_range_std_floor_m <= 0.0 || options.fiveg_loss_scale <= 0.0) {
        throw std::runtime_error("GNSS SPP-like and 5G-A noise parameters must be positive");
    }
    if (!std::isfinite(options.fiveg_min_prs_rsrp_dbm)) {
        throw std::runtime_error("5G-A PRS-RSRP threshold must be finite");
    }

    double raw_start =
        std::max(dataset.imu.front().t, dataset.gnss_position_fixes.front().t);
    // A GNSS outage must not truncate the timeline at the last available fix.
    // This trial always enables 5G-A timestamps (with or without range factors),
    // so the IMU/5G-A overlap defines the endpoint without future GNSS fixes.
    double raw_end = options.use_fiveg
        ? std::min(dataset.imu.back().t, dataset.fiveg_epochs.back().t)
        : std::min(dataset.imu.back().t, dataset.gnss_position_fixes.back().t);
    if (options.use_visual) {
        raw_start = std::max(raw_start, dataset.camera_frames.front().t);
        raw_end = std::min(raw_end, dataset.camera_frames.back().t);
    }
    if (options.use_lidar) {
        raw_start = std::max(raw_start, dataset.lidar_frames.front().t);
        raw_end = std::min(raw_end, dataset.lidar_frames.back().t);
    }
    if (options.use_fiveg) {
        raw_start = std::max(raw_start, dataset.fiveg_epochs.front().t);
        raw_end = std::min(raw_end, dataset.fiveg_epochs.back().t);
    }
    const double start_time = options.absolute_start_s >= 0.0
        ? std::max(raw_start, options.absolute_start_s) : raw_start + options.start_offset_s;
    const double end_time =
        options.absolute_end_s >= 0.0 ? std::min(raw_end, options.absolute_end_s) :
        (options.end_offset_s > 0.0 ? raw_start + options.end_offset_s : raw_end);
    if (start_time >= end_time) {
        throw std::runtime_error("Invalid fusion time range");
    }

    SensorInitialization sensor_initialization;
    const SensorInitialization* sensor_initialization_ptr = nullptr;
    if (dataset.format == DatasetFormat::StructuredExperiment) {
        sensor_initialization = initialize_from_imu_and_gnss(dataset, options);
        sensor_initialization_ptr = &sensor_initialization;
        std::cout << "offline_fusion: sensor initialization uses "
                  << sensor_initialization.gravity_samples << " gravity samples, heading at "
                  << sensor_initialization.heading_time << " s" << std::endl;
    }

    std::cout << "offline_fusion: building state timeline" << std::endl;
    std::vector<GraphNode> nodes = build_graph_nodes(
        dataset, start_time, end_time, options, sensor_initialization_ptr);
#ifdef OFFLINE_FUSION_HAS_OPENCV
    VisualMap visual_map;
    if (options.use_visual) {
        std::vector<VisualPose> initial_visual_poses;
        initial_visual_poses.reserve(nodes.size());
        for (const auto& node : nodes) {
            initial_visual_poses.push_back(
                {node.t, node.position, node.orientation});
        }
        std::cout << "offline_fusion: building visual map" << std::endl;
        visual_map = build_visual_map(
            dataset, initial_visual_poses, start_time, end_time, options);
        std::cout << "offline_fusion: visual map has " << visual_map.landmarks.size()
                  << " landmarks and " << visual_map.reprojection_factors
                  << " reprojection factors, initial reprojection RMSE "
                  << visual_map.initial_reprojection_rmse_px << " px" << std::endl;
    }
#endif

#ifdef OFFLINE_FUSION_HAS_LIDAR
    LidarOdometry lidar_odometry;
    if (options.use_lidar) {
        std::vector<LidarPose> initial_lidar_poses;
        initial_lidar_poses.reserve(nodes.size());
        for (const auto& node : nodes) {
            initial_lidar_poses.push_back({node.t, node.position, node.orientation});
        }
        std::cout << "offline_fusion: building LiDAR odometry" << std::endl;
        lidar_odometry = build_lidar_odometry(
            dataset, initial_lidar_poses, start_time, end_time, options);
        write_lidar_diagnostics_csv(
            options.output_csv.parent_path() / "lidar_pair_diagnostics.csv",
            lidar_odometry);
        std::cout << "offline_fusion: LiDAR accepted "
                  << lidar_odometry.constraints.size() << " / "
                  << lidar_odometry.attempted_pairs << " relative-pose constraints, mean RMSE "
                  << lidar_odometry.mean_rmse_m << " m, mean overlap "
                  << lidar_odometry.mean_overlap << std::endl;
    }
#endif

    bool prune_frontend_nodes = false;
#ifdef OFFLINE_FUSION_HAS_OPENCV
    prune_frontend_nodes = prune_frontend_nodes ||
        (options.use_visual && options.visual_prune_unconstrained_nodes);
#endif
#ifdef OFFLINE_FUSION_HAS_LIDAR
    prune_frontend_nodes = prune_frontend_nodes ||
        (options.use_lidar && options.lidar_prune_unconstrained_nodes);
#endif
    if (prune_frontend_nodes) {
        std::vector<bool> keep(nodes.size(), false);
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            keep[i] = nodes[i].has_gnss || nodes[i].has_fiveg;
        }
#ifdef OFFLINE_FUSION_HAS_OPENCV
        if (options.use_visual) {
            if (options.visual_prune_unconstrained_nodes) {
                for (const auto& landmark : visual_map.landmarks) {
                    for (const auto& observation : landmark.observations) {
                        keep[observation.node_index] = true;
                    }
                }
            } else {
                for (std::size_t i = 0; i < nodes.size(); ++i) {
                    keep[i] = keep[i] || nodes[i].is_visual_keyframe;
                }
            }
        }
#endif
#ifdef OFFLINE_FUSION_HAS_LIDAR
        if (options.use_lidar && !options.lidar_prune_unconstrained_nodes) {
            for (std::size_t i = 0; i < nodes.size(); ++i) {
                keep[i] = keep[i] || nodes[i].is_lidar_keyframe;
            }
        }
        for (const auto& constraint : lidar_odometry.constraints) {
            keep[constraint.source_node_index] = true;
            keep[constraint.target_node_index] = true;
        }
#endif

        const std::size_t invalid_index = std::numeric_limits<std::size_t>::max();
        std::vector<std::size_t> old_to_new(nodes.size(), invalid_index);
        std::vector<GraphNode> pruned_nodes;
        pruned_nodes.reserve(nodes.size());
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            if (!keep[i]) {
                continue;
            }
            old_to_new[i] = pruned_nodes.size();
            pruned_nodes.push_back(std::move(nodes[i]));
        }
#ifdef OFFLINE_FUSION_HAS_LIDAR
        for (auto& constraint : lidar_odometry.constraints) {
            constraint.source_node_index = old_to_new[constraint.source_node_index];
            constraint.target_node_index = old_to_new[constraint.target_node_index];
        }
#endif
#ifdef OFFLINE_FUSION_HAS_OPENCV
        for (auto& landmark : visual_map.landmarks) {
            for (auto& observation : landmark.observations) {
                observation.node_index = old_to_new[observation.node_index];
            }
        }
#endif
        std::cout << "offline_fusion: pruned unconstrained frontend state nodes from "
                  << nodes.size() << " to " << pruned_nodes.size() << std::endl;
        nodes = std::move(pruned_nodes);
    }

    const std::size_t node_count = nodes.size();
    const std::size_t gnss_node_count = static_cast<std::size_t>(std::count_if(
        nodes.begin(), nodes.end(), [](const GraphNode& node) { return node.has_gnss; }));
    const std::size_t fiveg_node_count = static_cast<std::size_t>(std::count_if(
        nodes.begin(), nodes.end(), [](const GraphNode& node) { return node.has_fiveg; }));
    const std::size_t visual_node_count = static_cast<std::size_t>(std::count_if(
        nodes.begin(), nodes.end(), [](const GraphNode& node) {
            return node.is_visual_keyframe;
        }));
#ifdef OFFLINE_FUSION_HAS_LIDAR
    const std::size_t lidar_node_count = static_cast<std::size_t>(std::count_if(
        nodes.begin(), nodes.end(), [](const GraphNode& node) {
            return node.is_lidar_keyframe;
        }));
#endif

    std::vector<std::array<double, 3>> positions;
    std::vector<std::array<double, 3>> velocities;
    std::vector<std::array<double, 4>> orientations;
    std::vector<std::array<double, 3>> accel_biases;
    std::vector<std::array<double, 3>> gyro_biases;
    positions.reserve(node_count);
    velocities.reserve(node_count);
    orientations.reserve(node_count);
    accel_biases.reserve(node_count);
    gyro_biases.reserve(node_count);

    for (const auto& node : nodes) {
        positions.push_back(to_array(node.position));
        velocities.push_back(to_array(node.velocity));
        orientations.push_back(to_array(node.orientation));
        accel_biases.push_back(to_array(node.accel_bias));
        gyro_biases.push_back(to_array(node.gyro_bias));
    }

    ceres::Problem problem;
    std::size_t fiveg_range_factor_count = 0;
    std::size_t fiveg_rsrp_rejected_count = 0;
    auto linear_solver_ordering = std::make_shared<ceres::ParameterBlockOrdering>();
    auto* quaternion_manifold = new ceres::QuaternionManifold;
    for (std::size_t i = 0; i < node_count; ++i) {
        problem.AddParameterBlock(positions[i].data(), 3);
        problem.AddParameterBlock(velocities[i].data(), 3);
        problem.AddParameterBlock(orientations[i].data(), 4, quaternion_manifold);
        problem.AddParameterBlock(accel_biases[i].data(), 3);
        problem.AddParameterBlock(gyro_biases[i].data(), 3);
        linear_solver_ordering->AddElementToGroup(positions[i].data(), 1);
        linear_solver_ordering->AddElementToGroup(velocities[i].data(), 1);
        linear_solver_ordering->AddElementToGroup(orientations[i].data(), 1);
        linear_solver_ordering->AddElementToGroup(accel_biases[i].data(), 1);
        linear_solver_ordering->AddElementToGroup(gyro_biases[i].data(), 1);
    }

    problem.AddResidualBlock(
        make_vector_cost<VectorPriorCost>(nodes.front().position, options.prior_position_std_m),
        nullptr,
        positions.front().data());
    problem.AddResidualBlock(
        make_vector_cost<VectorPriorCost>(nodes.front().velocity, options.prior_velocity_std_mps),
        nullptr,
        velocities.front().data());
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<QuaternionPriorCost, 3, 4>(
            new QuaternionPriorCost{nodes.front().orientation,
                                    1.0 / options.attitude_prior_std_rad}),
        nullptr,
        orientations.front().data());
    problem.AddResidualBlock(
        make_vector_cost<VectorPriorCost>(nodes.front().accel_bias,
                                          options.accel_bias_prior_std_mps2),
        nullptr,
        accel_biases.front().data());
    problem.AddResidualBlock(
        make_vector_cost<VectorPriorCost>(nodes.front().gyro_bias,
                                          options.gyro_bias_prior_std_rps),
        nullptr,
        gyro_biases.front().data());

    for (std::size_t i = 0; i < node_count; ++i) {
        if (!nodes[i].has_gnss) {
            continue;
        }
        auto* cost = new ceres::AutoDiffCostFunction<
            AxisWeightedPositionCost, 3, 3>(
            new AxisWeightedPositionCost{
                nodes[i].gnss_position,
                {
                    1.0 / nodes[i].gnss_position_std.x,
                    1.0 / nodes[i].gnss_position_std.y,
                    1.0 / nodes[i].gnss_position_std.z,
                },
            });
        problem.AddResidualBlock(
            cost, new ceres::HuberLoss(1.0), positions[i].data());
    }

    if (options.use_fiveg) {
        for (std::size_t i = 0; i < node_count; ++i) {
            if (!nodes[i].has_fiveg ||
                nodes[i].fiveg_epoch_index >= dataset.fiveg_epochs.size()) {
                continue;
            }
            const auto& epoch = dataset.fiveg_epochs[nodes[i].fiveg_epoch_index];
            for (const auto& observation : epoch.observations) {
                if (!observation.valid || !std::isfinite(observation.measured_range_m) ||
                    observation.measured_range_m <= 0.0) {
                    continue;
                }
                if (!std::isfinite(observation.prs_rsrp_dbm) ||
                    observation.prs_rsrp_dbm < options.fiveg_min_prs_rsrp_dbm) {
                    ++fiveg_rsrp_rejected_count;
                    continue;
                }
                const double sigma = std::max(
                    options.fiveg_range_std_floor_m, observation.range_std_m);
                auto* cost = new ceres::AutoDiffCostFunction<FiveGRangeCost, 1, 3>(
                    new FiveGRangeCost{
                        observation.gnb_position_ned,
                        observation.measured_range_m,
                        1.0 / sigma,
                    });
                problem.AddResidualBlock(
                    cost,
                    new ceres::CauchyLoss(options.fiveg_loss_scale),
                    positions[i].data());
                ++fiveg_range_factor_count;
            }
        }
    }

    for (std::size_t i = 0; i + 1 < node_count; ++i) {
        const ImuPreintegration preintegration =
            preintegrate_with_bias_jacobians(dataset, nodes[i].t, nodes[i + 1].t,
                options.imu_propagated_covariance ? &options : nullptr, nodes[i].accel_bias, nodes[i].gyro_bias);
        if (preintegration.samples == 0 || preintegration.dt <= 0.0) {
            throw std::runtime_error("No IMU samples available between adjacent state nodes");
        }

        auto* cost = new ceres::AutoDiffCostFunction<
            ImuPreintegrationCost, 15, 3, 3, 4, 3, 3, 3, 3, 4, 3, 3>(
            new ImuPreintegrationCost{
                preintegration,
                options.gravity_mps2,
                options.imu_propagated_covariance ? 1.0 : 1.0 / options.imu_position_std_m,
                options.imu_propagated_covariance ? 1.0 : 1.0 / options.imu_velocity_std_mps,
                options.imu_propagated_covariance ? 1.0 : 1.0 / options.imu_attitude_std_rad,
                options.imu_propagated_covariance ? 1.0 : 1.0 / options.accel_bias_rw_std_mps2,
                options.imu_propagated_covariance ? 1.0 : 1.0 / options.gyro_bias_rw_std_rps,
            });
        problem.AddResidualBlock(cost,
                                 nullptr,
                                 positions[i].data(),
                                 velocities[i].data(),
                                 orientations[i].data(),
                                 accel_biases[i].data(),
                                 gyro_biases[i].data(),
                                 positions[i + 1].data(),
                                 velocities[i + 1].data(),
                                 orientations[i + 1].data(),
                                 accel_biases[i + 1].data(),
                                 gyro_biases[i + 1].data());
    }

#ifdef OFFLINE_FUSION_HAS_LIDAR
    if (options.use_lidar) {
        for (const auto& constraint : lidar_odometry.constraints) {
            if (constraint.source_node_index >= node_count ||
                constraint.target_node_index >= node_count) {
                continue;
            }
            auto* cost = new ceres::AutoDiffCostFunction<
                LidarRelativePoseCost, 6, 3, 4, 3, 4>(
                new LidarRelativePoseCost{
                    constraint.translation_source_in_target_body,
                    constraint.rotation_source_to_target_body,
                    1.0 / constraint.translation_std_m,
                    1.0 / constraint.rotation_std_rad,
                });
            problem.AddResidualBlock(
                cost,
                new ceres::HuberLoss(2.0),
                positions[constraint.source_node_index].data(),
                orientations[constraint.source_node_index].data(),
                positions[constraint.target_node_index].data(),
                orientations[constraint.target_node_index].data());
        }
    }
#endif

#ifdef OFFLINE_FUSION_HAS_OPENCV
    std::vector<std::array<double, 3>> landmark_positions;
    if (options.use_visual) {
        landmark_positions.reserve(visual_map.landmarks.size());
        for (const auto& landmark : visual_map.landmarks) {
            landmark_positions.push_back(to_array(landmark.position_ned));
        }
        for (auto& landmark_position : landmark_positions) {
            problem.AddParameterBlock(landmark_position.data(), 3);
            linear_solver_ordering->AddElementToGroup(landmark_position.data(), 0);
        }

        for (std::size_t landmark_index = 0; landmark_index < visual_map.landmarks.size();
             ++landmark_index) {
            const auto& landmark = visual_map.landmarks[landmark_index];
            for (const auto& observation : landmark.observations) {
                if (observation.node_index >= node_count) {
                    continue;
                }
                auto* cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, 3, 4, 3>(
                    new ReprojectionCost{
                        observation.u_px,
                        observation.v_px,
                        options.camera_fx,
                        options.camera_fy,
                        options.camera_cx,
                        options.camera_cy,
                        options.camera_t_body_camera,
                        options.camera_pitch_deg * 3.14159265358979323846 / 180.0,
                        1.0 / options.visual_reprojection_std_px,
                    });
                problem.AddResidualBlock(cost,
                                         new ceres::HuberLoss(3.0),
                                         positions[observation.node_index].data(),
                                         orientations[observation.node_index].data(),
                                         landmark_positions[landmark_index].data());
            }
        }
    }
#endif

    ceres::Solver::Options solver_options;
    if (options.solver_max_iterations <= 0) throw std::runtime_error("solver-max-iterations must be positive");
    solver_options.max_num_iterations = options.solver_max_iterations;
    solver_options.function_tolerance = 1e-5;
    if (options.use_visual) {
#ifdef _MSC_VER
        solver_options.linear_solver_type = ceres::DENSE_SCHUR;
#else
        solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
#endif
    } else if (options.use_lidar || options.use_fiveg || node_count > 300) {
        solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    } else {
        solver_options.linear_solver_type = ceres::DENSE_QR;
    }
    if (options.use_visual) {
        solver_options.linear_solver_ordering = linear_solver_ordering;
    }
    solver_options.minimizer_progress_to_stdout = true;
    solver_options.num_threads = 4;

    ceres::Solver::Summary ceres_summary;
    std::cout << "offline_fusion: solving Ceres problem" << std::endl;
    ceres::Solve(solver_options, &problem, &ceres_summary);
    if (!ceres_summary.IsSolutionUsable()) {
        throw std::runtime_error("Ceres failed: " + ceres_summary.BriefReport());
    }

    FusionSummary local_summary;
    local_summary.start_time = start_time;
    local_summary.end_time = end_time;
    local_summary.imu_used = static_cast<std::size_t>(std::count_if(
        dataset.imu.begin(), dataset.imu.end(), [start_time, end_time](const ImuSample& sample) {
            return in_range(sample.t, start_time, end_time);
        }));
    local_summary.gnss_epochs_used = gnss_node_count;
    local_summary.states_written = node_count;
    local_summary.state_nodes = node_count;
    local_summary.prior_factors = 15;
    local_summary.gnss_factors = gnss_node_count * 3;
    local_summary.gnss_fix_factors = 0;
    local_summary.gnss_spp_epochs_solved = dataset.gnss_spp_diagnostics.epochs_solved;
    local_summary.gnss_spp_epochs_rejected = dataset.gnss_spp_diagnostics.epochs_rejected;
    local_summary.gnss_spp_mean_satellites = dataset.gnss_spp_diagnostics.mean_satellites;
    local_summary.gnss_spp_mean_postfit_rms_m =
        dataset.gnss_spp_diagnostics.mean_postfit_rms_m;
    local_summary.gnss_spp_max_postfit_rms_m =
        dataset.gnss_spp_diagnostics.max_postfit_rms_m;
    local_summary.gnss_spp_min_weight =
        dataset.gnss_spp_diagnostics.min_normalized_weight;
    local_summary.gnss_spp_max_weight =
        dataset.gnss_spp_diagnostics.max_normalized_weight;
    local_summary.gnss_fixed_satellite_consistency_rms_m =
        dataset.gnss_spp_diagnostics.fixed_satellite_consistency_rms_m;
    local_summary.gnss_fixed_satellite_consistency_max_m =
        dataset.gnss_spp_diagnostics.fixed_satellite_consistency_max_m;
    local_summary.gnss_fixed_satellite_model_consistent =
        dataset.gnss_spp_diagnostics.fixed_satellite_model_consistent;
    local_summary.fiveg_enabled = options.use_fiveg && fiveg_range_factor_count > 0;
    local_summary.fiveg_epochs_used = fiveg_node_count;
    local_summary.fiveg_range_factors = fiveg_range_factor_count;
    local_summary.fiveg_rsrp_rejected = fiveg_rsrp_rejected_count;
    local_summary.imu_factors = (node_count - 1) * 6;
    local_summary.attitude_factors = (node_count - 1) * 3;
    local_summary.bias_factors = (node_count - 1) * 6;
    if (sensor_initialization_ptr != nullptr) {
        constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
        local_summary.initialization_source = "imu_gravity_and_spp_like_gnss_velocity";
        local_summary.groundtruth_used_for_initialization = false;
        local_summary.groundtruth_used_for_frame_anchor =
            dataset.format == DatasetFormat::LegacyText;
        local_summary.initialization_gravity_samples =
            sensor_initialization.gravity_samples;
        local_summary.initialization_heading_time_s = sensor_initialization.heading_time;
        local_summary.initialization_roll_deg = sensor_initialization.roll_rad * kRadToDeg;
        local_summary.initialization_pitch_deg = sensor_initialization.pitch_rad * kRadToDeg;
        local_summary.initialization_yaw_deg = sensor_initialization.yaw_rad * kRadToDeg;
    }
#ifdef OFFLINE_FUSION_HAS_OPENCV
    local_summary.visual_enabled = options.use_visual && !visual_map.landmarks.empty();
    local_summary.camera_frames_loaded = visual_map.frames_loaded;
    local_summary.visual_keyframes = visual_node_count;
    local_summary.visual_tracks = visual_map.tracks;
    local_summary.visual_landmarks = visual_map.landmarks.size();
    local_summary.visual_reprojection_factors = visual_map.reprojection_factors;
    local_summary.visual_rejected_tracks = visual_map.rejected_tracks;
    local_summary.visual_initial_reprojection_observations =
        visual_map.initial_reprojection_observations;
    local_summary.visual_initial_reprojection_rmse_px =
        visual_map.initial_reprojection_rmse_px;
#else
    local_summary.visual_enabled = false;
    local_summary.camera_frames_loaded = dataset.camera_frames.size();
    local_summary.visual_keyframes = 0;
#endif
#ifdef OFFLINE_FUSION_HAS_LIDAR
    local_summary.lidar_enabled =
        options.use_lidar && !lidar_odometry.constraints.empty();
    local_summary.lidar_frames_loaded = lidar_odometry.frames_loaded;
    local_summary.lidar_keyframes = lidar_node_count;
    local_summary.lidar_constraints = lidar_odometry.constraints.size();
    local_summary.lidar_rejected_pairs = lidar_odometry.rejected_pairs;
    local_summary.lidar_correspondences = lidar_odometry.total_correspondences;
    local_summary.lidar_mean_icp_rmse_m = lidar_odometry.mean_rmse_m;
    local_summary.lidar_mean_overlap = lidar_odometry.mean_overlap;
    local_summary.lidar_mean_translation_std_m =
        lidar_odometry.mean_translation_std_m;
    local_summary.lidar_mean_rotation_std_rad =
        lidar_odometry.mean_rotation_std_rad;
#else
    local_summary.lidar_enabled = false;
    local_summary.lidar_frames_loaded = dataset.lidar_frames.size();
#endif
    local_summary.initial_cost = ceres_summary.initial_cost;
    local_summary.final_cost = ceres_summary.final_cost;
    local_summary.solver = "Ceres 2 nonlinear least squares with quaternion manifold";
    local_summary.termination = termination_name(ceres_summary.termination_type);

    std::vector<TrajectoryState> trajectory;
    trajectory.reserve(node_count);
    for (std::size_t i = 0; i < node_count; ++i) {
        TrajectoryState state;
        state.t = nodes[i].t;
        state.position_ned = to_vec3(positions[i]);
        state.velocity_ned = to_vec3(velocities[i]);
        state.q_body_to_ned = to_quat(orientations[i]);
        state.accel_bias = to_vec3(accel_biases[i]);
        state.gyro_bias = to_vec3(gyro_biases[i]);
        state.update_source = "ceres";
        if (local_summary.fiveg_enabled) {
            state.update_source += "_fiveg";
        }
        if (local_summary.lidar_enabled) {
            state.update_source += "_lidar";
        }
        if (local_summary.visual_enabled) {
            state.update_source += "_visual";
        }
        state.update_source += "_inertial_spp_like_gnss";
        state.update_source += "_factor_graph";
        trajectory.push_back(state);
    }

    if (summary != nullptr) {
        *summary = local_summary;
    }
    return trajectory;
}

}  // namespace offline_fusion
