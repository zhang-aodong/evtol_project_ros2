#include "offline_fusion/lidar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/flann.hpp>

#include "offline_fusion/dataset.hpp"
#include "offline_fusion/math.hpp"

namespace offline_fusion {
namespace {

struct VoxelKey {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const {
        const auto hx = std::hash<int>{}(key.x);
        const auto hy = std::hash<int>{}(key.y);
        const auto hz = std::hash<int>{}(key.z);
        return hx ^ (hy << 1U) ^ (hz << 2U);
    }
};

struct VoxelAccumulator {
    cv::Vec3d sum{0.0, 0.0, 0.0};
    std::size_t count = 0;
};

struct SelectedFrame {
    const LidarFrame* frame = nullptr;
    std::size_t node_index = 0;
};

struct IcpResult {
    bool valid = false;
    cv::Matx33d rotation = cv::Matx33d::eye();
    cv::Vec3d translation{0.0, 0.0, 0.0};
    double rmse_m = std::numeric_limits<double>::infinity();
    double overlap = 0.0;
    std::size_t correspondences = 0;
    double correction_translation_m = 0.0;
    double correction_rotation_rad = 0.0;
    double correction_translation_ratio = 0.0;
};

bool in_range(double t, double start, double end) {
    return t >= start && t <= end;
}

double squared_norm(const cv::Vec3d& value) {
    return value.dot(value);
}

cv::Matx33d rotation_matrix(const Quat& quaternion) {
    const Vec3 x = rotate(quaternion, {1.0, 0.0, 0.0});
    const Vec3 y = rotate(quaternion, {0.0, 1.0, 0.0});
    const Vec3 z = rotate(quaternion, {0.0, 0.0, 1.0});
    return cv::Matx33d(x.x, y.x, z.x,
                       x.y, y.y, z.y,
                       x.z, y.z, z.z);
}

Quat quaternion_from_rotation(const cv::Matx33d& rotation) {
    cv::Mat rotation_vector;
    cv::Rodrigues(cv::Mat(rotation), rotation_vector);
    return quat_from_delta_angle({rotation_vector.at<double>(0, 0),
                                  rotation_vector.at<double>(1, 0),
                                  rotation_vector.at<double>(2, 0)});
}

std::optional<std::size_t> find_pose_index(const std::vector<LidarPose>& poses, double t) {
    constexpr double kTolerance = 1e-5;
    const auto it = std::lower_bound(
        poses.begin(), poses.end(), t, [](const LidarPose& pose, double value) {
            return pose.t < value;
        });
    if (it != poses.end() && std::abs(it->t - t) <= kTolerance) {
        return static_cast<std::size_t>(std::distance(poses.begin(), it));
    }
    if (it != poses.begin()) {
        const auto previous = it - 1;
        if (std::abs(previous->t - t) <= kTolerance) {
            return static_cast<std::size_t>(std::distance(poses.begin(), previous));
        }
    }
    return std::nullopt;
}

std::vector<cv::Point3f> voxel_downsample(const LidarFrame& frame,
                                          const LidarPose& pose,
                                          const FusionOptions& options) {
    const std::vector<LidarPoint> points = load_lidar_points(frame);
    std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels;
    voxels.reserve(points.size());
    const double inverse_voxel = 1.0 / options.lidar_voxel_size_m;
    const double min_range_squared =
        options.lidar_min_range_m * options.lidar_min_range_m;
    const double max_range_squared =
        options.lidar_max_range_m * options.lidar_max_range_m;

    for (const auto& point : points) {
        cv::Vec3d xyz(point.x, point.y, point.z);
        if (options.lidar_points_vehicle_inertial_frame) {
            // VehicleInertialFrame -> body/LiDAR local frame. The input frame
            // uses the fixed NED axes; pose is the current body pose in NED.
            const Vec3 delta{xyz[0] - pose.position_ned.x,
                             xyz[1] - pose.position_ned.y,
                             xyz[2] - pose.position_ned.z};
            const Vec3 local = rotate_inverse(pose.q_body_to_ned, delta);
            xyz = cv::Vec3d(local.x - options.lidar_t_body_lidar.x,
                            local.y - options.lidar_t_body_lidar.y,
                            local.z - options.lidar_t_body_lidar.z);
        }
        const double range_squared = squared_norm(xyz);
        if (!std::isfinite(range_squared) || range_squared < min_range_squared ||
            range_squared > max_range_squared) {
            continue;
        }
        const VoxelKey key{
            static_cast<int>(std::floor(xyz[0] * inverse_voxel)),
            static_cast<int>(std::floor(xyz[1] * inverse_voxel)),
            static_cast<int>(std::floor(xyz[2] * inverse_voxel)),
        };
        auto& accumulator = voxels[key];
        accumulator.sum += xyz;
        ++accumulator.count;
    }

    std::vector<cv::Point3f> downsampled;
    downsampled.reserve(voxels.size());
    for (const auto& item : voxels) {
        const cv::Vec3d centroid =
            item.second.sum / static_cast<double>(item.second.count);
        downsampled.emplace_back(static_cast<float>(centroid[0]),
                                 static_cast<float>(centroid[1]),
                                 static_cast<float>(centroid[2]));
    }
    if (downsampled.size() <= static_cast<std::size_t>(options.lidar_max_points)) {
        return downsampled;
    }

    std::vector<cv::Point3f> limited;
    limited.reserve(static_cast<std::size_t>(options.lidar_max_points));
    for (int i = 0; i < options.lidar_max_points; ++i) {
        const std::size_t index = static_cast<std::size_t>(
            static_cast<double>(i) * static_cast<double>(downsampled.size()) /
            static_cast<double>(options.lidar_max_points));
        limited.push_back(downsampled[std::min(index, downsampled.size() - 1)]);
    }
    return limited;
}

cv::Vec3d transform_point(const cv::Matx33d& rotation,
                          const cv::Vec3d& translation,
                          const cv::Point3f& point) {
    return rotation * cv::Vec3d(point.x, point.y, point.z) + translation;
}

double rotation_angle(const cv::Matx33d& rotation) {
    const double cosine = std::clamp((cv::trace(cv::Mat(rotation))[0] - 1.0) * 0.5,
                                     -1.0,
                                     1.0);
    return std::acos(cosine);
}

std::vector<cv::Vec3d> estimate_target_normals(const cv::Mat& target_matrix,
                                                cv::flann::Index* target_index) {
    const int neighbor_count = std::min(12, target_matrix.rows);
    cv::Mat neighbor_indices;
    cv::Mat neighbor_distances;
    target_index->knnSearch(target_matrix,
                            neighbor_indices,
                            neighbor_distances,
                            neighbor_count,
                            cv::flann::SearchParams(32));
    std::vector<cv::Vec3d> normals(
        static_cast<std::size_t>(target_matrix.rows), cv::Vec3d(0.0, 0.0, 0.0));
    for (int i = 0; i < target_matrix.rows; ++i) {
        cv::Vec3d centroid(0.0, 0.0, 0.0);
        for (int k = 0; k < neighbor_count; ++k) {
            const int index = neighbor_indices.at<int>(i, k);
            centroid += cv::Vec3d(target_matrix.at<float>(index, 0),
                                  target_matrix.at<float>(index, 1),
                                  target_matrix.at<float>(index, 2));
        }
        centroid /= static_cast<double>(neighbor_count);

        cv::Matx33d covariance = cv::Matx33d::zeros();
        for (int k = 0; k < neighbor_count; ++k) {
            const int index = neighbor_indices.at<int>(i, k);
            const cv::Vec3d centered(
                static_cast<double>(target_matrix.at<float>(index, 0)) - centroid[0],
                static_cast<double>(target_matrix.at<float>(index, 1)) - centroid[1],
                static_cast<double>(target_matrix.at<float>(index, 2)) - centroid[2]);
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    covariance(row, col) += centered[row] * centered[col];
                }
            }
        }
        cv::Mat eigenvalues;
        cv::Mat eigenvectors;
        if (!cv::eigen(cv::Mat(covariance), eigenvalues, eigenvectors)) {
            continue;
        }
        const double eigenvalue_sum = cv::sum(eigenvalues)[0];
        if (eigenvalue_sum <= 1e-12 ||
            eigenvalues.at<double>(2, 0) / eigenvalue_sum > 0.15) {
            continue;
        }
        cv::Vec3d normal(eigenvectors.at<double>(2, 0),
                         eigenvectors.at<double>(2, 1),
                         eigenvectors.at<double>(2, 2));
        const double normal_norm = std::sqrt(squared_norm(normal));
        if (normal_norm > 1e-12) {
            normals[static_cast<std::size_t>(i)] = normal / normal_norm;
        }
    }
    return normals;
}

IcpResult run_icp(const std::vector<cv::Point3f>& source,
                  const std::vector<cv::Point3f>& target,
                  const cv::Matx33d& initial_rotation,
                  const cv::Vec3d& initial_translation,
                  const FusionOptions& options) {
    IcpResult result;
    result.rotation = initial_rotation;
    result.translation = initial_translation;
    if (source.size() < static_cast<std::size_t>(options.lidar_min_correspondences) ||
        target.size() < static_cast<std::size_t>(options.lidar_min_correspondences)) {
        return result;
    }

    cv::Mat target_matrix(static_cast<int>(target.size()), 3, CV_32F);
    for (std::size_t i = 0; i < target.size(); ++i) {
        target_matrix.at<float>(static_cast<int>(i), 0) = target[i].x;
        target_matrix.at<float>(static_cast<int>(i), 1) = target[i].y;
        target_matrix.at<float>(static_cast<int>(i), 2) = target[i].z;
    }
    cv::flann::Index target_index(target_matrix, cv::flann::KDTreeIndexParams(4));
    const std::vector<cv::Vec3d> target_normals =
        estimate_target_normals(target_matrix, &target_index);
    const double max_distance_squared =
        options.lidar_max_correspondence_distance_m *
        options.lidar_max_correspondence_distance_m;

    for (int iteration = 0; iteration < options.lidar_icp_max_iterations; ++iteration) {
        cv::Mat query_matrix(static_cast<int>(source.size()), 3, CV_32F);
        for (std::size_t i = 0; i < source.size(); ++i) {
            const cv::Vec3d transformed =
                transform_point(result.rotation, result.translation, source[i]);
            query_matrix.at<float>(static_cast<int>(i), 0) =
                static_cast<float>(transformed[0]);
            query_matrix.at<float>(static_cast<int>(i), 1) =
                static_cast<float>(transformed[1]);
            query_matrix.at<float>(static_cast<int>(i), 2) =
                static_cast<float>(transformed[2]);
        }

        cv::Mat indices;
        cv::Mat distances;
        target_index.knnSearch(query_matrix,
                               indices,
                               distances,
                               1,
                               cv::flann::SearchParams(32));
        cv::Mat normal_matrix = cv::Mat::zeros(6, 6, CV_64F);
        cv::Mat gradient = cv::Mat::zeros(6, 1, CV_64F);
        std::size_t usable_correspondences = 0;
        for (std::size_t i = 0; i < source.size(); ++i) {
            const float squared_distance = distances.at<float>(static_cast<int>(i), 0);
            if (!std::isfinite(squared_distance) || squared_distance > max_distance_squared) {
                continue;
            }
            const int target_point_index = indices.at<int>(static_cast<int>(i), 0);
            const auto& point = target[static_cast<std::size_t>(target_point_index)];
            const cv::Vec3d normal =
                target_normals[static_cast<std::size_t>(target_point_index)];
            if (squared_norm(normal) < 0.5) {
                continue;
            }
            const cv::Vec3d transformed =
                transform_point(result.rotation, result.translation, source[i]);
            const cv::Vec3d target_point(point.x, point.y, point.z);
            const double residual = normal.dot(transformed - target_point);
            const double huber_delta = 0.3;
            const double weight =
                std::abs(residual) <= huber_delta
                    ? 1.0
                    : huber_delta / std::max(std::abs(residual), 1e-12);
            const cv::Vec3d rotation_jacobian = transformed.cross(normal);
            const double jacobian[6] = {rotation_jacobian[0],
                                        rotation_jacobian[1],
                                        rotation_jacobian[2],
                                        normal[0],
                                        normal[1],
                                        normal[2]};
            for (int row = 0; row < 6; ++row) {
                gradient.at<double>(row, 0) += weight * jacobian[row] * residual;
                for (int col = 0; col < 6; ++col) {
                    normal_matrix.at<double>(row, col) +=
                        weight * jacobian[row] * jacobian[col];
                }
            }
            ++usable_correspondences;
        }
        if (usable_correspondences <
            static_cast<std::size_t>(options.lidar_min_correspondences)) {
            return result;
        }

        if (options.lidar_min_information_ratio > 0.0) {
            cv::Mat normalized(6,6,CV_64F);
            for (int r=0; r<6; ++r) for (int c=0; c<6; ++c) {
                if (normal_matrix.at<double>(r,r) <= 1e-12 || normal_matrix.at<double>(c,c) <= 1e-12)
                    return result;
                normalized.at<double>(r,c) = normal_matrix.at<double>(r,c) /
                    std::sqrt(normal_matrix.at<double>(r,r)*normal_matrix.at<double>(c,c));
            }
            cv::Mat eigenvalues;
            if (!cv::eigen(normalized,eigenvalues) || eigenvalues.at<double>(5) <
                options.lidar_min_information_ratio*eigenvalues.at<double>(0)) return result;
        }
        for (int dim = 0; dim < 6; ++dim) {
            normal_matrix.at<double>(dim, dim) += dim < 3 ? 1e-3 : 1e-2;
        }
        cv::Mat increment;
        if (!cv::solve(normal_matrix, -gradient, increment, cv::DECOMP_CHOLESKY)) {
            return result;
        }
        cv::Vec3d delta_rotation_vector(increment.at<double>(0, 0),
                                        increment.at<double>(1, 0),
                                        increment.at<double>(2, 0));
        cv::Vec3d delta_translation(increment.at<double>(3, 0),
                                    increment.at<double>(4, 0),
                                    increment.at<double>(5, 0));
        const double delta_angle = std::sqrt(squared_norm(delta_rotation_vector));
        if (delta_angle > 0.03) {
            delta_rotation_vector *= 0.03 / delta_angle;
        }
        const double delta_translation_norm = std::sqrt(squared_norm(delta_translation));
        if (delta_translation_norm > 0.2) {
            delta_translation *= 0.2 / delta_translation_norm;
        }
        cv::Mat delta_rotation_mat;
        cv::Rodrigues(cv::Mat(delta_rotation_vector), delta_rotation_mat);
        cv::Matx33d delta_rotation;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                delta_rotation(row, col) = delta_rotation_mat.at<double>(row, col);
            }
        }
        result.rotation = delta_rotation * result.rotation;
        result.translation = delta_rotation * result.translation + delta_translation;
        if (std::sqrt(squared_norm(delta_translation)) < 1e-4 &&
            rotation_angle(delta_rotation) < 1e-5) {
            break;
        }
    }

    cv::Mat query_matrix(static_cast<int>(source.size()), 3, CV_32F);
    for (std::size_t i = 0; i < source.size(); ++i) {
        const cv::Vec3d transformed =
            transform_point(result.rotation, result.translation, source[i]);
        query_matrix.at<float>(static_cast<int>(i), 0) = static_cast<float>(transformed[0]);
        query_matrix.at<float>(static_cast<int>(i), 1) = static_cast<float>(transformed[1]);
        query_matrix.at<float>(static_cast<int>(i), 2) = static_cast<float>(transformed[2]);
    }
    cv::Mat indices;
    cv::Mat distances;
    target_index.knnSearch(
        query_matrix, indices, distances, 1, cv::flann::SearchParams(64));
    double squared_error_sum = 0.0;
    for (int i = 0; i < distances.rows; ++i) {
        const double squared_distance = distances.at<float>(i, 0);
        if (std::isfinite(squared_distance) && squared_distance <= max_distance_squared) {
            squared_error_sum += squared_distance;
            ++result.correspondences;
        }
    }
    if (result.correspondences == 0) {
        return result;
    }
    result.rmse_m =
        std::sqrt(squared_error_sum / static_cast<double>(result.correspondences));
    result.overlap =
        static_cast<double>(result.correspondences) / static_cast<double>(source.size());

    const cv::Matx33d correction_rotation = result.rotation * initial_rotation.t();
    result.correction_translation_m =
        std::sqrt(squared_norm(result.translation - initial_translation));
    result.correction_rotation_rad = rotation_angle(correction_rotation);
    const double predicted_translation_m =
        std::sqrt(squared_norm(initial_translation));
    result.correction_translation_ratio =
        result.correction_translation_m / std::max(predicted_translation_m, 0.05);
    const bool correction_ratio_valid =
        options.lidar_max_correction_ratio <= 0.0 ||
        result.correction_translation_ratio <= options.lidar_max_correction_ratio;
    result.valid =
        result.correspondences >=
            static_cast<std::size_t>(options.lidar_min_correspondences) &&
        result.overlap >= options.lidar_min_overlap &&
        result.rmse_m <= options.lidar_max_icp_rmse_m &&
        result.correction_translation_m <=
            options.lidar_max_correction_translation_m &&
        result.correction_rotation_rad <= options.lidar_max_correction_rotation_rad &&
        correction_ratio_valid;
    return result;
}

std::pair<cv::Matx33d, cv::Vec3d> predicted_sensor_transform(
    const LidarPose& source,
    const LidarPose& target,
    const FusionOptions& options) {
    const Quat q_target_source = quat_multiply(
        quat_conjugate(target.q_body_to_ned), source.q_body_to_ned);
    const cv::Matx33d rotation = rotation_matrix(q_target_source);
    const Vec3 translated_source_lidar =
        source.position_ned + rotate(source.q_body_to_ned, options.lidar_t_body_lidar);
    const Vec3 translation_body =
        rotate_inverse(target.q_body_to_ned, translated_source_lidar - target.position_ned) -
        options.lidar_t_body_lidar;
    return {rotation,
            cv::Vec3d(translation_body.x, translation_body.y, translation_body.z)};
}

}  // namespace

std::vector<double> select_lidar_keyframe_times(const Dataset& dataset,
                                                double start_time,
                                                double end_time,
                                                const FusionOptions& options) {
    std::vector<double> keyframe_times;
    if (!options.use_lidar || dataset.lidar_frames.empty()) {
        return keyframe_times;
    }

    double next_keyframe_time = start_time;
    for (const auto& frame : dataset.lidar_frames) {
        if (!in_range(frame.t, start_time, end_time)) {
            continue;
        }
        if (keyframe_times.empty() || frame.t + 1e-6 >= next_keyframe_time) {
            keyframe_times.push_back(frame.t);
            next_keyframe_time = frame.t + options.lidar_keyframe_interval_s;
        }
    }
    return keyframe_times;
}

LidarOdometry build_lidar_odometry(const Dataset& dataset,
                                   const std::vector<LidarPose>& initial_poses,
                                   double start_time,
                                   double end_time,
                                   const FusionOptions& options) {
    LidarOdometry odometry;
    if (!options.use_lidar || dataset.lidar_frames.empty()) {
        return odometry;
    }
    if (initial_poses.empty()) {
        throw std::runtime_error("Cannot build LiDAR odometry without initial state poses");
    }
    if (options.lidar_keyframe_interval_s <= 0.0 ||
        options.lidar_voxel_size_m <= 0.0 || options.lidar_max_points <= 0 ||
        options.lidar_max_correspondence_distance_m <= 0.0 ||
        options.lidar_translation_std_m <= 0.0 ||
        options.lidar_rotation_std_rad <= 0.0 ||
        options.lidar_adaptive_max_scale < 1.0) {
        throw std::runtime_error("Invalid non-positive LiDAR configuration value");
    }

    std::vector<SelectedFrame> selected;
    double next_keyframe_time = start_time;
    for (const auto& frame : dataset.lidar_frames) {
        if (!in_range(frame.t, start_time, end_time)) {
            continue;
        }
        if (!selected.empty() && frame.t + 1e-6 < next_keyframe_time) {
            continue;
        }
        const auto node_index = find_pose_index(initial_poses, frame.t);
        if (!node_index.has_value()) {
            continue;
        }
        selected.push_back({&frame, *node_index});
        next_keyframe_time = frame.t + options.lidar_keyframe_interval_s;
    }
    odometry.keyframes = selected.size();
    if (selected.size() < 2) {
        return odometry;
    }

    std::vector<cv::Point3f> source_points =
        voxel_downsample(*selected.front().frame,
                         initial_poses[selected.front().node_index], options);
    ++odometry.frames_loaded;
    double rmse_sum = 0.0;
    double overlap_sum = 0.0;
    for (std::size_t i = 1; i < selected.size(); ++i) {
        std::vector<cv::Point3f> target_points =
            voxel_downsample(*selected[i].frame,
                             initial_poses[selected[i].node_index], options);
        ++odometry.frames_loaded;
        ++odometry.attempted_pairs;

        const LidarPose& source_pose = initial_poses[selected[i - 1].node_index];
        const LidarPose& target_pose = initial_poses[selected[i].node_index];
        const auto initial_transform =
            predicted_sensor_transform(source_pose, target_pose, options);
        const IcpResult icp = run_icp(source_points,
                                     target_points,
                                     initial_transform.first,
                                     initial_transform.second,
                                     options);
        const Quat relative_rotation = quaternion_from_rotation(icp.rotation);
        const Vec3 sensor_translation{
            icp.translation[0], icp.translation[1], icp.translation[2]};
        const Vec3 body_translation =
            sensor_translation - rotate(relative_rotation, options.lidar_t_body_lidar) +
            options.lidar_t_body_lidar;
        const Vec3 initial_delta_ned = target_pose.position_ned - source_pose.position_ned;
        const Vec3 estimated_delta_ned =
            rotate(target_pose.q_body_to_ned, body_translation * -1.0);
        double quality_scale = 1.0;
        if (options.lidar_adaptive_covariance && std::isfinite(icp.rmse_m)) {
            const double rmse_fraction =
                icp.rmse_m / std::max(options.lidar_max_icp_rmse_m, 1e-6);
            const double overlap_penalty =
                1.0 / std::sqrt(std::max(icp.overlap, 0.1));
            const double correction_penalty =
                1.0 + 0.5 * std::min(icp.correction_translation_ratio, 2.0);
            quality_scale = std::clamp(
                (1.0 + rmse_fraction) * overlap_penalty * correction_penalty,
                1.0,
                options.lidar_adaptive_max_scale);
        }
        const double translation_std_m =
            options.lidar_translation_std_m * quality_scale;
        const double rotation_std_rad =
            options.lidar_rotation_std_rad * quality_scale;
        odometry.pair_diagnostics.push_back({selected[i - 1].node_index,
                                             selected[i].node_index,
                                             source_pose.t,
                                             target_pose.t,
                                             icp.valid,
                                             icp.rmse_m,
                                             icp.overlap,
                                             icp.correspondences,
                                             std::sqrt(squared_norm(initial_transform.second)),
                                             std::sqrt(squared_norm(icp.translation)),
                                             icp.correction_translation_m,
                                             icp.correction_rotation_rad,
                                             icp.correction_translation_ratio,
                                             translation_std_m,
                                             rotation_std_rad,
                                             initial_delta_ned,
                                             estimated_delta_ned});
        if (!icp.valid) {
            ++odometry.rejected_pairs;
            source_points = std::move(target_points);
            continue;
        }
        odometry.constraints.push_back({selected[i - 1].node_index,
                                        selected[i].node_index,
                                        body_translation,
                                        relative_rotation,
                                        icp.rmse_m,
                                        icp.overlap,
                                        icp.correspondences,
                                        icp.correction_translation_m,
                                        icp.correction_translation_ratio,
                                        translation_std_m,
                                        rotation_std_rad});
        odometry.total_correspondences += icp.correspondences;
        rmse_sum += icp.rmse_m;
        overlap_sum += icp.overlap;
        odometry.mean_translation_std_m += translation_std_m;
        odometry.mean_rotation_std_rad += rotation_std_rad;
        source_points = std::move(target_points);
    }

    if (!odometry.constraints.empty()) {
        odometry.mean_rmse_m =
            rmse_sum / static_cast<double>(odometry.constraints.size());
        odometry.mean_overlap =
            overlap_sum / static_cast<double>(odometry.constraints.size());
        odometry.mean_translation_std_m /=
            static_cast<double>(odometry.constraints.size());
        odometry.mean_rotation_std_rad /=
            static_cast<double>(odometry.constraints.size());
    }
    return odometry;
}

void write_lidar_diagnostics_csv(const std::filesystem::path& path,
                                 const LidarOdometry& odometry) {
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("Failed to open LiDAR diagnostic output " + path.string());
    }
    output << "source_node_index,target_node_index,source_time_s,target_time_s,accepted,"
              "rmse_m,overlap,correspondences,initial_translation_norm_m,"
              "estimated_translation_norm_m,correction_translation_m,"
              "correction_rotation_rad,correction_translation_ratio,"
              "translation_std_m,rotation_std_rad,"
              "initial_delta_n_m,initial_delta_e_m,"
              "initial_delta_d_m,estimated_delta_n_m,estimated_delta_e_m,"
              "estimated_delta_d_m\n";
    output << std::fixed << std::setprecision(9);
    for (const auto& diagnostic : odometry.pair_diagnostics) {
        output << diagnostic.source_node_index << ','
               << diagnostic.target_node_index << ','
               << diagnostic.source_time_s << ','
               << diagnostic.target_time_s << ','
               << (diagnostic.accepted ? 1 : 0) << ','
               << diagnostic.correspondence_rmse_m << ','
               << diagnostic.overlap << ','
               << diagnostic.correspondences << ','
               << diagnostic.initial_translation_norm_m << ','
               << diagnostic.estimated_translation_norm_m << ','
               << diagnostic.correction_translation_m << ','
               << diagnostic.correction_rotation_rad << ','
               << diagnostic.correction_translation_ratio << ','
               << diagnostic.translation_std_m << ','
               << diagnostic.rotation_std_rad << ','
               << diagnostic.initial_delta_ned.x << ','
               << diagnostic.initial_delta_ned.y << ','
               << diagnostic.initial_delta_ned.z << ','
               << diagnostic.estimated_delta_ned.x << ','
               << diagnostic.estimated_delta_ned.y << ','
               << diagnostic.estimated_delta_ned.z << '\n';
    }
}

}  // namespace offline_fusion
