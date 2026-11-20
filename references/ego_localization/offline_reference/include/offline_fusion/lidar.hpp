#pragma once

#include <cstddef>
#include <filesystem>
#include <vector>

#include "offline_fusion/fusion.hpp"
#include "offline_fusion/types.hpp"

namespace offline_fusion {

struct LidarPose {
    double t = 0.0;
    Vec3 position_ned;
    Quat q_body_to_ned;
};

struct LidarRelativeConstraint {
    std::size_t source_node_index = 0;
    std::size_t target_node_index = 0;
    Vec3 translation_source_in_target_body;
    Quat rotation_source_to_target_body;
    double correspondence_rmse_m = 0.0;
    double overlap = 0.0;
    std::size_t correspondences = 0;
    double correction_translation_m = 0.0;
    double correction_translation_ratio = 0.0;
    double translation_std_m = 0.0;
    double rotation_std_rad = 0.0;
};

struct LidarPairDiagnostic {
    std::size_t source_node_index = 0;
    std::size_t target_node_index = 0;
    double source_time_s = 0.0;
    double target_time_s = 0.0;
    bool accepted = false;
    double correspondence_rmse_m = 0.0;
    double overlap = 0.0;
    std::size_t correspondences = 0;
    double initial_translation_norm_m = 0.0;
    double estimated_translation_norm_m = 0.0;
    double correction_translation_m = 0.0;
    double correction_rotation_rad = 0.0;
    double correction_translation_ratio = 0.0;
    double translation_std_m = 0.0;
    double rotation_std_rad = 0.0;
    Vec3 initial_delta_ned;
    Vec3 estimated_delta_ned;
};

struct LidarOdometry {
    std::vector<LidarRelativeConstraint> constraints;
    std::vector<LidarPairDiagnostic> pair_diagnostics;
    std::size_t frames_loaded = 0;
    std::size_t keyframes = 0;
    std::size_t attempted_pairs = 0;
    std::size_t rejected_pairs = 0;
    std::size_t total_correspondences = 0;
    double mean_rmse_m = 0.0;
    double mean_overlap = 0.0;
    double mean_translation_std_m = 0.0;
    double mean_rotation_std_rad = 0.0;
};

std::vector<double> select_lidar_keyframe_times(const Dataset& dataset,
                                                double start_time,
                                                double end_time,
                                                const FusionOptions& options);

LidarOdometry build_lidar_odometry(const Dataset& dataset,
                                   const std::vector<LidarPose>& initial_poses,
                                   double start_time,
                                   double end_time,
                                   const FusionOptions& options);

void write_lidar_diagnostics_csv(const std::filesystem::path& path,
                                 const LidarOdometry& odometry);

}  // namespace offline_fusion
