#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "offline_fusion/types.hpp"

namespace offline_fusion {

struct FusionOptions {
    std::filesystem::path data_dir = "simdata_v2";
    std::filesystem::path output_csv = std::filesystem::path("results") /
                                       "simdata_v2_ceres_visual" /
                                       "trajectory_est.csv";
    double absolute_start_s = -1.0;
    double absolute_end_s = -1.0;
    int solver_max_iterations = 100;
    double visual_min_triangulation_deg = 0.5;
    double lidar_min_information_ratio = 1e-4;
    double start_offset_s = 0.0;
    double end_offset_s = -1.0;
    double gravity_mps2 = 9.793;
    double initialization_gravity_window_s = 3.0;
    double initialization_gravity_tolerance_mps2 = 0.25;
    double initialization_max_gyro_rps = 0.02;
    double initialization_heading_speed_mps = 1.5;
    double prior_position_std_m = 0.05;
    double prior_velocity_std_mps = 0.05;
    double gnss_position_std_m = 3.0;
    double gnss_position_noise_scale = 1.0;
    bool use_fiveg = false;
    double fiveg_range_std_floor_m = 0.5;
    double fiveg_loss_scale = 2.0;
    double fiveg_min_prs_rsrp_dbm = -32.0;
    bool imu_propagated_covariance = true;
    double imu_accel_noise_density = 0.2; // (m/s^2)/sqrt(Hz), engineering default
    double imu_gyro_noise_density = 0.02; // (rad/s)/sqrt(Hz), engineering default
    double imu_accel_bias_density = 0.05; // (m/s^2)/sqrt(s)
    double imu_gyro_bias_density = 0.005; // (rad/s)/sqrt(s)
    double extra_state_interval_s = 0.0; // diagnostic nodes, default disabled
    double imu_position_std_m = 0.5;
    double imu_velocity_std_mps = 0.2;
    double imu_attitude_std_rad = 0.05;
    double attitude_prior_std_rad = 0.02;
    double accel_bias_prior_std_mps2 = 0.2;
    double gyro_bias_prior_std_rps = 0.02;
    double accel_bias_rw_std_mps2 = 0.05;
    double gyro_bias_rw_std_rps = 0.005;
    double normal_matrix_damping = 1e-8;
    bool use_visual = true;
    double visual_keyframe_interval_s = 0.25;
    int visual_max_features = 300;
    int visual_max_landmarks = 300;
    int visual_max_observations_per_landmark = 8;
    double visual_min_parallax_px = 12.0;
    double visual_reprojection_std_px = 2.0;
    double visual_max_initial_reprojection_px = 8.0;
    double visual_image_scale = 0.5;
    bool visual_prune_unconstrained_nodes = false;
    double camera_pitch_deg = 0.0;  // AirSim FRD pitch: negative looks down.
    double camera_fx = 3248.0;
    double camera_fy = 3248.0;
    double camera_cx = 1024.0;
    double camera_cy = 768.0;
    int camera_width = 2048;
    int camera_height = 1536;
    Vec3 camera_t_body_camera{0.5, 0.0, -0.1};
    bool use_lidar = false;
    double lidar_keyframe_interval_s = 0.2;
    double lidar_voxel_size_m = 0.5;
    double lidar_min_range_m = 0.5;
    double lidar_max_range_m = 100.0;
    int lidar_max_points = 5000;
    int lidar_icp_max_iterations = 15;
    int lidar_min_correspondences = 200;
    double lidar_max_correspondence_distance_m = 2.0;
    double lidar_min_overlap = 0.35;
    double lidar_max_icp_rmse_m = 0.8;
    double lidar_max_correction_translation_m = 0.4;
    double lidar_max_correction_rotation_rad = 0.15;
    double lidar_max_correction_ratio = -1.0;
    bool lidar_adaptive_covariance = false;
    double lidar_adaptive_max_scale = 6.0;
    bool lidar_prune_unconstrained_nodes = false;
    double lidar_translation_std_m = 0.25;
    double lidar_rotation_std_rad = 0.03;
    Vec3 lidar_t_body_lidar{0.0, 0.0, -0.1};
    // AirSim VehicleInertialFrame points are expressed in the fixed NED frame
    // anchored at the vehicle initial pose; sensor points are already local.
    bool lidar_points_vehicle_inertial_frame = false;
};

struct FusionSummary {
    double start_time = 0.0;
    double end_time = 0.0;
    std::size_t imu_used = 0;
    std::size_t gnss_epochs_used = 0;
    std::size_t states_written = 0;
    std::size_t state_nodes = 0;
    std::size_t prior_factors = 0;
    std::size_t gnss_factors = 0;
    std::size_t gnss_fix_factors = 0;
    std::size_t gnss_spp_epochs_solved = 0;
    std::size_t gnss_spp_epochs_rejected = 0;
    double gnss_spp_mean_satellites = 0.0;
    double gnss_spp_mean_postfit_rms_m = 0.0;
    double gnss_spp_max_postfit_rms_m = 0.0;
    double gnss_spp_min_weight = 0.0;
    double gnss_spp_max_weight = 0.0;
    double gnss_fixed_satellite_consistency_rms_m = 0.0;
    double gnss_fixed_satellite_consistency_max_m = 0.0;
    bool gnss_fixed_satellite_model_consistent = false;
    bool fiveg_enabled = false;
    std::size_t fiveg_epochs_used = 0;
    std::size_t fiveg_range_factors = 0;
    std::size_t fiveg_rsrp_rejected = 0;
    std::size_t imu_factors = 0;
    std::size_t attitude_factors = 0;
    std::size_t bias_factors = 0;
    bool visual_enabled = false;
    std::size_t camera_frames_loaded = 0;
    std::size_t visual_keyframes = 0;
    std::size_t visual_tracks = 0;
    std::size_t visual_landmarks = 0;
    std::size_t visual_reprojection_factors = 0;
    std::size_t visual_rejected_tracks = 0;
    std::size_t visual_initial_reprojection_observations = 0;
    double visual_initial_reprojection_rmse_px = 0.0;
    bool lidar_enabled = false;
    std::size_t lidar_frames_loaded = 0;
    std::size_t lidar_keyframes = 0;
    std::size_t lidar_constraints = 0;
    std::size_t lidar_rejected_pairs = 0;
    std::size_t lidar_correspondences = 0;
    double lidar_mean_icp_rmse_m = 0.0;
    double lidar_mean_overlap = 0.0;
    double lidar_mean_translation_std_m = 0.0;
    double lidar_mean_rotation_std_rad = 0.0;
    std::string initialization_source = "legacy_groundtruth";
    bool groundtruth_used_for_initialization = true;
    bool groundtruth_used_for_frame_anchor = false;
    std::size_t initialization_gravity_samples = 0;
    double initialization_heading_time_s = 0.0;
    double initialization_roll_deg = 0.0;
    double initialization_pitch_deg = 0.0;
    double initialization_yaw_deg = 0.0;
    double initial_cost = 0.0;
    double final_cost = 0.0;
    std::string solver = "dense normal-equation least squares";
    std::string termination = "completed";
};

std::vector<TrajectoryState> run_offline_gins(const Dataset& dataset,
                                              const FusionOptions& options,
                                              FusionSummary* summary);

void write_trajectory_csv(const std::filesystem::path& path,
                          const std::vector<TrajectoryState>& trajectory);

void write_summary_txt(const std::filesystem::path& output_csv,
                       const FusionOptions& options,
                       const FusionSummary& summary,
                       const Dataset& dataset);

}  // namespace offline_fusion
