#include <cmath>
#include <exception>
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <string>

#include "offline_fusion/dataset.hpp"
#include "offline_fusion/fusion.hpp"

namespace {

void print_usage() {
    std::cout
        << "Usage:\n"
        << "  offline_fusion [--data <simdata_dir>] [--output <trajectory_csv>] [options]\n\n"
        << "Options:\n"
        << "  --data <simdata_dir>           Dataset directory, default simdata_v2\n"
        << "  --output <trajectory_csv>      Output CSV, default results\\simdata_v2_ceres_visual\\trajectory_est.csv\n"
        << "  --load-only                    Load and validate all sensor streams without fusion\n"
        << "  --gnss-only                    Write per-epoch SPP-like GNSS trajectory without IMU/Ceres\n"
        << "  --start-offset <seconds>       Seconds after dataset start to begin fusion, default 0\n"
        << "  --end-offset <seconds>         Seconds after dataset start to end fusion, default full range\n"
        << "  --gravity <mps2>               NED gravity magnitude, default 9.793\n"
        << "  --init-gravity-window <s>      Initial IMU gravity search window, default 3.0\n"
        << "  --init-gravity-tolerance <mps2> Gravity-magnitude tolerance, default 0.25\n"
        << "  --init-heading-speed <mps>     Minimum GNSS horizontal speed, default 1.5\n"
        << "  --prior-position-std <m>       First-node position prior std, default 0.05\n"
        << "  --prior-velocity-std <mps>     First-node velocity prior std, default 0.05\n"
        << "  --attitude-prior-std <rad>     First-node attitude error prior std, default 0.02\n"
        << "  --accel-bias-prior-std <mps2>  First-node accel bias prior std, default 0.2\n"
        << "  --gyro-bias-prior-std <rps>    First-node gyro bias prior std, default 0.02\n"
        << "  --gnss-position-std <m>        Maximum/fallback SPP-like position factor std, default 3.0\n"
        << "  --gnss-position-noise-scale <x>  Scale derived SPP position std, default 1.0\n"
        << "  --disable-gnss-fix             Compatibility no-op; gps_fix never enters estimation\n"
        << "  --enable-fiveg                 Enable raw 5G-A Multi-RTT range factors\n"
        << "  --fiveg-range-std-floor <m>    Minimum 5G-A range std, default 0.5\n"
        << "  --fiveg-loss-scale <value>     Cauchy loss scale in sigma units, default 2.0\n"
        << "  --fiveg-min-prs-rsrp <dBm>     Minimum accepted PRS-RSRP, default -32\n"
        << "  --imu-legacy-weighting       Restore fixed interval IMU weights for comparison\n"
        << "  --imu-accel-noise-density <value> Accel white noise density, default 0.2\n"
        << "  --imu-gyro-noise-density <value> Gyro white noise density, default 0.02\n"
        << "  --imu-accel-bias-density <value> Accel bias RW density, default 0.05\n"
        << "  --imu-gyro-bias-density <value> Gyro bias RW density, default 0.005\n"
        << "  --extra-state-interval <s>   Diagnostic extra nodes without sensor factors\n"
        << "  --imu-position-std <m>         Legacy-only position factor std, default 0.5\n"
        << "  --imu-velocity-std <mps>       Legacy-only velocity factor std, default 0.2\n"
        << "  --imu-attitude-std <rad>       Legacy-only attitude factor std, default 0.05\n"
        << "  --accel-bias-rw-std <mps2>     Legacy-only accel bias factor std, default 0.05\n"
        << "  --gyro-bias-rw-std <rps>       Legacy-only gyro bias factor std, default 0.005\n"
        << "  --normal-damping <value>       Normal matrix diagonal damping, default 1e-8\n"
        << "  --disable-visual               Disable camera keyframes and reprojection factors\n"
        << "  --visual-keyframe-interval <s> Visual keyframe interval, default 0.25\n"
        << "  --visual-max-features <n>      Max tracked features, default 300\n"
        << "  --visual-max-landmarks <n>     Max optimized landmarks, default 300\n"
        << "  --visual-reprojection-std <px> Reprojection residual std, default 2.0\n"
        << "  --visual-min-parallax <px>     Minimum triangulation parallax, default 12\n"
        << "  --visual-image-scale <scale>   Image scale for frontend only, default 0.5\n"
        << "  --visual-prune-unconstrained-nodes Keep only state nodes used by reprojection factors\n"
        << "  --camera-fx <px>               Camera fx, default 3248\n"
        << "  --camera-fy <px>               Camera fy, default 3248\n"
        << "  --camera-cx <px>               Camera cx, default 1024\n"
        << "  --camera-cy <px>               Camera cy, default 768\n"
        << "  --camera-t-bc-x <m>            Camera origin x in body frame, default 0.5\n"
        << "  --camera-t-bc-y <m>            Camera origin y in body frame, default 0.0\n"
        << "  --camera-t-bc-z <m>            Camera origin z in body frame, default -0.1\n"
        << "  --enable-lidar                 Enable LiDAR ICP relative-pose factors\n"
        << "  --lidar-keyframe-interval <s>  LiDAR keyframe interval, default 0.2\n"
        << "  --lidar-voxel-size <m>         ICP voxel size, default 0.5\n"
        << "  --lidar-max-points <n>         Max downsampled points per scan, default 5000\n"
        << "  --lidar-max-correspondence <m> ICP correspondence gate, default 2.0\n"
        << "  --lidar-min-overlap <ratio>    Minimum accepted ICP overlap, default 0.35\n"
        << "  --lidar-max-rmse <m>           Maximum accepted ICP RMSE, default 0.8\n"
        << "  --lidar-max-correction-translation <m> Maximum ICP translation correction, default 0.4\n"
        << "  --lidar-max-correction-rotation <rad> Maximum ICP rotation correction, default 0.15\n"
        << "  --lidar-max-correction-ratio <ratio> Maximum translation correction / predicted motion; disabled by default\n"
        << "  --lidar-adaptive-covariance    Scale LiDAR residual std using ICP quality\n"
        << "  --lidar-adaptive-max-scale <ratio> Maximum adaptive std scale, default 6.0\n"
        << "  --lidar-prune-unconstrained-nodes Keep only state nodes used by accepted LiDAR factors\n"
        << "  --lidar-translation-std <m>    LiDAR translation residual std, default 0.25\n"
        << "  --lidar-rotation-std <rad>     LiDAR rotation residual std, default 0.03\n"
        << "  --absolute-start <value>    absolute_start_s\n"
        << "  --absolute-end <value>    absolute_end_s\n"
        << "  --solver-max-iterations <value>    solver_max_iterations\n"
        << "  --visual-min-triangulation-deg <value>    visual_min_triangulation_deg\n"
        << "  --lidar-min-information-ratio <value>    lidar_min_information_ratio\n"
        << "  --lidar-t-bl-x <value>    lidar_t_body_lidar.x\n"
        << "  --lidar-t-bl-y <value>    lidar_t_body_lidar.y\n"
        << "  --lidar-t-bl-z <value>    lidar_t_body_lidar.z\n"
        << "  --lidar-data-frame <sensor|vehicle-inertial>  input point frame\n"
        << "  --camera-pitch-deg <degrees>   AirSim mounting pitch\n"
        << "  --help                         Show this message\n";
}

bool read_option(int& i, int argc, char** argv, std::string* value) {
    if (i + 1 >= argc) {
        return false;
    }
    *value = argv[++i];
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    offline_fusion::FusionOptions options;
    bool load_only = false;
    bool gnss_only = false;
    bool legacy_imu_parameter_requested = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--imu-position-std" || arg == "--imu-velocity-std" ||
            arg == "--imu-attitude-std" || arg == "--accel-bias-rw-std" || arg == "--gyro-bias-rw-std")
            legacy_imu_parameter_requested = true;
        std::string value;
        if (arg == "--help" || arg == "-h") {
            print_usage();
            return 0;
        }
        if (arg == "--data" && read_option(i, argc, argv, &value)) {
            options.data_dir = value;
        } else if (arg == "--load-only") {
            load_only = true;
        } else if (arg == "--gnss-only") {
            gnss_only = true;
        } else if (arg == "--output" && read_option(i, argc, argv, &value)) {
            options.output_csv = value;
        } else if (arg == "--absolute-start" && read_option(i, argc, argv, &value)) {
            options.absolute_start_s = std::stod(value);
        } else if (arg == "--absolute-end" && read_option(i, argc, argv, &value)) {
            options.absolute_end_s = std::stod(value);
        } else if (arg == "--solver-max-iterations" && read_option(i, argc, argv, &value)) {
            options.solver_max_iterations = std::stoi(value);
        } else if (arg == "--visual-min-triangulation-deg" && read_option(i, argc, argv, &value)) {
            options.visual_min_triangulation_deg = std::stod(value);
        } else if (arg == "--lidar-min-information-ratio" && read_option(i, argc, argv, &value)) {
            options.lidar_min_information_ratio = std::stod(value);
        } else if (arg == "--lidar-t-bl-x" && read_option(i, argc, argv, &value)) {
            options.lidar_t_body_lidar.x = std::stod(value);
        } else if (arg == "--lidar-t-bl-y" && read_option(i, argc, argv, &value)) {
            options.lidar_t_body_lidar.y = std::stod(value);
        } else if (arg == "--lidar-t-bl-z" && read_option(i, argc, argv, &value)) {
            options.lidar_t_body_lidar.z = std::stod(value);
        } else if (arg == "--lidar-data-frame" && read_option(i, argc, argv, &value)) {
            if (value == "vehicle-inertial") options.lidar_points_vehicle_inertial_frame = true;
            else if (value == "sensor") options.lidar_points_vehicle_inertial_frame = false;
            else { std::cerr << "Unknown lidar data frame: " << value << "\n\n"; print_usage(); return 2; }
        } else if (arg == "--start-offset" && read_option(i, argc, argv, &value)) {
            options.start_offset_s = std::stod(value);
        } else if (arg == "--end-offset" && read_option(i, argc, argv, &value)) {
            options.end_offset_s = std::stod(value);
        } else if (arg == "--gravity" && read_option(i, argc, argv, &value)) {
            options.gravity_mps2 = std::stod(value);
        } else if (arg == "--init-gravity-window" && read_option(i, argc, argv, &value)) {
            options.initialization_gravity_window_s = std::stod(value);
        } else if (arg == "--init-gravity-tolerance" && read_option(i, argc, argv, &value)) {
            options.initialization_gravity_tolerance_mps2 = std::stod(value);
        } else if (arg == "--init-heading-speed" && read_option(i, argc, argv, &value)) {
            options.initialization_heading_speed_mps = std::stod(value);
        } else if (arg == "--prior-position-std" && read_option(i, argc, argv, &value)) {
            options.prior_position_std_m = std::stod(value);
        } else if (arg == "--prior-velocity-std" && read_option(i, argc, argv, &value)) {
            options.prior_velocity_std_mps = std::stod(value);
        } else if (arg == "--attitude-prior-std" && read_option(i, argc, argv, &value)) {
            options.attitude_prior_std_rad = std::stod(value);
        } else if (arg == "--accel-bias-prior-std" && read_option(i, argc, argv, &value)) {
            options.accel_bias_prior_std_mps2 = std::stod(value);
        } else if (arg == "--gyro-bias-prior-std" && read_option(i, argc, argv, &value)) {
            options.gyro_bias_prior_std_rps = std::stod(value);
        } else if (arg == "--gnss-position-std" && read_option(i, argc, argv, &value)) {
            options.gnss_position_std_m = std::stod(value);
        } else if (arg == "--gnss-position-noise-scale" && read_option(i, argc, argv, &value)) {
            options.gnss_position_noise_scale = std::stod(value);
        } else if (arg == "--disable-gnss-fix") {
            // Compatibility no-op: gps_fix is never added to the estimator.
        } else if (arg == "--enable-fiveg") {
            options.use_fiveg = true;
        } else if (arg == "--fiveg-range-std-floor" &&
                   read_option(i, argc, argv, &value)) {
            options.fiveg_range_std_floor_m = std::stod(value);
        } else if (arg == "--fiveg-loss-scale" && read_option(i, argc, argv, &value)) {
            options.fiveg_loss_scale = std::stod(value);
        } else if (arg == "--fiveg-min-prs-rsrp" &&
                   read_option(i, argc, argv, &value)) {
            options.fiveg_min_prs_rsrp_dbm = std::stod(value);
        } else if (arg == "--imu-legacy-weighting") {
            options.imu_propagated_covariance = false;
        } else if (arg == "--imu-accel-noise-density" && read_option(i, argc, argv, &value)) {
            options.imu_accel_noise_density = std::stod(value);
        } else if (arg == "--imu-gyro-noise-density" && read_option(i, argc, argv, &value)) {
            options.imu_gyro_noise_density = std::stod(value);
        } else if (arg == "--imu-accel-bias-density" && read_option(i, argc, argv, &value)) {
            options.imu_accel_bias_density = std::stod(value);
        } else if (arg == "--imu-gyro-bias-density" && read_option(i, argc, argv, &value)) {
            options.imu_gyro_bias_density = std::stod(value);
        } else if (arg == "--extra-state-interval" && read_option(i, argc, argv, &value)) {
            options.extra_state_interval_s = std::stod(value);
            if (!std::isfinite(options.extra_state_interval_s) || options.extra_state_interval_s < 0.001)
                throw std::runtime_error("extra-state-interval must be finite and >= 0.001 s");
        } else if (arg == "--imu-position-std" && read_option(i, argc, argv, &value)) {
            options.imu_position_std_m = std::stod(value);
        } else if (arg == "--imu-velocity-std" && read_option(i, argc, argv, &value)) {
            options.imu_velocity_std_mps = std::stod(value);
        } else if (arg == "--imu-attitude-std" && read_option(i, argc, argv, &value)) {
            options.imu_attitude_std_rad = std::stod(value);
        } else if (arg == "--accel-bias-rw-std" && read_option(i, argc, argv, &value)) {
            options.accel_bias_rw_std_mps2 = std::stod(value);
        } else if (arg == "--gyro-bias-rw-std" && read_option(i, argc, argv, &value)) {
            options.gyro_bias_rw_std_rps = std::stod(value);
        } else if (arg == "--normal-damping" && read_option(i, argc, argv, &value)) {
            options.normal_matrix_damping = std::stod(value);
        } else if (arg == "--disable-visual") {
            options.use_visual = false;
        } else if (arg == "--visual-keyframe-interval" && read_option(i, argc, argv, &value)) {
            options.visual_keyframe_interval_s = std::stod(value);
        } else if (arg == "--visual-max-features" && read_option(i, argc, argv, &value)) {
            options.visual_max_features = std::stoi(value);
        } else if (arg == "--visual-max-landmarks" && read_option(i, argc, argv, &value)) {
            options.visual_max_landmarks = std::stoi(value);
        } else if (arg == "--visual-reprojection-std" && read_option(i, argc, argv, &value)) {
            options.visual_reprojection_std_px = std::stod(value);
        } else if (arg == "--visual-min-parallax" && read_option(i, argc, argv, &value)) {
            options.visual_min_parallax_px = std::stod(value);
        } else if (arg == "--visual-image-scale" && read_option(i, argc, argv, &value)) {
            options.visual_image_scale = std::stod(value);
        } else if (arg == "--visual-prune-unconstrained-nodes") {
            options.visual_prune_unconstrained_nodes = true;
        } else if (arg == "--camera-pitch-deg" && read_option(i, argc, argv, &value)) {
            options.camera_pitch_deg = std::stod(value);
        } else if (arg == "--camera-fx" && read_option(i, argc, argv, &value)) {
            options.camera_fx = std::stod(value);
        } else if (arg == "--camera-fy" && read_option(i, argc, argv, &value)) {
            options.camera_fy = std::stod(value);
        } else if (arg == "--camera-cx" && read_option(i, argc, argv, &value)) {
            options.camera_cx = std::stod(value);
        } else if (arg == "--camera-cy" && read_option(i, argc, argv, &value)) {
            options.camera_cy = std::stod(value);
        } else if (arg == "--camera-t-bc-x" && read_option(i, argc, argv, &value)) {
            options.camera_t_body_camera.x = std::stod(value);
        } else if (arg == "--camera-t-bc-y" && read_option(i, argc, argv, &value)) {
            options.camera_t_body_camera.y = std::stod(value);
        } else if (arg == "--camera-t-bc-z" && read_option(i, argc, argv, &value)) {
            options.camera_t_body_camera.z = std::stod(value);
        } else if (arg == "--enable-lidar") {
            options.use_lidar = true;
        } else if (arg == "--lidar-keyframe-interval" &&
                   read_option(i, argc, argv, &value)) {
            options.lidar_keyframe_interval_s = std::stod(value);
        } else if (arg == "--lidar-voxel-size" && read_option(i, argc, argv, &value)) {
            options.lidar_voxel_size_m = std::stod(value);
        } else if (arg == "--lidar-max-points" && read_option(i, argc, argv, &value)) {
            options.lidar_max_points = std::stoi(value);
        } else if (arg == "--lidar-max-correspondence" &&
                   read_option(i, argc, argv, &value)) {
            options.lidar_max_correspondence_distance_m = std::stod(value);
        } else if (arg == "--lidar-min-overlap" && read_option(i, argc, argv, &value)) {
            options.lidar_min_overlap = std::stod(value);
        } else if (arg == "--lidar-max-rmse" && read_option(i, argc, argv, &value)) {
            options.lidar_max_icp_rmse_m = std::stod(value);
        } else if (arg == "--lidar-max-correction-translation" &&
                   read_option(i, argc, argv, &value)) {
            options.lidar_max_correction_translation_m = std::stod(value);
        } else if (arg == "--lidar-max-correction-rotation" &&
                   read_option(i, argc, argv, &value)) {
            options.lidar_max_correction_rotation_rad = std::stod(value);
        } else if (arg == "--lidar-max-correction-ratio" &&
                   read_option(i, argc, argv, &value)) {
            options.lidar_max_correction_ratio = std::stod(value);
        } else if (arg == "--lidar-adaptive-covariance") {
            options.lidar_adaptive_covariance = true;
        } else if (arg == "--lidar-adaptive-max-scale" &&
                   read_option(i, argc, argv, &value)) {
            options.lidar_adaptive_max_scale = std::stod(value);
        } else if (arg == "--lidar-prune-unconstrained-nodes") {
            options.lidar_prune_unconstrained_nodes = true;
        } else if (arg == "--lidar-translation-std" &&
                   read_option(i, argc, argv, &value)) {
            options.lidar_translation_std_m = std::stod(value);
        } else if (arg == "--lidar-rotation-std" && read_option(i, argc, argv, &value)) {
            options.lidar_rotation_std_rad = std::stod(value);
        } else {
            std::cerr << "Unknown or incomplete option: " << arg << "\n\n";
            print_usage();
            return 2;
        }
    }

    if (legacy_imu_parameter_requested && options.imu_propagated_covariance) {
        std::cerr << "Fixed interval IMU std options require --imu-legacy-weighting; use noise densities for the covariance model.\n";
        return 1;
    }
    try {
        const auto dataset = offline_fusion::load_dataset(options.data_dir);
        if (load_only) {
            const auto valid_fiveg_links = std::count_if(
                dataset.fiveg_epochs.begin(), dataset.fiveg_epochs.end(),
                [](const offline_fusion::FiveGEpoch& epoch) {
                    return std::any_of(
                        epoch.observations.begin(), epoch.observations.end(),
                        [](const offline_fusion::FiveGObservation& observation) {
                            return observation.valid;
                        });
                });
            std::size_t fiveg_observations = 0;
            std::size_t valid_fiveg_observations = 0;
            for (const auto& epoch : dataset.fiveg_epochs) {
                fiveg_observations += epoch.observations.size();
                valid_fiveg_observations += static_cast<std::size_t>(std::count_if(
                    epoch.observations.begin(), epoch.observations.end(),
                    [](const offline_fusion::FiveGObservation& observation) {
                        return observation.valid;
                    }));
            }
            std::size_t lidar_points_checked = 0;
            if (!dataset.lidar_frames.empty()) {
                lidar_points_checked =
                    offline_fusion::load_lidar_points(dataset.lidar_frames.front()).size();
            }

            std::cout << "offline_fusion dataset load completed\n";
            std::cout << "  format: " << offline_fusion::dataset_format_name(dataset.format)
                      << '\n';
            std::cout << "  time_origin_ns: " << dataset.time_origin_ns << '\n';
            std::cout << "  groundtruth_samples: " << dataset.groundtruth.size() << '\n';
            std::cout << "  imu_samples: " << dataset.imu.size() << '\n';
            std::cout << "  gnss_epochs: " << dataset.gnss_epochs.size() << '\n';
            std::cout << "  gnss_fixes: " << dataset.gnss_fixes.size() << '\n';
            std::cout << "  gnss_raw_derived_position_fixes: "
                      << dataset.gnss_position_fixes.size() << '\n';
            std::cout << "  gnss_spp_epochs_rejected: "
                      << dataset.gnss_spp_diagnostics.epochs_rejected << '\n';
            std::cout << "  gnss_spp_mean_satellites: "
                      << dataset.gnss_spp_diagnostics.mean_satellites << '\n';
            std::cout << "  gnss_spp_mean_postfit_rms_m: "
                      << dataset.gnss_spp_diagnostics.mean_postfit_rms_m << '\n';
            std::cout << "  gnss_fixed_satellite_consistency_rms_m: "
                      << dataset.gnss_spp_diagnostics.fixed_satellite_consistency_rms_m << '\n';
            std::cout << "  gnss_fixed_satellite_model_consistent: "
                      << (dataset.gnss_spp_diagnostics.fixed_satellite_model_consistent
                              ? "true" : "false") << '\n';
            std::cout << "  camera_frames: " << dataset.camera_frames.size() << '\n';
            std::cout << "  lidar_frames: " << dataset.lidar_frames.size() << '\n';
            std::cout << "  first_lidar_points_checked: " << lidar_points_checked << '\n';
            std::cout << "  fiveg_epochs: " << dataset.fiveg_epochs.size() << '\n';
            std::cout << "  fiveg_observations: " << fiveg_observations << '\n';
            std::cout << "  valid_fiveg_observations: " << valid_fiveg_observations << '\n';
            std::cout << "  fiveg_epochs_with_valid_links: " << valid_fiveg_links << '\n';
            if (!dataset.imu.empty()) {
                std::cout << "  imu_time_range_s: " << dataset.imu.front().t << " to "
                          << dataset.imu.back().t << '\n';
            }
            if (!dataset.groundtruth.empty()) {
                std::cout << "  groundtruth_time_range_s: "
                          << dataset.groundtruth.front().t << " to "
                          << dataset.groundtruth.back().t << '\n';
                std::cout << "  groundtruth_has_orientation: "
                          << (dataset.groundtruth.front().has_orientation ? "true" : "false")
                          << '\n';
            }
            return 0;
        }
        if (gnss_only) {
            std::vector<offline_fusion::TrajectoryState> trajectory;
            trajectory.reserve(dataset.gnss_position_fixes.size());
            for (const auto& fix : dataset.gnss_position_fixes) {
                offline_fusion::TrajectoryState state;
                state.t = fix.t;
                state.position_ned = fix.position_ned;
                state.velocity_ned = fix.velocity_ned;
                state.update_source = "per_epoch_spp_like_gnss";
                trajectory.push_back(state);
            }
            offline_fusion::FusionSummary summary;
            summary.start_time = trajectory.front().t;
            summary.end_time = trajectory.back().t;
            summary.gnss_epochs_used = trajectory.size();
            summary.states_written = trajectory.size();
            summary.state_nodes = trajectory.size();
            summary.gnss_factors = trajectory.size() * 3;
            summary.gnss_spp_epochs_solved = dataset.gnss_spp_diagnostics.epochs_solved;
            summary.gnss_spp_epochs_rejected = dataset.gnss_spp_diagnostics.epochs_rejected;
            summary.gnss_spp_mean_satellites = dataset.gnss_spp_diagnostics.mean_satellites;
            summary.gnss_spp_mean_postfit_rms_m =
                dataset.gnss_spp_diagnostics.mean_postfit_rms_m;
            summary.gnss_spp_max_postfit_rms_m =
                dataset.gnss_spp_diagnostics.max_postfit_rms_m;
            summary.gnss_spp_min_weight =
                dataset.gnss_spp_diagnostics.min_normalized_weight;
            summary.gnss_spp_max_weight =
                dataset.gnss_spp_diagnostics.max_normalized_weight;
            summary.gnss_fixed_satellite_consistency_rms_m =
                dataset.gnss_spp_diagnostics.fixed_satellite_consistency_rms_m;
            summary.gnss_fixed_satellite_consistency_max_m =
                dataset.gnss_spp_diagnostics.fixed_satellite_consistency_max_m;
            summary.gnss_fixed_satellite_model_consistent =
                dataset.gnss_spp_diagnostics.fixed_satellite_model_consistent;
            summary.initialization_source = dataset.format == offline_fusion::DatasetFormat::LegacyText
                ? "first_truth_position_frame_anchor_only"
                : "same_epoch_gnss_fix_virtual_satellite_frame_anchor";
            summary.groundtruth_used_for_initialization = false;
            summary.groundtruth_used_for_frame_anchor =
                dataset.format == offline_fusion::DatasetFormat::LegacyText;
            summary.solver = "per-epoch SPP-like weighted robust least squares";
            summary.termination = "completed";
            offline_fusion::write_trajectory_csv(options.output_csv, trajectory);
            offline_fusion::write_summary_txt(options.output_csv, options, summary, dataset);
            std::cout << "offline_fusion GNSS-only completed\n";
            std::cout << "  output: " << options.output_csv << '\n';
            std::cout << "  states_written: " << summary.states_written << '\n';
            std::cout << "  mean_postfit_rms_m: "
                      << summary.gnss_spp_mean_postfit_rms_m << '\n';
            return 0;
        }
        offline_fusion::FusionSummary summary;
        const auto trajectory = offline_fusion::run_offline_gins(dataset, options, &summary);
        offline_fusion::write_trajectory_csv(options.output_csv, trajectory);
        offline_fusion::write_summary_txt(options.output_csv, options, summary, dataset);

        std::cout << "offline_fusion completed\n";
        std::cout << "  output: " << options.output_csv << '\n';
        std::cout << "  states_written: " << summary.states_written << '\n';
        std::cout << "  gnss_epochs_used: " << summary.gnss_epochs_used << '\n';
        std::cout << "  gnss_fix_factors: " << summary.gnss_fix_factors << '\n';
        std::cout << "  gnss_spp_mean_postfit_rms_m: "
                  << summary.gnss_spp_mean_postfit_rms_m << '\n';
        std::cout << "  gnss_fixed_satellite_model_consistent: "
                  << (summary.gnss_fixed_satellite_model_consistent ? "true" : "false")
                  << '\n';
        std::cout << "  fiveg_enabled: " << (summary.fiveg_enabled ? "true" : "false")
                  << '\n';
        std::cout << "  fiveg_epochs_used: " << summary.fiveg_epochs_used << '\n';
        std::cout << "  fiveg_range_factors: " << summary.fiveg_range_factors << '\n';
        std::cout << "  fiveg_rsrp_rejected: " << summary.fiveg_rsrp_rejected << '\n';
        std::cout << "  visual_enabled: " << (summary.visual_enabled ? "true" : "false") << '\n';
        std::cout << "  visual_keyframes: " << summary.visual_keyframes << '\n';
        std::cout << "  visual_landmarks: " << summary.visual_landmarks << '\n';
        std::cout << "  visual_reprojection_factors: "
                  << summary.visual_reprojection_factors << '\n';
        std::cout << "  visual_initial_reprojection_rmse_px: "
                  << summary.visual_initial_reprojection_rmse_px << '\n';
        std::cout << "  lidar_enabled: " << (summary.lidar_enabled ? "true" : "false")
                  << '\n';
        std::cout << "  lidar_constraints: " << summary.lidar_constraints << '\n';
        std::cout << "  lidar_rejected_pairs: " << summary.lidar_rejected_pairs << '\n';
        std::cout << "  lidar_mean_icp_rmse_m: " << summary.lidar_mean_icp_rmse_m << '\n';
        std::cout << "  lidar_mean_overlap: " << summary.lidar_mean_overlap << '\n';
        std::cout << "  initial_cost: " << summary.initial_cost << '\n';
        std::cout << "  final_cost: " << summary.final_cost << '\n';
        std::cout << "  initialization_source: " << summary.initialization_source << '\n';
        std::cout << "  groundtruth_used_for_initialization: "
                  << (summary.groundtruth_used_for_initialization ? "true" : "false") << '\n';
        std::cout << "  initial_rpy_deg: " << summary.initialization_roll_deg << ", "
                  << summary.initialization_pitch_deg << ", "
                  << summary.initialization_yaw_deg << '\n';
        std::cout << "  note: gps_fix is loaded for interface/baseline only; carrier phase and Doppler are not used.\n";
    } catch (const std::exception& e) {
        std::cerr << "offline_fusion failed: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
