#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace offline_fusion {

enum class DatasetFormat {
    LegacyText,
    StructuredExperiment,
};

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Quat {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct ImuSample {
    double t = 0.0;
    Vec3 accel;
    Vec3 gyro;
    std::string phase;
};

struct GroundTruthSample {
    double t = 0.0;
    Vec3 position_ned;
    Vec3 velocity_ned;
    Quat q_body_to_ned;
    bool has_velocity = false;
    bool has_orientation = false;
    std::string phase;
};

struct GnssObservation {
    int prn = 0;
    double pseudorange_m = 0.0;
    double carrier_phase_cyc = 0.0;
    double doppler_hz = 0.0;
    double snr_dbhz = 0.0;
    double elevation_deg = 0.0;
    double azimuth_deg = 0.0;
};

struct GnssEpoch {
    double t = 0.0;
    std::vector<GnssObservation> observations;
    std::string phase;
};

struct GnssFixMeasurement {
    double t = 0.0;
    bool valid = false;
    int fix_type = 0;
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;
    double horizontal_std_m = 0.0;
    double vertical_std_m = 0.0;
    Vec3 position_ned;
    Vec3 velocity_ned;
    std::string phase;
};

struct GeodeticOrigin {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;
    bool valid = false;
};

struct GnssPositionFix {
    double t = 0.0;
    Vec3 position_ned;
    Vec3 velocity_ned;
    Vec3 position_std_ned{3.0, 3.0, 3.0};
    double clock_bias_m = 0.0;
    double postfit_rms_m = 0.0;
    double min_normalized_weight = 0.0;
    double max_normalized_weight = 0.0;
    std::size_t observations = 0;
    bool valid = false;
};

struct GnssSppDiagnostics {
    std::size_t epochs_total = 0;
    std::size_t epochs_solved = 0;
    std::size_t epochs_rejected = 0;
    std::size_t virtual_satellites = 0;
    std::size_t observations_used = 0;
    double mean_satellites = 0.0;
    double mean_postfit_rms_m = 0.0;
    double max_postfit_rms_m = 0.0;
    double min_normalized_weight = 0.0;
    double max_normalized_weight = 0.0;
    double fixed_satellite_consistency_rms_m = 0.0;
    double fixed_satellite_consistency_max_m = 0.0;
    bool fixed_satellite_model_consistent = false;
};

struct CameraFrame {
    std::string frame_id;
    double t = 0.0;
    std::filesystem::path image_path;
};

struct LidarPoint {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float intensity = 0.0F;
};

static_assert(sizeof(LidarPoint) == 4 * sizeof(float),
              "LiDAR point layout must match four packed float32 values");

struct LidarFrame {
    std::string frame_id;
    double t = 0.0;
    std::filesystem::path point_path;
    std::size_t point_count = 0;
};

struct FiveGObservation {
    std::string gnb_id;
    Vec3 gnb_position_ned;
    double prs_rsrp_dbm = 0.0;
    bool is_los = false;
    double range_std_m = 0.0;
    double measured_range_m = 0.0;
    double measured_rtt_s = 0.0;
    bool valid = false;
};

struct FiveGEpoch {
    double t = 0.0;
    std::vector<FiveGObservation> observations;
    std::string phase;
};

struct TrajectoryState {
    double t = 0.0;
    Vec3 position_ned;
    Vec3 velocity_ned;
    Quat q_body_to_ned;
    Vec3 accel_bias;
    Vec3 gyro_bias;
    std::string update_source;
};

struct Dataset {
    DatasetFormat format = DatasetFormat::LegacyText;
    std::int64_t time_origin_ns = 0;
    GeodeticOrigin geodetic_origin;
    std::vector<ImuSample> imu;
    std::vector<GroundTruthSample> groundtruth;
    std::vector<GnssEpoch> gnss_epochs;
    std::vector<GnssFixMeasurement> gnss_fixes;
    std::vector<GnssPositionFix> gnss_position_fixes;
    GnssSppDiagnostics gnss_spp_diagnostics;
    std::vector<CameraFrame> camera_frames;
    std::vector<LidarFrame> lidar_frames;
    std::vector<FiveGEpoch> fiveg_epochs;
};

}  // namespace offline_fusion
