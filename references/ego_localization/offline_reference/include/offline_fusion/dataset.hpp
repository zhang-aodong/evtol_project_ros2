#pragma once

#include <filesystem>
#include <vector>

#include "offline_fusion/types.hpp"

namespace offline_fusion {

std::vector<ImuSample> load_imu(const std::filesystem::path& path);
std::vector<GroundTruthSample> load_groundtruth(const std::filesystem::path& path);
std::vector<GnssEpoch> load_gnss_raw(const std::filesystem::path& path);
std::vector<CameraFrame> load_camera_frames(const std::filesystem::path& timestamp_path,
                                            const std::filesystem::path& image_dir);
std::vector<LidarPoint> load_lidar_points(const LidarFrame& frame);
std::vector<GnssPositionFix> derive_gnss_position_fixes(
    const std::vector<GnssEpoch>& epochs,
    const Vec3& anchor_position_ned,
    GnssSppDiagnostics* diagnostics = nullptr);
Dataset load_dataset(const std::filesystem::path& data_dir);
const char* dataset_format_name(DatasetFormat format);

Vec3 interpolate_groundtruth_position(const std::vector<GroundTruthSample>& gt, double t);
Quat nearest_groundtruth_orientation(const std::vector<GroundTruthSample>& gt, double t);
Vec3 estimate_groundtruth_velocity(const std::vector<GroundTruthSample>& gt, double t);

}  // namespace offline_fusion
