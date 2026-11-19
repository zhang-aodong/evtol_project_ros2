// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "common/Common.hpp"
#include "GameFramework/Actor.h"
#include "sensors/lidar/LidarSimple.hpp"
#include "NedTransform.h"
#include "LidarSharedMemoryWriter.h"
#include "LidarIdentitySharedMemoryWriter.h"
#include <memory>

// UnrealLidarSensor implementation that uses Ray Tracing in Unreal.
// The implementation uses a model similar to CARLA Lidar implementation.
// Thanks to CARLA folks for this.
class UnrealLidarSensor : public msr::airlib::LidarSimple
{
public:
    typedef msr::airlib::AirSimSettings AirSimSettings;

public:
    UnrealLidarSensor(const AirSimSettings::LidarSetting& setting,
                      AActor* actor, const NedTransform* ned_transform);

protected:
    virtual void getPointCloud(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
                               msr::airlib::TTimeDelta delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<int>& segmentation_cloud) override;

private:
    using Vector3r = msr::airlib::Vector3r;
    using VectorMath = msr::airlib::VectorMath;

    void createLasers();
    bool shootLaser(const msr::airlib::Pose& sensor_world, const Vector3r& direction_local,
                    const msr::airlib::LidarSimpleParams& params, Vector3r& point, int& segmentationID, uint32_t& object_id);

private:
    AActor* actor_;
    const NedTransform* ned_transform_;

    msr::airlib::vector<msr::airlib::real_T> laser_angles_;
    float current_horizontal_angle_ = 0.0f;
    std::unique_ptr<LidarSharedMemoryWriter> shared_memory_writer_;
    uint64_t shared_memory_frame_id_ = 0;
    std::unique_ptr<LidarIdentitySharedMemoryWriter> identity_writer_;
    std::vector<uint32_t> identity_points_;
    // Metadata is constant during one scan, but can change between scans.
    TMap<const AActor*, TPair<int, uint32_t>> scan_metadata_;
};
