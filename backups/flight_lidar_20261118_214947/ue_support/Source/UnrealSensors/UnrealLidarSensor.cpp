// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "UnrealLidarSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"

// ctor
UnrealLidarSensor::UnrealLidarSensor(const AirSimSettings::LidarSetting& setting,
                                     AActor* actor, const NedTransform* ned_transform)
    : LidarSimple(setting), actor_(actor), ned_transform_(ned_transform)
{
    createLasers();
    shared_memory_writer_ = std::make_unique<LidarSharedMemoryWriter>(setting.sensor_name, 65536, 8);
    identity_writer_ = std::make_unique<LidarIdentitySharedMemoryWriter>(setting.sensor_name, 65536, 8);
}

// initializes information based on lidar configuration
void UnrealLidarSensor::createLasers()
{
    const auto& params = getParams();

    const auto number_of_lasers = params.number_of_channels;

    if (number_of_lasers <= 0)
        return;

    // calculate verticle angle distance between each laser
    float delta_angle = 0;
    if (number_of_lasers > 1)
        delta_angle = (params.vertical_FOV_upper - (params.vertical_FOV_lower)) /
                      static_cast<float>(number_of_lasers - 1);

    // store vertical angles for each laser
    laser_angles_.clear();
    for (auto i = 0u; i < number_of_lasers; ++i) {
        const float vertical_angle = params.vertical_FOV_upper - static_cast<float>(i) * delta_angle;
        laser_angles_.emplace_back(vertical_angle);
    }
}

// returns a point-cloud for the tick
void UnrealLidarSensor::getPointCloud(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
                                      const msr::airlib::TTimeDelta delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<int>& segmentation_cloud)
{
    point_cloud.clear();
    segmentation_cloud.clear();
    auto& object_ids = identity_points_;
    object_ids.clear();

    const auto& params = getParams();
    const auto sensor_world = lidar_pose + vehicle_pose;
    scan_metadata_.Reset();
    const auto number_of_lasers = params.number_of_channels;

    // cap the points to scan via ray-tracing; this is currently needed for car/Unreal tick scenarios
    // since SensorBase mechanism uses the elapsed clock time instead of the tick delta-time.
    constexpr float MAX_POINTS_IN_SCAN = 1e+5f;
    // Wall-clock triggers and fixed-step simulation clocks do not advance by
    // the same amount during catch-up. Using sim delta here produced empty
    // frames and 20-degree wedges even though the publisher ran at 20 Hz.
    // A complete scan is an instantaneous FOV snapshot at this vehicle pose;
    // keep a fixed angular grid/ray budget rather than replaying elapsed turns.
    const auto scan_time = params.complete_scan_per_update
        ? 1.0 / std::max(1u, params.horizontal_rotation_frequency) : delta_time;
    uint32 total_points_to_scan = FMath::RoundHalfFromZero(params.points_per_second * scan_time);
    if (total_points_to_scan > MAX_POINTS_IN_SCAN) {
        total_points_to_scan = MAX_POINTS_IN_SCAN;
        UAirBlueprintLib::LogMessageString("Lidar: ", "Capping number of points to scan", LogDebugLevel::Failure);
    }

    // calculate number of points needed for each laser/channel
    const uint32 points_to_scan_with_one_laser = FMath::RoundHalfFromZero(total_points_to_scan / float(number_of_lasers));
    if (points_to_scan_with_one_laser <= 0) {
        //UAirBlueprintLib::LogMessageString("Lidar: ", "No points requested this frame", LogDebugLevel::Failure);
        return;
    }

    // calculate needed angle/distance between each point
    const float angle_distance_of_tick = params.complete_scan_per_update
        ? 360.0f : params.horizontal_rotation_frequency * 360.0f * delta_time;
    const float angle_distance_of_laser_measure = angle_distance_of_tick / points_to_scan_with_one_laser;

    // normalize FOV start/end
    const float laser_start = std::fmod(360.0f + params.horizontal_FOV_start, 360.0f);
    const float laser_end = std::fmod(360.0f + params.horizontal_FOV_end, 360.0f);

    // The angular grid and FOV are unchanged. Trigonometry for each horizontal
    // sample is shared by all channels instead of constructing 3 quaternions
    // and copying LidarSimpleParams for every ray.
    struct HorizontalSample { float cosine, sine; };
    std::vector<HorizontalSample> horizontal_samples;
    horizontal_samples.reserve(points_to_scan_with_one_laser);
    for (auto i = 0u; i < points_to_scan_with_one_laser; ++i) {
        const float angle = std::fmod(current_horizontal_angle_ + angle_distance_of_laser_measure * i, 360.0f);
        if (VectorMath::isAngleBetweenAngles(angle, laser_start, laser_end)) {
            const auto radians = msr::airlib::Utils::degreesToRadians(angle);
            horizontal_samples.push_back({std::cos(radians), std::sin(radians)});
        }
    }
    // shoot lasers
    for (auto laser = 0u; laser < number_of_lasers; ++laser) {
        const float vertical_angle = laser_angles_[laser];
        const auto pitch = msr::airlib::Utils::degreesToRadians(vertical_angle);
        const float cp = std::cos(pitch), sp = std::sin(pitch);

        for (const auto& sample : horizontal_samples) {
            const Vector3r direction_local(cp * sample.cosine, cp * sample.sine, -sp);

            Vector3r point;
            int segmentationID = -1;
            uint32_t object_id = 0;
            // shoot laser and get the impact point, if any
            if (shootLaser(sensor_world, direction_local, params, point, segmentationID, object_id)) {
                point_cloud.emplace_back(point.x());
                point_cloud.emplace_back(point.y());
                point_cloud.emplace_back(point.z());
                segmentation_cloud.emplace_back(segmentationID);
                object_ids.emplace_back(object_id);
            }
        }
    }

    current_horizontal_angle_ = std::fmod(current_horizontal_angle_ + angle_distance_of_tick, 360.0f);

    // Publish directly from the Unreal LiDAR producer. The writer only copies
    // the completed frame into a lock-free ring slot and never waits for Python.
    if (shared_memory_writer_ && shared_memory_writer_->isValid()) {
        const uint64_t frame_id = ++shared_memory_frame_id_;
        const uint64_t timestamp_ns = clock()->nowNanos();
        // Publish identities first; XYZ consumers then read the exact same slot.
        if (identity_writer_) identity_writer_->publish(frame_id, timestamp_ns, object_ids);
        shared_memory_writer_->publish(frame_id, timestamp_ns, point_cloud, params.complete_scan_per_update);
    }

    return;
}

// simulate shooting a laser via Unreal ray-tracing.
bool UnrealLidarSensor::shootLaser(const msr::airlib::Pose& sensor_world, const Vector3r& direction_local,
                                   const msr::airlib::LidarSimpleParams& params, Vector3r& point, int& segmentationID, uint32_t& object_id)
{
    // start position
    const Vector3r& start = sensor_world.position;

    // We need to compose rotations here rather than rotate a vector by a quaternion
    // Hence using coordOrientationAdd(..) rather than rotateQuaternion(..)

    // get ray quaternion in lidar frame (angles must be in radians)
    const Vector3r end = VectorMath::rotateVector(direction_local, sensor_world.orientation, true) * params.range + start;

    FHitResult hit_result = FHitResult(ForceInit);
    bool is_hit = UAirBlueprintLib::GetObstacle(actor_, ned_transform_->fromLocalNed(start), ned_transform_->fromLocalNed(end), hit_result, actor_, ECC_Visibility);

    if (is_hit) {
        //Store the segmentation id of the hit object.
        auto hitActor = hit_result.GetActor();
        if (const auto* metadata = scan_metadata_.Find(hitActor)) {
            segmentationID = metadata->Key;
            object_id = metadata->Value;
        }
        else if (hitActor != nullptr) {
            object_id = identity_writer_ ? identity_writer_->objectId(hitActor) : 0;
            TArray<UMeshComponent*> meshComponents;
            hitActor->GetComponents<UMeshComponent>(meshComponents);
            for (int i = 0; i < meshComponents.Num(); i++) {
                segmentationID = segmentationID == -1 ? meshComponents[i]->CustomDepthStencilValue : segmentationID;
            }
            scan_metadata_.Add(hitActor, TPair<int, uint32_t>(segmentationID, object_id));
        }

        if (false && UAirBlueprintLib::IsInGameThread()) {
            // Debug code for very specific cases.
            // Mostly shouldn't be needed. Use SimModeBase::drawLidarDebugPoints()
            DrawDebugPoint(
                actor_->GetWorld(),
                hit_result.ImpactPoint,
                5, //size
                FColor::Red,
                true, //persistent (never goes away)
                0.1 //point leaves a trail on moving object
            );
        }

        // decide the frame for the point-cloud
        if (params.data_frame == AirSimSettings::kVehicleInertialFrame) {
            // current detault behavior; though it is probably not very useful.
            // not changing the default for now to maintain backwards-compat.
            point = ned_transform_->toLocalNed(hit_result.ImpactPoint);
        }
        else if (params.data_frame == AirSimSettings::kSensorLocalFrame) {
            // point in vehicle intertial frame
            Vector3r point_v_i = ned_transform_->toLocalNed(hit_result.ImpactPoint);

            // tranform to lidar frame
            point = VectorMath::transformToBodyFrame(point_v_i, sensor_world, true);

            // The above should be same as first transforming to vehicle-body frame and then to lidar frame
            //    Vector3r point_v_b = VectorMath::transformToBodyFrame(point_v_i, vehicle_pose, true);
            //    point = VectorMath::transformToBodyFrame(point_v_b, lidar_pose, true);

            // On the client side, if it is needed to transform this data back to the world frame,
            // then do the equivalent of following,
            //     Vector3r point_w = VectorMath::transformToWorldFrame(point, lidar_pose + vehicle_pose, true);
            // See SimModeBase::drawLidarDebugPoints()

            // TODO: Optimization -- instead of doing this for every point, it should be possible to do this
            // for the point-cloud together? Need to look into matrix operations to do this together for all points.
        }
        else
            throw std::runtime_error("Unknown requested data frame");

        return true;
    }
    else {
        return false;
    }
}
