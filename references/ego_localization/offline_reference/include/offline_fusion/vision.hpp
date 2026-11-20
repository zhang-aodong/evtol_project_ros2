#pragma once

#include <cstddef>
#include <vector>

#include "offline_fusion/fusion.hpp"
#include "offline_fusion/types.hpp"

namespace offline_fusion {

struct VisualObservation {
    std::size_t node_index = 0;
    double u_px = 0.0;
    double v_px = 0.0;
};

struct VisualPose {
    double t = 0.0;
    Vec3 position_ned;
    Quat q_body_to_ned;
};

struct VisualLandmark {
    Vec3 position_ned;
    std::vector<VisualObservation> observations;
};

struct VisualMap {
    std::vector<VisualLandmark> landmarks;
    std::size_t frames_loaded = 0;
    std::size_t keyframes = 0;
    std::size_t tracks = 0;
    std::size_t rejected_tracks = 0;
    std::size_t reprojection_factors = 0;
    std::size_t initial_reprojection_observations = 0;
    double initial_reprojection_rmse_px = 0.0;
};

std::vector<double> select_visual_keyframe_times(const Dataset& dataset,
                                                 double start_time,
                                                 double end_time,
                                                 const FusionOptions& options);

VisualMap build_visual_map(const Dataset& dataset,
                           const std::vector<VisualPose>& initial_poses,
                           double start_time,
                           double end_time,
                           const FusionOptions& options);

}  // namespace offline_fusion
