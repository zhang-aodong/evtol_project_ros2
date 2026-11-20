#include "offline_fusion/vision.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <map>
#include <optional>
#include <stdexcept>
#include <unordered_map>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "offline_fusion/dataset.hpp"
#include "offline_fusion/math.hpp"

namespace offline_fusion {
namespace {

struct TrackObservation {
    std::size_t node_index = 0;
    double t = 0.0;
    cv::Point2f pixel;
};

bool in_range(double t, double start, double end) {
    return t >= start && t <= end;
}

cv::Mat load_gray_image(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Failed to open image " + path.string());
    }
    input.seekg(0, std::ios::end);
    const auto size = input.tellg();
    if (size <= 0) throw std::runtime_error("Empty image " + path.string());
    input.seekg(0);
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
    if (!input.read(reinterpret_cast<char*>(bytes.data()), size))
        throw std::runtime_error("Incomplete image read " + path.string());
    cv::Mat image = cv::imdecode(bytes, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        throw std::runtime_error("Failed to decode image " + path.string());
    }
    return image;
}

std::optional<std::size_t> find_state_index(const std::vector<double>& state_times, double t) {
    constexpr double kTolerance = 1e-5;
    const auto it = std::lower_bound(state_times.begin(), state_times.end(), t);
    if (it != state_times.end() && std::abs(*it - t) <= kTolerance) {
        return static_cast<std::size_t>(std::distance(state_times.begin(), it));
    }
    if (it != state_times.begin()) {
        const auto prev = it - 1;
        if (std::abs(*prev - t) <= kTolerance) {
            return static_cast<std::size_t>(std::distance(state_times.begin(), prev));
        }
    }
    return std::nullopt;
}

cv::Point2d normalize_pixel(const cv::Point2f& pixel, const FusionOptions& options) {
    return {
        (static_cast<double>(pixel.x) - options.camera_cx) / options.camera_fx,
        (static_cast<double>(pixel.y) - options.camera_cy) / options.camera_fy,
    };
}

cv::Point3d to_cv(const Vec3& value) {
    return {value.x, value.y, value.z};
}

Vec3 to_vec3(const cv::Point3d& value) {
    return {value.x, value.y, value.z};
}

double frontend_image_scale(const FusionOptions& options) {
    if (options.visual_image_scale <= 0.0) {
        throw std::runtime_error("visual_image_scale must be positive");
    }
    return std::min(1.0, options.visual_image_scale);
}

cv::Mat scale_gray_image(const cv::Mat& gray, double image_scale) {
    if (std::abs(image_scale - 1.0) < 1e-12) {
        return gray;
    }
    cv::Mat scaled;
    cv::resize(gray, scaled, cv::Size(), image_scale, image_scale, cv::INTER_AREA);
    return scaled;
}

cv::Point2f to_original_pixel(const cv::Point2f& pixel, double image_scale) {
    return {static_cast<float>(static_cast<double>(pixel.x) / image_scale),
            static_cast<float>(static_cast<double>(pixel.y) / image_scale)};
}

cv::Matx33d rotation_body_to_camera(const FusionOptions& options) {
    const double a = options.camera_pitch_deg * 3.14159265358979323846 / 180.0;
    const double c = std::cos(a), s = std::sin(a);
    return cv::Matx33d(0.0, 1.0, 0.0,
                       s, 0.0, c,
                       c, 0.0, -s);
}

cv::Matx33d rotation_body_to_world(const Quat& q_body_to_world) {
    const Vec3 x = rotate(q_body_to_world, {1.0, 0.0, 0.0});
    const Vec3 y = rotate(q_body_to_world, {0.0, 1.0, 0.0});
    const Vec3 z = rotate(q_body_to_world, {0.0, 0.0, 1.0});
    return cv::Matx33d(x.x, y.x, z.x,
                       x.y, y.y, z.y,
                       x.z, y.z, z.z);
}

cv::Matx34d projection_world_to_normalized_camera(const VisualPose& pose,
                                                  const FusionOptions& options) {
    const Vec3& p_wb_vec = pose.position_ned;
    const Quat& q_wb = pose.q_body_to_ned;
    const cv::Matx33d r_wb = rotation_body_to_world(q_wb);
    const cv::Matx33d r_bw = r_wb.t();
    const cv::Matx33d r_cb = rotation_body_to_camera(options);
    const cv::Matx33d r_cw = r_cb * r_bw;
    const cv::Vec3d p_wb(p_wb_vec.x, p_wb_vec.y, p_wb_vec.z);
    const cv::Vec3d t_bc(options.camera_t_body_camera.x,
                         options.camera_t_body_camera.y,
                         options.camera_t_body_camera.z);
    const cv::Vec3d t_cw = -(r_cw * p_wb) - r_cb * t_bc;

    return cv::Matx34d(r_cw(0, 0), r_cw(0, 1), r_cw(0, 2), t_cw(0),
                       r_cw(1, 0), r_cw(1, 1), r_cw(1, 2), t_cw(1),
                       r_cw(2, 0), r_cw(2, 1), r_cw(2, 2), t_cw(2));
}

Vec3 world_to_camera(const VisualPose& pose,
                     const Vec3& point_ned,
                     const FusionOptions& options) {
    const Vec3 p_body = rotate_inverse(pose.q_body_to_ned,
                                      point_ned - pose.position_ned) -
                        options.camera_t_body_camera;
    const auto p = rotation_body_to_camera(options) * cv::Vec3d(p_body.x, p_body.y, p_body.z);
    return {p[0], p[1], p[2]};
}

bool project_point(const std::vector<VisualPose>& initial_poses,
                   const Vec3& point_ned,
                   const TrackObservation& observation,
                   const FusionOptions& options,
                   cv::Point2d* projected) {
    if (observation.node_index >= initial_poses.size()) {
        return false;
    }
    const Vec3 p_camera = world_to_camera(
        initial_poses[observation.node_index], point_ned, options);
    if (p_camera.z <= 0.1 || !std::isfinite(p_camera.z)) {
        return false;
    }
    projected->x = options.camera_fx * p_camera.x / p_camera.z + options.camera_cx;
    projected->y = options.camera_fy * p_camera.y / p_camera.z + options.camera_cy;
    return std::isfinite(projected->x) && std::isfinite(projected->y);
}

// CAMERA_V3: select the strongest pair across a track, not first-versus-last.
struct TriangulationDiagnostic {
    const char* reason = "too_few_observations";
    double angle_deg = 0, baseline_m = 0, min_depth_m = 0, rms_px = 0;
    std::size_t first_index = 0, second_index = 0;
};

bool triangulate_track(const std::vector<VisualPose>& initial_poses,
                       const std::vector<TrackObservation>& observations,
                       const FusionOptions& options,
                       Vec3* point_ned,
                       double* squared_error_sum_out,
                       std::size_t* projected_count_out,
                       TriangulationDiagnostic* diagnostic = nullptr) {
    TriangulationDiagnostic local;
    auto& diag = diagnostic ? *diagnostic : local;
    diag = TriangulationDiagnostic{};
    if (observations.size() < 2) {
        return false;
    }

    for (const auto& o : observations) {
        if (o.node_index >= initial_poses.size()) {
            diag.reason = "invalid_node"; return false;
        }
    }
    auto world_ray = [&](const TrackObservation& o) {
        const auto uv = normalize_pixel(o.pixel, options);
        cv::Vec3d ray = rotation_body_to_camera(options).t() * cv::Vec3d(uv.x, uv.y, 1.0);
        ray = rotation_body_to_world(initial_poses[o.node_index].q_body_to_ned) * ray;
        return ray / cv::norm(ray);
    };
    std::vector<cv::Vec3d> rays;
    std::vector<Vec3> centers;
    for (const auto& o : observations) {
        rays.push_back(world_ray(o));
        const auto& pose = initial_poses[o.node_index];
        centers.push_back(pose.position_ned + rotate(pose.q_body_to_ned, options.camera_t_body_camera));
    }
    bool have_pair = false;
    bool had_geometric_baseline = false;
    for (std::size_t i=0; i<observations.size(); ++i) {
        for (std::size_t j=i+1; j<observations.size(); ++j) {
            const double baseline = norm(centers[j]-centers[i]);
            if (baseline <= 1e-6) continue;
            had_geometric_baseline = true;
            const double angle = std::acos(std::clamp(rays[i].dot(rays[j]),-1.0,1.0))*180.0/std::acos(-1.0);
            if (angle >= 90.0) continue; // inconsistent forward-view geometry
            // CAMERA_V4: among geometrically valid pairs, prefer the shortest
            // baseline. Long-span pairs amplify the error of the initial pose
            // trajectory under degraded GNSS/IMU.
            const double min_angle = options.visual_min_triangulation_deg;
            if (angle >= min_angle && (!have_pair || baseline < diag.baseline_m)) {
                have_pair=true; diag.angle_deg=angle; diag.baseline_m=baseline;
                diag.first_index=i; diag.second_index=j;
            }
        }
    }
    // Pixel displacement is rotation-dependent; use world-ray parallax instead.
    if (!have_pair) {
        diag.reason = had_geometric_baseline ? "low_angle" : "no_baseline";
        return false;
    }
    if (diag.angle_deg < options.visual_min_triangulation_deg) {
        diag.reason="low_angle"; return false;
    }
    const auto& first = observations[diag.first_index];
    const auto* last = &observations[diag.second_index];
    const cv::Matx34d p0 = projection_world_to_normalized_camera(
        initial_poses[first.node_index], options);
    const cv::Matx34d p1 = projection_world_to_normalized_camera(
        initial_poses[last->node_index], options);
    std::vector<cv::Point2d> pts0{normalize_pixel(first.pixel, options)};
    std::vector<cv::Point2d> pts1{normalize_pixel(last->pixel, options)};
    cv::Mat homogeneous;
    cv::triangulatePoints(p0, p1, pts0, pts1, homogeneous);
    const double w = homogeneous.at<double>(3, 0);
    if (std::abs(w) < 1e-12 || !std::isfinite(w)) {
        diag.reason="homogeneous_invalid";
        return false;
    }

    const cv::Point3d candidate(homogeneous.at<double>(0, 0) / w,
                                homogeneous.at<double>(1, 0) / w,
                                homogeneous.at<double>(2, 0) / w);
    *point_ned = to_vec3(candidate);
    if (!std::isfinite(point_ned->x) || !std::isfinite(point_ned->y) ||
        !std::isfinite(point_ned->z)) {
        diag.reason="nonfinite_point";
        return false;
    }

    double squared_error_sum = 0.0;
    std::size_t projected_count = 0;
    diag.min_depth_m = world_to_camera(initial_poses[first.node_index], *point_ned, options).z;
    for (const auto& observation : observations) {
        diag.min_depth_m = std::min(diag.min_depth_m,
            world_to_camera(initial_poses[observation.node_index], *point_ned, options).z);
        cv::Point2d projected;
        if (!project_point(initial_poses, *point_ned, observation, options, &projected)) {
            diag.reason="invalid_depth_or_projection";
            return false; // Never send behind-camera observations to the backend.
        }
        const double du = projected.x - observation.pixel.x;
        const double dv = projected.y - observation.pixel.y;
        squared_error_sum += du * du + dv * dv;
        ++projected_count;
    }
    if (projected_count < 2) {
        return false;
    }
    const double rms = std::sqrt(squared_error_sum / static_cast<double>(projected_count));
    diag.rms_px = rms;
    diag.reason = rms <= options.visual_max_initial_reprojection_px ? "accepted" : "reprojection";
    *squared_error_sum_out = squared_error_sum;
    *projected_count_out = projected_count;
    return rms <= options.visual_max_initial_reprojection_px;
}

void add_new_features(const cv::Mat& gray,
                      const FusionOptions& options,
                      std::vector<cv::Point2f>* points,
                      std::vector<int>* ids,
                      int* next_track_id) {
    const int needed = options.visual_max_features - static_cast<int>(points->size());
    if (needed <= 0) {
        return;
    }

    cv::Mat mask(gray.size(), CV_8UC1, cv::Scalar(255));
    const int radius = std::max(4, static_cast<int>(options.visual_min_parallax_px));
    for (const auto& point : *points) {
        cv::circle(mask, point, radius, cv::Scalar(0), -1);
    }

    std::vector<cv::Point2f> new_points;
    cv::goodFeaturesToTrack(gray,
                            new_points,
                            needed,
                            0.01,
                            options.visual_min_parallax_px,
                            mask);
    for (const auto& point : new_points) {
        points->push_back(point);
        ids->push_back((*next_track_id)++);
    }
}

void filter_tracks_with_fundamental_matrix(const std::vector<cv::Point2f>& previous,
                                           const std::vector<cv::Point2f>& current,
                                           const std::vector<int>& ids,
                                           std::vector<cv::Point2f>* filtered_points,
                                           std::vector<int>* filtered_ids) {
    if (current.size() < 8) {
        *filtered_points = current;
        *filtered_ids = ids;
        return;
    }

    std::vector<unsigned char> inlier_mask;
    cv::findFundamentalMat(previous, current, cv::FM_RANSAC, 1.5, 0.99, inlier_mask);
    filtered_points->clear();
    filtered_ids->clear();
    for (std::size_t i = 0; i < current.size(); ++i) {
        if (i < inlier_mask.size() && inlier_mask[i]) {
            filtered_points->push_back(current[i]);
            filtered_ids->push_back(ids[i]);
        }
    }
}

std::vector<VisualObservation> choose_landmark_observations(
    const std::vector<TrackObservation>& observations,
    int max_observations) {
    std::vector<VisualObservation> selected;
    if (observations.empty() || max_observations <= 0) {
        return selected;
    }

    const std::size_t target = std::min<std::size_t>(
        observations.size(), static_cast<std::size_t>(max_observations));
    selected.reserve(target);
    if (target == observations.size()) {
        for (const auto& observation : observations) {
            selected.push_back(
                {observation.node_index, observation.pixel.x, observation.pixel.y});
        }
        return selected;
    }

    for (std::size_t i = 0; i < target; ++i) {
        const std::size_t index =
            static_cast<std::size_t>(std::round(
                static_cast<double>(i) * static_cast<double>(observations.size() - 1) /
                static_cast<double>(target - 1)));
        const auto& observation = observations[index];
        selected.push_back({observation.node_index, observation.pixel.x, observation.pixel.y});
    }
    return selected;
}

}  // namespace

std::vector<double> select_visual_keyframe_times(const Dataset& dataset,
                                                 double start_time,
                                                 double end_time,
                                                 const FusionOptions& options) {
    std::vector<double> keyframe_times;
    if (!options.use_visual || dataset.camera_frames.empty()) {
        return keyframe_times;
    }

    double next_keyframe_time = start_time;
    for (const auto& frame : dataset.camera_frames) {
        if (!in_range(frame.t, start_time, end_time)) {
            continue;
        }
        if (keyframe_times.empty() || frame.t + 1e-6 >= next_keyframe_time) {
            keyframe_times.push_back(frame.t);
            next_keyframe_time = frame.t + options.visual_keyframe_interval_s;
        }
    }
    return keyframe_times;
}

VisualMap build_visual_map(const Dataset& dataset,
                           const std::vector<VisualPose>& initial_poses,
                           double start_time,
                           double end_time,
                           const FusionOptions& options) {
    VisualMap visual_map;
    if (!options.use_visual || dataset.camera_frames.empty()) {
        return visual_map;
    }
    if (initial_poses.empty()) {
        throw std::runtime_error("Cannot build visual map without initial state poses");
    }

    std::vector<double> state_times;
    state_times.reserve(initial_poses.size());
    for (const auto& pose : initial_poses) {
        state_times.push_back(pose.t);
    }

    cv::setNumThreads(2);
    const auto camera_times = select_visual_keyframe_times(dataset, start_time, end_time, options);
    if (!std::isfinite(options.visual_min_triangulation_deg) || options.visual_min_triangulation_deg <= 0.0)
        throw std::runtime_error("visual-min-triangulation-deg must be positive");
    cv::Mat previous_gray;
    std::vector<cv::Point2f> previous_points;
    std::vector<int> previous_ids;
    std::unordered_map<int, std::vector<TrackObservation>> track_observations;
    int next_track_id = 0;
    const double image_scale = frontend_image_scale(options);
    FusionOptions frontend_options = options;
    frontend_options.visual_min_parallax_px =
        std::max(2.0, options.visual_min_parallax_px * image_scale);

    for (const auto& frame : dataset.camera_frames) {
        if (!in_range(frame.t, start_time, end_time)) {
            continue;
        }

        const std::optional<std::size_t> node_index = find_state_index(state_times, frame.t);
        const bool is_keyframe = node_index.has_value() && find_state_index(camera_times, frame.t).has_value();

        cv::Mat gray = scale_gray_image(load_gray_image(frame.image_path), image_scale);
        ++visual_map.frames_loaded;

        if (previous_gray.empty() || previous_points.empty()) {
            add_new_features(
                gray, frontend_options, &previous_points, &previous_ids, &next_track_id);
            if (is_keyframe) ++visual_map.keyframes;
            for (std::size_t i = 0; is_keyframe && i < previous_points.size(); ++i) {
                track_observations[previous_ids[i]].push_back(
                    {*node_index, frame.t, to_original_pixel(previous_points[i], image_scale)});
            }
            previous_gray = gray;
            continue;
        }

        std::vector<cv::Point2f> tracked_points;
        std::vector<unsigned char> status;
        std::vector<float> errors;
        cv::calcOpticalFlowPyrLK(previous_gray,
                                 gray,
                                 previous_points,
                                 tracked_points,
                                 status,
                                 errors,
                                 cv::Size(21, 21),
                                 3);

        std::vector<cv::Point2f> backwards;
        std::vector<unsigned char> back_status;
        std::vector<float> back_errors;
        if (!tracked_points.empty()) cv::calcOpticalFlowPyrLK(gray, previous_gray, tracked_points,
            backwards, back_status, back_errors, cv::Size(21,21), 3);
        std::vector<cv::Point2f> valid_previous;
        std::vector<cv::Point2f> valid_current;
        std::vector<int> valid_ids;
        for (std::size_t i = 0; i < tracked_points.size(); ++i) {
            if (!status[i] || !back_status[i] || cv::norm(backwards[i] - previous_points[i]) > 1.0) {
                continue;
            }
            const auto& point = tracked_points[i];
            if (point.x < 0.0F || point.y < 0.0F ||
                point.x >= static_cast<float>(gray.cols) ||
                point.y >= static_cast<float>(gray.rows)) {
                continue;
            }
            valid_previous.push_back(previous_points[i]);
            valid_current.push_back(point);
            valid_ids.push_back(previous_ids[i]);
        }

        std::vector<cv::Point2f> current_points;
        std::vector<int> current_ids;
        filter_tracks_with_fundamental_matrix(valid_previous,
                                              valid_current,
                                              valid_ids,
                                              &current_points,
                                              &current_ids);
        add_new_features(
            gray, frontend_options, &current_points, &current_ids, &next_track_id);

        if (is_keyframe) ++visual_map.keyframes;
        for (std::size_t i = 0; is_keyframe && i < current_points.size(); ++i) {
            track_observations[current_ids[i]].push_back(
                {*node_index, frame.t, to_original_pixel(current_points[i], image_scale)});
        }

        previous_gray = gray;
        previous_points = std::move(current_points);
        previous_ids = std::move(current_ids);
    }

    std::vector<int> track_ids;
    track_ids.reserve(track_observations.size());
    for (const auto& item : track_observations) {
        if (item.second.size() >= 3) {
            track_ids.push_back(item.first);
        }
    }
    std::sort(track_ids.begin(), track_ids.end(), [&track_observations](int a, int b) {
        const auto na = track_observations[a].size(), nb = track_observations[b].size();
        return na == nb ? a < b : na > nb;
    });

    visual_map.tracks = track_ids.size();
    const auto diagnostic_path = options.output_csv.parent_path()/"visual_track_diagnostics.csv";
    if (!diagnostic_path.parent_path().empty()) std::filesystem::create_directories(diagnostic_path.parent_path());
    std::ofstream diagnostics(diagnostic_path);
    if (!diagnostics) throw std::runtime_error("Cannot write visual track diagnostics");
    diagnostics << "track_id,observations,start_s,end_s,reason,first_index,second_index,angle_deg,baseline_m,min_depth_m,rms_px\n";
    for (int track_id : track_ids) {
        if (visual_map.landmarks.size() >=
            static_cast<std::size_t>(std::max(0, options.visual_max_landmarks))) {
            break;
        }

        const auto& observations = track_observations[track_id];
        Vec3 point_ned;
        double squared_error_sum = 0.0;
        std::size_t projected_count = 0;
        TriangulationDiagnostic diag;
        const bool accepted = triangulate_track(initial_poses,
                               observations,
                               options,
                               &point_ned,
                               &squared_error_sum,
                               &projected_count, &diag);
        diagnostics << track_id << ',' << observations.size() << ',' << observations.front().t << ','
                    << observations.back().t << ',' << diag.reason << ',' << diag.first_index << ','
                    << diag.second_index << ',' << diag.angle_deg << ',' << diag.baseline_m << ','
                    << diag.min_depth_m << ',' << diag.rms_px << '\n';
        if (!accepted) {
            ++visual_map.rejected_tracks;
            continue;
        }

        VisualLandmark landmark;
        landmark.position_ned = point_ned;
        landmark.observations = choose_landmark_observations(
            observations, options.visual_max_observations_per_landmark);
        if (landmark.observations.size() < 2) {
            ++visual_map.rejected_tracks;
            continue;
        }
        visual_map.reprojection_factors += landmark.observations.size();
        visual_map.initial_reprojection_observations += projected_count;
        visual_map.initial_reprojection_rmse_px += squared_error_sum;
        visual_map.landmarks.push_back(std::move(landmark));
    }

    if (visual_map.initial_reprojection_observations > 0) {
        visual_map.initial_reprojection_rmse_px = std::sqrt(
            visual_map.initial_reprojection_rmse_px /
            static_cast<double>(visual_map.initial_reprojection_observations));
    }

    return visual_map;
}

}  // namespace offline_fusion
