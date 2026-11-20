#include "../src/vision.cpp"
#include <cassert>
void test_camera_frontend() {
    using namespace offline_fusion;
    FusionOptions options;
    options.camera_pitch_deg = -10;
    options.camera_t_body_camera = {0.3,0,0};
    VisualPose pose;
    pose.position_ned = {4,5,6};
    pose.q_body_to_ned = {1,0,0,0};
    const double a = -10 * std::acos(-1.0) / 180;
    Vec3 point{4.3+10*std::cos(a),5,6-10*std::sin(a)};
    Vec3 camera = world_to_camera(pose, point, options);
    assert(std::abs(camera.x)<1e-8 && std::abs(camera.y)<1e-8 && std::abs(camera.z-10)<1e-8);
    auto projection = projection_world_to_normalized_camera(pose, options);
    auto p = projection * cv::Vec4d(point.x,point.y,point.z,1);
    assert(std::abs(p[0])<1e-8 && std::abs(p[1])<1e-8 && std::abs(p[2]-10)<1e-8);

    // Pure camera rotation yields large pixel flow but no triangulation baseline.
    options.camera_pitch_deg=0;
    options.camera_t_body_camera={0,0,0};
    std::vector<VisualPose> poses(2);
    poses[0].q_body_to_ned={1,0,0,0};
    poses[1].q_body_to_ned=quat_from_delta_angle({0,0,0.05});
    const Vec3 landmark{10,1,0.5};
    std::vector<TrackObservation> obs;
    for (int i=0;i<2;++i) {
        auto c=world_to_camera(poses[i],landmark,options);
        obs.push_back({static_cast<std::size_t>(i),double(i),cv::Point2f(
            options.camera_fx*c.x/c.z+options.camera_cx,
            options.camera_fy*c.y/c.z+options.camera_cy)});
    }
    Vec3 estimated; double error; std::size_t count;
    assert(!triangulate_track(poses,obs,options,&estimated,&error,&count));
    // A genuine lateral translation provides depth and must remain usable.
    poses[1].q_body_to_ned={1,0,0,0};
    poses[1].position_ned={0,1,0};
    auto c=world_to_camera(poses[1],landmark,options);
    obs[1].pixel=cv::Point2f(options.camera_fx*c.x/c.z+options.camera_cx,
                           options.camera_fy*c.y/c.z+options.camera_cy);
    assert(triangulate_track(poses,obs,options,&estimated,&error,&count));
    assert(norm(estimated-landmark)<1e-3);
    // Each pair with frame zero is below 1.5 deg, but frames 1 and 2
    // provide sufficient parallax. The old first-frame-only search misses it.
    options.visual_min_triangulation_deg=1.5;
    poses.resize(3); obs.clear();
    for (int i=0; i<3; ++i) {
        poses[i].q_body_to_ned={1,0,0,0};
        poses[i].position_ned={0, i==0 ? 0.0 : (i==1 ? -0.2 : 0.2), 0};
        auto pc=world_to_camera(poses[i],landmark,options);
        obs.push_back({static_cast<std::size_t>(i),double(i),cv::Point2f(
            options.camera_fx*pc.x/pc.z+options.camera_cx,
            options.camera_fy*pc.y/pc.z+options.camera_cy)});
    }
    TriangulationDiagnostic diag;
    assert(triangulate_track(poses,obs,options,&estimated,&error,&count,&diag));
    assert(diag.first_index==1 && diag.second_index==2);
    assert(norm(estimated-landmark)<1e-3);
    options.visual_min_triangulation_deg=10;
    assert(!triangulate_track(poses,obs,options,&estimated,&error,&count,&diag));
    assert(std::string(diag.reason)=="low_angle");
}
