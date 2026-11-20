#include "../src/lidar.cpp"
#include <cassert>
int main() {
    using namespace offline_fusion;
    cv::setNumThreads(1);
    FusionOptions options;
    options.lidar_min_correspondences=100;
    std::vector<cv::Point3f> plane, corner;
    for(int i=1;i<=25;++i) for(int j=1;j<=25;++j) {
        float a=i*0.2f,b=j*0.2f;
        plane.emplace_back(a,b,0);
        corner.emplace_back(a,b,0);
        corner.emplace_back(0,a,b);
        corner.emplace_back(a,0,b);
    }
    auto flat=run_icp(plane,plane,cv::Matx33d::eye(),cv::Vec3d(0,0,0),options);
    assert(!flat.valid); // A single plane cannot constrain in-plane translation.
    auto solid=run_icp(corner,corner,cv::Matx33d::eye(),cv::Vec3d(0,0,0),options);
    assert(solid.valid);
    assert(solid.rmse_m<1e-6);
}
