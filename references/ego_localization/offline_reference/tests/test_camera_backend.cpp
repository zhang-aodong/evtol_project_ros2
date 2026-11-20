#include "../src/ceres_fusion.cpp"
#include <cassert>
void test_camera_frontend();
int main() {
    using namespace offline_fusion;
    const double a = -10.0 * std::acos(-1.0) / 180.0;
    const double position[3] = {0, 0, 0}, q[4] = {1, 0, 0, 0};
    // A world point ten metres along the pitched camera optical axis
    // must project to the principal point, including the lever arm.
    const double point[3] = {0.3 + 10*std::cos(a), 0, -10*std::sin(a)};
    ReprojectionCost cost{1024, 768, 3247.7131, 3247.7131, 1024, 768, {0.3,0,0}, a, 1};
    double residuals[2];
    cost(position, q, point, residuals);
    assert(std::abs(residuals[0]) < 1e-8 && std::abs(residuals[1]) < 1e-8);
    // With zero pitch the original axis permutation remains exact.
    cost.camera_pitch_rad = 0;
    cost.u_px = 1024 + 3247.7131*0.2;
    cost.v_px = 768 + 3247.7131*0.1;
    const double legacy_point[3] = {10.3, 2, 1};
    cost(position, q, legacy_point, residuals);
    assert(std::abs(residuals[0]) < 1e-8 && std::abs(residuals[1]) < 1e-8);
    test_camera_frontend();
    std::cout << "Camera pitch geometry and zero-pitch regression passed\n";
}
