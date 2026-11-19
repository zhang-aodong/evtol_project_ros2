// Verify optimized directions against AirSim's original quaternion composition.
// Build with matching AirLib/include and AirLib/deps/eigen3 include directories.
#include "common/Common.hpp"
#include <random>
#include <iostream>
int main() {
    using namespace msr::airlib;
    std::mt19937 generator(11732);
    std::uniform_real_distribution<float> angle(-3.14159265f, 3.14159265f);
    float max_error = 0;
    for (int i=0; i<20000; ++i) {
        const float pitch=angle(generator)*.5f, yaw=angle(generator);
        const auto sensor=VectorMath::toQuaternion(angle(generator),angle(generator),angle(generator));
        const auto vehicle=VectorMath::toQuaternion(angle(generator),angle(generator),angle(generator));
        const auto ray=VectorMath::toQuaternion(pitch,0,yaw);
        const auto original=VectorMath::coordOrientationAdd(VectorMath::coordOrientationAdd(ray,sensor),vehicle);
        const auto combined=VectorMath::coordOrientationAdd(sensor,vehicle);
        const Vector3r local(std::cos(pitch)*std::cos(yaw),std::cos(pitch)*std::sin(yaw),-std::sin(pitch));
        const Vector3r old_end=VectorMath::rotateVector(VectorMath::front(),original,true)*100;
        const Vector3r new_end=VectorMath::rotateVector(local,combined,true)*100;
        max_error=std::max(max_error,(old_end-new_end).norm());
    }
    std::cout << "max ray endpoint error at 100 m: " << max_error << " m\n";
    return max_error<.0002f ? 0 : 1;
}
