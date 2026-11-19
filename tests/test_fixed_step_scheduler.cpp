// Standalone timing regression for AirSim's fixed-step scheduler.
#include <atomic>
#include <chrono>
#include <thread>
#include <iostream>
namespace common_utils {
struct Utils {
    static void cleanupThread(std::thread& thread) { if (thread.joinable()) thread.join(); }
};
}
#include "../ue_support/Source/AirLib/include/common/common_utils/ScheduledExecutor.hpp"

int run(bool catchup) {
    std::atomic<int> steps{0};
    common_utils::ScheduledExecutor executor;
    executor.initialize([&](uint64_t) {
        // A 20 ms sensor update every 50 ms must not permanently slow time.
        const int count = ++steps;
        if (count % 17 == 0) std::this_thread::sleep_for(std::chrono::milliseconds(20));
        return true;
    }, 3000000, catchup);
    executor.start();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    executor.stop();
    const int measured = steps;
    std::cout << (catchup ? "fixed" : "legacy") << " steps=" << measured << std::endl;
    return measured;
}
int main() {
    const int legacy = run(false), fixed = run(true);
    if (legacy >= 600 || fixed < 630 || fixed > 710) return 1;
    std::atomic<int> steps{0};
    common_utils::ScheduledExecutor executor;
    executor.initialize([&](uint64_t) { ++steps; return true; }, 3000000, true);
    executor.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    executor.pause(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    const int before = steps;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    if (steps != before) return 2;
    executor.pause(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    executor.stop();
    const int resumed = steps - before;
    std::cout << "resumed steps=" << resumed << std::endl;
    if (resumed < 28 || resumed > 42) return 3;
    return 0;
}
