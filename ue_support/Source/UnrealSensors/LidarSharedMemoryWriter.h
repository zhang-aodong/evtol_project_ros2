#pragma once

#include <cstdint>
#include <string>
#include "common/Common.hpp"

class LidarSharedMemoryWriter
{
public:
    LidarSharedMemoryWriter(const std::string& lidar_name, uint32_t max_points = 65536, uint32_t slots = 8);
    ~LidarSharedMemoryWriter();
    bool publish(uint64_t frame_id, int64_t timestamp_ns, const msr::airlib::vector<msr::airlib::real_T>& point_cloud, bool complete_scan = false);
    bool isValid() const { return mapping_ != nullptr; }
private:
    void* mapping_ = nullptr;
    uint8_t* view_ = nullptr;
    std::string mapping_name_;
    uint32_t max_points_;
    uint32_t slots_;
};
