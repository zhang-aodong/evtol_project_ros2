#pragma once

#include "GameFramework/Actor.h"
#include "common/AirSimSettings.hpp"
#include <cstdint>
#include <string>
#include <vector>

// Independent evaluation channel. Does not alter semantic stencil IDs or XYZ.
class LidarIdentitySharedMemoryWriter
{
public:
    LidarIdentitySharedMemoryWriter(const std::string& sensor_name, uint32_t max_points = 65536, uint32_t slots = 8);
    ~LidarIdentitySharedMemoryWriter();
    uint32_t objectId(AActor* hit_actor) const;
    bool publish(uint64_t frame_id, uint64_t timestamp_ns, const std::vector<uint32_t>& ids);
private:
    void* mapping_ = nullptr;
    uint8_t* view_ = nullptr;
    uint32_t max_points_;
    uint32_t slots_;
    TMap<FName, uint32_t> vehicle_ids_;
};
