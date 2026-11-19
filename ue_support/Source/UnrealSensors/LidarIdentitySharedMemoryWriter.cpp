#include "LidarIdentitySharedMemoryWriter.h"
#include <cmath>
#include <cstring>
#include "HAL/PlatformAtomics.h"
#include "HAL/PlatformMisc.h"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include <windows.h>
#include "Windows/HideWindowsPlatformTypes.h"

namespace {
#pragma pack(push, 1)
struct IdentityRingHeader {
    uint32_t magic;
    uint16_t version, header_size;
    uint32_t slots, max_points, object_count, entry_size;
    uint64_t latest_frame_id;
};
struct IdentityEntry {
    uint32_t id;
    char vehicle_name[64];
    double x, y, z;
    uint8_t padding[4];
};
struct IdentitySlot {
    int64_t guard;
    uint64_t frame_id, timestamp_ns;
    uint32_t point_count, reserved;
    uint8_t padding[8];
};
#pragma pack(pop)
static_assert(sizeof(IdentityRingHeader) == 32, "identity ring layout");
static_assert(sizeof(IdentityEntry) == 96, "identity registry layout");
static_assert(sizeof(IdentitySlot) == 40, "identity slot layout");
constexpr uint32_t kMaxObjects = 128;
constexpr uint32_t kHeaderSize = sizeof(IdentityRingHeader) + kMaxObjects * sizeof(IdentityEntry);
uint32_t identitySlotSize(uint32_t max_points) { return (sizeof(IdentitySlot) + max_points * 4u + 63u) & ~63u; }
}

LidarIdentitySharedMemoryWriter::LidarIdentitySharedMemoryWriter(const std::string& sensor_name,
                                                               uint32_t max_points, uint32_t slots)
    : max_points_(max_points), slots_(slots)
{
    const auto& vehicles = msr::airlib::AirSimSettings::singleton().vehicles;
    if (vehicles.size() > kMaxObjects) return;
    for (const auto& vehicle : vehicles)
        if (vehicle.first.empty() || vehicle.first.size() >= 64) return;
    const uint64_t size = kHeaderSize + uint64_t(slots_) * identitySlotSize(max_points_);
    const std::string name = "AirSimLidarIdentity_" + sensor_name;
    mapping_ = CreateFileMappingA(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE,
                                 uint32_t(size >> 32), uint32_t(size), name.c_str());
    if (!mapping_) return;
    view_ = static_cast<uint8_t*>(MapViewOfFile(static_cast<HANDLE>(mapping_), FILE_MAP_ALL_ACCESS, 0, 0, size));
    if (!view_) { CloseHandle(static_cast<HANDLE>(mapping_)); mapping_ = nullptr; return; }
    std::memset(view_, 0, size);
    auto* header = reinterpret_cast<IdentityRingHeader*>(view_);
    header->version = 1; header->header_size = kHeaderSize;
    header->slots = slots_; header->max_points = max_points_; header->entry_size = sizeof(IdentityEntry);
    auto* entries = reinterpret_cast<IdentityEntry*>(view_ + sizeof(IdentityRingHeader));
    for (const auto& vehicle : vehicles) {
        const uint32_t index = header->object_count++;
        auto& entry = entries[index];
        entry.id = index + 1;
        std::memcpy(entry.vehicle_name, vehicle.first.data(), vehicle.first.size());
        const auto& position = vehicle.second->position;
        entry.x = std::isfinite(position.x()) ? position.x() : 0.;
        entry.y = std::isfinite(position.y()) ? position.y() : 0.;
        entry.z = std::isfinite(position.z()) ? position.z() : 0.;
        vehicle_ids_.Add(FName(UTF8_TO_TCHAR(vehicle.first.c_str())), entry.id);
    }
    FPlatformMisc::MemoryBarrier();
    header->magic = 0x44494C41;
}

LidarIdentitySharedMemoryWriter::~LidarIdentitySharedMemoryWriter()
{
    if (view_) UnmapViewOfFile(view_);
    if (mapping_) CloseHandle(static_cast<HANDLE>(mapping_));
}

uint32_t LidarIdentitySharedMemoryWriter::objectId(AActor* hit_actor) const
{
    // Pawn names are the AirSim vehicle API names. Child Actors may own a mesh.
    for (int depth = 0; hit_actor && depth < 8; ++depth) {
        if (const uint32_t* id = vehicle_ids_.Find(hit_actor->GetFName())) return *id;
        AActor* owner = hit_actor->GetOwner();
        hit_actor = owner ? owner : hit_actor->GetAttachParentActor();
    }
    return 0; // A real hit on a non-vehicle object, not "identity unavailable".
}

bool LidarIdentitySharedMemoryWriter::publish(uint64_t frame_id, uint64_t timestamp_ns,
                                            const std::vector<uint32_t>& ids)
{
    if (!view_) return false;
    const uint32_t count = ids.size() < max_points_ ? uint32_t(ids.size()) : max_points_;
    auto* slot = reinterpret_cast<IdentitySlot*>(view_ + kHeaderSize + (frame_id % slots_) * identitySlotSize(max_points_));
    const int64_t sequence = (slot->guard & ~int64_t(1)) + 1;
    FPlatformAtomics::InterlockedExchange(reinterpret_cast<volatile int64*>(&slot->guard), sequence);
    slot->frame_id = frame_id; slot->timestamp_ns = timestamp_ns; slot->point_count = count;
    if (count) std::memcpy(reinterpret_cast<uint8_t*>(slot) + sizeof(IdentitySlot), ids.data(), count * sizeof(uint32_t));
    FPlatformMisc::MemoryBarrier();
    FPlatformAtomics::InterlockedExchange(reinterpret_cast<volatile int64*>(&slot->guard), sequence + 1);
    FPlatformAtomics::InterlockedExchange(reinterpret_cast<volatile int64*>(&reinterpret_cast<IdentityRingHeader*>(view_)->latest_frame_id), static_cast<int64>(frame_id));
    return true;
}
#else
LidarIdentitySharedMemoryWriter::LidarIdentitySharedMemoryWriter(const std::string&, uint32_t m, uint32_t s) : max_points_(m), slots_(s) {}
LidarIdentitySharedMemoryWriter::~LidarIdentitySharedMemoryWriter() = default;
uint32_t LidarIdentitySharedMemoryWriter::objectId(AActor*) const { return 0; }
bool LidarIdentitySharedMemoryWriter::publish(uint64_t, uint64_t, const std::vector<uint32_t>&) { return false; }
#endif
