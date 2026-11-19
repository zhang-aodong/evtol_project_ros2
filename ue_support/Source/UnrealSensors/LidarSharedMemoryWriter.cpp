#include "LidarSharedMemoryWriter.h"
#include "HAL/PlatformAtomics.h"
#include "HAL/PlatformMisc.h"

#if PLATFORM_WINDOWS
#include <windows.h>
#include <algorithm>
#include <cstring>

#pragma pack(push, 1)
struct LidarRingHeader { uint32_t magic; uint16_t version; uint16_t header_size; uint32_t slots; uint32_t max_points; uint64_t reserved; uint64_t latest_frame_id; };
struct LidarSlotHeader { int64_t guard; uint64_t frame_id; uint64_t timestamp_ns; uint32_t point_count; uint32_t reserved; uint8_t padding[8]; };
#pragma pack(pop)

static constexpr uint32_t kMagic = 0x4D534C44;
static constexpr uint16_t kVersion = 1;
static uint32_t slotSize(uint32_t max_points) { return (sizeof(LidarSlotHeader) + max_points * 12u + 63u) & ~63u; }

LidarSharedMemoryWriter::LidarSharedMemoryWriter(const std::string& lidar_name, uint32_t max_points, uint32_t slots)
    : mapping_name_("AirSimLidar_" + lidar_name), max_points_(max_points), slots_(slots)
{
    const uint64_t size = sizeof(LidarRingHeader) + uint64_t(slots_) * slotSize(max_points_);
    mapping_ = CreateFileMappingA(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, uint32_t(size >> 32), uint32_t(size), mapping_name_.c_str());
    if (!mapping_) return;
    view_ = static_cast<uint8_t*>(MapViewOfFile(static_cast<HANDLE>(mapping_), FILE_MAP_ALL_ACCESS, 0, 0, size));
    if (!view_) { CloseHandle(static_cast<HANDLE>(mapping_)); mapping_ = nullptr; return; }
    auto* header = reinterpret_cast<LidarRingHeader*>(view_);
    if (GetLastError() != ERROR_ALREADY_EXISTS || header->magic != kMagic) {
        std::memset(view_, 0, size);
        header->magic = kMagic; header->version = kVersion; header->header_size = sizeof(LidarRingHeader);
        header->slots = slots_; header->max_points = max_points_;
    }
}

LidarSharedMemoryWriter::~LidarSharedMemoryWriter()
{
    if (view_) UnmapViewOfFile(view_);
    if (mapping_) CloseHandle(static_cast<HANDLE>(mapping_));
}

bool LidarSharedMemoryWriter::publish(uint64_t frame_id, int64_t timestamp_ns, const msr::airlib::vector<msr::airlib::real_T>& point_cloud, bool complete_scan)
{
    if (!view_ || point_cloud.size() % 3 != 0) return false;
    const uint32_t count = std::min<uint32_t>(uint32_t(point_cloud.size() / 3), max_points_);
    auto* header = reinterpret_cast<LidarRingHeader*>(view_);
    auto* slot = view_ + sizeof(LidarRingHeader) + (frame_id % slots_) * slotSize(max_points_);
    auto* slot_header = reinterpret_cast<LidarSlotHeader*>(slot);
    auto* guard = reinterpret_cast<volatile int64*>(&slot_header->guard);
    const int64 seq = std::llabs(FPlatformAtomics::InterlockedCompareExchange(guard, int64(0), int64(0))) | 1;
    // The guard must be observable before touching the payload. Plain stores
    // are not a cross-process seqlock and may be removed/reordered by /O2.
    FPlatformAtomics::InterlockedExchange(guard, seq);
    slot_header->frame_id = frame_id; slot_header->timestamp_ns = uint64_t(timestamp_ns);
    // v1 reserved bit 0 now certifies a full configured-FOV scan. Never mark a
    // truncated payload complete. Old readers remain binary compatible.
    slot_header->point_count = count;
    slot_header->reserved = complete_scan && point_cloud.size() / 3 <= max_points_ ? 1u : 0u;
    std::memcpy(slot + sizeof(LidarSlotHeader), point_cloud.data(), size_t(count) * 12u);
    FPlatformMisc::MemoryBarrier();
    FPlatformAtomics::InterlockedExchange(guard, seq + 1);
    FPlatformAtomics::InterlockedExchange(reinterpret_cast<volatile int64*>(&header->latest_frame_id), static_cast<int64>(frame_id));
    return true;
}
#else
LidarSharedMemoryWriter::LidarSharedMemoryWriter(const std::string&, uint32_t, uint32_t) {}
LidarSharedMemoryWriter::~LidarSharedMemoryWriter() = default;
bool LidarSharedMemoryWriter::publish(uint64_t, int64_t, const msr::airlib::vector<msr::airlib::real_T>&, bool) { return false; }
#endif
