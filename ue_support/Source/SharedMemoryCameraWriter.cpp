#include "SharedMemoryCameraWriter.h"

#include "HAL/PlatformAtomics.h"
#include "HAL/PlatformMisc.h"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include <Windows.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace
{
#pragma pack(push, 1)
struct SharedCameraRingHeader
{
    uint32 magic;
    uint16 version;
    uint16 header_size;
    uint32 slot_count;
    uint32 slot_header_size;
    uint32 width;
    uint32 height;
    uint32 channels;
    uint32 stride;
    uint32 pixel_format;
    uint32 slot_size;
    uint64 mapping_size;
    volatile int64 latest_frame_id;
    uint64 reserved;
};

struct SharedCameraSlotHeader
{
    volatile int64 guard;
    uint64 frame_id;
    uint64 timestamp_ns;
    uint32 width;
    uint32 height;
    uint32 stride;
    uint32 data_size;
    uint32 image_type;
    uint8 reserved[20];
};
#pragma pack(pop)

static_assert(sizeof(SharedCameraRingHeader) == 64, "Unexpected ring header size");
static_assert(sizeof(SharedCameraSlotHeader) == 64, "Unexpected slot header size");

uint32 alignToCacheLine(uint64 value)
{
    return static_cast<uint32>((value + 63ULL) & ~63ULL);
}
}

SharedMemoryCameraWriter::SharedMemoryCameraWriter(
    const FString& mapping_name,
    uint32 width,
    uint32 height,
    uint32 slot_count)
    : mapping_name_(mapping_name),
      width_(width),
      height_(height),
      stride_(width * 3),
      slot_count_(slot_count)
{
#if PLATFORM_WINDOWS
    if (width_ == 0 || height_ == 0 || slot_count_ == 0) {
        UE_LOG(LogTemp, Error, TEXT("AirSim shared memory camera has invalid dimensions or slot count"));
        return;
    }

    const uint64 frame_bytes = static_cast<uint64>(stride_) * height_;
    slot_size_ = alignToCacheLine(SlotHeaderSize + frame_bytes);
    mapping_size_ = RingHeaderSize + static_cast<uint64>(slot_count_) * slot_size_;

    const DWORD size_high = static_cast<DWORD>(mapping_size_ >> 32);
    const DWORD size_low = static_cast<DWORD>(mapping_size_ & 0xFFFFFFFFULL);
    HANDLE handle = CreateFileMappingW(
        INVALID_HANDLE_VALUE,
        nullptr,
        PAGE_READWRITE,
        size_high,
        size_low,
        *mapping_name_);
    if (handle == nullptr) {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("CreateFileMapping failed for %s: error=%lu"),
            *mapping_name_,
            GetLastError());
        return;
    }
    mapping_handle_ = handle;

    mapped_memory_ = static_cast<uint8*>(MapViewOfFile(
        handle,
        FILE_MAP_ALL_ACCESS,
        0,
        0,
        static_cast<SIZE_T>(mapping_size_)));
    if (mapped_memory_ == nullptr) {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("MapViewOfFile failed for %s: error=%lu"),
            *mapping_name_,
            GetLastError());
        close();
        return;
    }

    FMemory::Memzero(mapped_memory_, static_cast<SIZE_T>(mapping_size_));
    auto* header = reinterpret_cast<SharedCameraRingHeader*>(mapped_memory_);
    header->magic = Magic;
    header->version = Version;
    header->header_size = RingHeaderSize;
    header->slot_count = slot_count_;
    header->slot_header_size = SlotHeaderSize;
    header->width = width_;
    header->height = height_;
    header->channels = 3;
    header->stride = stride_;
    header->pixel_format = PixelFormatBgr8;
    header->slot_size = slot_size_;
    header->mapping_size = mapping_size_;
    FPlatformAtomics::InterlockedExchange(&header->latest_frame_id, 0);

    UE_LOG(
        LogTemp,
        Display,
        TEXT("AirSim shared memory camera created: %s, %ux%u, slots=%u, bytes=%llu"),
        *mapping_name_,
        width_,
        height_,
        slot_count_,
        mapping_size_);
#else
    UE_LOG(LogTemp, Error, TEXT("AirSim shared memory camera currently supports Windows only"));
#endif
}

SharedMemoryCameraWriter::~SharedMemoryCameraWriter()
{
    close();
}

bool SharedMemoryCameraWriter::isValid() const
{
    return mapped_memory_ != nullptr;
}

bool SharedMemoryCameraWriter::publish(
    const uint8* bgr_data,
    uint32 data_size,
    uint64 timestamp_ns,
    uint32 image_type,
    uint64& frame_id)
{
#if PLATFORM_WINDOWS
    const uint32 expected_size = stride_ * height_;
    if (!isValid() || bgr_data == nullptr || data_size != expected_size) {
        return false;
    }

    frame_id = ++next_frame_id_;
    const uint32 slot_index = static_cast<uint32>(frame_id % slot_count_);
    uint8* slot_memory = mapped_memory_ + RingHeaderSize +
                         static_cast<uint64>(slot_index) * slot_size_;
    auto* slot = reinterpret_cast<SharedCameraSlotHeader*>(slot_memory);

    FPlatformAtomics::InterlockedExchange(
        &slot->guard,
        static_cast<int64>(frame_id * 2 + 1));
    FPlatformMisc::MemoryBarrier();

    slot->frame_id = frame_id;
    slot->timestamp_ns = timestamp_ns;
    slot->width = width_;
    slot->height = height_;
    slot->stride = stride_;
    slot->data_size = data_size;
    slot->image_type = image_type;
    FMemory::Memcpy(slot_memory + SlotHeaderSize, bgr_data, data_size);

    FPlatformMisc::MemoryBarrier();
    FPlatformAtomics::InterlockedExchange(
        &slot->guard,
        static_cast<int64>(frame_id * 2 + 2));
    auto* header = reinterpret_cast<SharedCameraRingHeader*>(mapped_memory_);
    FPlatformAtomics::InterlockedExchange(
        &header->latest_frame_id,
        static_cast<int64>(frame_id));
    return true;
#else
    return false;
#endif
}

const FString& SharedMemoryCameraWriter::mappingName() const
{
    return mapping_name_;
}

void SharedMemoryCameraWriter::close()
{
#if PLATFORM_WINDOWS
    if (mapped_memory_ != nullptr) {
        UnmapViewOfFile(mapped_memory_);
        mapped_memory_ = nullptr;
    }
    if (mapping_handle_ != nullptr) {
        CloseHandle(static_cast<HANDLE>(mapping_handle_));
        mapping_handle_ = nullptr;
    }
#endif
}
