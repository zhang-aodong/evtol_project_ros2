#pragma once

#include "CoreMinimal.h"

class SharedMemoryCameraWriter
{
public:
    static constexpr uint32 Magic = 0x4D485341; // "ASHM" in little endian
    static constexpr uint16 Version = 1;
    static constexpr uint32 RingHeaderSize = 64;
    static constexpr uint32 SlotHeaderSize = 64;
    static constexpr uint32 PixelFormatBgr8 = 1;

    SharedMemoryCameraWriter(
        const FString& mapping_name,
        uint32 width,
        uint32 height,
        uint32 slot_count);
    ~SharedMemoryCameraWriter();

    bool isValid() const;
    bool publish(
        const uint8* bgr_data,
        uint32 data_size,
        uint64 timestamp_ns,
        uint32 image_type,
        uint64& frame_id);

    const FString& mappingName() const;

private:
    void close();

private:
    FString mapping_name_;
    uint32 width_ = 0;
    uint32 height_ = 0;
    uint32 stride_ = 0;
    uint32 slot_count_ = 0;
    uint32 slot_size_ = 0;
    uint64 mapping_size_ = 0;
    uint64 next_frame_id_ = 0;
    void* mapping_handle_ = nullptr;
    uint8* mapped_memory_ = nullptr;
};
