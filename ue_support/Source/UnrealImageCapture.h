#pragma once

#include "CoreMinimal.h"
#include "PIPCamera.h"
#include "common/ImageCaptureBase.hpp"
#include "common/common_utils/UniqueValueMap.hpp"
#include "SharedMemoryCameraWriter.h"
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

class UnrealImageCapture : public msr::airlib::ImageCaptureBase
{
public:
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

    UnrealImageCapture(const common_utils::UniqueValueMap<std::string, APIPCamera*>* cameras);
    virtual ~UnrealImageCapture();

    virtual void getImages(const std::vector<ImageRequest>& requests, std::vector<ImageResponse>& responses) const override;

private:
    void getSceneCaptureImage(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests,
                              std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses, bool use_safe_method) const;

    void addScreenCaptureHandler(UWorld* world);
    bool getScreenshotScreen(ImageType image_type, std::vector<uint8_t>& compressedPng);

    bool updateCameraVisibility(APIPCamera* camera, const msr::airlib::ImageCaptureBase::ImageRequest& request);

    void startActiveSharedMemoryStream(
        const std::vector<ImageRequest>& requests,
        double rate_hz) const;
    void stopActiveSharedMemoryStream() const;

private:
    const common_utils::UniqueValueMap<std::string, APIPCamera*>* cameras_;
    std::vector<uint8_t> last_compressed_png_;
    mutable FCriticalSection shared_memory_mutex_;
    mutable std::map<std::string, std::unique_ptr<SharedMemoryCameraWriter>> shared_memory_writers_;
    mutable std::mutex active_stream_mutex_;
    mutable std::condition_variable active_stream_condition_;
    mutable std::thread active_stream_thread_;
    mutable bool active_stream_stop_ = false;
};
