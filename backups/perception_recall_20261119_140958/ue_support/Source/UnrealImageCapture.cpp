#include "UnrealImageCapture.h"
#include "Async/Async.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "ImageUtils.h"
#include "Math/Float16Color.h"
#include "PixelFormat.h"
#include "RHIGPUReadback.h"
#include "RHICommandList.h"

#include "RenderRequest.h"
#include "AirSimCaptureVisibility.h"
#include "common/ClockFactory.hpp"
#include "Misc/ScopeLock.h"

#include <array>
#include <atomic>
#include <chrono>
#include <deque>

namespace
{
constexpr char SharedMemoryCameraPrefix[] = "shm:";
constexpr char SharedMemoryPushStartPrefix[] = "shm_push_start:";
constexpr char SharedMemoryPushStopPrefix[] = "shm_push_stop:";
constexpr uint32 SharedMemorySlotCount = 8;
constexpr uint32 ActiveGpuReadbackSlotCount = 4;

struct ActiveReadbackSource
{
    FTextureRenderTargetResource* render_target_resource = nullptr;
    FString mapping_name;
    uint32 width = 0;
    uint32 height = 0;
    uint32 image_type = 0;
};

struct ActiveReadbackOutput
{
    FString mapping_name;
    uint32 width = 0;
    uint32 height = 0;
    uint32 image_type = 0;
    EPixelFormat pixel_format = PF_Unknown;
    TUniquePtr<FRHIGPUTextureReadback> readback;
};

struct ActiveReadbackSlot
{
    std::atomic<bool> in_use{ false };
    std::atomic<bool> copy_enqueued{ false };
    uint64 timestamp_ns = 0;
    std::vector<ActiveReadbackOutput> outputs;
};

struct ActiveCompletedOutput
{
    std::string writer_key;
    uint32 image_type = 0;
    uint32 width = 0;
    uint32 height = 0;
    EPixelFormat pixel_format = PF_Unknown;
    std::vector<uint8_t> raw_bytes;
};

struct ActiveCompletedFrame
{
    uint64 timestamp_ns = 0;
    std::vector<ActiveCompletedOutput> outputs;
};

struct ActiveReadbackState
{
    ActiveReadbackState()
    {
        for (auto& slot : slots) {
            slot = std::make_shared<ActiveReadbackSlot>();
        }
    }

    std::array<std::shared_ptr<ActiveReadbackSlot>, ActiveGpuReadbackSlotCount> slots;
    std::map<std::string, SharedMemoryCameraWriter*> writers;
    std::mutex completed_mutex;
    std::condition_variable completed_condition;
    std::deque<ActiveCompletedFrame> completed_frames;
    std::atomic<bool> stop{ false };
    std::atomic<bool> game_capture_pending{ false };
    std::atomic<uint64> scheduled_count{ 0 };
    std::atomic<uint64> scheduled_output_count{ 0 };
    std::atomic<uint64> polled_output_count{ 0 };
    std::atomic<uint64> completed_count{ 0 };
    std::atomic<uint64> publish_attempt_count{ 0 };
    std::atomic<uint64> published_output_count{ 0 };
    std::atomic<uint64> empty_output_count{ 0 };
    std::atomic<uint64> completed_queue_drop_count{ 0 };
    std::atomic<uint64> unavailable_slot_count{ 0 };
};

void enqueueActiveReadbackPoll(const std::shared_ptr<ActiveReadbackState>& state)
{
    ENQUEUE_RENDER_COMMAND(AirSimPollActiveCameraReadbacks)
    ([state](FRHICommandListImmediate& RHICmdList) {
        for (const auto& slot : state->slots) {
            if (!slot->in_use.load(std::memory_order_acquire) ||
                !slot->copy_enqueued.load(std::memory_order_acquire)) {
                continue;
            }

            bool all_ready = true;
            for (const ActiveReadbackOutput& output : slot->outputs) {
                if (!output.readback || !output.readback->IsReady()) {
                    all_ready = false;
                    break;
                }
            }
            if (!all_ready) {
                continue;
            }

            ActiveCompletedFrame completed_frame;
            completed_frame.timestamp_ns = slot->timestamp_ns;
            bool read_ok = true;
            state->polled_output_count.fetch_add(
                static_cast<uint64>(slot->outputs.size()), std::memory_order_relaxed);
            for (ActiveReadbackOutput& output : slot->outputs) {
                void* mapped_data = nullptr;
                int32 row_pitch_pixels = 0;
                output.readback->LockTexture(RHICmdList, mapped_data, row_pitch_pixels);
                if (mapped_data == nullptr || row_pitch_pixels < static_cast<int32>(output.width)) {
                    read_ok = false;
                    if (mapped_data != nullptr) {
                        output.readback->Unlock();
                    }
                    break;
                }

                ActiveCompletedOutput completed_output;
                completed_output.writer_key = TCHAR_TO_UTF8(*output.mapping_name);
                completed_output.image_type = output.image_type;
                completed_output.width = output.width;
                completed_output.height = output.height;
                completed_output.pixel_format = output.pixel_format;

                const uint8* source_base = static_cast<const uint8*>(mapped_data);
                const size_t pixel_bytes = output.pixel_format == PF_FloatRGBA
                                               ? sizeof(FFloat16Color)
                                               : output.pixel_format == PF_B8G8R8A8 ? sizeof(FColor) : 0;
                if (pixel_bytes == 0) {
                    read_ok = false;
                    UE_LOG(
                        LogTemp,
                        Error,
                        TEXT("AirSim asynchronous camera unsupported pixel format %d"),
                        static_cast<int32>(output.pixel_format));
                }
                else {
                    const size_t compact_row_bytes = static_cast<size_t>(output.width) * pixel_bytes;
                    completed_output.raw_bytes.resize(compact_row_bytes * output.height);
                    for (uint32 y = 0; y < output.height; ++y) {
                        FMemory::Memcpy(
                            completed_output.raw_bytes.data() + static_cast<size_t>(y) * compact_row_bytes,
                            source_base + static_cast<size_t>(y) * row_pitch_pixels * pixel_bytes,
                            compact_row_bytes);
                    }
                }
                output.readback->Unlock();
                if (!read_ok) {
                    break;
                }
                completed_frame.outputs.push_back(std::move(completed_output));
            }

            if (read_ok && completed_frame.outputs.size() == slot->outputs.size()) {
                if (completed_frame.outputs.empty()) {
                    state->empty_output_count.fetch_add(1, std::memory_order_relaxed);
                }
                {
                    std::lock_guard<std::mutex> lock(state->completed_mutex);
                    if (state->completed_frames.size() >= ActiveGpuReadbackSlotCount) {
                        state->completed_frames.pop_front();
                        state->completed_queue_drop_count.fetch_add(1, std::memory_order_relaxed);
                    }
                    state->completed_frames.push_back(std::move(completed_frame));
                }
                state->completed_condition.notify_one();
                state->completed_count.fetch_add(1, std::memory_order_relaxed);
            }
            else {
                UE_LOG(LogTemp, Error, TEXT("AirSim asynchronous camera GPU readback failed"));
            }

            slot->copy_enqueued.store(false, std::memory_order_release);
            slot->in_use.store(false, std::memory_order_release);
        }
    });
}

void runActiveSharedMemoryPublisher(const std::shared_ptr<ActiveReadbackState>& state)
{
    while (true) {
        ActiveCompletedFrame completed_frame;
        {
            std::unique_lock<std::mutex> lock(state->completed_mutex);
            state->completed_condition.wait(lock, [state]() {
                return state->stop.load(std::memory_order_acquire) || !state->completed_frames.empty();
            });
            if (state->completed_frames.empty()) {
                if (state->stop.load(std::memory_order_acquire)) {
                    break;
                }
                continue;
            }
            completed_frame = std::move(state->completed_frames.front());
            state->completed_frames.pop_front();
        }

        for (const ActiveCompletedOutput& output : completed_frame.outputs) {
            std::vector<uint8_t> bgr_bytes(
                static_cast<size_t>(output.width) * output.height * 3);
            uint8_t* destination = bgr_bytes.data();

            if (output.pixel_format == PF_FloatRGBA) {
                const FFloat16Color* source =
                    reinterpret_cast<const FFloat16Color*>(output.raw_bytes.data());
                const size_t pixel_count = static_cast<size_t>(output.width) * output.height;
                for (size_t index = 0; index < pixel_count; ++index) {
                    const FColor color = FLinearColor(source[index]).ToFColor(false);
                    *destination++ = color.B;
                    *destination++ = color.G;
                    *destination++ = color.R;
                }
            }
            else if (output.pixel_format == PF_B8G8R8A8) {
                const FColor* source = reinterpret_cast<const FColor*>(output.raw_bytes.data());
                const size_t pixel_count = static_cast<size_t>(output.width) * output.height;
                for (size_t index = 0; index < pixel_count; ++index) {
                    *destination++ = source[index].B;
                    *destination++ = source[index].G;
                    *destination++ = source[index].R;
                }
            }
            else {
                continue;
            }

            state->publish_attempt_count.fetch_add(1, std::memory_order_relaxed);
            uint64 frame_id = 0;
            const auto writer_it = state->writers.find(output.writer_key);
            const bool published =
                writer_it != state->writers.end() && writer_it->second != nullptr &&
                writer_it->second->isValid() &&
                writer_it->second->publish(
                    bgr_bytes.data(),
                    static_cast<uint32>(bgr_bytes.size()),
                    completed_frame.timestamp_ns,
                    output.image_type,
                    frame_id);
            if (published) {
                state->published_output_count.fetch_add(1, std::memory_order_relaxed);
            }
            else {
                UE_LOG(
                    LogTemp,
                    Error,
                    TEXT("AirSim asynchronous camera failed to publish shared memory output %s"),
                    UTF8_TO_TCHAR(output.writer_key.c_str()));
            }
        }
    }
}

bool parseActiveStreamStart(
    const std::string& requested_name,
    std::string& stream_key,
    double& rate_hz)
{
    const size_t prefix_size = sizeof(SharedMemoryPushStartPrefix) - 1;
    if (requested_name.compare(0, prefix_size, SharedMemoryPushStartPrefix) != 0) {
        return false;
    }

    const std::string suffix = requested_name.substr(prefix_size);
    const size_t rate_separator = suffix.rfind(':');
    if (rate_separator == std::string::npos) {
        return false;
    }
    stream_key = suffix.substr(0, rate_separator);
    rate_hz = FCString::Atod(UTF8_TO_TCHAR(suffix.substr(rate_separator + 1).c_str()));
    return !stream_key.empty() && rate_hz >= 0.1 && rate_hz <= 120.0;
}

bool parseActiveStreamStop(const std::string& requested_name, std::string& stream_key)
{
    const size_t prefix_size = sizeof(SharedMemoryPushStopPrefix) - 1;
    if (requested_name.compare(0, prefix_size, SharedMemoryPushStopPrefix) != 0) {
        return false;
    }
    stream_key = requested_name.substr(prefix_size);
    return !stream_key.empty();
}

bool parseSharedMemoryCameraName(
    const std::string& requested_name,
    std::string& actual_camera_name,
    FString& stream_key)
{
    if (requested_name.compare(0, sizeof(SharedMemoryCameraPrefix) - 1, SharedMemoryCameraPrefix) != 0) {
        actual_camera_name = requested_name;
        return false;
    }

    const std::string suffix = requested_name.substr(sizeof(SharedMemoryCameraPrefix) - 1);
    const size_t separator = suffix.rfind(':');
    actual_camera_name = separator == std::string::npos
                             ? suffix
                             : suffix.substr(separator + 1);
    stream_key = UTF8_TO_TCHAR(suffix.c_str());
    return !actual_camera_name.empty();
}

FString sanitizeSharedMemoryName(const FString& value)
{
    FString sanitized;
    sanitized.Reserve(value.Len());
    for (const TCHAR character : value) {
        sanitized.AppendChar(FChar::IsAlnum(character) ? character : TEXT('_'));
    }
    return sanitized;
}

FString makeSharedMemoryName(
    const FString& stream_key,
    int width,
    int height,
    int image_type)
{
    return FString::Printf(
        TEXT("Local\\AirSim_%s_%dx%d_%d"),
        *sanitizeSharedMemoryName(stream_key),
        width,
        height,
        image_type);
}
}

UnrealImageCapture::UnrealImageCapture(const common_utils::UniqueValueMap<std::string, APIPCamera*>* cameras)
    : cameras_(cameras)
{
    //TODO: explore screenshot option
    //addScreenCaptureHandler(camera->GetWorld());
}

UnrealImageCapture::~UnrealImageCapture()
{
    stopActiveSharedMemoryStream();
}

void UnrealImageCapture::getImages(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests,
                                   std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses) const
{
    if (!requests.empty()) {
        std::string stop_stream_key;
        if (parseActiveStreamStop(requests.front().camera_name, stop_stream_key)) {
            stopActiveSharedMemoryStream();
            responses.push_back(ImageResponse());
            responses.back().message = std::string("shared_memory_stopped|") + stop_stream_key;
            return;
        }

        std::string stream_key;
        double rate_hz = 0.0;
        if (parseActiveStreamStart(requests.front().camera_name, stream_key, rate_hz)) {
            std::vector<ImageRequest> active_requests = requests;
            for (ImageRequest& request : active_requests) {
                std::string request_stream_key;
                double request_rate_hz = 0.0;
                if (!parseActiveStreamStart(request.camera_name, request_stream_key, request_rate_hz) ||
                    request_stream_key != stream_key || FMath::Abs(request_rate_hz - rate_hz) > 0.0001) {
                    responses.push_back(ImageResponse());
                    responses.back().message = "all active shared-memory requests must use the same stream and rate";
                    return;
                }
                request.camera_name = std::string(SharedMemoryCameraPrefix) + stream_key;
            }

            stopActiveSharedMemoryStream();
            // A newly enabled segmentation capture component returns two
            // transition frames before the stencil palette is ready in UE4.27.
            // Publish/discard those frames inside the start command and return
            // only the third, valid frame to the consumer.
            constexpr int ActiveStreamWarmupCaptures = 3;
            for (int warmup_index = 0; warmup_index < ActiveStreamWarmupCaptures; ++warmup_index) {
                std::vector<ImageResponse> warmup_responses;
                getSceneCaptureImage(active_requests, warmup_responses, false);
                if (warmup_index + 1 == ActiveStreamWarmupCaptures) {
                    responses = std::move(warmup_responses);
                }
            }
            bool start_ok = responses.size() == active_requests.size();
            for (const ImageResponse& response : responses) {
                start_ok = start_ok && response.message.compare(0, 14, "shared_memory|") == 0;
            }
            if (start_ok) {
                startActiveSharedMemoryStream(active_requests, rate_hz);
            }
            return;
        }
    }

    if (cameras_->valsSize() == 0) {
        for (unsigned int i = 0; i < requests.size(); ++i) {
            responses.push_back(ImageResponse());
            responses[responses.size() - 1].message = "camera is not set";
        }
    }
    else
        getSceneCaptureImage(requests, responses, false);
}

void UnrealImageCapture::startActiveSharedMemoryStream(
    const std::vector<ImageRequest>& requests,
    double rate_hz) const
{
    {
        std::lock_guard<std::mutex> lock(active_stream_mutex_);
        active_stream_stop_ = false;
    }

    const auto readback_state = std::make_shared<ActiveReadbackState>();
    {
        FScopeLock lock(&shared_memory_mutex_);
        for (const auto& writer : shared_memory_writers_) {
            readback_state->writers.emplace(writer.first, writer.second.get());
        }
    }

    const auto request_list = std::make_shared<const std::vector<ImageRequest>>(requests);
    active_stream_thread_ = std::thread([this, request_list, rate_hz, readback_state]() {
        using Clock = std::chrono::steady_clock;
        const auto period = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / rate_hz));
        auto next_capture = Clock::now() + period;

        auto schedule_capture = [this, request_list, readback_state]() {
            if (readback_state->stop.load(std::memory_order_acquire) ||
                readback_state->game_capture_pending.exchange(true, std::memory_order_acq_rel)) {
                return;
            }

            std::shared_ptr<ActiveReadbackSlot> selected_slot;
            for (const auto& slot : readback_state->slots) {
                bool expected = false;
                if (slot->in_use.compare_exchange_strong(
                        expected,
                        true,
                        std::memory_order_acq_rel)) {
                    selected_slot = slot;
                    slot->copy_enqueued.store(false, std::memory_order_release);
                    break;
                }
            }
            if (!selected_slot) {
                readback_state->game_capture_pending.store(false, std::memory_order_release);
                readback_state->unavailable_slot_count.fetch_add(1, std::memory_order_relaxed);
                return;
            }

            AsyncTask(ENamedThreads::GameThread, [this, request_list, readback_state, selected_slot]() {
                if (readback_state->stop.load(std::memory_order_acquire)) {
                    selected_slot->in_use.store(false, std::memory_order_release);
                    readback_state->game_capture_pending.store(false, std::memory_order_release);
                    return;
                }

                // Keep the largest scene target allocated. Without this UE4
                // reallocates between the viewport size and the 2K captures
                // on almost every frame, which stalls the render thread.
                if (IConsoleVariable* resize_method =
                        IConsoleManager::Get().FindConsoleVariable(TEXT("r.SceneRenderTargetResizeMethod"))) {
                    resize_method->Set(2, ECVF_SetByCode);
                }
                if (IConsoleVariable* force_resize_method =
                        IConsoleManager::Get().FindConsoleVariable(TEXT("r.SceneRenderTargetResizeMethodForceOverride"))) {
                    force_resize_method->Set(1, ECVF_SetByCode);
                }

                const auto sources = std::make_shared<std::vector<ActiveReadbackSource>>();
                std::vector<USceneCaptureComponent2D*> capture_components;
                for (const ImageRequest& request : *request_list) {
                    std::string actual_camera_name;
                    FString stream_key;
                    if (!parseSharedMemoryCameraName(request.camera_name, actual_camera_name, stream_key)) {
                        continue;
                    }
                    APIPCamera* camera = cameras_->at(actual_camera_name);
                    USceneCaptureComponent2D* capture = camera->getCaptureComponent(request.image_type, false);
                    if (capture == nullptr || capture->TextureTarget == nullptr) {
                        continue;
                    }
                    ActiveReadbackSource source;
                    source.render_target_resource = capture->TextureTarget->GameThread_GetRenderTargetResource();
                    source.width = static_cast<uint32>(capture->TextureTarget->SizeX);
                    source.height = static_cast<uint32>(capture->TextureTarget->SizeY);
                    source.image_type = static_cast<uint32>(request.image_type);
                    source.mapping_name = makeSharedMemoryName(
                        stream_key,
                        static_cast<int>(source.width),
                        static_cast<int>(source.height),
                        static_cast<int>(source.image_type));
                    sources->push_back(std::move(source));
                    capture_components.push_back(capture);
                }

                if (sources->size() != request_list->size()) {
                    UE_LOG(LogTemp, Error, TEXT("AirSim asynchronous camera could not resolve all capture targets"));
                    selected_slot->in_use.store(false, std::memory_order_release);
                    readback_state->game_capture_pending.store(false, std::memory_order_release);
                    return;
                }
                for (USceneCaptureComponent2D* capture : capture_components) {
                    // CaptureScene submits the scene capture immediately. The
                    // readback command queued below therefore follows both
                    // captures without depending on editor viewport redraws.
                    ApplyAirSimCaptureVisibility(capture);
                    capture->CaptureScene();
                }

                ENQUEUE_RENDER_COMMAND(AirSimEnqueueActiveCameraReadbacks)
                ([readback_state, selected_slot, sources](FRHICommandListImmediate& RHICmdList) {
                    selected_slot->outputs.clear();
                    selected_slot->outputs.reserve(sources->size());
                    selected_slot->timestamp_ns =
                        static_cast<uint64>(msr::airlib::ClockFactory::get()->nowNanos());

                    bool enqueue_ok = true;
                    for (const ActiveReadbackSource& source : *sources) {
                        if (source.render_target_resource == nullptr) {
                            enqueue_ok = false;
                            break;
                        }
                        const FTexture2DRHIRef texture =
                            source.render_target_resource->GetRenderTargetTexture();
                        if (!texture.IsValid()) {
                            enqueue_ok = false;
                            break;
                        }

                        ActiveReadbackOutput output;
                        output.mapping_name = source.mapping_name;
                        output.width = source.width;
                        output.height = source.height;
                        output.image_type = source.image_type;
                        output.pixel_format = texture->GetFormat();
                        output.readback = MakeUnique<FRHIGPUTextureReadback>(
                            FName(*FString::Printf(
                                TEXT("AirSimCameraReadback_%u"),
                                source.image_type)));
                        output.readback->EnqueueCopy(
                            RHICmdList,
                            texture.GetReference(),
                            FResolveRect(0, 0, source.width, source.height));
                        selected_slot->outputs.push_back(std::move(output));
                    }

                    if (enqueue_ok && selected_slot->outputs.size() == sources->size()) {
                        readback_state->scheduled_output_count.fetch_add(
                            static_cast<uint64>(selected_slot->outputs.size()),
                            std::memory_order_relaxed);
                        selected_slot->copy_enqueued.store(true, std::memory_order_release);
                        readback_state->scheduled_count.fetch_add(1, std::memory_order_relaxed);
                    }
                    else {
                        UE_LOG(LogTemp, Error, TEXT("AirSim asynchronous camera failed to enqueue GPU readback"));
                        selected_slot->in_use.store(false, std::memory_order_release);
                    }
                });
                readback_state->game_capture_pending.store(false, std::memory_order_release);
            });
        };

        std::thread publisher_thread([readback_state]() {
            runActiveSharedMemoryPublisher(readback_state);
        });

        UE_LOG(
            LogTemp,
            Display,
            TEXT("AirSim asynchronous active shared-memory camera started: %.3f Hz, outputs=%d, readback_slots=%u"),
            rate_hz,
            static_cast<int>(request_list->size()),
            ActiveGpuReadbackSlotCount);

        while (true) {
            {
                std::unique_lock<std::mutex> lock(active_stream_mutex_);
                if (active_stream_condition_.wait_until(
                        lock,
                        next_capture,
                        [this]() { return active_stream_stop_; })) {
                    break;
                }
            }

            enqueueActiveReadbackPoll(readback_state);
            schedule_capture();

            next_capture += period;
            const auto now = Clock::now();
            if (now > next_capture) {
                next_capture = now;
            }
        }

        readback_state->stop.store(true, std::memory_order_release);
        readback_state->completed_condition.notify_all();
        publisher_thread.join();
        UE_LOG(
            LogTemp,
            Display,
            TEXT("AirSim asynchronous active shared-memory camera stopped: scheduled=%llu, scheduled_outputs=%llu, completed=%llu, polled_outputs=%llu, publish_attempts=%llu, published_outputs=%llu, empty_outputs=%llu, queue_drops=%llu, writers=%d, no_slot=%llu"),
            static_cast<unsigned long long>(readback_state->scheduled_count.load()),
            static_cast<unsigned long long>(readback_state->scheduled_output_count.load()),
            static_cast<unsigned long long>(readback_state->completed_count.load()),
            static_cast<unsigned long long>(readback_state->polled_output_count.load()),
            static_cast<unsigned long long>(readback_state->publish_attempt_count.load()),
            static_cast<unsigned long long>(readback_state->published_output_count.load()),
            static_cast<unsigned long long>(readback_state->empty_output_count.load()),
            static_cast<unsigned long long>(readback_state->completed_queue_drop_count.load()),
            static_cast<int>(readback_state->writers.size()),
            static_cast<unsigned long long>(readback_state->unavailable_slot_count.load()));
    });
}

void UnrealImageCapture::stopActiveSharedMemoryStream() const
{
    std::thread thread_to_join;
    {
        std::lock_guard<std::mutex> lock(active_stream_mutex_);
        active_stream_stop_ = true;
        active_stream_condition_.notify_all();
        if (active_stream_thread_.joinable()) {
            thread_to_join = std::move(active_stream_thread_);
        }
    }
    if (thread_to_join.joinable()) {
        thread_to_join.join();
    }
}

void UnrealImageCapture::getSceneCaptureImage(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests,
                                               std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses, bool use_safe_method) const
{
    std::vector<ImageRequest> actual_requests = requests;
    std::vector<bool> shared_memory_requests(requests.size(), false);
    std::vector<FString> shared_memory_stream_keys(requests.size());
    for (size_t i = 0; i < requests.size(); ++i) {
        std::string actual_camera_name;
        shared_memory_requests[i] = parseSharedMemoryCameraName(
            requests[i].camera_name,
            actual_camera_name,
            shared_memory_stream_keys[i]);
        actual_requests[i].camera_name = actual_camera_name;
    }

    std::vector<std::shared_ptr<RenderRequest::RenderParams>> render_params;
    std::vector<std::shared_ptr<RenderRequest::RenderResult>> render_results;

    bool visibilityChanged = false;
    for (unsigned int i = 0; i < requests.size(); ++i) {
        APIPCamera* camera = cameras_->at(actual_requests.at(i).camera_name);
        //TODO: may be we should have these methods non-const?
        visibilityChanged = const_cast<UnrealImageCapture*>(this)->updateCameraVisibility(camera, actual_requests[i]) || visibilityChanged;
    }

    if (use_safe_method && visibilityChanged) {
        // We don't do game/render thread synchronization for safe method.
        // We just blindly sleep for 200ms (the old way)
        std::this_thread::sleep_for(std::chrono::duration<double>(0.2));
    }

    UGameViewportClient* gameViewport = nullptr;
    for (unsigned int i = 0; i < requests.size(); ++i) {
        APIPCamera* camera = cameras_->at(actual_requests.at(i).camera_name);
        if (gameViewport == nullptr) {
            gameViewport = camera->GetWorld()->GetGameViewport();
        }

        responses.push_back(ImageResponse());
        ImageResponse& response = responses.at(i);

        UTextureRenderTarget2D* textureTarget = nullptr;
        USceneCaptureComponent2D* capture = camera->getCaptureComponent(actual_requests[i].image_type, false);
        if (capture == nullptr) {
            response.message = "Can't take screenshot because none camera type is not active";
        }
        else if (capture->TextureTarget == nullptr) {
            response.message = "Can't take screenshot because texture target is null";
        }
        else
            textureTarget = capture->TextureTarget;

        render_params.push_back(std::make_shared<RenderRequest::RenderParams>(capture, textureTarget, actual_requests[i].pixels_as_float, actual_requests[i].compress));
    }

    if (nullptr == gameViewport) {
        return;
    }

    auto query_camera_pose_cb = [this, &actual_requests, &responses]() {
        size_t count = actual_requests.size();
        for (size_t i = 0; i < count; i++) {
            const ImageRequest& request = actual_requests.at(i);
            APIPCamera* camera = cameras_->at(request.camera_name);
            ImageResponse& response = responses.at(i);
            auto camera_pose = camera->getPose();
            response.camera_position = camera_pose.position;
            response.camera_orientation = camera_pose.orientation;
        }
    };
    RenderRequest render_request{ gameViewport, std::move(query_camera_pose_cb) };

    render_request.getScreenshot(render_params.data(), render_results, render_params.size(), use_safe_method);

    for (unsigned int i = 0; i < requests.size(); ++i) {
        const ImageRequest& request = actual_requests.at(i);
        ImageResponse& response = responses.at(i);

        response.camera_name = request.camera_name;
        response.time_stamp = render_results[i]->time_stamp;
        bool published_to_shared_memory = false;
        if (shared_memory_requests[i] && !request.compress && !request.pixels_as_float) {
            const FString mapping_name = makeSharedMemoryName(
                shared_memory_stream_keys[i],
                render_results[i]->width,
                render_results[i]->height,
                static_cast<int>(request.image_type));
            const std::string writer_key(TCHAR_TO_UTF8(*mapping_name));
            uint64 frame_id = 0;
            {
                FScopeLock lock(&shared_memory_mutex_);
                auto writer_it = shared_memory_writers_.find(writer_key);
                if (writer_it == shared_memory_writers_.end()) {
                    writer_it = shared_memory_writers_.emplace(
                        writer_key,
                        std::make_unique<SharedMemoryCameraWriter>(
                            mapping_name,
                            static_cast<uint32>(render_results[i]->width),
                            static_cast<uint32>(render_results[i]->height),
                            SharedMemorySlotCount)).first;
                }
                if (writer_it->second->isValid()) {
                    published_to_shared_memory = writer_it->second->publish(
                        render_results[i]->image_data_uint8.GetData(),
                        static_cast<uint32>(render_results[i]->image_data_uint8.Num()),
                        static_cast<uint64>(render_results[i]->time_stamp),
                        static_cast<uint32>(request.image_type),
                        frame_id);
                }
            }
            if (published_to_shared_memory) {
                response.message = std::string("shared_memory|") + writer_key + "|" + std::to_string(frame_id);
            }
            else {
                response.message = "shared_memory_error";
            }
        }

        if (!published_to_shared_memory) {
            response.image_data_uint8 = std::vector<uint8_t>(render_results[i]->image_data_uint8.GetData(), render_results[i]->image_data_uint8.GetData() + render_results[i]->image_data_uint8.Num());
            response.image_data_float = std::vector<float>(render_results[i]->image_data_float.GetData(), render_results[i]->image_data_float.GetData() + render_results[i]->image_data_float.Num());
        }

        if (use_safe_method) {
            // Currently, we don't have a way to synthronize image capturing and camera pose when safe method is used,
            APIPCamera* camera = cameras_->at(request.camera_name);
            msr::airlib::Pose pose = camera->getPose();
            response.camera_position = pose.position;
            response.camera_orientation = pose.orientation;
        }
        response.pixels_as_float = request.pixels_as_float;
        response.compress = request.compress;
        response.width = render_results[i]->width;
        response.height = render_results[i]->height;
        response.image_type = request.image_type;
    }
}

bool UnrealImageCapture::updateCameraVisibility(APIPCamera* camera, const msr::airlib::ImageCaptureBase::ImageRequest& request)
{
    bool visibilityChanged = false;
    if (!camera->getCameraTypeEnabled(request.image_type)) {
        camera->setCameraTypeEnabled(request.image_type, true);
        visibilityChanged = true;
    }

    return visibilityChanged;
}

bool UnrealImageCapture::getScreenshotScreen(ImageType image_type, std::vector<uint8_t>& compressedPng)
{
    FScreenshotRequest::RequestScreenshot(false); // This is an async operation
    return true;
}

void UnrealImageCapture::addScreenCaptureHandler(UWorld* world)
{
    static bool is_installed = false;

    if (!is_installed) {
        UGameViewportClient* ViewportClient = world->GetGameViewport();
        ViewportClient->OnScreenshotCaptured().Clear();
        ViewportClient->OnScreenshotCaptured().AddLambda(
            [this](int32 SizeX, int32 SizeY, const TArray<FColor>& Bitmap) {
                // Make sure that all alpha values are opaque.
                TArray<FColor>& RefBitmap = const_cast<TArray<FColor>&>(Bitmap);
                for (auto& Color : RefBitmap)
                    Color.A = 255;

                TArray<uint8_t> last_compressed_png;
                FImageUtils::CompressImageArray(SizeX, SizeY, RefBitmap, last_compressed_png);
                last_compressed_png_ = std::vector<uint8_t>(last_compressed_png.GetData(), last_compressed_png.GetData() + last_compressed_png.Num());
            });

        is_installed = true;
    }
}
