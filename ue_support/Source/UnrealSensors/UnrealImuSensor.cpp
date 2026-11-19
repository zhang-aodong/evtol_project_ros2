#include "UnrealImuSensor.h"
#include <cstring>
#include <atomic>
#if PLATFORM_WINDOWS
#include <windows.h>
#include <intrin.h>
#endif

class ImuSharedMemoryWriter {
public:
    struct Header { uint32_t magic; uint16_t version, header_size; uint32_t slots, slot_size; uint64_t latest; uint8_t padding[40]; };
    struct Slot { int64_t guard; uint64_t frame_id, stamp; double values[10]; uint8_t padding[24]; };
    static_assert(sizeof(Header)==64 && sizeof(Slot)==128,"IMU ring layout");
    ImuSharedMemoryWriter(const std::string& name) {
#if PLATFORM_WINDOWS
        const uint32_t size=sizeof(Header)+1024*sizeof(Slot);
        mapping_=CreateFileMappingA(INVALID_HANDLE_VALUE,nullptr,PAGE_READWRITE,0,size,name.c_str());
        if (!mapping_) throw std::runtime_error("Cannot create IMU shared memory");
        if (GetLastError()==ERROR_ALREADY_EXISTS) {
            CloseHandle(static_cast<HANDLE>(mapping_));mapping_=nullptr;
            throw std::runtime_error("IMU shared memory already exists: stop previous writers/readers before restarting UE");
        }
        view_=static_cast<uint8_t*>(MapViewOfFile(static_cast<HANDLE>(mapping_),FILE_MAP_ALL_ACCESS,0,0,size));
        if (!view_) { CloseHandle(static_cast<HANDLE>(mapping_));mapping_=nullptr;throw std::runtime_error("Cannot map IMU shared memory"); }
        std::memset(view_,0,size);
        auto* header=reinterpret_cast<Header*>(view_);
        header->version=1;header->header_size=sizeof(Header);header->slots=1024;header->slot_size=sizeof(Slot);
        std::atomic_thread_fence(std::memory_order_seq_cst);header->magic=0x554D4945;
#endif
    }
    ~ImuSharedMemoryWriter() {
#if PLATFORM_WINDOWS
        if(view_)UnmapViewOfFile(view_);
        if(mapping_)CloseHandle(static_cast<HANDLE>(mapping_));
#endif
    }
    void publish(const msr::airlib::ImuBase::Output& output) {
#if PLATFORM_WINDOWS
        if (!view_)return;
        auto* header=reinterpret_cast<Header*>(view_);
        const uint64_t id=header->latest+1;
        auto* slot=reinterpret_cast<Slot*>(view_+sizeof(Header))+(id%1024);
        _InterlockedIncrement64(reinterpret_cast<volatile LONG64*>(&slot->guard));
        slot->frame_id=id;slot->stamp=output.time_stamp;
        const auto& q=output.orientation;const auto& w=output.angular_velocity;const auto& a=output.linear_acceleration;
        const double values[10]={q.x(),q.y(),q.z(),q.w(),w.x(),w.y(),w.z(),a.x(),a.y(),a.z()};
        std::memcpy(slot->values,values,sizeof(values));
        _InterlockedIncrement64(reinterpret_cast<volatile LONG64*>(&slot->guard));
        _InterlockedExchange64(reinterpret_cast<volatile LONG64*>(&header->latest),id);
#endif
    }
private:
    void* mapping_=nullptr;uint8_t* view_=nullptr;
};

UnrealImuSensor::UnrealImuSensor(const msr::airlib::AirSimSettings::ImuSetting& setting,const std::string& vehicle)
    : ImuSimple(setting),writer_(new ImuSharedMemoryWriter("AirSimImu_"+vehicle+"_"+setting.sensor_name)) {}
UnrealImuSensor::~UnrealImuSensor()=default;
void UnrealImuSensor::update() { ImuSimple::update();publishOutput(); }
void UnrealImuSensor::resetImplementation() { ImuSimple::resetImplementation();publishOutput(); }
void UnrealImuSensor::publishOutput() { if(writer_)writer_->publish(getOutput()); }
