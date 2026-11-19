#pragma once
#include "sensors/imu/ImuSimple.hpp"
#include <memory>
class ImuSharedMemoryWriter;

// Preserve ImuSimple physics/noise; retain every real native sample for readers.
class UnrealImuSensor : public msr::airlib::ImuSimple {
public:
    UnrealImuSensor(const msr::airlib::AirSimSettings::ImuSetting& setting, const std::string& vehicle);
    ~UnrealImuSensor();
    void update() override;
private:
    void resetImplementation() override;
    void publishOutput();
    std::unique_ptr<ImuSharedMemoryWriter> writer_;
};
