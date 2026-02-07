#pragma once
#include "platform.hpp"

namespace vislib::platform {
    
template <typename Controller_t, typename Time_t> class GyroPlatform : public Platform<Controller_t> {
protected:
    calculators::GyroPidCalculator<Time_t> calculator{};
    core::UniquePtr<gyro::YawGetter<core::Angle<>>> yawGetter{};
    core::TimeGetter<Time_t> timeGetter{};
    core::Angle<> headAngle{};
    
    bool isSyncHeadWithDir = false;
    
public:
    
    GyroPlatform(
        const calculators::GyroPidCalculator<Time_t>& calculator,
        core::UniquePtr<gyro::YawGetter<core::Angle<>>>& yawGetter,
        core::TimeGetter<Time_t>& timeGetter,
        const PlatformMotorConfig& configuration,
        size_t parallelismPrecision = 0) noexcept
        : calculator(calculator), yawGetter(core::move(yawGetter)), timeGetter(core::move(timeGetter)), Platform<Controller_t>(configuration, parallelismPrecision) {
        
    }
    
    GyroPlatform() = default;
    GyroPlatform(const GyroPlatform&) = default;
    GyroPlatform(GyroPlatform&&) = default;
    GyroPlatform& operator=(const GyroPlatform&) = default;
    GyroPlatform& operator=(GyroPlatform&&) = default;
    ~GyroPlatform() = default;
    
    void setHead(const core::Angle<>& angle) noexcept {
        headAngle = angle;
    }
    
    core::Angle<> getHead() const noexcept {
        return headAngle;
    }
    
    core::Error go(const double speed, const core::Angle<>& angle, bool isAngleRelative = false,  bool enableHeadSync = false, const double angularSpeed = 0, const double speedK = 1) noexcept {
        
        auto time = timeGetter();
        
        core::Result<core::Angle<>> yaw = yawGetter->getYaw();
        if(yaw.isError()) return yaw.error();
        
        if(enableHeadSync) {
            headAngle = angle;
        }
        
        core::Result<PlatformMotorSpeeds> speeds = calculator.calculateSpeeds(
            time,
            isAngleRelative ? yaw() - angle : angle,
            yaw.Value(),
            headAngle,
            speed,
            speedK,
            angularSpeed
        );
        
        if (speeds.isError()) return speeds.error();
        
        core::Error err = this->setSpeeds(speeds());
        
        if(err.isError()) return err;
        
        return {};
    }
    
};

} //vislib::platform
