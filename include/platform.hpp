#pragma once

#include "motor.hpp"
#include "pid.hpp"

namespace vislib::platform {

using PlatformMotorConfig = core::Array<motor::MotorInfo>;
using PlatformMotorSpeeds = core::Array<motor::Speed>;

inline PlatformMotorConfig updateParallelAxisesForMotors(PlatformMotorConfig config, size_t precision) noexcept {
    for(size_t i = 0; i < config.Size(); i++) {
        config[i].parallelAxisesAmount = 1;
    }
    
    for(size_t i = 0; i < config.Size(); i++) {
        for(size_t j = i + 1; j < config.Size(); j++) {
            const double diff = static_cast<double>(lround(core::absF(config[i].anglePos - config[j].anglePos) * pow(10, precision)));
            if(diff == 0 || diff == 180) {
                config[i].parallelAxisesAmount++;
                config[j].parallelAxisesAmount++;
            }
        }
    }
    
    return config;
}

template<typename Controller> class Platform {
protected:
    core::Array<Controller> _controllers;
    
public:

    Platform(PlatformMotorConfig configuration, size_t parallelismPrecision = 0) noexcept {
        configuration = updateParallelAxisesForMotors(configuration, parallelismPrecision);
        _controllers = core::Array<Controller>(configuration.Size());
        for (size_t i = 0; i < _controllers.Size(); i++) {
            Controller ctrl(configuration[i]);
            _controllers[i] = ctrl;
        }
    }
    
    [[nodiscard]] core::Error setSpeeds(PlatformMotorSpeeds speeds) noexcept {
        if (speeds.Size() != _controllers.Size()) {
            return {core::ErrorCode::invalidArgument, "Cannot apply speeds set to controller set as there are different amount of them"};
        }
        
        core::Error err;
        
        for(size_t i = 0; i < _controllers.Size(); i++) {
            core::Error e = _controllers.at(i)().setSpeed(speeds.at(i)());
            if(e) {
                err.errcode = e.errcode;
                err.msg = err.msg + "\nAnother error encountered: Could not apply speed to motor controller, error encountered: " + e.msg;
            }
        }
        
        return err;
    }
    
    [[nodiscard]] core::Error setSpeedsInRanges(PlatformMotorSpeeds speeds, core::Array<motor::SpeedRange> ranges) noexcept {
        if (speeds.Size() != _controllers.Size() || speeds.Size() != ranges.Size()) {
            return {core::ErrorCode::invalidArgument,
                "Cannot apply speeds from different ranges set to controller set as there are different amounts of them"};
        }
        
        core::Error err;
        
        for(size_t i = 0; i < _controllers.Size(); i++) {
            core::Error e = _controllers.at(i)().setSpeedInRange(speeds.at(i)(), ranges[i]);
            if(e) {
                err.errcode = e.errcode;
                err.msg = err.msg + "\nAnother error encountered: Could not apply speed to motor controller, error encountered: " + e.msg;
            }
        }
        
        return err;
    }
    
    template<typename C> [[nodiscard]] core::Error init(const core::Array<C>& ports) noexcept {
        
        for(size_t i = 0; i < _controllers.Size(); i++) {
            
            auto t = _controllers.at(i);
            auto p = ports.at(i);
            
            if (t.isError()) {
                return {core::ErrorCode::initFailed,
                    "failed initializing one of the platform motors, invalid motor controller with index" 
                    + core::to_string(i) + ": " + t.error().msg};
            }
            
            if (p.isError()) {
                return {core::ErrorCode::invalidArgument,
                    "failed initializing one of the platform motors, invalid port array was given at index " 
                    + core::to_string(i) + " : " + p.error().msg};
            }
            
            auto e = t().init(p());
            
            if(e) {
                return {core::ErrorCode::initFailed,
                    "failed initializing one of the platform motors, failed motor controller initialization at index "
                    + core::to_string(i) + " and port with value " + core::to_string(static_cast<size_t>(p())) + ": " + e.msg};
            }
        }
        
        return core::ErrorCode::success;
    }
    
    const core::Array<Controller>& controllers() const noexcept {
        return _controllers;
    }
    
};

namespace calculators {
    
[[nodiscard]] inline core::Result<motor::Speed> calculateMotorLinearSpeed(const motor::MotorInfo& info, double angle, const motor::Speed& speed) noexcept {
    if(info.parallelAxisesAmount == 0) {
        return core::Error(core::ErrorCode::invalidArgument, "amount of motors with parallel movement axises cannot be zero in motor config");
    }

    if(!info.interfaceSpeedRange.contains(speed)) {
        return core::Error(core::ErrorCode::outOfRange, "the given speed is not in the configured motor interface speed range");
    }

    return core::cosDegrees(angle - info.anglePos) * speed / info.parallelAxisesAmount / info.wheelR;

}

[[nodiscard]] inline double calculateMotorSpeedLinearFromAngular(const motor::MotorInfo& info, const double angularSpeed) noexcept {
    return angularSpeed * info.distance / (info.wheelR != 0 ? info.wheelR : 1);
}

[[nodiscard]] inline core::Result<PlatformMotorSpeeds> calculatePlatformSpeeds(
        const PlatformMotorConfig& config,
        const double angle,
        const motor::Speed& speed,
        const double speedK = 1,
        const double angularSpeed = 0
    ) noexcept {

    PlatformMotorSpeeds speeds(config.Size());

    for(size_t i = 0; i < speeds.Size(); i++) {

        core::Result<motor::Speed> l = calculateMotorLinearSpeed(config[i], angle, speed * speedK);
        if(l) return l.error();


        speeds[i] = l() + calculateMotorSpeedLinearFromAngular(config[i], angularSpeed);
    }

    return speeds;
}

template<typename TimeType> class GyroPidCalculator {
public:
    PIDRegulator<double, TimeType>& pid;
    core::UniquePtr<gyro::YawGetter<double>> getter;

    GyroPidCalculator(const PIDRegulator<TimeType>& pid, core::UniquePtr<gyro::YawGetter<double>>&& getter) noexcept : pid(pid), getter(core::move(getter)) { }

    core::Result<PlatformMotorSpeeds> calculateSpeeds(
        TimeType time,
        const PlatformMotorConfig& config,
        const motor::Speed& speed,
        const double speedK = 1,
        const double angularSpeed = 0
    ) noexcept {
        core::Result<double> angle = getter->getYaw();

        if (angle.isError()) return angle.error();

        return calculatePlatformSpeeds(config, angle, speed, speedK, angularSpeed + pid.compute(angle, time));
    }
};

} // namespace vislib::platform::calculators

} //namespace vislib::platform

