#pragma once

#include <vislib.hpp>
#include "Arduino.h"

namespace vislib::motor {

using Speed = double;

using SpeedRange = core::Range<Speed>;

class MotorInfo {
public:
    double anglePos = 0;
    double distance = 1;
    double wheelR = 1;
    SpeedRange speedRange;
    SpeedRange interfaceSpeedRange;
    bool isReversed = false;
    
    size_t parallelAxisesAmount = 1;
    
    MotorInfo() = default;
    
    MotorInfo(double p_ap, double p_d, double p_wr, const SpeedRange& p_speed, const SpeedRange& p_interfaceSpeed, bool p_reversed = false) noexcept
    : anglePos(p_ap), distance(p_d), wheelR(p_wr), speedRange(p_speed), interfaceSpeedRange(p_interfaceSpeed), isReversed(p_reversed) {}
    
};

namespace controllers {

class MotorInfoIncluded {
protected:
    MotorInfo info;
public:
    
    MotorInfoIncluded() = default;
    
    MotorInfoIncluded(const MotorInfo& p_info) noexcept : info(p_info) {}
    
    virtual inline MotorInfo Info() const {
        return info;
    }

    virtual ~MotorInfoIncluded() = default;
};

template <typename T> class InitializationController {
public:
    virtual core::Error init(T) = 0;

    virtual ~InitializationController() = default;
};

class SpeedController {
public:

    virtual core::Error setSpeed(Speed) = 0;
    virtual core::Result<Speed> getSpeed() const = 0;

    virtual ~SpeedController() = default;
};

class RangedSpeedController : public MotorInfoIncluded, public SpeedController {
protected:
    virtual core::Error setSpeedRaw(Speed) = 0;
    virtual core::Result<Speed> getSpeedRaw() const = 0;
public:
    using MotorInfoIncluded::MotorInfoIncluded;
    
    [[nodiscard]] virtual inline core::Error setSpeed(Speed speed) noexcept override {
        return setSpeedRaw(info.interfaceSpeedRange.mapValueToRange(info.interfaceSpeedRange.restrict(info.isReversed ? -speed : speed), info.speedRange));
    }
    
    [[nodiscard]] virtual core::Result<Speed> getSpeed() const noexcept override {
        core::Result<Speed> rawSpeed = getSpeedRaw();
        if(rawSpeed) return rawSpeed;
        
        Speed mapped = info.speedRange.mapValueToRange(rawSpeed(), info.interfaceSpeedRange);
        
        return info.isReversed ? -mapped : mapped;
    }

    virtual inline bool inSpeedRange(Speed speed) const noexcept {
        return info.interfaceSpeedRange.contains(speed);
    }
    
    [[nodiscard]] virtual inline core::Error setSpeedInRange(Speed speed, SpeedRange range) noexcept {
        return setSpeed(info.interfaceSpeedRange.mapValueFromRange(range.restrict(speed), range));
    }

    virtual ~RangedSpeedController() override = default;
};

} //namespace vislib::motor::controllers

} //namespace vislib::motor
