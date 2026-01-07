#pragma once

#include <vislib.hpp>

namespace vislib {

template <typename T> struct TMPResult {
    T position{};
    T speed{};
    T acceleration{};
};

template <typename T, typename TimeType = T> class TrapezoidalMotionProfile {
protected:
    T acceleration{};
    T speedLimit{};
    
    T t1{};
    T t2{};
    T t3{};
    
    T x1{};
    T x2{};
    
    TimeType startTime{};
    
    T x0{};
    T xt{};
    
    char s = 1;
    
    bool isConfiguredFlag = false;
    
    
public:
    
    TrapezoidalMotionProfile() = default;
    
    TrapezoidalMotionProfile(const T& acceleration, const T& speedLimit) noexcept(core::numberNoexcept<T>())
    : acceleration(acceleration), speedLimit(speedLimit) { }
    
    inline constexpr T getAcceleration() const noexcept(core::numberNoexcept<T>()) {
        return acceleration;
    }
    
    inline constexpr T getSpeedLimit() const noexcept(core::numberNoexcept<T>()) {
        return speedLimit;
    }
    
    [[nodiscard]] core::Error validCheck() noexcept(core::numberNoexcept<T>()) {
        if(x0 == xt) {
            endMotion();
            return {core::ErrorCode::reachedTheTarget, "The motion starting position is the same as final destination"};
        }
        
        if(acceleration <= 0) {
            endMotion();
            return {core::ErrorCode::invalidConfiguration, "The motion controller doesn't support acceleration equal or below zero and is supposed to work with positive values"};
        }
        
        if(speedLimit <= 0) {
            endMotion();
            return {core::ErrorCode::invalidConfiguration, "The motion controller doesn't support speed limit equal or below zero and is supposed to work with positive values"};
        }
        
        return core::ErrorCode::success;
    }
    
    [[nodiscard]] core::Error isConfiguredAsErr() const noexcept(core::numberNoexcept<T>()) {
        if(isConfiguredFlag) return core::ErrorCode::success;
        
        core::Error err = validCheck();
        err.msg = "Trapezoidal Motion Profile wasn't configured or was ill-configured: " + err.msg;
        
        return err;
    }
    
    inline constexpr bool isConfigured() const noexcept {
        return isConfiguredFlag;
    }
    
    inline void endMotion() noexcept(core::numberNoexcept<T, TimeType>()) {
        *this = TrapezoidalMotionProfile();
    }
    
    [[nodiscard]] core::Error startMotion(const T& startPosition, const T& targetPosition, const TimeType& startTime = TimeType{}) noexcept(core::numberNoexcept<T, TimeType>()) {
        
        isConfiguredFlag = false;
        
        x0 = startPosition;
        xt = targetPosition;
        
        core::Error err = isConfiguredAsErr();
        
        if (err) return err;
        
        s = core::signF(xt - x0);
        
        speedLimit = s * core::minF(speedLimit, sqrt(core::absF(acceleration * (xt - x0))));
        
        t1 = core::absF(speedLimit) / acceleration;
        x1 = x0 + s * acceleration * t1 * t1 / 2.0;
        
        t2 = t1 + (xt + x0 - 2 * x1) / speedLimit;
        x2 = x1 + acceleration * (t2 - t1);
        
        t3 = t1 + t2;
        
        this->startTime = startTime;
        
        isConfiguredFlag = true;
        
        return core::ErrorCode::success;
    }
    
    
    
    [[nodiscard]] core::Result<TMPResult<T>> calculateMotion(const TimeType& timePoint) const noexcept(core::numberNoexcept<T, TimeType>()) {
        
        core::Error err = validCheck();
        
        if (err.errcode == core::ErrorCode::reachedTheTarget) return {err.errcode, "Already supposed to have reached the requested target position. No farther motion calculation will be conducted, you should call endMotion() or start new motion"};
        if(err) return err;
        
        TMPResult<T> result;
        
        TimeType t = timePoint - startTime;
        
        if (t < 0) return {core::ErrorCode::invalidArgument, "The given motion time point is less than the start of motion time. This is invalid behaviour, you should properly configure TMP and pass right time"};
        
        if(0 <= t && t <= t1) {
            result.position = x0 + s * acceleration * t * t / 2.0;
            result.speed = acceleration * t;
            result.acceleration = acceleration;
            
        } else if(t1 < t && t < t2) {
            result.position = x1 + speedLimit * (t - t1);
            result.speed = speedLimit;
            result.acceleration = 0;
            
        } else if(t2 <= t && t <= t3) {
            result.position = x2 + speedLimit * (t - t2) - s * acceleration * (t - t2) * (t - t2) / 2.0;
            result.speed = speedLimit - acceleration * (t - t2);
            result.acceleration = -acceleration;
            
        }
        
        return result;
        
    }

};     

template <typename T, typename TimeType = T> using TMP = TrapezoidalMotionProfile<T, TimeType>;

} // namespace vislib

