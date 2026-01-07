#pragma once

#include <vislib.hpp>

namespace vislib {

template <typename T, typename TimeType = size_t> class PIDRegulator {
protected:
    T Kp{};
    T Ki{};
    T Kd{};
    T errold{};
    T integral{};
    T target{};
    TimeType prevTime{};
    
public:
    PIDRegulator(const T& Kp, const T& Ki, const T& Kd, const T& Target = T{}) noexcept(core::numberNoexcept<T>()) : Kp(Kp), Ki(Ki), Kd(Kd), target(target) {}

    
    [[nodiscard]] T compute(const T& measured, const T& target, const TimeType& time) noexcept(core::numberNoexcept<T, TimeType>()) {
        T error = target - measured;
        
        if (prevTime == 0) {
            prevTime = time;
            errold = error;
            return Kp * error;
        }
        
        TimeType timeStep = time - prevTime;
        
        
        integral += error * timeStep;
        
        T derivative = (timeStep > 0) ? (error - errold) / static_cast<T>(timeStep) : 0;
        
        T output = Kp * error + Ki * integral + Kd * derivative;
        
        errold = error;
        prevTime = time;
        
        return output;
        
    }
    
    inline T compute(const T& measured, const TimeType& time) noexcept(core::numberNoexcept<T, TimeType>()) {
        return compute(measured, this->target, time);
    }
    
    inline void setTarget(const T& target) noexcept(core::numberNoexcept<T>()) {
        this->target = target;
    }
    
    inline constexpr T getTarget() const noexcept {
        return target;
    }
    
};

} // namespace vislib
