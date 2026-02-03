#pragma once

#include <vislib.hpp>

namespace vislib::gyro {

template <typename T> struct YPR {
    T yaw;
    T pitch;
    T roll;
    
    YPR() = default;
    
    YPR(const T& p_yaw, const T& p_pitch, const T& p_roll) : yaw(p_yaw), pitch(p_pitch), roll(p_roll) {}
    
    YPR(const YPR<T>& ) = default;
    
    YPR(YPR<T>&&) = default;
    
    YPR<T>& operator=(const YPR<T>& other) = default;
    
    YPR<T>& operator=(YPR<T>&& other) = default;
};

template <typename T> using Acceleration = core::Vector<T>;
template <typename T> using AngularSpeed = core::Vector<T>;

template <typename UpdateParameterType> class BaseGyroController {
public:
    virtual core::Error update(UpdateParameterType) = 0;
    virtual core::Error calibrate() = 0;
    virtual ~BaseGyroController() = default;
};

template <typename T, typename TimeType = T, typename WT = T> struct YPRElementCalculatorConfig {
    WT integralWeight = WT(1);
    T offset{};
    core::Integrator<T, TimeType> integrator{};
    
};
template <typename YPRType, typename AccAngularSpeedType> class GyroData {
private:
    using AccT = Acceleration<AccAngularSpeedType>;
    using SpeedT = AngularSpeed<AccAngularSpeedType>;
public:
    YPR<YPRType> ypr;
    Acceleration<AccAngularSpeedType> acceleration;
    AngularSpeed<AccAngularSpeedType> speed;
    
    
    GyroData() = default;
    
    GyroData(const YPR<YPRType>& ypr, const AccT& acceleration, const SpeedT& speed)
        noexcept(core::numberNoexcept<YPRType>() && core::numberNoexcept<AccAngularSpeedType>()) : ypr(ypr), acceleration(acceleration), speed(speed) {}
    
    GyroData(const GyroData& other) noexcept(core::numberNoexcept<YPRType>() && core::numberNoexcept<AccAngularSpeedType>())
        : ypr(other.ypr), acceleration(other.acceleration), speed(other.speed) {}
    
    GyroData(GyroData&& other) noexcept(core::numberNoexcept<YPRType>() && core::numberNoexcept<AccAngularSpeedType>())
        : ypr(core::move(other.ypr)), acceleration(core::move(other.acceleration)), speed(core::move(other.speed)) {}
    
    GyroData(YPR<YPRType>&& ypr, AccT&& acceleration, SpeedT&& speed) noexcept(core::numberNoexcept<YPRType>() && core::numberNoexcept<AccAngularSpeedType>())
        : ypr(core::move(ypr)), acceleration(core::move(acceleration)), speed(core::move(speed)) {}
    
    GyroData<YPRType, AccAngularSpeedType>& operator=(const GyroData& other) noexcept(core::numberNoexcept<YPRType>() && core::numberNoexcept<AccAngularSpeedType>()) = default;
    
    GyroData<YPRType, AccAngularSpeedType>& operator=(GyroData&& other) noexcept(core::numberNoexcept<YPRType>() && core::numberNoexcept<AccAngularSpeedType>()) {
        ypr = core::move(other.ypr);
        acceleration =  core::move(other.acceleration);
        speed = core::move(other.speed);
        
        return *this;
    }
};

// getters

template <typename T> class AccelerationGetter {
public:
    virtual core::Result<Acceleration<T>> getAcceleration() const noexcept(core::numberNoexcept<T>()) = 0;
    virtual ~AccelerationGetter() = default;
};

template <typename T> class AngularSpeedGetter {
public:
    virtual core::Result<AngularSpeed<T>> getAngularSpeed() const noexcept(core::numberNoexcept<T>()) = 0;
    virtual ~AngularSpeedGetter() = default;
};

template <typename T> class YawGetter {
public:
    virtual core::Result<T> getYaw() const noexcept(core::numberNoexcept<T>()) = 0;
    virtual ~YawGetter() = default;
};

template <typename T> class PitchGetter {
public:
    virtual core::Result<T> getPitch() const noexcept(core::numberNoexcept<T>()) = 0;
    virtual ~PitchGetter() = default;
};

template <typename T> class RollGetter {
public:
    virtual core::Result<T> getRoll() const noexcept(core::numberNoexcept<T>()) = 0;
    virtual ~RollGetter() = default;
};

template <typename T> class YPRGetter : virtual public YawGetter<T>, virtual public PitchGetter<T>, virtual public RollGetter<T> {
public:
    virtual core::Result<YPR<T>> getYPR() const noexcept(core::numberNoexcept<T>()) {
        
        core::Result<T> yaw = this->getYaw();
        if(yaw) return yaw.error();
        
        core::Result<T> pitch = this->getPitch();
        if(pitch) return pitch.error();
        
        core::Result<T> roll = this->getRoll();
        if(roll) return roll.error();
        
        return YPR<T>{yaw(), pitch(), roll()};
    }
    
    virtual ~YPRGetter() override = default;
};

// calculators

template <typename T, typename TimeType = T, typename WT = T> class YawCalculator : virtual public YawGetter<T>, virtual public AngularSpeedGetter<T> {
protected:
    YPRElementCalculatorConfig<T, TimeType, WT> yawConfig{};
    
    virtual core::Error internalYawInit(const YPRElementCalculatorConfig<T, TimeType, WT>& config)
    noexcept(core::numberNoexcept<T>() && core::numberNoexcept<TimeType>()) {
        
        return core::ErrorCode::success;
    };
    
    virtual core::Result<T> internalNonIntegralPartYawCalculation() const {
        return T();
    }
    
public:
    
    virtual core::Error initYawCalculator(const YPRElementCalculatorConfig<T, TimeType, WT>& config)
    noexcept(core::numberNoexcept<T>() && core::numberNoexcept<TimeType>()) {
        
        yawConfig = config;
        yawConfig.integrator.setIntegral(yawConfig.integrator.getIntegral() + yawConfig.offset);
        
        return internalYawInit(yawConfig);
    }

    virtual core::Result<T> calculateYaw(const TimeType& currentTime)
    noexcept(core::numberNoexcept<T>() && core::numberNoexcept<TimeType>()) {
        
        core::Result<AngularSpeed<T>> angularSpeed = this->getAngularSpeed();
        if(angularSpeed) return angularSpeed.Err();
        
        core::Result<T> temp = yawConfig.integrator.update(currentTime, angularSpeed().at(0));
        if(temp) return temp.Err();
        
        core::Result<T> nonIntegral = internalNonIntegralPartYawCalculation();
        if(nonIntegral) return nonIntegral.Err();
        
        yawConfig.integrator.setIntegral(temp() * yawConfig.integralWeight + nonIntegral() * (T(1) - yawConfig.integralWeight));
        
        return yawConfig.integrator.getIntegral();
        
    }
    
    virtual ~YawCalculator() override = default;
};

template <typename T, typename TimeType = T, typename WT = T> class PitchCalculator : virtual public PitchGetter<T>, virtual public AngularSpeedGetter<T> {
protected:
    YPRElementCalculatorConfig<T, TimeType, WT> pitchConfig{};
    
    virtual core::Error internalPitchInit(const YPRElementCalculatorConfig<T, TimeType, WT>& config)
    noexcept(core::numberNoexcept<T>() && core::numberNoexcept<TimeType>()) {
        
        return core::ErrorCode::success;
    };
    
    virtual core::Result<T> internalNonIntegralPartPitchCalculation() const {
        return T();
    }
    
public:
    
    virtual core::Error initPitchCalculator(const YPRElementCalculatorConfig<T, TimeType, WT>& config)
    noexcept(core::numberNoexcept<T>() && core::numberNoexcept<TimeType>()) {
        
        pitchConfig = config;
        pitchConfig.integrator.setIntegral(pitchConfig.integrator.getIntegral() + pitchConfig.offset);
        
        return internalPitchInit(pitchConfig);
    }

    virtual core::Result<T> calculatePitch(const TimeType& currentTime)
    noexcept(core::numberNoexcept<T>() && core::numberNoexcept<TimeType>()) {
        
        core::Result<AngularSpeed<T>> angularSpeed = this->getAngularSpeed();
        if(angularSpeed) return angularSpeed.Err();
        
        core::Result<T> temp = pitchConfig.integrator.update(currentTime, angularSpeed().at(1));
        if(temp) return temp.Err();
        
        core::Result<T> nonIntegral = internalNonIntegralPartPitchCalculation();
        if(nonIntegral) return nonIntegral.Err();
        
        pitchConfig.integrator.setIntegral(temp() * pitchConfig.integralWeight + nonIntegral() * (T(1) - pitchConfig.integralWeight));
        
        return pitchConfig.integrator.getIntegral();
        
    }
    
    virtual ~PitchCalculator() override = default;
};

template <typename T, typename TimeType = T, typename WT = T> class RollCalculator : virtual public RollGetter<T>, virtual public AngularSpeedGetter<T> {
protected:
    YPRElementCalculatorConfig<T, TimeType, WT> rollConfig{};
    
    virtual core::Error internalRollInit(const YPRElementCalculatorConfig<T, TimeType, WT>& config)
    noexcept(core::numberNoexcept<T>() && core::numberNoexcept<TimeType>()) {
        
        return core::ErrorCode::success;
    };
    
    virtual core::Result<T> internalNonIntegralPartRollCalculation() const {
        return T();
    }
    
public:
    
    virtual core::Error initRollCalculator(const YPRElementCalculatorConfig<T, TimeType, WT>& config)
    noexcept(core::numberNoexcept<T>() && core::numberNoexcept<TimeType>()) {
        
        rollConfig = config;
        rollConfig.integrator.setIntegral(rollConfig.integrator.getIntegral() + rollConfig.offset);
        
        return internalRollInit(rollConfig);
    }

    virtual core::Result<T> calculateRoll(const TimeType& currentTime)
    noexcept(core::numberNoexcept<T>() && core::numberNoexcept<TimeType>()) {
        
        core::Result<AngularSpeed<T>> angularSpeed = this->getAngularSpeed();
        if(angularSpeed) return angularSpeed.Err();
        
        core::Result<T> temp = rollConfig.integrator.update(currentTime, angularSpeed().at(2));
        if(temp) return temp.Err();
        
        core::Result<T> nonIntegral = internalNonIntegralPartRollCalculation();
        if(nonIntegral) return nonIntegral.Err();
        
        rollConfig.integrator.setIntegral(temp() * rollConfig.integralWeight + nonIntegral() * (T(1) - rollConfig.integralWeight));
        
        return rollConfig.integrator.getIntegral();
        
    }
    
    virtual ~RollCalculator() override = default;
};

template <typename T, typename TimeType = T, typename WT = T> class YPRCalculator :
    virtual public YawCalculator<T, TimeType, WT>,
    virtual public PitchCalculator<T, TimeType, WT>,
    virtual public RollCalculator<T, TimeType, WT> {
public:

    virtual core::Error initCalculator(const YPRElementCalculatorConfig<T, TimeType, WT>& yawConfig,
                                    const YPRElementCalculatorConfig<T, TimeType, WT>& pitchConfig,
                                    const YPRElementCalculatorConfig<T, TimeType, WT>& rollConfig)
        noexcept(core::numberNoexcept<T>() && core::numberNoexcept<TimeType>()) {
            
            core::Error err = this->initYawCalculator(yawConfig);
            if(err) return err;
            
            err = this->initPitchCalculator(pitchConfig);
            if(err) return err;
            
            err = this->initRollCalculator(rollConfig);
            if(err) return err;
            
            return core::ErrorCode::success;
        }

    virtual core::Result<YPR<T>> calculateYPR(const TimeType& currentTime) noexcept(core::numberNoexcept<T>()) {
        
        core::Result<T> yaw = this->calculateYaw(currentTime);
        if(yaw) return yaw.Err();
        
        core::Result<T> pitch = this->calculatePitch(currentTime);
        if(pitch) return pitch.Err();
        
        core::Result<T> roll = this->calculateRoll(currentTime);
        if(roll) return roll.Err();
        
        return YPR<T>{yaw(), pitch(), roll()};
    }
    
    virtual ~YPRCalculator() override = default;
};


//extended calculators

template <typename T, typename TimeType = T, typename WT = T> class PitchCalculatorWithAcceleration :
    virtual public PitchCalculator<T, TimeType, WT>,
    virtual public AccelerationGetter<T> {
protected:
    virtual core::Result<T> internalNonIntegralPartPitchCalculation() const override {
        core::Result<Acceleration<T>> acceleration = this->getAcceleration();
        if(acceleration) return acceleration.Err();
        
        // T accX = acceleration().at(0);
        T accY = acceleration().at(1);
        T accZ = acceleration().at(2);
        
        T temp = sqrt(accY * accY + accZ * accZ);
        if(temp == T(0)) return T(0);
        
        
        return core::rad2Deg(atan2(accY, temp));
    }

public:

    virtual ~PitchCalculatorWithAcceleration() override = default;
};

template <typename T, typename TimeType = T, typename WT = T> class RollCalculatorWithAcceleration
    : virtual public RollCalculator<T, TimeType, WT>, virtual public AccelerationGetter<T> {
protected:
    virtual core::Result<T> internalNonIntegralPartRollCalculation() const override {
        core::Result<Acceleration<T>> acceleration = this->getAcceleration();
        if(acceleration) return acceleration.Err();
        
        T accY = acceleration().at(1);
        T accZ = acceleration().at(2);
        
        if(accZ == T(0)) return T(90);
        
        
        return core::rad2Deg(atan2(accY, accZ));
    }

public:

    virtual ~RollCalculatorWithAcceleration() override = default;
};

template <typename T, typename TimeType = T, typename WT = T> class YPRCalculatorWithAcceleration
    : public virtual YPRCalculator<T, TimeType>, public PitchCalculatorWithAcceleration<T, TimeType>, public RollCalculatorWithAcceleration<T, TimeType, WT> {
public:
    virtual ~YPRCalculatorWithAcceleration() override = default;
};

// controllers

template <typename YPRType, typename AccAngularSpeedType> class GyroDataGetter
    : public YPRGetter<YPRType>, virtual public AccelerationGetter<AccAngularSpeedType>, virtual public AngularSpeedGetter<AccAngularSpeedType> {

public:
    virtual core::Result<GyroData<YPRType, AccAngularSpeedType>> getGyroData() const noexcept(core::numberNoexcept<YPRType>() && core::numberNoexcept<AccAngularSpeedType>()) {
        
        core::Result<YPR<YPRType>> ypr = this->getYPR();
        if(ypr) return ypr.error();
        
        core::Result<core::Vector<AccAngularSpeedType>> acceleration = this->getAcceleration();
        if(acceleration) return acceleration.error();
        
        core::Result<core::Vector<AccAngularSpeedType>> speed = this->getAngularSpeed();
        if(speed) return speed.error();
        
        return GyroData<YPRType, AccAngularSpeedType>{ypr(), acceleration(), speed()};
    }
    
    virtual ~GyroDataGetter() override = default;
};

template <typename YPRType, typename TimeType = YPRType, typename WT = YPRType, typename AccAngularSpeedType = YPRType> class GyroDataCalculator
    : virtual public YPRCalculator<YPRType, TimeType, WT>, virtual public AccelerationGetter<AccAngularSpeedType>, virtual public AngularSpeedGetter<AccAngularSpeedType> {

public:
    virtual core::Result<GyroData<YPRType, AccAngularSpeedType>> calculateGyroData(const TimeType& current) noexcept(core::numberNoexcept<YPRType>() && core::numberNoexcept<AccAngularSpeedType>()) {
        
        core::Result<YPR<YPRType>> ypr = this->calculateYPR(current);
        if(ypr) return ypr.Err();
        
        core::Result<Acceleration<AccAngularSpeedType>> acceleration = this->getAcceleration();
        if(acceleration) return acceleration.Err();
        
        core::Result<AngularSpeed<AccAngularSpeedType>> speed = this->getAngularSpeed();
        if(speed) return speed.Err();
        
        return GyroData<YPRType, AccAngularSpeedType>{ypr(), acceleration(), speed()};
    }
    
    virtual ~GyroDataCalculator() override = default;
};

template <typename YPRType, typename TimeType = YPRType, typename WT = YPRType, typename AccAngularSpeedType = YPRType> class GyroDataCalculatorWithAcceleration
    : virtual public YPRCalculatorWithAcceleration<YPRType, TimeType, WT>, public GyroDataCalculator<YPRType, TimeType, WT, AccAngularSpeedType> {
public:
    
    using YPRCalculatorWithAcceleration<YPRType, TimeType, WT>::calculateYPR;

    virtual ~GyroDataCalculatorWithAcceleration() override = default;
};

template <typename YPRType, typename TimeType = YPRType, typename WT = YPRType, typename AccAngularSpeedType = YPRType, typename UpdateParameterType = TimeType> class UltimateGyroCalculator :
    public BaseGyroController<UpdateParameterType>,
    public GyroDataGetter<YPRType, AccAngularSpeedType>,
    public GyroDataCalculatorWithAcceleration<YPRType, TimeType, WT, AccAngularSpeedType> {

public:
    
    virtual inline vislib::core::Result<YPRType> getYaw() const noexcept override {
        return this->yawConfig.integrator.getIntegral();
    }

    virtual inline vislib::core::Result<YPRType> getPitch() const noexcept override {
        return this->pitchConfig.integrator.getIntegral();
    }

    virtual inline vislib::core::Result<YPRType> getRoll() const noexcept override {
        return this->rollConfig.integrator.getIntegral();
    }
    
    virtual inline core::Error calibrate() noexcept override {
        
        this->yawConfig.offset += this->getYaw()();
        this->yawConfig.integrator.setIntegral(this->yawConfig.offset);
        
        this->rollConfig.offset += this->getRoll()();
        this->rollConfig.integrator.setIntegral(this->rollConfig.offset);
        
        this->pitchConfig.offset += this->getPitch()();
        this->pitchConfig.integrator.setIntegral(this->pitchConfig.offset);
        
        return {};
    }
    
    virtual inline core::Error update(UpdateParameterType currentTime) override {
        auto e = this->calculateYPR(currentTime);

        if (e) return e.Err();

        return {};
    }
    
    virtual ~UltimateGyroCalculator() override = default;
};

template <typename YPRType, typename TimeType = YPRType, typename AccAngularSpeedType = YPRType, typename UpdateParameterType = TimeType> class UltimateGyroGetter :
    public BaseGyroController<UpdateParameterType>,
    public GyroDataGetter<YPRType, AccAngularSpeedType> {

public:
    
    virtual ~UltimateGyroGetter() override = default;
};

} // namespace vislib::gyro
