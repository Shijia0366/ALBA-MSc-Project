#pragma once
#include <Arduino.h>

class StepperA4988{
public:
    StepperA4988(int stepPin, int dirPin, int enablePin);

    void begin();

    void enable();
    void disable();

    void setSpeedStepsPerSec(float speedStepsPerSec);
    float getCommandSpeedStepsPerSec() const;
    float getAppliedSpeedStepsPerSec() const;

    // A one-shot encoder stop, checked by the pulse ISR as well as main.cpp.
    // The counter must outlive this driver (the project's global encoder does).
    void armPositionStop(const volatile long* counter, long target, int direction);
    void clearPositionStop();
    bool positionStopReached() const;

    void stop();
    void update();

private:
    int stepPin_;
    int dirPin_;
    int enablePin_;

    volatile bool enabled_;
    float commandStepsPerSec_;
    float appliedStepsPerSec_;
    uint32_t lastRampMicros_;
    bool dirLevel_;

    float maxStepsPerSec_;
    // Conservative initial ramp; mechanical acceleration still needs validation.
    static constexpr float ACCELERATION_STEPS_S2 = 200.0f;
    static constexpr uint32_t TICK_US = 25;
    static constexpr uint32_t RATE_SCALE = 1000;
    static constexpr uint32_t PHASE_LIMIT = (1000000UL / TICK_US) * RATE_SCALE;
    static constexpr uint32_t SERVICE_TIMEOUT_US = 250000;
    static constexpr uint32_t DIR_HOLD_US = 5;

    hw_timer_t* timer_;
    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    volatile uint32_t rateMilliStepsS_;
    volatile uint32_t phase_;
    volatile bool stepHigh_;
    volatile uint8_t dirWaitTicks_;
    volatile uint32_t lastServiceMicros_;
    volatile uint32_t stoppedAtMicros_;
    volatile bool timingFault_;
    bool timingFaultReported_;
    uint32_t stepMask_;

    const volatile long* encoderCounter_;
    volatile long stopTargetCount_;
    volatile int8_t stopDirection_;
    volatile bool positionStopArmed_;
    volatile bool positionStopHit_;

    static StepperA4988* instance_;
    static void IRAM_ATTR onTimer();
    void IRAM_ATTR tick();
    void IRAM_ATTR haltPulsesLocked(uint32_t now);
    void publishAppliedSpeed(uint32_t now);

};
