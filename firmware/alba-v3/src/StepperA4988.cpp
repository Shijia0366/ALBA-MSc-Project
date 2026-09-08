#include "StepperA4988.h"
#include <math.h>
#include <esp_arduino_version.h>
#include <soc/gpio_struct.h>

// ===================== Direction polarity =====================
//
// Flip this if a positive speed command drives the motor the "wrong" way
// physically. It only swaps which DIR level a positive command maps to; it
// does NOT change how the STEP pulses are generated.
//
// Leave this at false unless a positive speed command drives the mechanism in
// the wrong physical direction. Changing this value requires recompilation.
static const bool INVERT_DIRECTION = false;

StepperA4988* StepperA4988::instance_ = nullptr;

StepperA4988::StepperA4988(int stepPin, int dirPin, int enablePin)
    : stepPin_(stepPin),
      dirPin_(dirPin),
      enablePin_(enablePin),
      enabled_(false),
      commandStepsPerSec_(0.0f),
      appliedStepsPerSec_(0.0f),
      lastRampMicros_(0),
      dirLevel_(HIGH),
      maxStepsPerSec_(3000.0f),
      timer_(nullptr),
      rateMilliStepsS_(0),
      phase_(0),
      stepHigh_(false),
      dirWaitTicks_(0),
      lastServiceMicros_(0),
      stoppedAtMicros_(0),
      timingFault_(false),
      timingFaultReported_(false),
      stepMask_(0),
      encoderCounter_(nullptr),
      stopTargetCount_(0),
      stopDirection_(0),
      positionStopArmed_(false),
      positionStopHit_(false) {}

void StepperA4988::begin() {
    if (timer_ != nullptr) {
        disable();
        return;
    }
    pinMode(stepPin_, OUTPUT);
    pinMode(dirPin_, OUTPUT);
    pinMode(enablePin_, OUTPUT);
    
    digitalWrite(stepPin_, LOW);
    digitalWrite(dirPin_, HIGH);

    // A4988 enabled when enable pin is LOW, so set HIGH to disable by default
    digitalWrite(enablePin_, HIGH);  // disabled by default
    enabled_ = false;

    // This project has one motor and STEP=18. Direct ISR GPIO writes below
    // deliberately support only output pins in the ESP32's low GPIO bank.
    if (stepPin_ < 0 || stepPin_ >= 32 ||
        (instance_ != nullptr && instance_ != this)) {
        Serial.println("ERROR: STEP timer requires one motor on GPIO 0..31");
        return;
    }
    stepMask_ = 1UL << stepPin_;
    lastRampMicros_ = lastServiceMicros_ = stoppedAtMicros_ = micros();
    instance_ = this;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    timer_ = timerBegin(1000000);
#else
    timer_ = timerBegin(0, 80, true);  // ESP32 timer 0, 1 MHz clock
#endif
    if (timer_ == nullptr) {
        Serial.println("ERROR: cannot allocate STEP timer; motor stays disabled");
        return;
    }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    timerAttachInterrupt(timer_, &StepperA4988::onTimer);
    timerAlarm(timer_, TICK_US, true, 0);
#else
    timerAttachInterrupt(timer_, &StepperA4988::onTimer, false);
    timerAlarmWrite(timer_, TICK_US, true);
    timerAlarmEnable(timer_);
#endif
}

void StepperA4988::enable() {
    if (timer_ == nullptr) {
        Serial.println("ERROR: STEP timer unavailable; motor stays disabled");
        return;
    }
    const uint32_t now = micros();
    portENTER_CRITICAL(&mux_);
    if (timingFault_) {
        haltPulsesLocked(now);
        commandStepsPerSec_ = appliedStepsPerSec_ = 0.0f;
        timingFault_ = false;
        timingFaultReported_ = false;
    }
    lastServiceMicros_ = now;
    enabled_ = true;
    digitalWrite(enablePin_, LOW);  // A4988 enabled
    portEXIT_CRITICAL(&mux_);
    lastRampMicros_ = now;
}

void StepperA4988::disable() {
    const uint32_t now = micros();
    portENTER_CRITICAL(&mux_);
    enabled_ = false;
    haltPulsesLocked(now);
    commandStepsPerSec_ = 0.0f;
    appliedStepsPerSec_ = 0.0f;
    digitalWrite(enablePin_, HIGH); // A4988 disabled
    portEXIT_CRITICAL(&mux_);
    lastRampMicros_ = now;
}

void StepperA4988::setSpeedStepsPerSec(float speedStepsPerSec) {
    // A zero command is an immediate stop, never a deceleration request.
    if (!isfinite(speedStepsPerSec) || speedStepsPerSec == 0.0f) {
        stop();
        return;
    }
    if (speedStepsPerSec > maxStepsPerSec_) speedStepsPerSec = maxStepsPerSec_;
    if (speedStepsPerSec < -maxStepsPerSec_) speedStepsPerSec = -maxStepsPerSec_;
    portENTER_CRITICAL(&mux_);
    const bool blocked = timingFault_ || positionStopHit_;
    portEXIT_CRITICAL(&mux_);
    if (!blocked) commandStepsPerSec_ = speedStepsPerSec;
}

float StepperA4988::getCommandSpeedStepsPerSec() const {
    return commandStepsPerSec_;  // Target, not measured motor velocity.
}

float StepperA4988::getAppliedSpeedStepsPerSec() const {
    return appliedStepsPerSec_;
}

void IRAM_ATTR StepperA4988::haltPulsesLocked(uint32_t now) {
    if (rateMilliStepsS_ != 0 || stepHigh_) stoppedAtMicros_ = now;
    rateMilliStepsS_ = 0;
    phase_ = 0;
    stepHigh_ = false;
    dirWaitTicks_ = 0;
    GPIO.out_w1tc = stepMask_;
}

void StepperA4988::stop() {
    const uint32_t now = micros();
    portENTER_CRITICAL(&mux_);
    haltPulsesLocked(now);
    portEXIT_CRITICAL(&mux_);
    commandStepsPerSec_ = appliedStepsPerSec_ = 0.0f;
    lastRampMicros_ = now;
    // ENABLE deliberately stays unchanged: stopping retains holding torque.
}

void StepperA4988::armPositionStop(const volatile long* counter, long target,
                                  int direction) {
    portENTER_CRITICAL(&mux_);
    if (positionStopHit_) {
        haltPulsesLocked(micros());
        commandStepsPerSec_ = appliedStepsPerSec_ = 0.0f;
    }
    encoderCounter_ = counter;
    stopTargetCount_ = target;
    stopDirection_ = direction > 0 ? 1 : (direction < 0 ? -1 : 0);
    positionStopHit_ = false;
    positionStopArmed_ = counter != nullptr;
    portEXIT_CRITICAL(&mux_);
}

void StepperA4988::clearPositionStop() {
    portENTER_CRITICAL(&mux_);
    // Check and clear atomically: an ISR trip between a caller's check and
    // this function must not revive the old speed when the gate is removed.
    if (positionStopHit_) {
        haltPulsesLocked(micros());
        commandStepsPerSec_ = appliedStepsPerSec_ = 0.0f;
    }
    positionStopArmed_ = false;
    positionStopHit_ = false;
    encoderCounter_ = nullptr;
    portEXIT_CRITICAL(&mux_);
}

bool StepperA4988::positionStopReached() const {
    portENTER_CRITICAL(&mux_);
    const bool reached = positionStopHit_;
    portEXIT_CRITICAL(&mux_);
    return reached;
}

void StepperA4988::publishAppliedSpeed(uint32_t now) {
    const uint32_t rate = static_cast<uint32_t>(fabsf(appliedStepsPerSec_) * RATE_SCALE + 0.5f);
    bool desiredDir = appliedStepsPerSec_ >= 0.0f ? LOW : HIGH;
    if (INVERT_DIRECTION) desiredDir = !desiredDir;

    portENTER_CRITICAL(&mux_);
    if (!enabled_ || timingFault_ || positionStopHit_ || rate == 0) {
        haltPulsesLocked(now);
    } else if (desiredDir != dirLevel_) {
        // Never change DIR on a live pulse. First reach zero, then observe
        // hold time; the ISR adds two quiet ticks after changing DIR.
        if (rateMilliStepsS_ != 0) {
            haltPulsesLocked(now);
            appliedStepsPerSec_ = 0.0f;
        } else if (static_cast<uint32_t>(now - stoppedAtMicros_) < DIR_HOLD_US) {
            appliedStepsPerSec_ = 0.0f;
        } else {
            digitalWrite(dirPin_, desiredDir);
            dirLevel_ = desiredDir;
            dirWaitTicks_ = 2;
            rateMilliStepsS_ = rate;
        }
    } else {
        // Keep accumulated phase when speed changes: repeated calls must not
        // restart the waveform or postpone its next pulse indefinitely.
        rateMilliStepsS_ = rate;
    }
    portEXIT_CRITICAL(&mux_);
}

void StepperA4988::update() {
    const uint32_t now = micros();
    uint32_t dtUs = now - lastRampMicros_;
    portENTER_CRITICAL(&mux_);
    const bool fault = timingFault_;
    const bool hit = positionStopHit_;
    const bool enabled = enabled_;
    lastServiceMicros_ = now;
    portEXIT_CRITICAL(&mux_);
    if (fault || hit || !enabled) {
        lastRampMicros_ = now;
        appliedStepsPerSec_ = 0.0f;
        if (fault || hit) commandStepsPerSec_ = 0.0f;
        if (fault && !timingFaultReported_) {
            timingFaultReported_ = true;
            Serial.println("ERROR: STEP service timeout; pulses stopped. Use h, then e to re-arm.");
        }
        return;
    }
    if (commandStepsPerSec_ == 0.0f) {
        stop();
        return;
    }
    // Integrate ramps at <= 1 kHz; tiny per-loop float additions accumulate
    // rounding error and make acceleration depend on loop frequency.
    if (dtUs < 1000) return;
    lastRampMicros_ = now;
    // A stalled loop must not produce a large catch-up acceleration jump.
    if (dtUs > 20000) dtUs = 20000;
    const float increment = ACCELERATION_STEPS_S2 * (dtUs / 1000000.0f);
    float rampTarget = commandStepsPerSec_;
    if ((appliedStepsPerSec_ > 0.0f && rampTarget < 0.0f) ||
        (appliedStepsPerSec_ < 0.0f && rampTarget > 0.0f)) rampTarget = 0.0f;
    const float difference = rampTarget - appliedStepsPerSec_;
    if (fabsf(difference) <= increment) appliedStepsPerSec_ = rampTarget;
    else appliedStepsPerSec_ += difference > 0.0f ? increment : -increment;
    publishAppliedSpeed(now);
}

void IRAM_ATTR StepperA4988::onTimer() {
    if (instance_ != nullptr) instance_->tick();
}

void IRAM_ATTR StepperA4988::tick() {
    // No Serial, allocation, floating-point operations or controller calls in
    // this ISR. 40 kHz integer phase accumulation gives 25 us quantization.
    const uint32_t now = micros();
    portENTER_CRITICAL_ISR(&mux_);
    if (positionStopArmed_ && !positionStopHit_ && encoderCounter_ != nullptr) {
        const long count = *encoderCounter_;
        if (stopDirection_ == 0 ||
            (stopDirection_ > 0 && count >= stopTargetCount_) ||
            (stopDirection_ < 0 && count <= stopTargetCount_)) {
            positionStopHit_ = true;
            haltPulsesLocked(now);
        }
    }
    if (!enabled_ || timingFault_ || positionStopHit_ || rateMilliStepsS_ == 0) {
        portEXIT_CRITICAL_ISR(&mux_);
        return;
    }
    if (static_cast<uint32_t>(now - lastServiceMicros_) > SERVICE_TIMEOUT_US) {
        timingFault_ = true;
        haltPulsesLocked(now);
        portEXIT_CRITICAL_ISR(&mux_);
        return;
    }
    if (stepHigh_) {
        GPIO.out_w1tc = stepMask_;
        stepHigh_ = false;
    }
    if (dirWaitTicks_ != 0) {
        --dirWaitTicks_;
        portEXIT_CRITICAL_ISR(&mux_);
        return;
    }
    phase_ += rateMilliStepsS_;
    if (phase_ >= PHASE_LIMIT) {
        phase_ -= PHASE_LIMIT;
        GPIO.out_w1ts = stepMask_;
        stepHigh_ = true;
    }
    portEXIT_CRITICAL_ISR(&mux_);
}
