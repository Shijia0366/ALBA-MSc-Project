#include "FlowMeter.h"
#include <math.h>

namespace {
constexpr uint32_t NO_PULSE_TIMEOUT_US = 1500000UL;
constexpr float FLOW_FILTER_ALPHA = 0.25f;
// DIGITEN B07HH16XC1 sensor: 1..30 L/min. Allow 25% headroom for
// calibration/pulse-spacing variation; do NOT reject low-flow pulses.
constexpr float MAX_RATED_FLOW_L_MIN = 30.0f;
constexpr float MAX_FLOW_HEADROOM = 1.25f;
}

FlowMeter* FlowMeter::instance_ = nullptr;

FlowMeter::FlowMeter(int pin, float pulsesPerLiter)
  : pin_(pin),
    pulsesPerLiter_(pulsesPerLiter),
    pulseCount_(0),
    lastPulseMicros_(0),
    pulseIntervalMicros_(0),
    lastEdgeMicros_(0),
    edgeSeen_(false),
    minPulseIntervalMicros_(0),
    lastProcessedPulseCount_(0),
    rawFlowLMin_(0.0f),
    filteredFlowLMin_(0.0f) {
  if (isfinite(pulsesPerLiter_) && pulsesPerLiter_ > 0.0f) {
    const double interval = 60000000.0 /
        (pulsesPerLiter_ * static_cast<double>(MAX_RATED_FLOW_L_MIN) * MAX_FLOW_HEADROOM);
    minPulseIntervalMicros_ = interval < 1.0 ? 1 :
        (interval > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(interval));
  }
}

void FlowMeter::begin() {
  pinMode(pin_, INPUT);

  instance_ = this;

  attachInterrupt(digitalPinToInterrupt(pin_), isrRouter, FALLING);
}

void IRAM_ATTR FlowMeter::isrRouter() {
  if (instance_ != nullptr) {
    instance_->handleInterrupt();
  }
}

void IRAM_ATTR FlowMeter::handleInterrupt() {
  const uint32_t nowMicros = micros();

  // Measure the quiet interval from EVERY edge, not just accepted edges.
  // Otherwise a continuous noise burst could be decimated into plausible
  // pulses. Rejected edges never advance the volume or accepted-pulse clock.
  const uint32_t edgeInterval = nowMicros - lastEdgeMicros_;
  const bool reject = edgeSeen_ && edgeInterval < minPulseIntervalMicros_;
  lastEdgeMicros_ = nowMicros;
  edgeSeen_ = true;
  if (reject) return;

  if (pulseCount_ == 0) {
    // The first pulse establishes the time reference; it has no interval yet.
    pulseIntervalMicros_ = 0;
  } else {
    // Unsigned subtraction remains correct across micros() wraparound.
    pulseIntervalMicros_ = nowMicros - lastPulseMicros_;
  }

  lastPulseMicros_ = nowMicros;
  pulseCount_++;
}

void FlowMeter::update(float dt) {
  (void)dt;

  noInterrupts();
  const uint32_t countNow = pulseCount_;
  const uint32_t lastPulseMicros = lastPulseMicros_;
  const uint32_t pulseIntervalMicros = pulseIntervalMicros_;
  interrupts();

  if (countNow != lastProcessedPulseCount_) {
    lastProcessedPulseCount_ = countNow;

    if (pulseIntervalMicros == 0 || pulsesPerLiter_ <= 0.0f) {
      rawFlowLMin_ = 0.0f;
      filteredFlowLMin_ = 0.0f;
    } else {
      const float pulseFrequencyHz = 1000000.0f / static_cast<float>(pulseIntervalMicros);
      const float newRawFlowLMin = pulseFrequencyHz / pulsesPerLiter_ * 60.0f;

      if (isfinite(newRawFlowLMin) && newRawFlowLMin >= 0.0f) {
        rawFlowLMin_ = newRawFlowLMin;
        filteredFlowLMin_ =
            FLOW_FILTER_ALPHA * rawFlowLMin_ +
            (1.0f - FLOW_FILTER_ALPHA) * filteredFlowLMin_;
      } else {
        rawFlowLMin_ = 0.0f;
        filteredFlowLMin_ = 0.0f;
      }
    }
  }

  const uint32_t nowMicros = micros();
  if (countNow == 0 ||
      static_cast<uint32_t>(nowMicros - lastPulseMicros) > NO_PULSE_TIMEOUT_US) {
    rawFlowLMin_ = 0.0f;
    filteredFlowLMin_ = 0.0f;
  }
}

uint32_t FlowMeter::getPulseCount() const {
  noInterrupts();
  const uint32_t value = pulseCount_;
  interrupts();
  return value;
}

uint32_t FlowMeter::getLastPulseMicros() const {
  noInterrupts();
  const uint32_t value = lastPulseMicros_;
  interrupts();
  return value;
}

uint32_t FlowMeter::getPulseIntervalMicros() const {
  noInterrupts();
  const uint32_t value = pulseIntervalMicros_;
  interrupts();
  return value;
}

float FlowMeter::getRawFlowLMin() const {
  return rawFlowLMin_;
}

float FlowMeter::getFilteredFlowLMin() const {
  return filteredFlowLMin_;
}

float FlowMeter::getFlowLMin() const {
  // Compatibility alias: existing control/status code consumes filtered flow.
  return filteredFlowLMin_;
}

void FlowMeter::reset() {
  noInterrupts();
  pulseCount_ = 0;
  lastPulseMicros_ = 0;
  pulseIntervalMicros_ = 0;
  lastEdgeMicros_ = 0;
  edgeSeen_ = false;
  lastProcessedPulseCount_ = 0;
  rawFlowLMin_ = 0.0f;
  filteredFlowLMin_ = 0.0f;
  interrupts();
}
