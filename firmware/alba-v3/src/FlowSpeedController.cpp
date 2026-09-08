#include "FlowSpeedController.h"

FlowSpeedController::FlowSpeedController(float baseSpeedStepsPerSec,
                                         float flowGain,
                                         float minSpeedStepsPerSec,
                                         float maxSpeedStepsPerSec)
  : baseSpeedStepsPerSec_(baseSpeedStepsPerSec),
    flowGain_(flowGain),
    minSpeedStepsPerSec_(minSpeedStepsPerSec),
    maxSpeedStepsPerSec_(maxSpeedStepsPerSec),
    enabled_(false) {}

void FlowSpeedController::setEnabled(bool enabled) {
  enabled_ = enabled;
}

bool FlowSpeedController::isEnabled() const {
  return enabled_;
}

void FlowSpeedController::setBaseSpeed(float baseSpeedStepsPerSec) {
  baseSpeedStepsPerSec_ = baseSpeedStepsPerSec;
}

void FlowSpeedController::setFlowGain(float flowGain) {
  flowGain_ = flowGain;
}

float FlowSpeedController::getBaseSpeed() const {
  return baseSpeedStepsPerSec_;
}

float FlowSpeedController::getFlowGain() const {
  return flowGain_;
}

float FlowSpeedController::compute(float flowLMin) const {
  if (!enabled_) {
    return 0.0f;
  }

  // Positive flow-speed relationship:
  // larger flow -> larger motor speed
  float speedCmd = baseSpeedStepsPerSec_ + flowGain_ * flowLMin;

  return clamp(speedCmd, minSpeedStepsPerSec_, maxSpeedStepsPerSec_);
}

float FlowSpeedController::clamp(float value, float minVal, float maxVal) const {
  if (value > maxVal) return maxVal;
  if (value < minVal) return minVal;
  return value;
}