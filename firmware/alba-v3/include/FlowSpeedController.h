#pragma once

#include <Arduino.h>

class FlowSpeedController {
public:
  FlowSpeedController(float baseSpeedStepsPerSec,
                      float flowGain,
                      float minSpeedStepsPerSec,
                      float maxSpeedStepsPerSec);

  void setEnabled(bool enabled);
  bool isEnabled() const;

  void setBaseSpeed(float baseSpeedStepsPerSec);
  void setFlowGain(float flowGain);

  float getBaseSpeed() const;
  float getFlowGain() const;

  float compute(float flowLMin) const;

private:
  float baseSpeedStepsPerSec_;
  float flowGain_;
  float minSpeedStepsPerSec_;
  float maxSpeedStepsPerSec_;
  bool enabled_;

  float clamp(float value, float minVal, float maxVal) const;
};