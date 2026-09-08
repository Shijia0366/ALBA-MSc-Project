#pragma once
#include <Arduino.h>

class PositionController {
public:
    PositionController(float kp, float maxSpeedStepsPerSec);

    void setTargetRev(float targetRev);
    float getTargetRev() const;

    void setKp(float kp);
    float getKp() const;

    float compute(float currentPositionRev);

private:
    float kp_;
    float maxSpeedStepsPerSec_;
    float targetRev_;

    float clamp(float value, float minVal, float maxVal);
};