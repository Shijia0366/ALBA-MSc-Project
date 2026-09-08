#include "PositionController.h"

PositionController::PositionController(float kp, float maxSpeedStepsPerSec)
    : kp_(kp), 
      maxSpeedStepsPerSec_(maxSpeedStepsPerSec), 
      targetRev_(0.0f) {}

void PositionController::setTargetRev(float targetRev) {
    targetRev_ = targetRev;
}

float PositionController::getTargetRev() const {
    return targetRev_;
}

void PositionController::setKp(float kp) {
    kp_ = kp;
}

float PositionController::getKp() const {
    return kp_;
}

float PositionController::compute(float currentPositionRev){
    float errorRev = targetRev_ - currentPositionRev;
    float cmdStepsPerSec = kp_ * errorRev;

    // Clamp command to max speed
    cmdStepsPerSec = clamp(cmdStepsPerSec, -maxSpeedStepsPerSec_, maxSpeedStepsPerSec_);

    // Deadband to prevent oscillation around target
    if (abs(errorRev) < 0.005f){
        cmdStepsPerSec = 0.0f;
    }
    return cmdStepsPerSec;
}

float PositionController::clamp(float value, float minVal, float maxVal) {
    if (value > maxVal) return maxVal;
    if (value < minVal) return minVal;
    return value;
}