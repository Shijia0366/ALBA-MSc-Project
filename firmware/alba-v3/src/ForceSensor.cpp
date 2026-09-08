#include "ForceSensor.h"
#include "ForceCalibration.h"

ForceSensor::ForceSensor(int pin)
    : pin_(pin),
      raw_(0),
      voltage_(0.0f)
{
}

void ForceSensor::begin() {
    pinMode(pin_, INPUT);
}

void ForceSensor::update() {
    raw_ = analogRead(pin_);
    // voltage_ = analogReadMilliVolts(pin_) / 1000.0f;
    voltage_ = 3.3f * raw_ / 4095.0f;
}

int ForceSensor::getRaw() const {
    return raw_;
}

float ForceSensor::getVoltage() const {
    return voltage_;
}

float ForceSensor::getResistanceOhm() const {
    const float VCC = 3.3f;
    // Measured/confirmed fixed divider resistor in the current hardware.
    const float RFIXED = 10000.0f;

    if (voltage_ <= 0.001f) {
        return 1.0e9f;
    }

    float v = voltage_;

    // Near 3.3 V the divider is saturated, and the resistance calculation
    // becomes numerically unstable. Treat anything very close to VCC as
    // saturated instead of allowing conductance/force to explode.
    if (v >= VCC - 0.05f) {
        v = VCC - 0.05f;
    }

    return RFIXED * (VCC / v - 1.0f);
}

float ForceSensor::getConductanceMicroS() const {
    float resistanceOhm = getResistanceOhm();

    if (resistanceOhm <= 0.0f || resistanceOhm >= 1.0e8f) {
        return 0.0f;
    }

    return 1000000.0f / resistanceOhm;
}

float ForceSensor::getForceN() const {
    const float raw = static_cast<float>(raw_);

    if (raw <= ForceCalibration::RAW_POINTS[0]) {
        return ForceCalibration::FORCE_POINTS_N[0];
    }

    // Piecewise-linear interpolation keeps every calibration point exact and
    // makes future recalibration independent of a particular curve model.
    for (size_t i = 1; i < ForceCalibration::POINT_COUNT; ++i) {
        if (raw <= ForceCalibration::RAW_POINTS[i]) {
            const float rawLow = ForceCalibration::RAW_POINTS[i - 1];
            const float rawHigh = ForceCalibration::RAW_POINTS[i];
            const float forceLow = ForceCalibration::FORCE_POINTS_N[i - 1];
            const float forceHigh = ForceCalibration::FORCE_POINTS_N[i];
            const float fraction = (raw - rawLow) / (rawHigh - rawLow);
            return forceLow + fraction * (forceHigh - forceLow);
        }
    }

    // Above the final measured point, extrapolate using only the final segment.
    // This is intentionally capped and must not be treated as calibrated data.
    const size_t last = ForceCalibration::POINT_COUNT - 1;
    const float rawLow = ForceCalibration::RAW_POINTS[last - 1];
    const float rawHigh = ForceCalibration::RAW_POINTS[last];
    const float forceLow = ForceCalibration::FORCE_POINTS_N[last - 1];
    const float forceHigh = ForceCalibration::FORCE_POINTS_N[last];
    const float slope = (forceHigh - forceLow) / (rawHigh - rawLow);
    float forceN = forceHigh + slope * (raw - rawHigh);

    if (forceN > ForceCalibration::MAX_EXTRAPOLATED_FORCE_N) {
        forceN = ForceCalibration::MAX_EXTRAPOLATED_FORCE_N;
    }
    return forceN;
}
