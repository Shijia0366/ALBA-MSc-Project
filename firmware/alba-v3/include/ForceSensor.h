#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

#include <Arduino.h>

class ForceSensor {
public:
    ForceSensor(int pin);

    void begin();

    void update();

    int getRaw() const;
    float getVoltage() const;
    float getResistanceOhm() const;
    float getConductanceMicroS() const;
    float getForceN() const;

private:
    int pin_;
    int raw_;
    float voltage_;
};

#endif