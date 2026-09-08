#pragma once
#include <Arduino.h>

class FlowMeter {
public:
    FlowMeter(int pin, float pulsesPerLiter);

    void begin();
    void update(float dt);

    uint32_t getPulseCount() const;
    uint32_t getLastPulseMicros() const;
    uint32_t getPulseIntervalMicros() const;
    float getRawFlowLMin() const;
    float getFilteredFlowLMin() const;
    float getFlowLMin() const;

    void reset();

private:
    int pin_;
    float pulsesPerLiter_;

    volatile uint32_t pulseCount_;
    volatile uint32_t lastPulseMicros_;
    volatile uint32_t pulseIntervalMicros_;
    volatile uint32_t lastEdgeMicros_;
    volatile bool edgeSeen_;
    uint32_t minPulseIntervalMicros_;
    uint32_t lastProcessedPulseCount_;

    float rawFlowLMin_;
    float filteredFlowLMin_;

    static FlowMeter* instance_;
    static void IRAM_ATTR isrRouter();
    void IRAM_ATTR handleInterrupt();
};
