#pragma once

#include <Arduino.h>

class EncoderReader {
public:
  EncoderReader(int pinA, int pinB, float countsPerRev);

  void begin();
  void update(float dt);

  long getCount() const;
  float getPositionRev() const;
  float getVelocityRevPerSec() const;

  void zero();

  // Read-only, lifetime-stable counter for the STEP ISR's one-shot stop gate.
  // Aligned long reads are atomic on this project's 32-bit ESP32.
  const volatile long* counterForStepIsr() const { return &count_; }

private:
  int pinA_;
  int pinB_;

  float countsPerRev_;

  volatile long count_;
  volatile int lastEncoded_;

  long lastCount_;

  float positionRev_;
  float velocityRevPerSec_;

  static EncoderReader* instance_;
  static void IRAM_ATTR isrRouter();
  void IRAM_ATTR handleInterrupt();
};
