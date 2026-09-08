#pragma once

#include <Arduino.h>

class SolenoidValve {
public:
  SolenoidValve(int controlPin, bool activeHigh = true);

  void begin();

  void on();
  void off();
  void toggle();

  bool isOn() const;

private:
  int controlPin_;
  bool activeHigh_;
  bool isOn_;

  void writeState();
};