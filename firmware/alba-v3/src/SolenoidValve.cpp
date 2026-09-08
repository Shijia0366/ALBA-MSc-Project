#include "SolenoidValve.h"

SolenoidValve::SolenoidValve(int controlPin, bool activeHigh)
  : controlPin_(controlPin),
    activeHigh_(activeHigh),
    isOn_(false) {}

void SolenoidValve::begin() {
  pinMode(controlPin_, OUTPUT);
  off();  // safe default
}

void SolenoidValve::on() {
  isOn_ = true;
  writeState();
}

void SolenoidValve::off() {
  isOn_ = false;
  writeState();
}

void SolenoidValve::toggle() {
  isOn_ = !isOn_;
  writeState();
}

bool SolenoidValve::isOn() const {
  return isOn_;
}

void SolenoidValve::writeState() {
  if (activeHigh_) {
    digitalWrite(controlPin_, isOn_ ? HIGH : LOW);
  } else {
    digitalWrite(controlPin_, isOn_ ? LOW : HIGH);
  }
}