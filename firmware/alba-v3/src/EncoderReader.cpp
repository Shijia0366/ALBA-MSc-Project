#include "EncoderReader.h"

EncoderReader* EncoderReader::instance_ = nullptr;

EncoderReader::EncoderReader(int pinA, int pinB, float countsPerRev)
  : pinA_(pinA),
    pinB_(pinB),
    countsPerRev_(countsPerRev),
    count_(0),
    lastEncoded_(0),
    lastCount_(0),
    positionRev_(0.0f),
    velocityRevPerSec_(0.0f) {}

void EncoderReader::begin() {
  pinMode(pinA_, INPUT);
  pinMode(pinB_, INPUT);

  int A = digitalRead(pinA_);
  int B = digitalRead(pinB_);
  lastEncoded_ = (A << 1) | B;

  instance_ = this;

  attachInterrupt(digitalPinToInterrupt(pinA_), isrRouter, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB_), isrRouter, CHANGE);
}

void IRAM_ATTR EncoderReader::isrRouter() {
  if (instance_ != nullptr) {
    instance_->handleInterrupt();
  }
}

void IRAM_ATTR EncoderReader::handleInterrupt() {
  int A = digitalRead(pinA_);
  int B = digitalRead(pinB_);

  int encoded = (A << 1) | B;
  int sum = (lastEncoded_ << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    count_++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    count_--;
  }

  lastEncoded_ = encoded;
}

void EncoderReader::update(float dt) {
  if (dt <= 0.0f) {
    return;
  }

  noInterrupts();
  long countNow = count_;
  interrupts();

  long delta = countNow - lastCount_;
  lastCount_ = countNow;

  positionRev_ = countNow / countsPerRev_;
  velocityRevPerSec_ = (delta / countsPerRev_) / dt;
}

long EncoderReader::getCount() const {
  noInterrupts();
  long c = count_;
  interrupts();
  return c;
}

float EncoderReader::getPositionRev() const {
  return positionRev_;
}

float EncoderReader::getVelocityRevPerSec() const {
  return velocityRevPerSec_;
}

void EncoderReader::zero() {
  noInterrupts();
  count_ = 0;
  interrupts();

  lastCount_ = 0;
  positionRev_ = 0.0f;
  velocityRevPerSec_ = 0.0f;
}