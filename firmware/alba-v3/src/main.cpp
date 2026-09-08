#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "StepperA4988.h"
#include "EncoderReader.h"
#include "FlowMeter.h"
#include "PositionController.h"
#include "SolenoidValve.h"
#include "FlowSpeedController.h"
#include "ForceSensor.h"
#include "ForceCalibration.h"


// ===================== Pins =====================

const int STEP_PIN = 18;
const int DIR_PIN  = 19;
const int EN_PIN   = 21;

// New PCB (SCH_Schematic1_20260819): ENC_A and ENC_B land on swapped GPIOs vs the
// old board. Header U11-10 = GPIO26 carries ENC_A, U11-9 = GPIO25 carries ENC_B.
// Signals pass through an SN74LVC14A Schmitt INVERTER (5V-tolerant inputs, 3V3 VCC),
// so A and B both arrive inverted -- which preserves quadrature direction.
// ENC_Z is wired to GPIO33 (U11-8) but unused by this firmware.
const int ENC_A_PIN = 26;
const int ENC_B_PIN = 25;

const int FLOW_PIN = 27;
const int VALVE_PIN = 32;

const int FORCE1_PIN = 34;
// ===================== Constants =====================

// AMT112S capacitive encoder, default 400 PPR, x4 quadrature decoding = 1600 counts/rev.
// Programmable 48..4096 PPR via AMT Viewpoint. VERIFY by hand-turning exactly one
// revolution and reading enc_count before trusting pos_rev / vel_rev_s.
const float ENCODER_COUNTS_PER_REV = 400.0f * 4.0f;

// Single flow calibration source: four 60 mL syringe trials produced
constexpr float FLOW_PULSES_PER_LITER = 1380.0f;

// ===================== Voiding Configuration =====================

// Runtime-configurable with: SET_VOLUME <mL>
float targetVoidVolumeMl = 300.0f;

// Reserved for the next-stage flow PID. It is not used to stop a void and it
// does not change the motor command in this target-volume-only implementation.
float targetFlowMlPerSec = 12.0f;

// ===================== Objects =====================

StepperA4988 motor(STEP_PIN, DIR_PIN, EN_PIN);
EncoderReader encoder(ENC_A_PIN, ENC_B_PIN, ENCODER_COUNTS_PER_REV);
FlowMeter flowMeter(FLOW_PIN, FLOW_PULSES_PER_LITER);
PositionController positionController(800.0f, 200.0f);
bool positionControlEnabled = false;
SolenoidValve valve(VALVE_PIN, true);
FlowSpeedController flowSpeedController(0.0f, 50.0f, 0.0f, 800.0f);
bool csvLoggingEnabled = false;
ForceSensor forceSensor1(FORCE1_PIN);

// ===================== Voiding Supervisor =====================

enum class VoidingState : uint8_t {
  IDLE,
  VOIDING,
  TARGET_REACHED
};

VoidingState voidingState = VoidingState::IDLE;
unsigned long voidStartPulseCount = 0;
unsigned long voidPulseDelta = 0;
unsigned long voidStartMillis = 0;
unsigned long voidElapsedMillis = 0;
float measuredFlowMlPerSec = 0.0f;
float flowErrorMlPerSec = 0.0f;
float voidedVolumeMl = 0.0f;
bool targetReachedLatched = false;
bool motorPositionStopEnabled = false;
float motorPositionStopRev = 0.0f;
long motorPositionStopCount = 0;
int8_t motorPositionStopDirection = 0;

// ===================== Timing =====================

unsigned long lastControlMicros = 0;
const unsigned long CONTROL_PERIOD_US = 10000;  // 100 Hz

unsigned long lastPrintMillis = 0;
const unsigned long PRINT_PERIOD_MS = 100;       // 10 Hz

const char* getVoidingStateName() {
  switch (voidingState) {
    case VoidingState::VOIDING:
      return "VOIDING";
    case VoidingState::TARGET_REACHED:
      return "TARGET_REACHED";
    case VoidingState::IDLE:
    default:
      return "IDLE";
  }
}

// Placeholder for the next-stage flow PID integral/derivative reset. The
// current FlowSpeedController has no stored PID state, so only the reserved
// error value needs clearing in this version.
void resetFlowPidState() {
  flowErrorMlPerSec = 0.0f;
}

// Reuses the existing 'h' command's stop semantics: no more STEP pulses, flow
// controller off, position controller off, and active-high valve commanded
// OFF. Driver enable state is intentionally preserved to avoid changing the
// existing hold/release behavior without hardware confirmation.
void stopVoidingActuatorsSafely() {
  flowSpeedController.setEnabled(false);
  positionControlEnabled = false;
  motorPositionStopEnabled = false;
  motor.stop();
  motor.clearPositionStop();
  valve.off();
}

void updateVoidingMeasurements() {
  measuredFlowMlPerSec = flowMeter.getFilteredFlowLMin() * (1000.0f / 60.0f);

  // Reserved for the future flow PID. Volume error is deliberately not used
  // as a motor-control input.
  flowErrorMlPerSec = targetFlowMlPerSec - measuredFlowMlPerSec;

  if (voidingState != VoidingState::VOIDING) {
    return;
  }

  // getPulseCount() performs the same interrupt-protected atomic read used by
  // the existing FlowMeter code. Unsigned subtraction also tolerates wraparound.
  const unsigned long currentPulseCount = flowMeter.getPulseCount();
  voidPulseDelta = currentPulseCount - voidStartPulseCount;
  voidedVolumeMl =
      static_cast<float>(voidPulseDelta) * 1000.0f / FLOW_PULSES_PER_LITER;
  voidElapsedMillis = millis() - voidStartMillis;
}

bool startVoiding() {
  if (!isfinite(targetVoidVolumeMl) || targetVoidVolumeMl <= 0.0f) {
    Serial.println("ERROR: target void volume must be greater than 0 mL");
    return false;
  }

  // Establish a baseline without clearing the ISR-owned lifetime pulse count.
  voidStartPulseCount = flowMeter.getPulseCount();
  voidPulseDelta = 0;
  voidedVolumeMl = 0.0f;
  voidStartMillis = millis();
  voidElapsedMillis = 0;
  targetReachedLatched = false;
  voidingState = VoidingState::VOIDING;
  resetFlowPidState();

  Serial.print("VOID_START: target_mL=");
  Serial.print(targetVoidVolumeMl, 3);
  Serial.print(", baseline_pulses=");
  Serial.println(voidStartPulseCount);
  return true;
}

void finishVoiding() {
  if (voidingState != VoidingState::VOIDING || targetReachedLatched) {
    return;
  }

  targetReachedLatched = true;
  voidingState = VoidingState::TARGET_REACHED;
  stopVoidingActuatorsSafely();
  resetFlowPidState();

  float overshootMl = voidedVolumeMl - targetVoidVolumeMl;
  if (overshootMl < 0.0f) {
    overshootMl = 0.0f;
  }

  // This completion block is reached once per void because the state and latch
  // are changed before printing.
  Serial.println();
  Serial.println("VOID_COMPLETE: target volume reached");
  Serial.print("  actual_volume_mL=");
  Serial.println(voidedVolumeMl, 3);
  Serial.print("  target_volume_mL=");
  Serial.println(targetVoidVolumeMl, 3);
  Serial.print("  pulse_delta=");
  Serial.println(voidPulseDelta);
  Serial.print("  elapsed_s=");
  Serial.println(voidElapsedMillis / 1000.0f, 3);
  Serial.print("  overshoot_mL=");
  Serial.println(overshootMl, 3);
}

void stopVoidingManually() {
  updateVoidingMeasurements();
  stopVoidingActuatorsSafely();
  resetFlowPidState();

  if (voidingState == VoidingState::VOIDING) {
    voidingState = VoidingState::IDLE;
  }
}

void updateMotorPositionStop() {
  if (!motorPositionStopEnabled) {
    return;
  }

  const long currentCount = encoder.getCount();
  const float currentPositionRev = currentCount / ENCODER_COUNTS_PER_REV;
  const bool targetReached =
      motor.positionStopReached() || motorPositionStopDirection == 0 ||
      (motorPositionStopDirection > 0 && currentCount >= motorPositionStopCount) ||
      (motorPositionStopDirection < 0 && currentCount <= motorPositionStopCount);

  if (!targetReached) {
    return;
  }

  const float reachedTargetRev = motorPositionStopRev;
  stopVoidingManually();

  Serial.println();
  Serial.print("MOTOR_POS_STOP_REACHED: target_rev=");
  Serial.print(reachedTargetRev, 6);
  Serial.print(", actual_rev=");
  Serial.println(currentPositionRev, 6);
}

void updateVoidingSupervisor() {
  updateVoidingMeasurements();

  // Future PID path (not implemented here):
  // flowErrorMlPerSec = targetFlowMlPerSec - measuredFlowMlPerSec;

  // Current task: target volume is only the upper-level end condition.
  if (voidingState == VoidingState::VOIDING &&
      !targetReachedLatched &&
      voidedVolumeMl >= targetVoidVolumeMl) {
    finishVoiding();
  }
}

void printVoidingStatus() {
  updateVoidingMeasurements();
  Serial.println();
  Serial.print("VOID_STATUS: state=");
  Serial.print(getVoidingStateName());
  Serial.print(", flow_mL_s=");
  Serial.print(measuredFlowMlPerSec, 3);
  Serial.print(", voided_mL=");
  Serial.print(voidedVolumeMl, 3);
  Serial.print(", target_mL=");
  Serial.print(targetVoidVolumeMl, 3);
  Serial.print(", pulse_delta=");
  Serial.print(voidPulseDelta);
  Serial.print(", elapsed_s=");
  Serial.print(voidElapsedMillis / 1000.0f, 3);
  Serial.print(", target_reached=");
  Serial.println(targetReachedLatched ? 1 : 0);
}

// ===================== Serial Commands =====================

void handleSingleCharCommand(char cmd) {
  if (cmd == 'e') {
    motor.enable();
    Serial.println("Motor enabled");
  }

  else if (cmd == 'd') {
    motor.disable();
    Serial.println("Motor disabled");
  }

  else if (cmd == 'f') {
    motor.setSpeedStepsPerSec(200.0f);
    Serial.println("Command: forward 200 steps/s");
  }

  else if (cmd == 'b') {
    motor.setSpeedStepsPerSec(-200.0f);
    Serial.println("Command: backward 200 steps/s");
  }

  else if (cmd == 's') {
    positionControlEnabled = false;
    motor.stop();
    Serial.println("Command: stop, position control disabled");
  }

  else if (cmd == '+') {
    float current = motor.getCommandSpeedStepsPerSec();

    if (current == 0.0f) {
      current = 200.0f;
    } else {
      current *= 1.2f;
    }

    motor.setSpeedStepsPerSec(current);

    Serial.print("steps/s = ");
    Serial.println(motor.getCommandSpeedStepsPerSec());
  }

  else if (cmd == '-') {
    float current = motor.getCommandSpeedStepsPerSec();
    current *= 0.8f;

    motor.setSpeedStepsPerSec(current);

    Serial.print("steps/s = ");
    Serial.println(motor.getCommandSpeedStepsPerSec());
  }

  else if (cmd == 'z') {
    if (motorPositionStopEnabled) {
      Serial.println("ERROR: use SET_POS_STOP OFF before zeroing an armed position stop");
      return;
    }
    encoder.zero();
    Serial.println("Encoder zeroed");
  }

  else if (cmd == 'r') {
    if (voidingState == VoidingState::VOIDING) {
      Serial.println("ERROR: cannot reset lifetime flow pulses while VOIDING");
    } else {
      flowMeter.reset();
      voidStartPulseCount = 0;
      voidPulseDelta = 0;
      voidedVolumeMl = 0.0f;
      voidElapsedMillis = 0;
      targetReachedLatched = false;
      voidingState = VoidingState::IDLE;
      Serial.println("Flowmeter lifetime pulse count reset");
    }
  }

  else if (cmd == 'p') {
    positionControlEnabled = !positionControlEnabled;

    if (positionControlEnabled) {
      positionController.setTargetRev(encoder.getPositionRev());
      Serial.print("Position control enabled. Holding current position: ");
      Serial.println(positionController.getTargetRev(), 4);
    } else {
      motor.stop();
      Serial.println("Position control disabled");
    }
  }

  else if (cmd == '1') {
    positionController.setTargetRev(1.0f);
    Serial.println("Position control target: 1.0 rev");
  }

  else if (cmd == '2') {
    positionController.setTargetRev(2.0f);
    Serial.println("Position control target: 2.0 rev");
  }

  else if (cmd == '0') {
    positionController.setTargetRev(0.0f);
    Serial.println("Position control target: 0.0 rev");
  }

  else if (cmd == 'v') {
    valve.on();
    Serial.println("Valve ON");
  }

  else if (cmd == 'c') {
    valve.off();
    Serial.println("Valve OFF");
  }

  else if (cmd == 'x') {
    valve.toggle();
    Serial.print("Valve toggled: ");
    Serial.println(valve.isOn() ? "ON" : "OFF");
  }

  else if (cmd == 'a') {
    startVoiding();
  }

  else if (cmd == 'g') {
    if (voidingState == VoidingState::VOIDING || startVoiding()) {
      valve.on();
      flowSpeedController.setEnabled(true);
      Serial.println("Flow-speed controller ON, valve opened");
    }
  }

  else if (cmd == 'h') {
    stopVoidingManually();
    Serial.println("Flow-speed controller OFF, valve closed");
  }

  else if (cmd == 'L') {
    csvLoggingEnabled = !csvLoggingEnabled;

    if (csvLoggingEnabled) {
      Serial.println();
      Serial.println("time_s,encoder_count,motor_pos_rev,motor_vel_rev_s,motor_cmd_steps_s,valve_state,void_state,flow_pulses,flow_pulse_interval_us,flow_mL_s,flow_mL_s_filtered,voided_volume_mL");
    } else {
      Serial.println();
      Serial.println("CSV logging OFF, returning to live status line");
    }
  }
}

void handleTextCommand(char* command) {
  const size_t commandLength = strlen(command);
  size_t trimmedLength = commandLength;
  while (trimmedLength > 0 && command[trimmedLength - 1] == ' ') {
    command[--trimmedLength] = '\0';
  }

  if (strncmp(command, "SET_VOLUME", strlen("SET_VOLUME")) == 0) {
    const char* valueText = command + strlen("SET_VOLUME");
    while (*valueText == ' ') {
      ++valueText;
    }

    char* endPtr = nullptr;
    const float requestedVolumeMl = strtof(valueText, &endPtr);
    while (endPtr != nullptr && *endPtr == ' ') {
      ++endPtr;
    }

    if (valueText == endPtr || endPtr == nullptr || *endPtr != '\0' ||
        !isfinite(requestedVolumeMl) || requestedVolumeMl <= 0.0f) {
      Serial.println("ERROR: use SET_VOLUME <mL> with a value greater than 0");
      return;
    }

    if (voidingState == VoidingState::VOIDING) {
      Serial.println("ERROR: stop the current void before changing target volume");
      return;
    }

    targetVoidVolumeMl = requestedVolumeMl;
    Serial.print("Target void volume set to ");
    Serial.print(targetVoidVolumeMl, 3);
    Serial.println(" mL");
  }
  else if (strncmp(command, "SET_SPEED", strlen("SET_SPEED")) == 0) {
    const char* valueText = command + strlen("SET_SPEED");
    while (*valueText == ' ') {
      ++valueText;
    }

    char* endPtr = nullptr;
    const float requestedSpeedStepsPerSec = strtof(valueText, &endPtr);
    while (endPtr != nullptr && *endPtr == ' ') {
      ++endPtr;
    }

    if (valueText == endPtr || endPtr == nullptr || *endPtr != '\0' ||
        !isfinite(requestedSpeedStepsPerSec) ||
        requestedSpeedStepsPerSec < -3000.0f ||
        requestedSpeedStepsPerSec > 3000.0f) {
      Serial.println("ERROR: use SET_SPEED <steps/s> with a value from -3000 to 3000");
      return;
    }

    motor.setSpeedStepsPerSec(requestedSpeedStepsPerSec);
    Serial.print("Motor speed command set to ");
    Serial.print(motor.getCommandSpeedStepsPerSec(), 3);
    Serial.println(" steps/s");
  }
  else if (strncmp(command, "SET_POS_STOP", strlen("SET_POS_STOP")) == 0) {
    const char* valueText = command + strlen("SET_POS_STOP");
    while (*valueText == ' ') {
      ++valueText;
    }

    if (strcmp(valueText, "OFF") == 0) {
      motorPositionStopEnabled = false;
      // If it already tripped, cancel its residual target before releasing
      // the ISR gate. OFF alone must not restart a previously stopped motor.
      if (motor.positionStopReached()) motor.stop();
      motor.clearPositionStop();
      Serial.println("Motor position auto-stop disabled");
      return;
    }

    char* endPtr = nullptr;
    const float requestedStopRev = strtof(valueText, &endPtr);
    while (endPtr != nullptr && *endPtr == ' ') {
      ++endPtr;
    }

    if (valueText == endPtr || endPtr == nullptr || *endPtr != '\0' ||
        !isfinite(requestedStopRev)) {
      Serial.println("ERROR: use SET_POS_STOP <rev> or SET_POS_STOP OFF");
      return;
    }

    const long startCount = encoder.getCount();
    const double targetCount = static_cast<double>(requestedStopRev) * ENCODER_COUNTS_PER_REV;
    const int8_t direction = targetCount > startCount ? 1 : (targetCount < startCount ? -1 : 0);
    const double stopCount = direction > 0 ? ceil(targetCount) : floor(targetCount);
    if (stopCount < INT32_MIN || stopCount > INT32_MAX) {
      Serial.println("ERROR: position target exceeds the encoder counter range");
      return;
    }
    if (motor.positionStopReached()) motor.stop();
    motorPositionStopCount = static_cast<long>(stopCount);
    motorPositionStopDirection = direction;
    motor.armPositionStop(encoder.counterForStepIsr(), motorPositionStopCount, direction);
    motorPositionStopRev = requestedStopRev;
    motorPositionStopEnabled = true;
    Serial.print("Motor position auto-stop set to ");
    Serial.print(motorPositionStopRev, 6);
    Serial.println(" rev");
  }
  else if (strcmp(command, "STOP_VOID") == 0) {
    stopVoidingManually();
    Serial.println("Voiding stopped manually; motor command stopped and valve closed");
  }
  else if (strcmp(command, "STATUS") == 0) {
    printVoidingStatus();
  }
  else {
    Serial.print("ERROR: unknown text command: ");
    Serial.println(command);
  }
}

void handleSerial() {
  static char textCommandBuffer[64];
  static size_t textCommandLength = 0;
  static bool receivingTextCommand = false;
  static bool discardingTextCommand = false;

  while (Serial.available()) {
    const char cmd = static_cast<char>(Serial.read());

    if (discardingTextCommand) {
      if (cmd == '\r' || cmd == '\n') {
        discardingTextCommand = false;
      }
      continue;
    }

    if (receivingTextCommand) {
      if (cmd == '\r' || cmd == '\n') {
        textCommandBuffer[textCommandLength] = '\0';
        handleTextCommand(textCommandBuffer);
        textCommandLength = 0;
        receivingTextCommand = false;
      } else if (textCommandLength < sizeof(textCommandBuffer) - 1) {
        textCommandBuffer[textCommandLength++] = cmd;
      } else {
        Serial.println("ERROR: text command too long");
        textCommandLength = 0;
        receivingTextCommand = false;
        discardingTextCommand = true;
      }
      continue;
    }

    // All supported text commands begin with uppercase S. Existing one-byte
    // commands remain immediate and do not require a line ending.
    if (cmd == 'S') {
      textCommandBuffer[0] = cmd;
      textCommandLength = 1;
      receivingTextCommand = true;
    } else if (cmd != '\r' && cmd != '\n') {
      handleSingleCharCommand(cmd);
    }
  }
}

// ===================== Periodic Sensor Update =====================

void updateSensors() {
  unsigned long now = micros();

  if (now - lastControlMicros < CONTROL_PERIOD_US) {
    return;
  }

  float dt = (now - lastControlMicros) / 1000000.0f;
  lastControlMicros = now;

  encoder.update(dt);
  flowMeter.update(dt);
  forceSensor1.update();
}

// ===================== Print Data =====================

void printData() {
  if (millis() - lastPrintMillis < PRINT_PERIOD_MS) {
    return;
  }

  lastPrintMillis = millis();

  if (csvLoggingEnabled) {
    const unsigned long nowMillis = millis();
    const uint32_t flowPulseCount = flowMeter.getPulseCount();
    const uint32_t flowPulseIntervalMicros = flowMeter.getPulseIntervalMicros();
    const float filteredFlowLMin = flowMeter.getFilteredFlowLMin();
    const float filteredFlowMlPerSec = filteredFlowLMin * (1000.0f / 60.0f);

    Serial.print(nowMillis / 1000.0f, 4);
    Serial.print(",");
    Serial.print(encoder.getCount());
    Serial.print(",");
    Serial.print(encoder.getPositionRev(), 6);
    Serial.print(",");
    Serial.print(encoder.getVelocityRevPerSec(), 6);
    Serial.print(",");
    Serial.print(motor.getCommandSpeedStepsPerSec(), 3);
    Serial.print(",");
    Serial.print(valve.isOn() ? 1 : 0);
    Serial.print(",");
    Serial.print(getVoidingStateName());
    Serial.print(",");
    Serial.print(flowPulseCount);
    Serial.print(",");
    Serial.print(flowPulseIntervalMicros);
    Serial.print(",");
    Serial.print(measuredFlowMlPerSec, 6);
    Serial.print(",");
    Serial.print(filteredFlowMlPerSec, 6);
    Serial.print(",");
    Serial.println(voidedVolumeMl, 6);
    return;
  }

  Serial.print("\r");  // move cursor back to start of same line

  Serial.print("t_ms=");
  Serial.print(millis());

  Serial.print(", enc_count=");
  Serial.print(encoder.getCount());

  Serial.print(", pos_rev=");
  Serial.print(encoder.getPositionRev(), 4);

  Serial.print(", vel_rev_s=");
  Serial.print(encoder.getVelocityRevPerSec(), 4);

  Serial.print(", flow_pulses=");
  Serial.print(flowMeter.getPulseCount());

  Serial.print(", cmd_steps_s=");
  Serial.print(motor.getCommandSpeedStepsPerSec(), 1);

  Serial.print(", target_rev=");
  Serial.print(positionController.getTargetRev(), 4);

  Serial.print(", pos_ctrl=");
  Serial.print(positionControlEnabled ? 1 : 0);

  Serial.print(", valve=");
  Serial.print(valve.isOn() ? 1 : 0);

  Serial.print(", state=");
  Serial.print(getVoidingStateName());

  Serial.print(", flow_mL_s=");
  Serial.print(measuredFlowMlPerSec, 3);

  Serial.print(", voided_mL=");
  Serial.print(voidedVolumeMl, 2);

  Serial.print(", target_mL=");
  Serial.print(targetVoidVolumeMl, 2);

  Serial.print(", void_pulses=");
  Serial.print(voidPulseDelta);

  Serial.print(", void_t_s=");
  Serial.print(voidElapsedMillis / 1000.0f, 2);

  Serial.print(", reached=");
  Serial.print(targetReachedLatched ? 1 : 0);

  Serial.print("                                        ");  // clears leftover old characters
}

// ===================== Setup / Loop =====================

void setup() {
  Serial.begin(115200);
  delay(1000);

  motor.begin();
  encoder.begin();
  flowMeter.begin();
  valve.begin();
  forceSensor1.begin();

  lastControlMicros = micros();

  Serial.println();
  Serial.println("FIRMWARE BUILD ID: main / fixed-ID flow capture / 2026-08-28");
  Serial.print("PINS -> STEP=GPIO");   Serial.print(STEP_PIN);
  Serial.print("  DIR=GPIO");          Serial.print(DIR_PIN);
  Serial.print("  EN=GPIO");           Serial.print(EN_PIN);
  Serial.print("  ENC_A=GPIO");        Serial.print(ENC_A_PIN);
  Serial.print("  ENC_B=GPIO");        Serial.print(ENC_B_PIN);
  Serial.print("  FLOW=GPIO");         Serial.print(FLOW_PIN);
  Serial.print("  VALVE=GPIO");        Serial.print(VALVE_PIN);
  Serial.print("  FORCE=GPIO");        Serial.println(FORCE1_PIN);
  Serial.print("ENCODER_COUNTS_PER_REV = "); Serial.println(ENCODER_COUNTS_PER_REV);
  Serial.print("FLOW pulses/L (preliminary) = "); Serial.println(FLOW_PULSES_PER_LITER, 3);
  Serial.print("TARGET void volume = "); Serial.print(targetVoidVolumeMl, 3); Serial.println(" mL");
  Serial.print("TARGET flow reference (future PID) = "); Serial.print(targetFlowMlPerSec, 3); Serial.println(" mL/s");
  Serial.print("FORCE CALIBRATION: ");  Serial.println(ForceCalibration::VERSION);
  Serial.print("  points=");            Serial.print(ForceCalibration::POINT_COUNT);
  Serial.print("  raw range 0..");      Serial.print(ForceCalibration::RAW_POINTS[ForceCalibration::POINT_COUNT-1], 1);
  Serial.print("  ->  0..");            Serial.print(ForceCalibration::FORCE_POINTS_N[ForceCalibration::POINT_COUNT-1], 3);
  Serial.println(" N (beyond this = extrapolated, NOT calibrated)");
  Serial.println();
  Serial.println("PlatformIO modular motor + encoder + flowmeter test ready.");
  Serial.println("Commands:");
  Serial.println("e = enable motor");
  Serial.println("d = disable motor");
  Serial.println("f = forward");
  Serial.println("b = backward");
  Serial.println("s = stop");
  Serial.println("+ = faster");
  Serial.println("- = slower");
  Serial.println("z = zero encoder");
  Serial.println("r = reset lifetime flow pulse count (not while VOIDING)");
  Serial.println("p = toggle position control");
  Serial.println("1 = set position target to 1.0 rev");
  Serial.println("2 = set position target to 2.0 rev");
  Serial.println("0 = set position target to 0.0 rev");
  Serial.println("v = valve ON");
  Serial.println("c = valve OFF");
  Serial.println("x = toggle valve");
  Serial.println("a = start/restart relative-volume recording only");
  Serial.println("g = start flow-speed controller and open valve");
  Serial.println("h = stop flow-speed controller and close valve");
  Serial.println("L = toggle CSV logging mode");
  Serial.println("SET_VOLUME <mL> = set target volume (must be > 0)");
  Serial.println("SET_SPEED <steps/s> = set exact motor command (-3000 to 3000)");
  Serial.println("SET_POS_STOP <rev|OFF> = set or disable motor position auto-stop");
  Serial.println("STOP_VOID = stop voiding, motor command, and valve");
  Serial.println("STATUS = print voiding status");
  Serial.println("Text commands must end with CR or LF (press Enter)");
}

void loop() {
  handleSerial();
  updateSensors();
  updateVoidingSupervisor();
  updateMotorPositionStop();

  if (positionControlEnabled) {
    float currentPosRev = encoder.getPositionRev();
    float cmd = positionController.compute(currentPosRev);
    motor.setSpeedStepsPerSec(cmd);
  }

  if (flowSpeedController.isEnabled()) {
    valve.on();

    float q = flowMeter.getFlowLMin();
    float uCmd = flowSpeedController.compute(q);

    motor.setSpeedStepsPerSec(uCmd);
  }

  motor.update();
  printData();
}
