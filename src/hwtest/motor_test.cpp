// Wiring test for the Triangle maquette. Run with the straws disconnected.
//
// Upload with the micro_hwtest environment, then open the serial monitor:
//   pio run -e micro_hwtest -t upload
//   pio device monitor -e micro_hwtest
//
// Moves one motor at a time by a small angle and reports:
//  - how far the encoder moved, and whether that matches the steps sent
//    (a mismatch usually means the DIP switches aren't at 8000 steps/rev)
//  - whether the encoder counts the same way as the steps, or the opposite way
//  - index (Z) pulses, if YEL is wired
// Watch which way each pivot turns for a "+" move. The sequence sheet views it
// from the left elevation, with clockwise = forward.

#include <Arduino.h>
#include <AccelStepper.h>
#include <Encoder.h>
#include "config.h"  // same settings and pins as the main sketch

AccelStepper stepperLeft(AccelStepper::DRIVER, LEFT_PUL, LEFT_DIR);
AccelStepper stepperRight(AccelStepper::DRIVER, RIGHT_PUL, RIGHT_DIR);
Encoder encoderLeft(LEFT_ENC_A, LEFT_ENC_B);
Encoder encoderRight(RIGHT_ENC_A, RIGHT_ENC_B);

struct Axis {
  const char *name;
  AccelStepper &stepper;
  Encoder &encoder;
  uint8_t indexPin;
  long startSteps;
  long startCount;
  bool indexWasHigh;
};

Axis left = {"Left", stepperLeft, encoderLeft, LEFT_INDEX, 0, 0, false};
Axis right = {"Right", stepperRight, encoderRight, RIGHT_INDEX, 0, 0, false};

// PUL/DIR pins, for the 't' wire check
const uint8_t SIGNAL_PINS[] = {LEFT_PUL, LEFT_DIR, RIGHT_PUL, RIGHT_DIR};
const char *const SIGNAL_NAMES[] = {"Left PUL", "Left DIR", "Right PUL", "Right DIR"};
int signalCheckState = -1;  // -1 = all LOW, otherwise index of the one pin set HIGH

const int STEP_SIZES[] = {5, 10, 45, 90};
const int STEP_SIZE_COUNT = sizeof(STEP_SIZES) / sizeof(STEP_SIZES[0]);
int stepSizeIndex = 1;
Axis *moving = nullptr;

void printHelp() {
  Serial.println();
  Serial.println(F("Triangle wiring test (straws OFF)"));
  Serial.println(F("  l / L   left motor  + / - one step size"));
  Serial.println(F("  r / R   right motor + / - one step size"));
  Serial.print(F("  s       change step size (now "));
  Serial.print(STEP_SIZES[stepSizeIndex]);
  Serial.println(F(" deg at the pivot)"));
  Serial.println(F("  p       print positions"));
  Serial.println(F("  z       set both positions to zero"));
  Serial.println(F("  t       wire check: set one PUL/DIR pin HIGH at a time"));
  Serial.println(F("  x       stop immediately"));
  Serial.println(F("  h       this help"));
  Serial.println();
}

void initAxis(Axis &a) {
  a.stepper.setMinPulseWidth(MIN_PULSE_WIDTH);
  a.stepper.setMaxSpeed(PIVOT_SPEED * STEP_PER_PIVOT_DEGREE);
  a.stepper.setAcceleration(PIVOT_ACCEL * STEP_PER_PIVOT_DEGREE);
  a.stepper.setCurrentPosition(0);
  pinMode(a.indexPin, INPUT_PULLUP);  // pulled up so an unwired YEL stays quiet
  a.indexWasHigh = digitalRead(a.indexPin);
}

void printPosition(Axis &a) {
  Serial.print(a.name);
  Serial.print(F(": steps "));
  Serial.print(a.stepper.currentPosition());
  Serial.print(F(" ("));
  Serial.print(a.stepper.currentPosition() / STEP_PER_PIVOT_DEGREE, 1);
  Serial.print(F(" deg), encoder "));
  Serial.println(a.encoder.read());
}

void startMove(Axis &a, int direction) {
  if (moving) {
    Serial.println(F("Still moving, wait or press x"));
    return;
  }
  int degrees = STEP_SIZES[stepSizeIndex] * direction;
  a.startSteps = a.stepper.currentPosition();
  a.startCount = a.encoder.read();
  a.stepper.move(lround(degrees * STEP_PER_PIVOT_DEGREE));
  moving = &a;

  Serial.print(a.name);
  Serial.print(F(" moving "));
  if (degrees > 0) Serial.print('+');
  Serial.print(degrees);
  Serial.println(F(" deg at the pivot. Watch which way it turns."));
}

void finishMove(Axis &a) {
  long steps = a.stepper.currentPosition() - a.startSteps;
  long counts = a.encoder.read() - a.startCount;

  Serial.print(a.name);
  Serial.print(F(": sent "));
  Serial.print(steps);
  Serial.print(F(" steps, encoder moved "));
  Serial.print(counts);
  Serial.print(F(" counts (expected about "));
  Serial.print(static_cast<float>(steps) / STEP_PER_COUNT, 0);
  Serial.println(F(" either way)"));

  if (counts == 0) {
    Serial.println(F("  No encoder counts: check the encoder wiring (RED, BLK, BRN, BLU)"));
    Serial.println(F("  If the motor didn't move either, check the driver wiring and power"));
    return;
  }

  float stepsPerCount = static_cast<float>(steps) / counts;
  if (fabs(fabs(stepsPerCount) - STEP_PER_COUNT) > 0.2) {
    Serial.print(F("  Steps per count is "));
    Serial.print(fabs(stepsPerCount), 2);
    Serial.print(F(", expected "));
    Serial.print(STEP_PER_COUNT);
    Serial.println(F(": check DIP SW4-6 (all OFF = 8000 steps/rev), or the motor slipped"));
  } else {
    Serial.println(F("  Steps and encoder agree"));
  }

  if (stepsPerCount > 0) {
    Serial.println(F("  Encoder counts the SAME way as the steps"));
  } else {
    Serial.println(F("  Encoder counts the OPPOSITE way to the steps"));
  }
}

void checkIndex(Axis &a) {
  bool high = digitalRead(a.indexPin);
  if (high && !a.indexWasHigh) {
    Serial.print(a.name);
    Serial.print(F(" index pulse at encoder "));
    Serial.println(a.encoder.read());
  }
  a.indexWasHigh = high;
}

// Wire check: each press sets one PUL/DIR pin HIGH and the rest LOW, then all LOW,
// so each wire can be traced with a meter between the driver terminals and the Micro's GND.
void stepSignalCheck() {
  signalCheckState++;
  if (signalCheckState >= static_cast<int>(sizeof(SIGNAL_PINS))) signalCheckState = -1;

  for (uint8_t i = 0; i < sizeof(SIGNAL_PINS); i++) {
    digitalWrite(SIGNAL_PINS[i], static_cast<int>(i) == signalCheckState ? HIGH : LOW);
  }

  if (signalCheckState < 0) {
    Serial.println(F("All PUL/DIR pins LOW: every PUL and DIR terminal should read under 0.5 V"));
  } else {
    Serial.print(F("Only "));
    Serial.print(SIGNAL_NAMES[signalCheckState]);
    Serial.println(F(" HIGH: that terminal should read about 4-5 V, the other PUL/DIR terminals under 0.5 V"));
  }
  Serial.println(F("  OPTO terminals should read 5 V throughout. Press t for the next pin"));
}

void handleCommand(char c) {
  switch (c) {
    case 't':
      if (moving) {
        Serial.println(F("Stop first (x)"));
        break;
      }
      stepSignalCheck();
      break;
    case 'l': startMove(left, 1); break;
    case 'L': startMove(left, -1); break;
    case 'r': startMove(right, 1); break;
    case 'R': startMove(right, -1); break;
    case 's':
      stepSizeIndex = (stepSizeIndex + 1) % STEP_SIZE_COUNT;
      Serial.print(F("Step size "));
      Serial.print(STEP_SIZES[stepSizeIndex]);
      Serial.println(F(" deg at the pivot"));
      break;
    case 'p':
      printPosition(left);
      printPosition(right);
      break;
    case 'z':
      if (moving) {
        Serial.println(F("Stop first (x)"));
        break;
      }
      stepperLeft.setCurrentPosition(0);
      stepperRight.setCurrentPosition(0);
      encoderLeft.write(0);
      encoderRight.write(0);
      Serial.println(F("Zeroed"));
      break;
    case 'x':
      if (moving) {
        moving->stepper.setCurrentPosition(moving->stepper.currentPosition());  // also sets speed to 0
        Serial.println(F("Stopped"));
        finishMove(*moving);
        moving = nullptr;
      }
      break;
    case 'h':
    case '?':
      printHelp();
      break;
    default:
      break;  // ignore newlines etc.
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000) {}  // Micro: wait for the serial monitor, but not forever

  initAxis(left);
  initAxis(right);
  printHelp();
}

void loop() {
  if (Serial.available()) {
    handleCommand(Serial.read());
  }

  if (moving) {
    moving->stepper.run();
    if (moving->stepper.distanceToGo() == 0) {
      finishMove(*moving);
      moving = nullptr;
    }
  }

  checkIndex(left);
  checkIndex(right);
}
