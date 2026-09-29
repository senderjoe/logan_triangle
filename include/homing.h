#pragma once

#include <AccelStepper.h>
#include <Encoder.h>

// One motor with its encoder
struct Axis {
  const char *name;
  AccelStepper &stepper;
  Encoder &encoder;
  uint8_t indexPin;
  int forward;  // sign of a forward move in steps (sequence sheet direction): left -1, right +1
};

void initHoming(Axis &left, Axis &right);  // call once in setup()
bool isCalibrated();                        // true once step 2 has been saved

// Home both motors on their index pulses and move to the start pose. Blocks; x stops it.
bool homeAll(Axis &left, Axis &right);

// Calibration step 1 (key i): straws off. Each motor turns until its index pulse, and the
// encoders count from there.
void runFindIndex(Axis &left, Axis &right);

// Calibration step 2 (key o): saves the current pose as the start pose, as each pulse's distance
// from it, in EEPROM. Use after step 1, or after homing to move the start pose: switch the motor
// power off and set the straws by hand first. The Micro must stay powered from step 1 or homing
// until then. Returns true if saved.
bool runSetStartPose(Axis &left, Axis &right);
