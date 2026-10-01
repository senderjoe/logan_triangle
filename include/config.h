#pragma once

#include <Arduino.h>

// Hardware and motion settings. The choreography itself is in sequence.h.

#define STEP_PER_REVOLUTION 8000  // motor steps per rev, DM320T DIP SW4-6 all OFF
#define COUNT_PER_REVOLUTION 4000 // encoder counts per motor rev
#define GEAR_RATIO 1.0            // belt: 16T on motor, 16T on pivot, 194 mm 2GT belt (was 36T:16T, 2.25x, until Oct 2026)

// All angles, speeds and accelerations are at the pivot, not the motor
#define STEP_PER_PIVOT_REVOLUTION (STEP_PER_REVOLUTION / GEAR_RATIO)
#define STEP_PER_PIVOT_DEGREE (STEP_PER_PIVOT_REVOLUTION / 360.0)
#define PIVOT_SPEED 12   // degrees/s, from the sequence sheet: 90 deg in 9 s
#define PIVOT_ACCEL 12   // degrees/s/s, peak: moves ease in and out (S-curve, see main.cpp)
#define RUNSPEED (PIVOT_SPEED * STEP_PER_PIVOT_DEGREE)
#define ACCELL (PIVOT_ACCEL * STEP_PER_PIVOT_DEGREE)

#define MIN_PULSE_WIDTH 20 // microseconds, DM320T needs at least 7.5

// If the encoder and the step count disagree by more than this (pivot degrees) at the end of a stage,
// something is jammed or an encoder has failed, so the sequence stops rather than "correcting"
#define MAX_CORRECTION_DEGREES 5

const long STEP_PER_COUNT = STEP_PER_REVOLUTION / COUNT_PER_REVOLUTION;  // motor steps per encoder count (2)

// Both encoders count the opposite way to the steps (checked with src/hwtest 26 Sep 2026:
// + steps turns the left pivot anticlockwise and the right pivot clockwise, viewed from the left elevation)
const int ENCODER_SIGN = -1;

// The drivers are held off until r, then switched on. Wait this long for the jerk as they take
// hold of the motors before reading the encoders (ms).
#define ENABLE_SETTLE_MS 500

// Pin map (Arduino Micro). Two layouts:
//  - the prototype wiring, see src/docs/wiring.pdf
//  - the carrier board, see src/docs/board.pdf. Each connector sits on the same strips as the
//    Micro pins it uses, so the pins are chosen to line up with the connectors.
#define CARRIER_BOARD true   // true for the carrier board, or the prototype wired from wiring-v6.pdf

// DRIVER_ENABLE switches both drivers' ENA inputs through Q1 (2N2222, with a pull-up to 5V).
// LOW = drivers on. HIGH, or not driven (at power-on, during an upload or a reset) = drivers off,
// motors free.
// The index (Z) pins are only used by the test sketch (src/hwtest).
#if CARRIER_BOARD
// Reserved, not used yet: A5 = START button (to GND)
const uint8_t LEFT_PUL = 15, LEFT_DIR = 14;                       // D15, D14
const uint8_t RIGHT_PUL = A3, RIGHT_DIR = A2;
const uint8_t LEFT_ENC_A = 0, LEFT_ENC_B = 1, LEFT_INDEX = 2;     // A and B both on interrupt pins
const uint8_t RIGHT_ENC_A = 7, RIGHT_ENC_B = 6, RIGHT_INDEX = 10; // A on an interrupt pin
const uint8_t DRIVER_ENABLE = A0;
#else
// Reserved, not wired or used yet: D10 = START button (to GND)
const uint8_t LEFT_PUL = A4, LEFT_DIR = A3;
const uint8_t RIGHT_PUL = A1, RIGHT_DIR = A0;
const uint8_t LEFT_ENC_A = 3, LEFT_ENC_B = 4, LEFT_INDEX = 5;    // A on an interrupt pin
const uint8_t RIGHT_ENC_A = 7, RIGHT_ENC_B = 8, RIGHT_INDEX = 9;
const uint8_t DRIVER_ENABLE = A2;
#endif

inline long pivotDegreesToSteps(float degrees) {
  return lround(degrees * STEP_PER_PIVOT_DEGREE);
}

inline float countsToPivotDegrees(long counts) {
  return counts * STEP_PER_COUNT / STEP_PER_PIVOT_DEGREE;
}
