#pragma once

#include <Arduino.h>

// Hardware and motion settings. The choreography itself is in sequence.h.

#define STEP_PER_REVOLUTION 8000  // motor steps per rev, DM320T DIP SW4-6 all OFF
#define COUNT_PER_REVOLUTION 4000 // encoder counts per motor rev
#define GEAR_RATIO (36.0 / 16.0)  // belt: 36T on motor, 16T on pivot, so the pivot turns 2.25x the motor

// All angles, speeds and accelerations are at the pivot, not the motor
#define STEP_PER_PIVOT_REVOLUTION (STEP_PER_REVOLUTION / GEAR_RATIO)
#define STEP_PER_PIVOT_DEGREE (STEP_PER_PIVOT_REVOLUTION / 360.0)
#define PIVOT_SPEED 10   // degrees/s, from the sequence sheet: 90 deg in 9 s
#define PIVOT_ACCEL 5   // degrees/s/s
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

// Homing on the encoder index pulse (see homing.cpp)
#define HOME_AT_POWER_ON true     // home automatically at power-on, once calibrated
#define HOME_DELAY_MS 2000        // wait this long after power-on for the drivers to start
#define HOME_SPEED 10             // degrees/s while searching with the straws on
#define HOME_ACCEL 20             // degrees/s/s while homing
#define HOME_LATCH_SPEED 2        // degrees/s for the final approach onto the pulse
#define HOME_BACKOFF 5            // degrees: back off this far before the final approach
#define HOME_WINDOW 15            // degrees: search this far either side of where the pulse should be.
                                  // At power-on the straws must be within this of the start pose.
#define FIND_INDEX_SPEED 60       // degrees/s for calibration step 1 (straws off)
#define FIND_INDEX_ACCEL 60       // degrees/s/s for calibration step 1

// Pin map (Arduino Micro). Two layouts:
//  - the prototype wiring, see src/docs/wiring.pdf
//  - the carrier board, see src/docs/board.pdf. Each connector sits on the same strips as the
//    Micro pins it uses, so the pins are chosen to line up with the connectors.
#define CARRIER_BOARD false  // set to true once the carrier board is built

#if CARRIER_BOARD
// Reserved, not used yet: A0 = right ENA, A4 = left ENA, A5 = START button (to GND)
const uint8_t LEFT_PUL = 15, LEFT_DIR = 14;                       // D15, D14
const uint8_t RIGHT_PUL = A3, RIGHT_DIR = A2;
const uint8_t LEFT_ENC_A = 0, LEFT_ENC_B = 1, LEFT_INDEX = 2;     // A and B both on interrupt pins
const uint8_t RIGHT_ENC_A = 7, RIGHT_ENC_B = 6, RIGHT_INDEX = 10; // A on an interrupt pin
#else
// Reserved, not wired or used yet: A2 = right ENA, A5 = left ENA, D10 = START button (to GND)
const uint8_t LEFT_PUL = A4, LEFT_DIR = A3;
const uint8_t RIGHT_PUL = A1, RIGHT_DIR = A0;
const uint8_t LEFT_ENC_A = 3, LEFT_ENC_B = 4, LEFT_INDEX = 5;    // A on an interrupt pin
const uint8_t RIGHT_ENC_A = 7, RIGHT_ENC_B = 8, RIGHT_INDEX = 9;
#endif

inline long pivotDegreesToSteps(float degrees) {
  return lround(degrees * STEP_PER_PIVOT_DEGREE);
}

inline float countsToPivotDegrees(long counts) {
  return counts * STEP_PER_COUNT / STEP_PER_PIVOT_DEGREE;
}
