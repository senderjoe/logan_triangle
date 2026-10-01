#pragma once

// The choreography. One row per stage, run top to bottom. Edit the table, then upload again.
//
// left, right  Angle to move to, in degrees at the pivot. 0 is the power-on position.
//              Left:  negative = forward (clockwise viewed from the left elevation)
//              Right: positive = forward
//              HOLD = leave that motor where it is.
// hold         How long to stay in this pose once both motors have arrived (ms).
// speed        Cruise speed, pivot degrees/s. 0 = the normal speed (PIVOT_SPEED in config.h).
// accel        Peak acceleration for this stage, pivot degrees/s/s. 0 = the normal (PIVOT_ACCEL).
//              Moves ease in and out (S-curve), so this peak is only reached halfway through
//              each ease. Easing up to speed takes about 1.6 x speed / accel seconds.
//              Applies to both motors.

const int HOLD = 9999;

struct Stage {
  const char *name;
  int left;
  int right;
  unsigned int hold;
  int speed;
  int accel;
};

const Stage SEQUENCE[] = {
  // name                      left   right   hold  speed  accel
  {"frame",                    HOLD,     57,  2000,     0,     0},
  {"pose 1",                    -20,   HOLD,  2000,     0,     0},
  {"pose",                      -65,     88,  2000,     0,     0},
  {"balance",                   -90,   HOLD,  2000,     0,     0},
  {"frame 2",                  -107,    144,  2000,     0,     0},
  {"stretch",                  -147,   HOLD,  2000,     0,     0},
  {"fall through",             -162,   HOLD,     0,     0,    0},
  {"fall through",             -157,   HOLD,  2000,     40,    40},
  {"short straw horizontal",   -148,   HOLD,  2000,     0,     0},
  {"pose 2 (L)",                -90,    132,  2000,     0,     0},
  {"pose 3 (A)",                  0,     44,  2000,     0,     0},
  {"pose (V)",                  -50,     94,  1000,     0,     0},
  {"reach for high wire",      HOLD,     34,     0,     0,     0},
  {"reach for high wire",       -44,     22,     0,     0,     0},
  {"run to end of high wire",     0,      0,     0,     0,     0},
};

const int STAGE_COUNT = sizeof(SEQUENCE) / sizeof(SEQUENCE[0]);

const bool LOOPING = false;                 // true = start again after the last stage
const unsigned long LOOP_PAUSE_MS = 10000;  // pause before starting again when LOOPING
