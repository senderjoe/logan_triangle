#pragma once

// The choreography. One row per stage, run top to bottom. Edit the table, then upload again.
//
// left, right  Angle to move to, in degrees at the pivot. 0 is the power-on position.
//              Left:  negative = forward (clockwise viewed from the left elevation)
//              Right: positive = forward
//              HOLD = leave that motor where it is.
// hold         How long to stay in this pose once both motors have arrived (ms).
// speed        Pivot degrees/s for this stage. 0 = the normal speed (PIVOT_SPEED in main.cpp).
// accel        Pivot degrees/s/s for this stage. 0 = the normal acceleration (PIVOT_ACCEL).
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
  {"frame",                    HOLD,     55,  2000,     0,     0},
  {"pose 1",                    -22,   HOLD,  2000,     0,     0},
  {"pose",                      -65,     88,  2000,     0,     0},
  {"balance",                   -90,   HOLD,  2000,     0,     0},
  {"frame 2",                  -100,    148,  2000,     0,     0},
  {"stretch",                  -150,   HOLD,  2000,     0,     0},
  {"fall through",             -165,   HOLD,   500,     0,    56},
  {"short straw horizontal",   -150,   HOLD,  2000,     0,     0},
  {"pose 2 (L)",                -90,    132,  2000,     0,     0},
  {"pose 3 (A)",                  0,     44,  2000,     0,     0},
  {"pose (V)",                  -50,     94,  1000,     0,     0},
  {"reach for high wire",      HOLD,     34,     0,     0,     0},
  {"reach for high wire",       -44,     27,     0,     0,     0},
  {"run to end of high wire",     0,      0,     0,     0,     0},
};

const int STAGE_COUNT = sizeof(SEQUENCE) / sizeof(SEQUENCE[0]);

const bool LOOPING = false;                 // true = start again after the last stage
const unsigned long LOOP_PAUSE_MS = 10000;  // pause before starting again when LOOPING
