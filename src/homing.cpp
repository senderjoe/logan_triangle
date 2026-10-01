// Homing on the encoder index (Z) pulse, and the two-step calibration that sets it up.
//
// Each encoder gives one index pulse per motor turn, which is every 810 degrees at the pivot,
// so there is at most one within a straw's range of travel.
//
// Calibration step 1 (straws off) turns each motor onto its pulse and counts from there.
// The motor power is then switched off (the Micro stays on, so the encoders keep counting),
// the straws are fitted and set to the start pose by hand, and step 2 records how far the
// start pose is from the pulse, in EEPROM.
//
// Homing searches for the pulse where the calibration says it should be, then moves by the
// recorded distance, so the start pose is exactly repeatable whatever the motors did at power-on.

#include "homing.h"
#include <EEPROM.h>
#include "config.h"

namespace {

const uint16_t CAL_MAGIC = 0x7A14;  // bumped for the 1:1 belts and new pins: old calibrations are ignored

struct Calibration {
  uint16_t magic;
  long indexLeft;   // where the index pulse is, measured from the start pose, in encoder counts
  long indexRight;
};

Calibration cal;

// Where the index pulses are in the encoders' current count, once known: after step 1 (at 0),
// or after homing (at their calibrated positions). The encoders keep counting while the Micro
// is on, even with the motor power off, so this stays valid until the Micro restarts.
bool indexKnown = false;
long indexLeftNow;
long indexRightNow;

enum MoveResult { REACHED, FOUND_INDEX, ABORTED };

void setAxisMotion(Axis &a, float speed, float accel) {
  a.stepper.setMaxSpeed(speed * STEP_PER_PIVOT_DEGREE);
  a.stepper.setAcceleration(accel * STEP_PER_PIVOT_DEGREE);
}

void stopDead(Axis &a) {
  a.stepper.setCurrentPosition(a.stepper.currentPosition());  // also sets speed to 0
}

long readPosition(Axis &a) {
  return a.encoder.read() * ENCODER_SIGN;
}

// Set both the encoder and the step count to this position (encoder counts)
void setPosition(Axis &a, long counts) {
  a.encoder.write(counts * ENCODER_SIGN);
  a.stepper.setCurrentPosition(counts * STEP_PER_COUNT);
}

// Forward degrees (sequence sheet direction) of a position in encoder counts
float forwardDegrees(Axis &a, long counts) {
  return countsToPivotDegrees(counts) * a.forward;
}

bool abortRequested() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'x' || c == 'X') return true;
  }
  return false;
}

// Wait for a key and return true if it was y
bool confirmed() {
  while (Serial.available()) Serial.read();  // ignore anything typed earlier
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\r' || c == '\n') continue;
      return c == 'y' || c == 'Y';
    }
  }
}

// Move to target (steps), optionally stopping dead on the index pulse's rising edge
MoveResult moveWatching(Axis &a, long target, bool watchIndex) {
  a.stepper.moveTo(target);
  bool wasHigh = digitalRead(a.indexPin);
  while (a.stepper.distanceToGo() != 0) {
    if (abortRequested()) {
      stopDead(a);
      Serial.println(F("Stopped"));
      return ABORTED;
    }
    a.stepper.run();
    if (watchIndex) {
      bool high = digitalRead(a.indexPin);
      if (high && !wasHigh) {
        stopDead(a);
        return FOUND_INDEX;
      }
      wasHigh = high;
    }
  }
  return REACHED;
}

bool moveBoth(Axis &l, Axis &r, long targetL, long targetR) {
  l.stepper.moveTo(targetL);
  r.stepper.moveTo(targetR);
  while (l.stepper.distanceToGo() != 0 || r.stepper.distanceToGo() != 0) {
    if (abortRequested()) {
      stopDead(l);
      stopDead(r);
      Serial.println(F("Stopped"));
      return false;
    }
    l.stepper.run();
    r.stepper.run();
  }
  return true;
}

// Catch the pulse the same way every time: back off forward past it, then creep back onto it.
// The pulse is a few counts wide, so where it is first seen depends on the direction of travel.
bool latchIndex(Axis &a) {
  long backoff = pivotDegreesToSteps(HOME_BACKOFF) * a.forward;

  setAxisMotion(a, HOME_SPEED, HOME_ACCEL);
  if (moveWatching(a, a.stepper.currentPosition() + backoff, false) == ABORTED) return false;

  setAxisMotion(a, HOME_LATCH_SPEED, HOME_ACCEL);
  MoveResult result = moveWatching(a, a.stepper.currentPosition() - 2 * backoff, true);
  if (result == REACHED) {
    Serial.print(a.name);
    Serial.println(F(": lost the index pulse on the final approach"));
  }
  return result == FOUND_INDEX;
}

// Homing, straws on and at or near the start pose: look for the pulse where the calibration
// says it is, within HOME_WINDOW either side. Goes towards it first, then back the other way.
bool findIndexExpected(Axis &a, long indexCounts) {
  setAxisMotion(a, HOME_SPEED, HOME_ACCEL);
  float expected = forwardDegrees(a, indexCounts);  // forward degrees from the start pose
  float nearEnd = expected >= 0 ? expected + HOME_WINDOW : expected - HOME_WINDOW;
  float farEnd = expected >= 0 ? expected - HOME_WINDOW : expected + HOME_WINDOW;

  long from = a.stepper.currentPosition();
  MoveResult result = moveWatching(a, from + pivotDegreesToSteps(nearEnd) * a.forward, true);
  if (result == REACHED) {
    result = moveWatching(a, from + pivotDegreesToSteps(farEnd) * a.forward, true);
  }
  if (result == REACHED) {
    Serial.print(a.name);
    Serial.println(F(": no index pulse where expected. Is the motor power on, the straw near"));
    Serial.println(F("  its start pose, and the YEL wire connected?"));
  }
  return result == FOUND_INDEX;
}

// Calibration step 1, straws off: turn the motor up to a full turn to find the pulse
bool findIndexOnMotor(Axis &a) {
  setAxisMotion(a, FIND_INDEX_SPEED, FIND_INDEX_ACCEL);
  long turn = lround(1.1 * STEP_PER_REVOLUTION);
  MoveResult result = moveWatching(a, a.stepper.currentPosition() + turn * a.forward, true);
  if (result == REACHED) {
    Serial.print(a.name);
    Serial.println(F(": no index pulse in a full motor turn. Is the YEL wire connected?"));
  }
  return result == FOUND_INDEX;
}

// Report where the pulse is relative to the start pose, and warn if homing needs care
void reportIndex(Axis &a, long indexCounts) {
  float ahead = forwardDegrees(a, indexCounts);
  Serial.print(a.name);
  Serial.print(F(": index pulse is "));
  Serial.print(fabs(ahead), 1);
  Serial.println(ahead >= 0 ? F(" deg forward of the start pose") : F(" deg behind the start pose"));

  if (ahead < -2) {
    Serial.println(F("  Homing will take this straw that far behind the start pose, plus up to 15 deg."));
    Serial.println(F("  If there isn't room, move the pulley on its motor shaft and calibrate again."));
  } else if (ahead > 90) {
    Serial.println(F("  Homing will swing this straw that far forward. Check it has room."));
  }
}

}  // namespace

void initHoming(Axis &left, Axis &right) {
  pinMode(left.indexPin, INPUT_PULLUP);   // pulled up so an unconnected YEL wire stays quiet
  pinMode(right.indexPin, INPUT_PULLUP);
  EEPROM.get(0, cal);
}

bool isCalibrated() {
  return cal.magic == CAL_MAGIC;
}

bool homeAll(Axis &left, Axis &right) {
  if (!isCalibrated()) {
    Serial.println(F("Not calibrated yet: press i (straws off), then o"));
    return false;
  }

  Serial.println(F("Homing on the index pulses (x = stop)"));
  // Right first: homing left first makes the straws bind
  Axis *axes[] = {&right, &left};
  long index[] = {cal.indexRight, cal.indexLeft};
  for (int i = 0; i < 2; i++) {
    if (!findIndexExpected(*axes[i], index[i]) || !latchIndex(*axes[i])) {
      Serial.println(F("Homing failed. Press h to try again"));
      return false;
    }
    setPosition(*axes[i], index[i]);  // the pulse is at its calibrated distance from the start pose
  }
  indexKnown = true;
  indexLeftNow = cal.indexLeft;
  indexRightNow = cal.indexRight;

  Serial.println(F("Moving to the start pose"));
  setAxisMotion(left, PIVOT_SPEED, PIVOT_ACCEL);
  setAxisMotion(right, PIVOT_SPEED, PIVOT_ACCEL);
  if (!moveBoth(left, right, 0, 0)) return false;

  Serial.println(F("Homed: at the start pose"));
  return true;
}

void runFindIndex(Axis &left, Axis &right) {
  Serial.println(F("Calibration step 1: find the index pulses."));
  Serial.println(F("The straws must be off: each motor will turn up to a full turn."));
  Serial.println(F("Press y to go, any other key to cancel."));
  if (!confirmed()) {
    Serial.println(F("Cancelled"));
    return;
  }

  indexKnown = false;
  Axis *axes[] = {&left, &right};
  for (Axis *a : axes) {
    if (!findIndexOnMotor(*a) || !latchIndex(*a)) {
      Serial.println(F("Step 1 stopped"));
      return;
    }
    setPosition(*a, 0);  // count from the pulse
    Serial.print(a->name);
    Serial.println(F(": found its index pulse"));
  }
  indexKnown = true;
  indexLeftNow = 0;
  indexRightNow = 0;

  Serial.println(F("Both motors are on their index pulses. Now:"));
  Serial.println(F("  1. Switch off the motor power. Keep the USB connected so the encoders keep counting."));
  Serial.println(F("  2. Fit the straws and set them to the start pose by hand."));
  Serial.println(F("  3. Press o to save."));
}

bool runSetStartPose(Axis &left, Axis &right) {
  if (!indexKnown) {
    Serial.println(F("The index pulses aren't known yet: home first (h), or do step 1 (i)."));
    Serial.println(F("The Micro must stay powered from then until o."));
    return false;
  }

  // The straws are at the new start pose now: measure the pulses from here
  cal.magic = CAL_MAGIC;
  cal.indexLeft = indexLeftNow - readPosition(left);
  cal.indexRight = indexRightNow - readPosition(right);
  EEPROM.put(0, cal);
  indexLeftNow = cal.indexLeft;
  indexRightNow = cal.indexRight;

  setPosition(left, 0);
  setPosition(right, 0);
  Serial.println(F("Saved. This pose is now the start pose (0)."));
  reportIndex(left, cal.indexLeft);
  reportIndex(right, cal.indexRight);
  Serial.println(F("Switch the motor power back on, then press r to start. It corrects any power-on jerk."));
  return true;
}
