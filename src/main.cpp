#include <Arduino.h>
#include <AccelStepper.h>
#include <Encoder.h>
#include "config.h"    // hardware settings and pins
#include "sequence.h"  // the stage angles and timings

AccelStepper stepperLeft(AccelStepper::DRIVER, LEFT_PUL, LEFT_DIR);
AccelStepper stepperRight(AccelStepper::DRIVER, RIGHT_PUL, RIGHT_DIR);  // Right = back, yellow straw

Encoder encoderLeft(LEFT_ENC_A, LEFT_ENC_B);
Encoder encoderRight(RIGHT_ENC_A, RIGHT_ENC_B);

// Smooth (S-curve) moves. Each move eases in and out: the acceleration rises and falls along a
// sine curve instead of switching on and off, so the flexible straw linkage isn't kicked into
// bouncing at the start and end of a move. AccelStepper still makes the step pulses.
struct SmoothMove {
  AccelStepper *stepper;
  bool active;
  long start;                 // steps
  long distance;              // steps, signed
  float speed;                // cruise speed, steps/s
  float ramp;                 // time to ease up to cruise speed, and to ease down from it, s
  float total;                // time for the whole move, s
  unsigned long startMicros;
  unsigned long planMicros;   // when the curve was last worked out
  long target;                // where the motor should be now, steps
  bool finished;              // the curve has reached its end
};

SmoothMove moveLeft = {&stepperLeft, false};
SmoothMove moveRight = {&stepperRight, false};

// How hard a motor catches up if it falls behind the planned curve, in steps/s per step behind
const float FOLLOW_GAIN = 50;
// How often the curve is worked out (sin/cos are slow on this chip); steps are made in between
const unsigned long PLAN_INTERVAL_US = 1000;

bool driversOn = false;    // the drivers start off, held off by Q1 (see config.h)
bool countUnknown = true;  // true when the drivers may have just come on, so the step count
                           // no longer matches the drivers (see syncFromEncoders)

void doRotation(AccelStepper &stepper, float rotations); 
void doRotationAdditonal(AccelStepper &stepper, float rotations);
void rotateDegrees(AccelStepper &stepper, int degrees);
void rotateDegreesAdditional(AccelStepper &stepper, int degrees);
void rotateTo(AccelStepper &stepper, int degrees);
void initStepper(AccelStepper &stepper);
bool stageComplete();
void checkPosition();
void initStage(int stageNo);
void resetPosition();
void printPositions();
bool positionsAgree();
void handleSerial();
void restartSequence();
void stopNow();
void continueSequence();
void syncFromEncoders();
void printEncoderPositions();
void driversOff();
void ensureDriversOn();
void setHomeHere();
void startSmoothMove(SmoothMove &m, long target, float speed, float accel);
void updateSmoothMove(SmoothMove &m);
int currentStage;
bool returningHome = false;  // true while moving back to 0 before a restart
bool holding = false;        // true while holding a pose after a stage has arrived
unsigned long holdStart;
int stoppedStage = -1;       // stage that was interrupted by x or the safety check, for c
bool stoppedHolding = false; // true if it was stopped while holding that stage's pose
long positionLeft;
long positionRight;
long targetLeft;
long targetRight;

// Start-up: the straws are assumed to be in the start pose, where the last show left them.
// The drivers are held off until r, so the motors stay free and don't jerk. The encoders count
// from now, so this pose is 0. When r switches the drivers on, each motor jerks to wherever its
// driver happens to be; the encoders measure that, and r moves the straws back to 0 before the
// sequence starts.
void setup() {
  pinMode(DRIVER_ENABLE, OUTPUT);
  driversOff();

  Serial.begin(9600);
  while (!Serial && millis() < 3000) {}  // Micro: wait for the serial monitor, but not forever
  Serial.println(F("Triangle setup"));

  initStepper(stepperLeft);
  initStepper(stepperRight);

  positionLeft = 0;
  positionRight = 0;
  targetLeft = 0;
  targetRight = 0;

  currentStage = -1;  // don't start the sequence until r

  Serial.println(F("Motors off. The current pose is the start pose."));
  Serial.println(F("Keys: r = start (motor power must be on), x = stop now, c = continue after a stop,"));
  Serial.println(F("      p = print encoder positions, f = free the motors, h = make the current pose home"));
}

// True when nothing is moving, so the motors can be freed or home set
bool idle() {
  if (currentStage != -1 || returningHome) {
    Serial.println(F("Stop first (x)"));
    return false;
  }
  return true;
}


void loop() {
  handleSerial();

  if (returningHome && stageComplete()) {
    returningHome = false;
    printPositions();
    if (positionsAgree()) {
      initStage(0);
    }
  }

  if (currentStage != -1 && stageComplete()) {
    if (!holding) {
      // Just arrived: check against the encoders, then hold the pose
      printPositions();
      if (positionsAgree()) {
        holding = true;
        holdStart = millis();
      } else {
        stoppedStage = currentStage;  // c retries this stage
        stoppedHolding = false;
        currentStage = -1;  // stop the sequence
        Serial.println(F("Press c to retry this stage from the encoder position, or r to restart"));
      }
    } else if (millis() - holdStart >= SEQUENCE[currentStage].hold) {
      initStage(currentStage + 1);
    }
  }

  updateSmoothMove(moveLeft);
  updateSmoothMove(moveRight);

  checkPosition();
  
}

// Keyboard control from the serial monitor
void handleSerial() {
  if (!Serial.available()) return;

  switch (Serial.read()) {
    case 'r':
    case 'R':
      restartSequence();
      break;
    case 'f':
    case 'F':
      if (idle()) {
        driversOff();
        Serial.println(F("Motors free: set the straws by hand, then press h to make this pose home"));
      }
      break;
    case 'h':
    case 'H':
      if (idle()) setHomeHere();
      break;
    case 'x':
    case 'X':
      stopNow();
      break;
    case 'c':
    case 'C':
      continueSequence();
      break;
    case 'p':
    case 'P':
      printEncoderPositions();
      break;
    default:
      break;  // ignore newlines etc.
  }
}

// Switch the drivers off: the motors go free and can be turned by hand. The encoders keep counting.
void driversOff() {
  digitalWrite(DRIVER_ENABLE, HIGH);  // Q1 on, so ENA draws current: drivers disabled
  driversOn = false;
}

// Switch the drivers on, if they're off, and wait for the jerk as they take hold of the motors.
// The step count then no longer matches the drivers, so the next syncFromEncoders resets it.
void ensureDriversOn() {
  if (driversOn) return;
  Serial.println(F("Motors on"));
  digitalWrite(DRIVER_ENABLE, LOW);
  delay(ENABLE_SETTLE_MS);
  driversOn = true;
  countUnknown = true;
}

// Make the pose the straws are in now the start pose (0)
void setHomeHere() {
  encoderLeft.write(0);
  encoderRight.write(0);
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  stoppedStage = -1;
  // With the drivers on, they're holding the motors here, so the count matches.
  // With them off, the count is reset when they come on (ensureDriversOn).
  if (driversOn) countUnknown = false;
  Serial.println(F("Home set: this pose is now the start pose"));
}

// Return both pivots to 0, then run the sequence from stage 0.
// Takes the current position from the encoders first, so 0 is the pose the straws were in
// at start-up (or when h was pressed), even if the motors have jerked or slipped since.
void restartSequence() {
  Serial.println(F("Restart: returning to the start pose"));
  currentStage = -1;  // pause stage advance until home
  holding = false;
  stoppedStage = -1;

  ensureDriversOn();
  syncFromEncoders();
  startSmoothMove(moveLeft, 0, PIVOT_SPEED, PIVOT_ACCEL);
  startSmoothMove(moveRight, 0, PIVOT_SPEED, PIVOT_ACCEL);
  returningHome = true;
}

// Stop both motors immediately, without decelerating
void stopNow() {
  if (currentStage != -1) {
    stoppedStage = currentStage;
    stoppedHolding = holding;
  }
  moveLeft.active = false;
  moveRight.active = false;
  stepperLeft.setCurrentPosition(stepperLeft.currentPosition());    // also sets speed to 0
  stepperRight.setCurrentPosition(stepperRight.currentPosition());
  currentStage = -1;
  returningHome = false;
  holding = false;
  Serial.println(stoppedStage != -1 ? F("Stopped. Press c to continue or r to restart") : F("Stopped. Press r to restart"));
}

// Carry on from where x (or the safety check) stopped the sequence: finish the interrupted
// stage's move, or go to the next stage if it was stopped while holding a pose
void continueSequence() {
  if (currentStage != -1 || returningHome) {
    Serial.println(F("Already running"));
    return;
  }
  if (stoppedStage == -1) {
    Serial.println(F("Nothing to continue: press r to start"));
    return;
  }

  int stage = stoppedHolding ? stoppedStage + 1 : stoppedStage;
  stoppedStage = -1;
  Serial.println(F("Continuing"));
  ensureDriversOn();
  syncFromEncoders();
  initStage(stage);
}

// Print where the encoders say the pivots are, in pivot degrees from the start pose
void printEncoderPositions() {
  checkPosition();
  Serial.print(F("Encoders (deg): left "));
  Serial.print(positionLeft * STEP_PER_COUNT / STEP_PER_PIVOT_DEGREE, 1);
  Serial.print(F(", right "));
  Serial.print(positionRight * STEP_PER_COUNT / STEP_PER_PIVOT_DEGREE, 1);
  Serial.print(F("   (counts: "));
  Serial.print(positionLeft);
  Serial.print(F(", "));
  Serial.print(positionRight);
  Serial.println(F(")"));
}

// Set the motors' step positions from the encoders, e.g. after a power-up jerk or a nudge
// Before r or c moves anything: stop any move in progress, and reset the step count from the
// encoders only if the count can't be trusted:
//  - the drivers may have just come on. The jerk as they switch on moves each motor to wherever
//    its driver happens to be, so the encoder is the best guide to where the driver now is.
//  - or a motor is more than a full step out, so it has slipped.
// Otherwise the count still matches where the drivers are holding the motors, and resetting it
// from the encoders would add friction's small shortfall to the next move.
void syncFromEncoders() {
  checkPosition();
  const long fullStep = STEP_PER_REVOLUTION / 200;  // 1.8 degree motor
  long left = positionLeft * STEP_PER_COUNT;
  long right = positionRight * STEP_PER_COUNT;
  bool slipped = labs(left - stepperLeft.currentPosition()) > fullStep ||
                 labs(right - stepperRight.currentPosition()) > fullStep;

  moveLeft.active = false;
  moveRight.active = false;

  if (countUnknown || slipped) {
    stepperLeft.setCurrentPosition(left);    // also stops any move in progress
    stepperRight.setCurrentPosition(right);
    countUnknown = false;
    Serial.println(F("Step count reset from the encoders"));
  } else {
    stepperLeft.setCurrentPosition(stepperLeft.currentPosition());    // just stop
    stepperRight.setCurrentPosition(stepperRight.currentPosition());
  }

  Serial.print(F("Position from encoders (deg): left "));
  Serial.print(countsToPivotDegrees(positionLeft), 1);
  Serial.print(F(", right "));
  Serial.println(countsToPivotDegrees(positionRight), 1);
}

void printPositions() {
  
  Serial.print(F("Left, "));
  Serial.print(positionLeft);
  Serial.print(F(", "));
  Serial.print(stepperLeft.currentPosition());
  Serial.print(F(", Right,  "));
  Serial.print(positionRight);
  Serial.print(F(", "));
  Serial.print(stepperRight.currentPosition());
  Serial.println();
    
}

// Rotate by pivot revolutions
void doRotation(AccelStepper &stepper, float rotations){
  stepper.moveTo(stepper.currentPosition() + lround(STEP_PER_PIVOT_REVOLUTION * rotations));
}

// Add rotation to target positon, pivot revolutions. Useful for resetting
void doRotationAdditonal(AccelStepper &stepper, float rotations){
  stepper.moveTo(stepper.targetPosition() + lround(STEP_PER_PIVOT_REVOLUTION * rotations));
}

// Rotate by pivot degrees
void rotateDegrees(AccelStepper &stepper, int degrees){
  stepper.moveTo(stepper.currentPosition() + pivotDegreesToSteps(degrees));
}

// Add rotation to target position, by pivot degrees
void rotateDegreesAdditional(AccelStepper &stepper, int degrees){
  stepper.moveTo(stepper.targetPosition() + pivotDegreesToSteps(degrees));
}

// Rotate to an absolute pivot angle (0 = the start pose)
void rotateTo(AccelStepper &stepper, int degrees){
  stepper.moveTo(pivotDegreesToSteps(degrees));
}


void initStepper(AccelStepper &stepper) {
  stepper.setMinPulseWidth(MIN_PULSE_WIDTH);
  stepper.setMaxSpeed(RUNSPEED);
  stepper.setAcceleration(ACCELL); // set acceleration
  stepper.setCurrentPosition(0); // set position
}

bool stageComplete() {
  return !moveLeft.active && !moveRight.active;
}

// Plan a smooth move to target (steps). speed is the cruise speed in pivot degrees/s, accel the
// peak acceleration in pivot degrees/s/s, reached halfway through each ease.
void startSmoothMove(SmoothMove &m, long target, float speed, float accel) {
  m.start = m.stepper->currentPosition();
  m.distance = target - m.start;
  if (m.distance == 0) {
    m.active = false;
    return;
  }

  float d = labs(m.distance);
  float v = speed * STEP_PER_PIVOT_DEGREE;
  float a = accel * STEP_PER_PIVOT_DEGREE;
  // A sine-shaped ease up to speed v, peaking at acceleration a, takes pi*v/(2a) seconds and covers
  // v/2 steps per second of it. If easing up and down would take more than the whole move, the
  // move is too short to reach v, so lower v until the two eases just meet.
  if (PI * v * v / (2 * a) > d) v = sqrt(2 * a * d / PI);

  m.speed = v;
  m.ramp = PI * v / (2 * a);
  m.total = 2 * m.ramp + (d - v * m.ramp) / v;
  m.startMicros = micros();
  m.planMicros = m.startMicros - PLAN_INTERVAL_US;  // plan straight away
  m.target = m.start;
  m.finished = false;
  m.active = true;
}

// Distance and speed u seconds into an ease up from rest
static float easeDistance(const SmoothMove &m, float u) {
  return m.speed * (u / 2 - m.ramp / (2 * PI) * sin(PI * u / m.ramp));
}
static float easeSpeed(const SmoothMove &m, float u) {
  return m.speed * (1 - cos(PI * u / m.ramp)) / 2;
}

// Step the motor along its planned curve. Call as often as possible.
void updateSmoothMove(SmoothMove &m) {
  if (!m.active) return;

  // Every PLAN_INTERVAL_US: work out where the motor should be now, and set the step speed
  // to match the curve, plus a little extra if it has fallen behind
  unsigned long now = micros();
  if (now - m.planMicros >= PLAN_INTERVAL_US) {
    m.planMicros = now;
    float t = (now - m.startMicros) * 1e-6;
    float d = labs(m.distance);
    float s, v;  // steps from the start, and speed in steps/s
    if (t >= m.total) {
      s = d;
      v = 0;
      m.finished = true;
    } else if (t < m.ramp) {
      s = easeDistance(m, t);
      v = easeSpeed(m, t);
    } else if (t > m.total - m.ramp) {
      s = d - easeDistance(m, m.total - t);
      v = easeSpeed(m, m.total - t);
    } else {
      s = m.speed * m.ramp / 2 + m.speed * (t - m.ramp);
      v = m.speed;
    }
    m.target = m.start + (m.distance > 0 ? lround(s) : -lround(s));
    long behind = m.target - m.stepper->currentPosition();
    float stepSpeed = v + FOLLOW_GAIN * labs(behind);
    m.stepper->setSpeed(behind >= 0 ? stepSpeed : -stepSpeed);
  }

  if (m.stepper->currentPosition() != m.target) {
    m.stepper->runSpeed();  // makes one step if it's time for the next one
  } else if (m.finished) {
    m.stepper->setSpeed(0);
    m.active = false;
  }
}

void checkPosition() {

  // read encoder position, in the same direction as the steps (see ENCODER_SIGN in config.h)
  positionLeft = encoderLeft.read() * ENCODER_SIGN;
  positionRight = encoderRight.read() * ENCODER_SIGN;

}


// Safety check at the end of each stage: false if either motor is more than MAX_CORRECTION_DEGREES
// from where it was sent, which means a jam, slipped steps or an encoder fault.
//
// The step count is deliberately NOT reset to the encoder here. The driver holds the motor where
// the step pulses put it, and friction leaves the motor a little short of that. Resetting the count
// to the encoder would make the next move start counting from the motor instead of from the driver,
// carrying this pose's shortfall into the next one.
bool positionsAgree() {
  long left = positionLeft * STEP_PER_COUNT;
  long right = positionRight * STEP_PER_COUNT;
  long limit = pivotDegreesToSteps(MAX_CORRECTION_DEGREES);

  if (labs(left - stepperLeft.currentPosition()) > limit ||
      labs(right - stepperRight.currentPosition()) > limit) {
    Serial.println(F("Encoder and steps disagree: stopping. Check for a jam or an encoder fault."));
    return false;
  }
  return true;
}

// Start a stage from the SEQUENCE table in sequence.h
void initStage(int stageNo) {
  holding = false;

  if (stageNo >= STAGE_COUNT) {
    if (!LOOPING) {
      currentStage = -1;
      Serial.println(F("Sequence finished. Press r to run it again"));
      return;
    }
    Serial.println(F("Looping"));
    delay(LOOP_PAUSE_MS);
    stageNo = 0;
  }

  currentStage = stageNo;
  const Stage &stage = SEQUENCE[stageNo];

  Serial.print(F("Stage "));
  Serial.print(stageNo);
  Serial.print(F(": "));
  Serial.println(stage.name);

  float speed = stage.speed ? stage.speed : PIVOT_SPEED;
  float accel = stage.accel ? stage.accel : PIVOT_ACCEL;
  if (stage.left != HOLD) startSmoothMove(moveLeft, pivotDegreesToSteps(stage.left), speed, accel);
  if (stage.right != HOLD) startSmoothMove(moveRight, pivotDegreesToSteps(stage.right), speed, accel);
}

// Reset to starting position during testing
void resetPosition(){
  // stage 0,1
  rotateDegrees(stepperRight, -58);
  rotateDegrees(stepperLeft, 25);

  // stage 2
  rotateDegreesAdditional(stepperLeft, 40);
  rotateDegreesAdditional(stepperRight, -30);
  // stage 3
  rotateDegreesAdditional(stepperLeft, 20);
  // stage 4
  rotateDegreesAdditional(stepperLeft, 18);
  rotateDegreesAdditional(stepperRight, -55);
  // stage 5
  rotateDegreesAdditional(stepperLeft, 50);
  // stage 6
  rotateDegreesAdditional(stepperLeft, 20); 
  // stage 7
  rotateDegreesAdditional(stepperLeft, -10); 

  while (!stageComplete() ) {
    stepperLeft.run();   
    stepperRight.run();
  }

  delay(3000);

}
