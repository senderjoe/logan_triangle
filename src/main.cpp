#include <Arduino.h>
#include <AccelStepper.h>
#include <Encoder.h>
#include "config.h"    // hardware settings and pins
#include "sequence.h"  // the stage angles and timings
#include "homing.h"    // index pulse homing and calibration

AccelStepper stepperLeft(AccelStepper::DRIVER, LEFT_PUL, LEFT_DIR);
AccelStepper stepperRight(AccelStepper::DRIVER, RIGHT_PUL, RIGHT_DIR);  // Right = back, yellow straw

Encoder encoderLeft(LEFT_ENC_A, LEFT_ENC_B);
Encoder encoderRight(RIGHT_ENC_A, RIGHT_ENC_B);

Axis leftAxis = {"Left", stepperLeft, encoderLeft, LEFT_INDEX, -1};
Axis rightAxis = {"Right", stepperRight, encoderRight, RIGHT_INDEX, 1};
bool homed = false;  // true once the start pose has been set from the index pulses
bool countUnknown = true;  // true when the motor power may have just come on, so the step count
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
void setMotion(float speed, float accel);
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

void setup() {
  Serial.begin(9600);
  Serial.println(F("Triangle setup"));

  initStepper(stepperLeft);
  initStepper(stepperRight);

  positionLeft = 0;
  positionRight = 0;
  targetLeft = 0;
  targetRight = 0;
  
  currentStage = -1;  // don't start the sequence until r
  initHoming(leftAxis, rightAxis);

  if (isCalibrated()) {
    // Find the start pose from the index pulses, whatever the motors did at power-on
    if (HOME_AT_POWER_ON) {
      delay(HOME_DELAY_MS);
      homed = homeAll(leftAxis, rightAxis);
      if (homed) countUnknown = false;
    }
  } else {
    // Not calibrated: the encoders count from this moment, so the pose the straws are in now is 0.
    // Switch the motor power on after this, then press r: any jerk as the drivers power up
    // is measured by the encoders and corrected before the sequence starts.
    Serial.println(F("Not calibrated for homing: encoders zeroed at the current pose."));
    Serial.println(F("Switch on the motor power, then press r to start."));
  }

  Serial.println(F("Keys: r = restart, x = stop now, c = continue after a stop, p = print encoder positions"));
  Serial.println(F("      h = home, i = calibrate step 1 (straws off), o = save the current pose as the start pose"));
}

// True when nothing is moving, so homing or calibration can take over the motors
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

  stepperLeft.run();   
  stepperRight.run(); 

  checkPosition();
  
}

// Keyboard control from the serial monitor
void handleSerial() {
  if (!Serial.available()) return;

  switch (Serial.read()) {
    case 'r':
    case 'R':
      if (isCalibrated() && !homed) {
        homed = homeAll(leftAxis, rightAxis);
        if (!homed) break;
        countUnknown = false;
      }
      restartSequence();
      break;
    case 'h':
    case 'H':
      if (idle()) {
        stoppedStage = -1;
        homed = homeAll(leftAxis, rightAxis);
        if (homed) countUnknown = false;
        setMotion(PIVOT_SPEED, PIVOT_ACCEL);
      }
      break;
    case 'i':
    case 'I':
      if (idle()) {
        stoppedStage = -1;
        homed = false;
        runFindIndex(leftAxis, rightAxis);
        countUnknown = true;  // the motor power is switched off and on during calibration
        setMotion(PIVOT_SPEED, PIVOT_ACCEL);
      }
      break;
    case 'o':
    case 'O':
      if (idle()) {
        stoppedStage = -1;
        homed = runSetStartPose(leftAxis, rightAxis);
        countUnknown = true;  // the motor power is switched back on after this
        setMotion(PIVOT_SPEED, PIVOT_ACCEL);
      }
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

// Return both pivots to 0, then run the sequence from stage 0.
// Takes the current position from the encoders first, so 0 is the pose the straws were in
// when the Micro started, even if the motors have jerked or slipped since.
void restartSequence() {
  Serial.println(F("Restart: returning to the start pose"));
  currentStage = -1;  // pause stage advance until home
  holding = false;
  stoppedStage = -1;

  syncFromEncoders();
  setMotion(PIVOT_SPEED, PIVOT_ACCEL);  // in case it stopped during a stage with its own speed
  rotateTo(stepperLeft, 0);
  rotateTo(stepperRight, 0);
  returningHome = true;
}

// Stop both motors immediately, without decelerating
void stopNow() {
  if (currentStage != -1) {
    stoppedStage = currentStage;
    stoppedHolding = holding;
  }
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
//  - the motor power may have just come on. The power-on jerk moves each motor to wherever its
//    driver happens to be, so the encoder is the best guide to where the driver now is.
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

// Set speed and acceleration for both motors, in pivot degrees/s and degrees/s/s
void setMotion(float speed, float accel) {
  stepperLeft.setMaxSpeed(speed * STEP_PER_PIVOT_DEGREE);
  stepperRight.setMaxSpeed(speed * STEP_PER_PIVOT_DEGREE);
  stepperLeft.setAcceleration(accel * STEP_PER_PIVOT_DEGREE);
  stepperRight.setAcceleration(accel * STEP_PER_PIVOT_DEGREE);
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
  return (
    stepperLeft.distanceToGo() == 0 &&
    stepperRight.distanceToGo() == 0 
  );
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

  setMotion(stage.speed ? stage.speed : PIVOT_SPEED, stage.accel ? stage.accel : PIVOT_ACCEL);
  if (stage.left != HOLD) rotateTo(stepperLeft, stage.left);
  if (stage.right != HOLD) rotateTo(stepperRight, stage.right);
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
