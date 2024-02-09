#include <Arduino.h>
#include <AccelStepper.h>

// define step constants
#define FULLSTEP 4
#define STEP_PER_REVOLUTION 2048 // this value is from datasheet
#define RUNSPEED 50
#define ACCELL 50

#define LOOPING false

// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
AccelStepper stepperLeft(FULLSTEP, 11, 9, 10, 8);
AccelStepper stepperRight(FULLSTEP, 6, 4, 5, 3);  // Right = back, yellow straw

void doRotation(AccelStepper &stepper, float rotations); 
void doRotationAdditonal(AccelStepper &stepper, float rotations);
void rotateDegrees(AccelStepper &stepper, int degrees);
void rotateDegreesAdditional(AccelStepper &stepper, int degrees);
void initStepper(AccelStepper &stepper);
bool stageComplete();
void initStage(int stageNo);
void resetPosition();
int currentStage;

void setup() {
  Serial.begin(9600);

  initStepper(stepperLeft);
  initStepper(stepperRight);
  
  // resetPosition();
  initStage(0);
}


void loop() {
  if (stageComplete() && currentStage != -1) {
    initStage(currentStage + 1);
  }

  stepperLeft.run();   
  stepperRight.run();   
}

// Rotate by revolutions
void doRotation(AccelStepper &stepper, float rotations){
  stepper.moveTo(stepper.currentPosition() + static_cast<int>(STEP_PER_REVOLUTION * rotations)); 
}

// Add rotation to target positon, revolutions
void doRotationAdditonal(AccelStepper &stepper, float rotations){
  stepper.moveTo(stepper.targetPosition() + static_cast<int>(STEP_PER_REVOLUTION * rotations)); 
}

// Rotate by degrees
void rotateDegrees(AccelStepper &stepper, int degrees){
  int steps = static_cast<int>(degrees / 360.0 * STEP_PER_REVOLUTION);
  stepper.moveTo(stepper.currentPosition() + steps); 
}

// Add rotation to target position, by degrees
void rotateDegreesAdditional(AccelStepper &stepper, int degrees){
  int steps = static_cast<int>(degrees / 360.0 * STEP_PER_REVOLUTION);
  stepper.moveTo(stepper.targetPosition() + steps); 
}


void initStepper(AccelStepper &stepper) {
  stepper.setSpeed(RUNSPEED);
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

void initStage(int stageNo) {

  currentStage = stageNo;
  Serial.print("Stage ");
  Serial.print(stageNo);
  Serial.println("");

  switch(stageNo) {
    case 0:  // to "frame"
      rotateDegrees(stepperRight, 58);  
      break;
    case 1: // to "pose 1"
      delay(2000);
      rotateDegrees(stepperLeft, -25);  
      break;
    case 2:
      delay(2000);
      rotateDegrees(stepperLeft, -40);
      rotateDegrees(stepperRight, 35);
      break;
    case 3:  // to "balance"
      delay(2000);
      rotateDegrees(stepperLeft, -20);
      break;
    case 4:  // to "frame 2"
      delay(2000);
      rotateDegrees(stepperLeft, -18);  
      rotateDegrees(stepperRight, 55);  
      break;
    case 5:  // to "stretch"
      delay(2000);
      rotateDegrees(stepperLeft, -50);  
      break;
    case 6:  // to "fall through"
      delay(2000);
      rotateDegrees(stepperLeft, -20);  
      stepperLeft.setAcceleration(500);  // increase acc. to speed up fall through
      break;
    case 7:  // to "short straw horizontal"
      rotateDegrees(stepperLeft, 20);
      stepperLeft.setAcceleration(ACCELL);
      break;
    case 8:  // to "pose 2" (L)
      delay(2000);
      rotateDegrees(stepperLeft, 60); 
      rotateDegrees(stepperRight, -15);   
      break;
    case 9:  // to "pose 3" (_A)
      delay(2000);
      rotateDegrees(stepperLeft, 93); 
      rotateDegrees(stepperRight, -88);   
      break;
    case 10:  // to (V)
      delay(2000);
      rotateDegrees(stepperLeft, -50); 
      rotateDegrees(stepperRight, 50);   
      break;
    case 11:  // "reach for high wire"
      delay(1000);
      rotateDegrees(stepperRight, -95);  
      break;
    case 12:  // "run to end of high wire"
      // delay(500);
      rotateDegrees(stepperLeft, 50); 
      rotateDegrees(stepperRight, -20);   
      break;

    default:
      if (LOOPING) {
        delay(10000);
        initStage(0);
      } else {
        currentStage = -1;
      }

  }
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
