#include <Arduino.h>
#include <AccelStepper.h>
#include <Encoder.h>

// define step constants
#define STEP_PER_REVOLUTION 3200 // this value is from datasheet
#define COUNT_PER_REVOLUTION 4000 // encoder counts per rev
// #define COUNT_PER_STEP COUNT_PER_REVOLUTION / STEP_PER_REVOLUTION // encoder counts per step
#define RUNSPEED 80
#define ACCELL 80

#define LOOPING false

// Pins entered PULSE, DIRECTION 
AccelStepper stepperLeft(AccelStepper::DRIVER, 15, 14);
AccelStepper stepperRight(AccelStepper::DRIVER, 23, 22);  // Right = back, yellow straw

Encoder encoderLeft(7, 8);
Encoder encoderRight(3, 4);

float COUNT_PER_STEP = static_cast<float>(COUNT_PER_REVOLUTION) / STEP_PER_REVOLUTION;  // encoder counts per step

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
void updateStepperPositions();
int currentStage;
int positionLeft;
int positionRight;
int targetLeft;
int targetRight;

void setup() {
  Serial.begin(9600);
  Serial.println("Triangle setup");

  initStepper(stepperLeft);
  initStepper(stepperRight);

  positionLeft = 0;
  positionRight = 0;
  targetLeft = 0;
  targetRight = 0;
  
  // resetPosition();
  initStage(0);
}


void loop() {
  if (stageComplete() && currentStage != -1) {
    printPositions();
    updateStepperPositions();
    printPositions();
    initStage(currentStage + 1);
  }

  stepperLeft.run();   
  stepperRight.run(); 

  checkPosition();
  
}

void printPositions() {
  
  Serial.print("Left, ");
  Serial.print(positionLeft);
  Serial.print(", ");
  Serial.print(stepperLeft.currentPosition());
  Serial.print(", Right,  ");
  Serial.print(positionRight);
  Serial.print(", ");
  Serial.print(stepperRight.currentPosition());
  Serial.println();
    
}

// Rotate by revolutions
void doRotation(AccelStepper &stepper, float rotations){
  stepper.moveTo(stepper.currentPosition() + static_cast<int>(STEP_PER_REVOLUTION * rotations)); 
}

// Add rotation to target positon, revolutions. Useful for resetting
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

// Rotate to an absolute position
void rotateTo(AccelStepper &stepper, int degrees){
  int steps = static_cast<int>(degrees / 360.0 * STEP_PER_REVOLUTION);
  stepper.moveTo(steps); 
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

void checkPosition() {

  // read encoder position
  positionLeft = encoderLeft.read() * -1;
  positionRight = encoderRight.read() * -1;
  
}


// Use encoder position to set motor step position
void updateStepperPositions() {
  stepperLeft.setCurrentPosition(static_cast<int>(positionLeft / COUNT_PER_STEP));
  stepperRight.setCurrentPosition(static_cast<int>(positionRight / COUNT_PER_STEP));
}

void initStage(int stageNo) {

  currentStage = stageNo;
  Serial.print("Stage ");
  Serial.print(stageNo);
  Serial.println("");

  switch(stageNo) {
    case 0:  // to "frame"
      rotateTo(stepperRight, 58);  
      break;
    case 1: // to "pose 1"
      delay(2000);
      rotateTo(stepperLeft, -25);  
      break;
    case 2:
      delay(2000);
      rotateTo(stepperLeft, -65);
      rotateTo(stepperRight, 93);
      break;
    case 3:  // to "balance"
      delay(2000);
      rotateTo(stepperLeft, -85);
      break;
    case 4:  // to "frame 2"
      delay(2000);
      rotateTo(stepperLeft, -103);  
      rotateTo(stepperRight, 148);  
      break;
    case 5:  // to "stretch"
      delay(2000);
      rotateTo(stepperLeft, -153);  
      break;
    case 6:  // to "fall through"
      delay(2000);
      rotateTo(stepperLeft, -173);  
      stepperLeft.setAcceleration(500);  // increase acc. to speed up fall through
      break;
    case 7:  // to "short straw horizontal"
      rotateTo(stepperLeft, -153);
      stepperLeft.setAcceleration(ACCELL);
      break;
    case 8:  // to "pose 2" (L)
      delay(2000);
      rotateTo(stepperLeft, -93); 
      rotateTo(stepperRight, 133);   
      break;
    case 9:  // to "pose 3" (_A)
      delay(2000);
      rotateTo(stepperLeft, 0); 
      rotateTo(stepperRight, 45);   
      break;
    case 10:  // to (V)
      delay(2000);
      rotateTo(stepperLeft, -50); 
      rotateTo(stepperRight, 95);   
      break;
    case 11:  // "reach for high wire"
      delay(1000);
      rotateTo(stepperRight, 20);  
      break;
    case 12:  // "run to end of high wire"
      // delay(500);
      rotateTo(stepperLeft, 0); 
      rotateTo(stepperRight, 0);   
      break;
    case 13:  // reset to 0 position
      // delay(500);
      rotateTo(stepperLeft, 0); 
      rotateTo(stepperRight, 0);   
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
