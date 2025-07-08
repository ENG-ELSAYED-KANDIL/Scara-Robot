#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

#define limitSwitch1 9 // joint 1
#define limitSwitch2 10 // joint 2
#define limitSwitch3 11 // joint 3
#define limitSwitchZ A3 // z axis

AccelStepper stepper1(1, 2, 5);
AccelStepper stepper2(1, 3, 6);
AccelStepper stepper3(1, 4, 7);
AccelStepper stepperZ(1, 12, 13);

int stepperPosition1;
int stepperPosition2;
int stepperPosition3;
int stepperPositionZ;

int maxDistance1;
int maxDistance2;
int maxDistance3;
int maxDistanceZ;

void setup() {
  Serial.begin(9600);

  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  pinMode(limitSwitch3, INPUT_PULLUP);
  pinMode(limitSwitchZ, INPUT_PULLUP);

  // Invert Pins for Correct Direction
  // stepper1.setPinsInverted(true, false, false);
  // stepper2.setPinsInverted(true, false, false);

  // Stepper Motors Max Speed
  stepper1.setMaxSpeed(50);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(50);
  stepper2.setAcceleration(500);
  stepper3.setMaxSpeed(50);
  stepper3.setAcceleration(500);
  stepperZ.setMaxSpeed(100);
  stepperZ.setAcceleration(500);

  getLimits();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void getLimits() {
  delay(10000);
  // homeZ();
  // homeStepper3();
  // homeStepper2();
  homeStepper1();
}

// Z axis
void homeZ() {
  stepperZ.setCurrentPosition(0);
  stepperZ.setSpeed(50);

  while (!digitalRead(limitSwitchZ)) {
    stepperZ.runSpeed();
  }

  maxDistanceZ = stepperZ.currentPosition();
  Serial.print("Max Z Distance: ");
  Serial.println(maxDistanceZ);

  stepperZ.setCurrentPosition(maxDistanceZ);

  stepperPositionZ = 3000;
  stepperZ.moveTo(stepperPositionZ);

  while (stepperZ.currentPosition() != stepperPositionZ) {
    stepperZ.run();
  }

  Serial.print("Safe Position Reached: ");
  Serial.println(stepperZ.currentPosition());
}

// gripper joint
void homeStepper3() {
  stepper3.setCurrentPosition(0); //from safe point
  stepper3.setSpeed(-50);

  while (!digitalRead(limitSwitch3)) {
    stepper3.runSpeed();
  }

  maxDistance3 = stepper3.currentPosition();
  Serial.print("Max Joint 3 Distance: ");
  Serial.println(maxDistance3);

  delay(20);

  stepper3.setCurrentPosition(maxDistance3);

  stepperPosition3 = 0;
  stepper3.moveTo(stepperPosition3);

  while (stepper3.currentPosition() != stepperPosition3) {
    stepper3.run();
  }

  Serial.print("Safe Position Reached: ");
  Serial.println(stepper3.currentPosition());
}

//link 2
void homeStepper2() {
  stepper2.setCurrentPosition(0); // from safe point
  stepper2.setSpeed(-50);

  while (!digitalRead(limitSwitch2)) {
    stepper2.runSpeed();
  }

  maxDistance2 = stepper2.currentPosition();
  Serial.print("Max Joint 2 Distance: ");
  Serial.println(maxDistance2);

  delay(20);
  
  stepper2.setCurrentPosition(maxDistance2);

  stepperPosition2 = 0;
  stepper2.moveTo(stepperPosition2);

  while (stepper2.currentPosition() != stepperPosition2) {
    stepper2.run();
  }

  Serial.print("Safe Position Reached: ");
  Serial.println(stepper2.currentPosition());
}

//link 1
void homeStepper1() {
  stepper1.setCurrentPosition(0); 
  stepper1.setSpeed(-50);

  while (!digitalRead(limitSwitch1)) {
    stepper1.runSpeed();
  }

  maxDistance1 = stepper1.currentPosition();
  Serial.print("Max Joint 1 Distance: ");
  Serial.println(maxDistance1);

  delay(20);

  stepper1.setCurrentPosition(maxDistance1); // الموضع الجديد بتاع الموتور
  stepperPosition1 = 0;
  stepper1.moveTo(stepperPosition1); // هنا الموتور هيبدأ يتحرك للموضع الجديد ول هو منتصف المسافه 

  while (stepper1.currentPosition() != stepperPosition1) {
    stepper1.run();
  }
  Serial.print("Safe Position Reached: ");
  Serial.println(stepper1.currentPosition());
}

