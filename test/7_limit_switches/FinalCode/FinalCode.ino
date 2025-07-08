#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

/////////////////////////////////////////////////////
#define X_LIMIT_MIN 9     // X- //stepper1
#define X_LIMIT_MAX A0    // X+ //stepper1

#define Y_LIMIT_MIN 10    // Y- //stepper2
#define Y_LIMIT_MAX A1    // Y+ //stepper2

#define Z_LIMIT_MIN 11    // Z- //stepperz
#define Z_LIMIT_MAX A2    // Z+ //stepperz

#define GRIPPER_LIMIT A3    //stepper3
//////////////////////////////////////////////////////////

AccelStepper stepper1(1, 2, 5); //link 1 // x in driver 
AccelStepper stepper3(1, 3, 6); //link 2 // y in driver 
AccelStepper stepper2(1, 4, 7); // gripper //z in driver 
AccelStepper stepperZ(1, 12, 13); // z axis // A in driver

int stepperPosition1 = 0;
int stepperPosition2 = 0;
int stepperPosition3 = 0;
int stepperPositionZ = 0;

int maxDistance1 = 1823; // joint 1
int maxDistance2 = 6429; // joint 2
int maxDistance3 = 4767; // joint 3
int maxDistanceZ = 0; // Z axis

void setup() {
  Serial.begin(9600);
  pinMode(8, OUTPUT);
  pinMode(X_LIMIT_MIN, INPUT_PULLUP);
  pinMode(X_LIMIT_MAX, INPUT_PULLUP);
  pinMode(Y_LIMIT_MIN, INPUT_PULLUP);
  pinMode(Y_LIMIT_MAX, INPUT_PULLUP);
  pinMode(Z_LIMIT_MIN, INPUT_PULLUP);
  pinMode(Z_LIMIT_MAX, INPUT_PULLUP);

  // Invert Pins for Correct Direction
                              // direction , enable , stop
  // stepper1.setPinsInverted(true, false, false);
  // stepper2.setPinsInverted(true, false, false);
  // stepper3.setPinsInverted(true, false, false);
  // stepperZ.setPinsInverted(true, false, false);


  // Stepper Motors Max Speed
  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(2000);
  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(2000);
  stepper3.setMaxSpeed(4000);
  stepper3.setAcceleration(2000);
  stepperZ.setMaxSpeed(4000);
  stepperZ.setAcceleration(2000);

  getLimits();
}

void loop() {

  // put your main code here, to run repeatedly:

}

void getLimits() {
  delay(1000);
  //homeZ();
  homeStepper2();
  homeStepper1();
  homeStepper3();
}

// //z axis
// void homeZ() {
//   stepperZ.setCurrentPosition(0);
//   stepperZ.setSpeed(-1000);

//   while (digitalRead(Z_LIMIT_MIN) == 0) {
//     stepperZ.runSpeed();
//   }

//   stepperZ.setCurrentPosition(0);
//   delay(200);

//   stepperZ.moveTo(10000);
//   while (stepperZ.distanceToGo() != 0) {
//     stepperZ.run();
//     if (digitalRead(Z_LIMIT_MAX) == 1) {
//       break;
//     }
//   }

//   maxDistanceZ = stepperZ.currentPosition();
//   Serial.print("Max Z Distance: ");
//   Serial.println(maxDistanceZ);

//   stepperZ.setCurrentPosition(maxDistanceZ / 2);
//   stepperPositionZ = 0;
//   stepperZ.moveTo(stepperPositionZ);

//   while (stepperZ.distanceToGo() != 0) {
//     stepperZ.run();
//   }

//   Serial.print("Safe Position Reached: ");
//   Serial.println(stepperZ.currentPosition());
// }


// joint 1
void homeStepper1() {
  stepper1.setCurrentPosition(0);
  stepper1.setSpeed(1100);

  while (digitalRead(X_LIMIT_MIN) == 1) {
    stepper1.runSpeed();
  }

  delay(20);

  stepper1.setCurrentPosition(maxDistance1);

  stepperPosition1 = 0;
  stepper1.moveTo(stepperPosition1);

  while (stepper1.currentPosition() != stepperPosition1) {
    stepper1.run();
  }

  Serial.print("Safe Position Reached: ");
  Serial.println(stepper1.currentPosition());
}


void homeStepper2() {
  stepper2.setCurrentPosition(0);
  stepper2.setSpeed(1300);

  while (digitalRead(Y_LIMIT_MIN) == 1) {
    stepper2.runSpeed();
  }

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


void homeStepper3() {
  stepper3.setCurrentPosition(0);
  stepper3.setSpeed(1300);

  while (digitalRead(Z_LIMIT_MIN) == 1) {
    stepper3.runSpeed();
  }

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
