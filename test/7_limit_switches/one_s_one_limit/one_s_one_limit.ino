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
long start , end , timer;
AccelStepper stepperZ(1, 2, 5); //link 1 // x in driver 
AccelStepper stepper1(1, 3, 6); //link 2 // y in driver 
AccelStepper stepper2(1, 4, 7); // gripper //z in driver 
AccelStepper stepper3(1, 12, 13); // z axis // A in driver

int stepperPosition1 = 0;
int stepperPosition2 = 0;
int stepperPosition3 = 0;
int stepperPositionZ = 0;


int maxDistance1 = 0; // joint 1
int maxDistance2 = 0; // joint 2
int maxDistance3 = 0; // joint 3
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
  //homeStepper2();
  homeStepper1();
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
  // stepper3.setCurrentPosition(0);
  // stepper3.setSpeed(1300);

  // while (digitalRead(Z_LIMIT_MIN) == 1) {
  //   stepper3.runSpeed();
  // }

  // maxDistance1 = stepper3.currentPosition();
  // Serial.print("Max Joint 1 Distance: ");
  // Serial.println(maxDistance1);

  // delay(20);

  // stepper3.setCurrentPosition(maxDistance1);

  // stepperPosition1 = 0;
  // stepper3.moveTo(stepperPosition1);

  // while (stepper3.currentPosition() != stepperPosition1) {
  //   stepper3.run();
  // }
  start = micros();
  stepper3.setCurrentPosition(0);
  stepper3.setSpeed(1100);
  stepper3.moveTo(3000);
  
  while (stepper3.currentPosition() != 3000) {
    stepper3.run();
  }
  end=micros();
  timer = end - start;
  Serial.print("Safe Position Reached: ");
  Serial.print(timer);
  Serial.println(stepper3.currentPosition());
}

