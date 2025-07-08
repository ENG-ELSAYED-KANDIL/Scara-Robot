#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

/////////////////////////////////////////////////////
#define X_LIMIT_MIN 9     // X- //stepper1
#define X_LIMIT_MAX A0    // X+ //stepper1

#define Y_LIMIT_MIN 10    // Y- //stepper2
#define Y_LIMIT_MAX A1    // Y+ //stepper2

#define Z_LIMIT_MIN A2   // Z- //stepperz
#define Z_LIMIT_MAX A1   // Z+ //stepperz

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

void homeStepper1() {
  stepper3.setCurrentPosition(0);       // Start from known 180° position
  stepper3.setSpeed(1100);              // Set movement speed

  // Move toward limit switch
  while (digitalRead(X_LIMIT_MIN) == 1) {
    stepper3.runSpeed();
  }

  maxDistance1 = stepper3.currentPosition();
  home = maxDistance1 - 1823;           // 7427 is the number of steps for 180°
  Serial.print("Max Joint 1 Distance: ");
  Serial.println(maxDistance1);

  // Move to home (0° position)
  stepper3.moveTo(home);
  while (stepper3.currentPosition() != home) {
    stepper3.runSpeed();
  }

  stepper3.setCurrentPosition(0);       // Set current position as 0°
  Serial.println(home);                 // This value represents 180°
}


