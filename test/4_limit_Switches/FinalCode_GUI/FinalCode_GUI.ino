/*
   Arduino based SCARA Robot 
   by Dejan, www.HowToMechatronics.com
   AccelStepper: http://www.airspayce.com/mikem/arduino/AccelStepper/index.html

*/
#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

#define limitSwitch1 9 // joint 1
#define limitSwitch2 10 // joint 2
#define limitSwitch3 11 // joint 3 
#define limitSwitchZ A3

// Define the stepper motors and the pins the will use
AccelStepper stepper1(1, 2, 5); // (Type:driver, STEP, DIR)
AccelStepper stepper2(1, 3, 6);
AccelStepper stepper3(1, 4, 7);
AccelStepper stepperZ(1, 12, 13);

Servo gripperServo;  // create servo object to control a servo

long maxDistance_Z = 5000;
long maxDistance_S1 = 100;
long maxDistance_S2 = 100;
long maxDistance_S3 = 100;

double x = 10.0;
double y = 10.0;
double L1 = 228; // L1 = 228mm
double L2 = 136.5; // L2 = 136.5mm
double theta1, theta2, phi, z;

int stepper1Position, stepper2Position, stepper3Position, stepperZPosition;

const float theta1AngleToSteps = 44.444444;
const float theta2AngleToSteps = 35.555555;
const float phiAngleToSteps = 10;
const float zDistanceToSteps = 100;

byte inputValue[5];
int k = 0;
int data[10];

int theta1Array[100];
int theta2Array[100];
int phiArray[100];
int zArray[100];
int gripperArray[100];
int positionsCounter = 0;

void setup() {
  Serial.begin(115200);

  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  pinMode(limitSwitch3, INPUT_PULLUP);
  pinMode(limitSwitchZ, INPUT_PULLUP);

  // Stepper motors max speed
  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(2000);
  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(2000);
  stepper3.setMaxSpeed(4000);
  stepper3.setAcceleration(2000);
  stepperZ.setMaxSpeed(4000);
  stepperZ.setAcceleration(2000);

  gripperServo.attach(A0, 600, 2500);
  // initial servo value - open gripper
  data[6] = 180;
  gripperServo.write(data[6]);
  delay(1000);
  data[5] = 100;
  homing();
}

void loop() {

  if (Serial.available()) {
    String content = Serial.readStringUntil('\n');
    for (int i = 0; i < 9; i++) {                  
        int index = content.indexOf(",");
        data[i] = atol(content.substring(0, index).c_str());
        content = content.substring(index + 1);
      }
    /*
     data[0] - SAVE button status
     data[1] - RUN button status
     data[2] - Joint 1 angle
     data[3] - Joint 2 angle
     data[4] - Joint 3 angle
     data[5] - Z position
     data[6] - Gripper value
     data[7] - Speed value
     data[8] - Acceleration value
    */
    // If SAVE button is pressed, store the data into the appropriate arrays
    if (data[0] == 1) {
      theta1Array[positionsCounter] = data[2] * theta1AngleToSteps; //store the values in steps = angles * angleToSteps variable
      theta2Array[positionsCounter] = data[3] * theta2AngleToSteps;
      phiArray[positionsCounter] = data[4] * phiAngleToSteps;
      zArray[positionsCounter] = data[5] * zDistanceToSteps;
      gripperArray[positionsCounter] = data[6];
      positionsCounter++;
    }
    // clear data
    if (data[0] == 2) {
      // Clear the array data to 0
      memset(theta1Array, 0, sizeof(theta1Array));
      memset(theta2Array, 0, sizeof(theta2Array));
      memset(phiArray, 0, sizeof(phiArray));
      memset(zArray, 0, sizeof(zArray));
      memset(gripperArray, 0, sizeof(gripperArray));
      positionsCounter = 0;
    }
  }
  
  // If RUN button is pressed
  while (data[1] == 1) {
    stepper1.setSpeed(data[7]);
    stepper2.setSpeed(data[7]);
    stepper3.setSpeed(data[7]);
    stepperZ.setSpeed(data[7]);
    stepper1.setAcceleration(data[8]);
    stepper2.setAcceleration(data[8]);
    stepper3.setAcceleration(data[8]);
    stepperZ.setAcceleration(data[8]);

    // execute the stored steps
    for (int i = 0; i <= positionsCounter - 1; i++) {
      if (data[1] == 0) {
        break;
      }
      stepper1.moveTo(theta1Array[i]);
      stepper2.moveTo(theta2Array[i]);
      stepper3.moveTo(phiArray[i]);
      stepperZ.moveTo(zArray[i]);
      while (stepper1.currentPosition() != theta1Array[i] || stepper2.currentPosition() != theta2Array[i] || stepper3.currentPosition() != phiArray[i] || stepperZ.currentPosition() != zArray[i]) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
        stepperZ.run();
      }
      if (i == 0) {
        gripperServo.write(gripperArray[i]);
      }
      else if (gripperArray[i] != gripperArray[i - 1]) {
        gripperServo.write(gripperArray[i]);
        delay(800); // wait 0.8s for the servo to grab or drop - the servo is slow
      }

      //check for change in speed and acceleration or program stop
      if (Serial.available()) {
          String content = Serial.readStringUntil('\n');
          for (int i = 0; i < 9; i++) {                  
              int index = content.indexOf(",");
              data[i] = atol(content.substring(0, index).c_str());
              content = content.substring(index + 1);
            }

        if (data[1] == 0) {
          break;
        }
        // change speed and acceleration while running the program
        stepper1.setSpeed(data[7]);
        stepper2.setSpeed(data[7]);
        stepper3.setSpeed(data[7]);
        stepperZ.setSpeed(data[7]);
        stepper1.setAcceleration(data[8]);
        stepper2.setAcceleration(data[8]);
        stepper3.setAcceleration(data[8]);
        stepperZ.setAcceleration(data[8]);
      }
    }
    /*
      // execute the stored steps in reverse
      for (int i = positionsCounter - 2; i >= 0; i--) {
      if (data[1] == 0) {
        break;
      }
      stepper1.moveTo(theta1Array[i]);
      stepper2.moveTo(theta2Array[i]);
      stepper3.moveTo(phiArray[i]);
      stepperZ.moveTo(zArray[i]);
      while (stepper1.currentPosition() != theta1Array[i] || stepper2.currentPosition() != theta2Array[i] || stepper3.currentPosition() != phiArray[i] || stepperZ.currentPosition() != zArray[i]) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
        stepperZ.run();
      }
      gripperServo.write(gripperArray[i]);

      if (Serial.available()) {
        content = Serial.readString(); // Read the incomding data from Processing
        // Extract the data from the string and put into separate integer variables (data[] array)
        for (int i = 0; i < 10; i++) {
          int index = content.indexOf(","); // locate the first ","
          data[i] = atol(content.substring(0, index).c_str()); //Extract the number from start to the ","
          content = content.substring(index + 1); //Remove the number from the string
        }
        if (data[1] == 0) {
          break;
        }
      }
      }
    */
  }

  stepper1Position = data[2] * theta1AngleToSteps;
  stepper2Position = data[3] * theta2AngleToSteps;
  stepper3Position = data[4] * phiAngleToSteps;
  stepperZPosition = data[5] * zDistanceToSteps;

  stepper1.setSpeed(data[7]);
  stepper2.setSpeed(data[7]);
  stepper3.setSpeed(data[7]);
  stepperZ.setSpeed(data[7]);

  stepper1.setAcceleration(data[8]);
  stepper2.setAcceleration(data[8]);
  stepper3.setAcceleration(data[8]);
  stepperZ.setAcceleration(data[8]);

  stepper1.moveTo(stepper1Position);
  stepper2.moveTo(stepper2Position);
  stepper3.moveTo(stepper3Position);
  stepperZ.moveTo(stepperZPosition);

  while (stepper1.currentPosition() != stepper1Position || stepper2.currentPosition() != stepper2Position || stepper3.currentPosition() != stepper3Position || stepperZ.currentPosition() != stepperZPosition) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepperZ.run();
  }
  delay(100);
  gripperServo.write(data[6]);
  delay(300);
}

void serialFlush() {
  while (Serial.available() > 0) {  //while there are characters in the serial buffer, because Serial.available is >0
    Serial.read();         // get one character
  }
}

void homing() {
  // Homing stepperZ
  while (digitalRead(limitSwitchZ) != 1) {
    stepperZ.setSpeed(1500);
    stepperZ.runSpeed();
    stepperZ.setCurrentPosition(maxDistance_Z); // When limit switch pressed set position to 0 steps
  }
  delay(20);
  stepperZ.moveTo(10000); // هتتغير علي حسب ال maxDistance_Z
  while (stepperZ.currentPosition() != 10000) {
    stepperZ.run();
  }

  // Homing Stepper3
  while (digitalRead(limitSwitch3) != 1) {
    stepper3.setSpeed(-1100);
    stepper3.runSpeed();
    stepper3.setCurrentPosition(-maxDistance_S3); // When limit switch pressed set position to 0 steps
  }
  delay(20);

  stepper3.moveTo(0);
  while (stepper3.currentPosition() != 0) {
    stepper3.run();
  }

  // Homing Stepper2
  while (digitalRead(limitSwitch2) != 1) {
    stepper2.setSpeed(-1300);
    stepper2.runSpeed();
    stepper2.setCurrentPosition(-maxDistance_S2); // When limit switch pressed set position to -5440 steps
  }
  delay(20);

  stepper2.moveTo(0);
  while (stepper2.currentPosition() != 0) {
    stepper2.run();
  }

  // Homing Stepper1
  while (digitalRead(limitSwitch1) != 1) {
    stepper1.setSpeed(-1200);
    stepper1.runSpeed();
    stepper1.setCurrentPosition(-maxDistance_S1); // When limit switch pressed set position to 0 steps
  }
  delay(20);
  stepper1.moveTo(-50);
  while (stepper1.currentPosition() != -50) {
    stepper1.run();
  }
  stepper1.setCurrentPosition(0);
}
