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

#define GRIPPER_LIMIT A0    //stepper3
//////////////////////////////////////////////////////////

Servo gripperServo;

AccelStepper stepper1(1, 2, 5); //link 1 // x in driver 
AccelStepper stepper2(1, 3, 6);  //link 2 // y in driver 
AccelStepper stepper3(1, 4, 7); // gripper //z in driver 
AccelStepper stepperZ(1, 12, 13); // z axis // A in driver

int stepperPosition1 = 0;
int stepperPosition2 = 0;
int stepperPosition3 = 0;
int stepperPositionZ = 0;

int maxDistance1 = 1823; // joint 1
int maxDistance2 = 6429; // joint 2
int maxDistance3 = 4767; // joint 3
int maxDistanceZ = 0; // Z axis


int stepper1Position, stepper2Position, stepper3Position, stepperZPosition;

const float theta1AngleToSteps = 41.2611111;
const float theta2AngleToSteps = 40.7111111;
const float phiAngleToSteps    = 40.7111111;
const float zDistanceToSteps   = 100;

String content = "";

int data[10];

int theta1Array[100];
int theta2Array[100];
int phiArray[100];
int zArray[100];
int gripperArray[100];
int positionsCounter = 0;

void setup() {
  Serial.begin(9600);
  pinMode(8, OUTPUT);
  pinMode(X_LIMIT_MIN, INPUT_PULLUP);
  pinMode(X_LIMIT_MAX, INPUT_PULLUP);
  pinMode(Y_LIMIT_MIN, INPUT_PULLUP);
  pinMode(Y_LIMIT_MAX, INPUT_PULLUP);
  pinMode(GRIPPER_LIMIT,INPUT_PULLUP);

  // Invert Pins for Correct Direction
                              // direction , enable , stop
  stepper1.setPinsInverted(true, false, false);
  stepper2.setPinsInverted(true, false, false);
  stepper3.setPinsInverted(true, false, false);
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

  gripperServo.attach(11);
  data[6] = 90;
  gripperServo.write(data[6]);

  getLimits();
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
      //stepperZ.moveTo(zArray[i]);
      while (stepper1.currentPosition() != theta1Array[i] || stepper2.currentPosition() != theta2Array[i] || stepper3.currentPosition() != phiArray[i]) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
        // stepperZ.run();
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

  if (data[1] == 0)
  {
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
  // stepperZ.moveTo(stepperZPosition);

  while (stepper1.currentPosition() != stepper1Position || stepper2.currentPosition() != stepper2Position || stepper3.currentPosition() != stepper3Position ) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
    //stepperZ.run();
  
  }
  delay(100);
  gripperServo.write(data[6]);
  delay(300);
  }
}

void serialFlush() {
  while (Serial.available() > 0) {  //while there are characters in the serial buffer, because Serial.available is >0
    Serial.read();         // get one character
  }
}

void getLimits() {
  delay(1000);
  //homeZ();
  homeStepper2();
  homeStepper1();
  homeStepper3();
}

// joint 1
void homeStepper1() {
  stepper1.setCurrentPosition(0);
  stepper1.setSpeed(-1100);

  while (digitalRead(X_LIMIT_MIN) == 1) {
    stepper1.runSpeed();
  }

  delay(20);

  stepper1.setCurrentPosition(-maxDistance1);

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
  stepper2.setSpeed(-1300);

  while (digitalRead(Y_LIMIT_MIN) == 1) {
    stepper2.runSpeed();
  }

  delay(20);

  stepper2.setCurrentPosition(-maxDistance2);

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
  stepper3.setSpeed(-1300);

  while (digitalRead(GRIPPER_LIMIT) == 1) {
    stepper3.runSpeed();
  }

  delay(20);

  stepper3.setCurrentPosition(-maxDistance3);

  stepperPosition3 = 0;
  stepper3.moveTo(stepperPosition3);

  while (stepper3.currentPosition() != stepperPosition3) {
    stepper3.run();
  }

  Serial.print("Safe Position Reached: ");
  Serial.println(stepper3.currentPosition());
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


