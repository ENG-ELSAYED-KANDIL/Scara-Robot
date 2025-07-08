// Final Code 4 DOF Scara Robot

#include <math.h>
#include <AccelStepper.h>
#include <Servo.h>

// #define limitSwitch1 9 // joint 1
// #define limitSwitch2 10 // joint 2
// #define limitSwitch3 11 // joint 3
// #define limitSwitchZ A3 // z axis

/////////////////////// LIMT SWITCH //////////////////////////////
#define X_LIMIT_MIN 9     // X- //stepper1
#define X_LIMIT_MAX A0    // X+ //stepper1

#define Y_LIMIT_MIN 10    // Y- //stepper2
#define Y_LIMIT_MAX A1    // Y+ //stepper2

#define Z_LIMIT_MIN 11    // Z- //stepperz
#define Z_LIMIT_MAX A2    // Z+ //stepperz

#define servo_pin    12
#define GRIPPER_LIMIT A3    //stepper3
/////////////////////// LIMT SWITCH //////////////////////////////

/////////// ULTRA SONIC
#define trig 5
#define echo 8

long duration , distance;

AccelStepper stepper1(1, 2, 5); //link 1 
AccelStepper stepper2(1, 3, 6); // link 2
AccelStepper stepper3(1, 4, 7); // rotation gripper
AccelStepper stepperZ(1, 12, 13); // z axis

Servo gripperServo;

double l0= 68;
double L1 = 228;
double L2 = 136.5;

double x = 200;
double y = 200;

long initial_homing = 1;

long maxDistance_Z = 5000;
long maxDistance_S1 = 100;
long maxDistance_S2 = 100;
long maxDistance_S3 = 100;

float theta1, theta2, phi;

int stepper1Position, stepper2Position, stepper3Position, stepperZPosition;

const float theta1AngleToSteps = 44.444444;
const float theta2AngleToSteps = 35.555555;
const float phiAngleToSteps = 10;
const float zDistanceToSteps = 100;

int data[9];

int theta1Array[100];
int theta2Array[100];
int phiArray[100];
int zArray[100];
int gripperArray[100];
int positionsCounter = 0;
bool readyForUltrasonic = false; 
static float lastSentDistance = -1;
void setup()
{
  Serial.begin(9600);

  // Setup limit switches with pullup resistors
  pinMode(X_LIMIT_MIN, INPUT_PULLUP);
  pinMode(X_LIMIT_MAX, INPUT_PULLUP);
  pinMode(Y_LIMIT_MIN, INPUT_PULLUP);
  pinMode(Y_LIMIT_MAX, INPUT_PULLUP);
  pinMode(Z_LIMIT_MIN, INPUT_PULLUP);
  pinMode(Z_LIMIT_MAX, INPUT_PULLUP);

  //setup ultra sonic
  pinMode(trig, OUTPUT); 
  pinMode(echo, INPUT);

  
  // Invert Pins for Correct Direction
                              // enable , direction , stop
  // stepper1.setPinsInverted(false, true, false);
  // stepper2.setPinsInverted(false, true, false);
  // stepper3.setPinsInverted(false, true, false);
  // stepperZ.setPinsInverted(false, true, false);

  stepperZ.setMaxSpeed(4000);
  stepperZ.setAcceleration(2000);
  stepper3.setMaxSpeed(4000);
  stepper3.setAcceleration(2000);
  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(2000);
  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(2000);

  inverseKinematics(x, y, &theta1, &theta2, &phi);
  Serial.print("Theta1: "); Serial.println(theta1);
  Serial.print("Theta2: "); Serial.println(theta2);
  Serial.print("Phi: "); Serial.println(phi); 

  gripperServo.attach(servo_pin, 600, 2500);
  // initial servo value - open gripper
  int open_gripper = 180;
  gripperServo.write(open_gripper);
  delay(1000);
  homing();

}

void loop()
{
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
      if (data[1] != 1){
      readyForUltrasonic = true;}
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
      while (stepper1.currentPosition() != theta1Array[i] || stepper2.currentPosition() != theta2Array[i] || stepper3.currentPosition() != phiArray[i]) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
      }
      delay(1000);
      digitalWrite(trig, LOW);
      delayMicroseconds(2);
      digitalWrite(trig, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig, LOW);

      duration = pulseIn(echo, HIGH);
      distance = (duration / 2.0) * 0.0343;
      data[5] = distance;
      stepperZPosition = distance * zDistanceToSteps;

      stepperZ.moveTo(stepperZPosition);
      while (stepperZ.currentPosition() != stepperZPosition) {
        if (digitalRead(Z_LIMIT_MIN) == HIGH || digitalRead(Z_LIMIT_MAX) == HIGH) {
          stepperZ.stop();
          Serial.println("⚠️ Z Limit!");
          break;
        }
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
      while (stepper1.currentPosition() != theta1Array[i] || stepper2.currentPosition() != theta2Array[i] || stepper3.currentPosition() != phiArray[i]) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
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

  if (stepperZ.currentPosition() != 2500){
      stepperZ.moveTo(2500);
      while (stepperZ.currentPosition() != 2500) {
        if (digitalRead(Z_LIMIT_MIN) == HIGH || digitalRead(Z_LIMIT_MAX) == HIGH) {
        stepperZ.stop();
        Serial.println("⚠️ Z Limit Reached!");
        break;
        }
        stepperZ.run();
      }
  }

  stepper1Position = data[2] * theta1AngleToSteps;
  stepper2Position = data[3] * theta2AngleToSteps;
  stepper3Position = data[4] * phiAngleToSteps;
  //stepperZPosition = data[5] * zDistanceToSteps;

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

  while (stepper1.currentPosition() != stepper1Position || stepper2.currentPosition() != stepper2Position || stepper3.currentPosition() != stepper3Position) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }


  if (readyForUltrasonic) {
    // قراءة الألترا سونيك لحساب Z
    digitalWrite(trig, LOW); 
    delayMicroseconds(2);  
    digitalWrite(trig, HIGH);  
    delayMicroseconds(10);  
    digitalWrite(trig, LOW);

    duration = pulseIn(echo, HIGH);
    distance = (duration / 2) * 0.0343; // cm

    if (distance != lastSentDistance) {
      Serial.print("Z:"); // كلمة مفتاحية للتعرف على القيمة
      Serial.println(distance);
      lastSentDistance = distance;
    }

    stepperZPosition = distance * zDistanceToSteps;
    stepperZ.moveTo(stepperZPosition);

    while(stepperZ.currentPosition() != stepperZPosition) {
        if (digitalRead(Z_LIMIT_MIN) == HIGH || digitalRead(Z_LIMIT_MAX) == HIGH) {
            stepperZ.stop();
            Serial.println("⚠️ Z Limit!");
        }
        stepperZ.run();
    }

    delay(1000);
    gripperServo.write(data[6]); // close gripper
    delay(300);
    // بعد ما يخلص الالترا سونيك، نقفل الفلاغ
    readyForUltrasonic = false;
  }

}


void inverseKinematics(float x, float y, float* theta1, float* theta2, float* phi) {
  float max_reach = L1 + L2;
  float min_reach = abs(L1 - L2);
  float dist = sqrt(x * x + y * y);

  if (dist > max_reach || dist < min_reach) {
        Serial.println("❌ Error: Target outside reachable area!");
        return;
    }
  // x=x-L0;
  float denominator = 2 * L1 * L2;
  float numerator = (x*x + y*y - L1*L1 - L2*L2);
  float fraction = numerator / denominator;
  if (fraction > 1 || fraction < -1) {
    Serial.println("❌ النقطة خارج نطاق الوصول!");
    return;
  }
  float t2 = -acos(fraction);
  

  *theta2 = round(t2 * 180.0 / PI * 10) / 10;

  float t1 = atan2(y, x) - atan2(L2 * sin(t2), L1 + L2 * cos(t2));
  *theta1 = round(t1 * 180.0 / PI * 10) / 10;

  *phi = 90 + *theta1 + *theta2;
  *phi = -(*phi);

  if (x < 0 && y < 0) {
    *phi = 270 - *theta1 - *theta2;
  }


  if (abs(*phi) > 165) {
    *phi = 180 + *phi;
  }
  // عرض النتائج
  // Serial.print("theta1 : ");
  // Serial.print(*theta1);
  // Serial.println("°");
  
  // Serial.print("theta2 :");
  // Serial.print(*theta2);
  // Serial.println("°");
  
  // Serial.print(" (z): ");
  // Serial.print(z);
  // Serial.println(" mm");

  
}

// Home All Stepper Motors
void homing()
{
  Serial.println("Stepper is homing...");
  homeAxis_Z();
  //  homeAxis_S1();
  //  homeAxis_S2();
  //  homeAxis_S3();

  Serial.println("Homing Completed!");

  Serial.println(stepperZ.currentPosition());
}

// Home The Z Axis Stepper Motor
void homeAxis_Z()
{
  stepperZ.setSpeed(1500);
  initial_homing = 1;

  while (!digitalRead(Z_LIMIT_MAX))
  {
    stepperZ.moveTo(initial_homing);
    initial_homing++;
    stepperZ.run();
    delay(5);
  }
  stepperZ.setCurrentPosition(maxDistance_Z);
  delay(20);
  stepperZ.moveTo(2500);

  while (stepperZ.currentPosition() != 2500) {
    stepperZ.run();
  }
  Serial.println("Z Homing Completed.");
}

// Home The S1 Stepper Motor 
void homeAxis_S1()
{
  stepper1.setSpeed(-1200);
  initial_homing = 1;

  while (!digitalRead(X_LIMIT_MAX))
  {
    stepper1.moveTo(initial_homing);
    initial_homing++;
    stepper1.run();
    delay(5);
  }

  stepper1.setCurrentPosition(maxDistance_S1);
  initial_homing = maxDistance_S1 - 1;
  delay(5);
  
  while (digitalRead(X_LIMIT_MAX))
  {
    stepper1.moveTo(initial_homing);
    stepper1.run();
    initial_homing--;
    delay(5);
  }
  stepper1.setCurrentPosition(0);

  Serial.println("S1 Homing Completed.");
}

// Home The S2 Stepper Motor
void homeAxis_S2()
{
  stepper2.setSpeed(-1300);
  initial_homing = 1;

  while (!digitalRead(Y_LIMIT_MAX))
  {
    stepper2.moveTo(initial_homing);
    initial_homing++;
    stepper2.run();
    delay(5);
  }

  stepper2.setCurrentPosition(maxDistance_S2);
  delay(20);
  stepper2.moveTo(0);

  while (stepper2.currentPosition() != 0)
  {
    stepper2.run();
  }
  Serial.println("S2 Homing Completed.");
}

// Home The S3 Stepper Motor (gripper)
void homeAxis_S3()
{
  stepper3.setSpeed(-1100);
  initial_homing = 1;

  while (!digitalRead(GRIPPER_LIMIT))
  {
    stepper3.moveTo(initial_homing);
    initial_homing++;
    stepper3.run();
    delay(5);
  }

  stepper3.setCurrentPosition(maxDistance_S3);
  delay(20);
  stepper3.moveTo(0);

  while (stepper3.currentPosition() != 0)
  {
    stepper3.run();
  }
  Serial.println("S3 Homing Completed.");
}


void checkLimits() {
    if (digitalRead(X_LIMIT_MIN) == HIGH || digitalRead(X_LIMIT_MAX) == HIGH) {
        stepper1.stop();
        Serial.println("⚠️ X Limit!");
    }
    if (digitalRead(Y_LIMIT_MIN) == HIGH || digitalRead(Y_LIMIT_MAX) == HIGH) {
        stepper2.stop();
        Serial.println("⚠️ Y Limit!");
    }
}