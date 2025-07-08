// Final Code 4 DOF Scara Robot

#include <math.h>
#include <AccelStepper.h>
#include <Servo.h>

// #define limitSwitch1 9 // joint 1
// #define limitSwitch2 10 // joint 2
// #define limitSwitch3 11 // joint 3
// #define limitSwitchZ A3 // z axis

/////////////////////// LIMT SWITCH //////////////////////////////
#define limitSwitch1 9    // X+ //stepper1
#define limitSwitch2 10    // Y+ //stepper2
#define limitSwitch3 11    //stepper3
#define limitSwitchZ A3    // Z+ //stepperz

#define servo_pin    12
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


void setup()
{
  Serial.begin(9600);

  // Setup limit switches with pullup resistors
  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  pinMode(limitSwitch3, INPUT_PULLUP);
  pinMode(limitSwitchZ, INPUT_PULLUP);

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
      stepper1Position = theta1 * theta1AngleToSteps;
      stepper2Position = theta2 * theta2AngleToSteps;
      stepper3Position = phi * phiAngleToSteps;

      Serial.print("stepper1Position: ");
      Serial.println(stepper1Position);
      Serial.print("stepper2Position: ");
      Serial.println(stepper2Position);
      Serial.print("stepper3Position: ");
      Serial.println(stepper3Position);

      stepper1.setSpeed(2500);
      stepper2.setSpeed(2500);
      stepper3.setSpeed(2500);
      stepperZ.setSpeed(2500);

      stepper1.setAcceleration(2000);
      stepper2.setAcceleration(2000);
      stepper3.setAcceleration(2000);
      stepperZ.setAcceleration(2000);

      // تحريك المحركات
      stepper1.moveTo(stepper1Position);
      stepper2.moveTo(stepper2Position);
      stepper3.moveTo(stepper3Position);

      while (stepper1.currentPosition() != stepper1Position ||
             stepper2.currentPosition() != stepper2Position ||
             stepper3.currentPosition() != stepper3Position) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
      }
      

      // قراءة الألترا سونيك لحساب Z
      // digitalWrite(trig, LOW); 
      // delayMicroseconds(2);  
      // digitalWrite(trig, HIGH);  
      // delayMicroseconds(10);  
      // digitalWrite(trig, LOW);

      // duration = pulseIn(echo, HIGH);
      // distance = (duration / 2) * 0.0343; // cm

      // stepperZPosition = distance * zDistanceToSteps;
      // stepperZ.moveTo(stepperZPosition);

      // while(stepperZ.currentPosition() != stepperZPosition) {
      //   stepperZ.run();
      // }

      delay(1000);
      gripperServo.write(90); // close gripper
      delay(300);
}


void inverseKinematics(float x, float y, float* theta1, float* theta2, float* phi) {

  float max_reach = L1 + L2;
  float min_reach = abs(L1 - L2);
  float dist = sqrt(x * x + y * y);

  if (dist > max_reach || dist < min_reach) {
        Serial.println("❌ Error: Target outside reachable area!");
        return;
    }
  // y=y-L0;
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

  while (!digitalRead(limitSwitchZ))
  {
    stepperZ.moveTo(initial_homing);
    initial_homing++;
    stepperZ.run();
    delay(5);
  }
  stepperZ.setCurrentPosition(maxDistance_Z);
  delay(20);
  stepperZ.moveTo(2500); /// maxDistance_Z ----;

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

  while (!digitalRead(limitSwitch1))
  {
    stepper1.moveTo(initial_homing);
    initial_homing++;
    stepper1.run();
    delay(5);
  }

  stepper1.setCurrentPosition(maxDistance_S1);
  initial_homing = maxDistance_S1 - 1;
  delay(5);
  
  while (digitalRead(limitSwitch1))
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

  while (!digitalRead(limitSwitch2))
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

  while (!digitalRead(limitSwitch3))
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

