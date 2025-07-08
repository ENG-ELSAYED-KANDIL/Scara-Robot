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
int maxDistanceZ = -20000; // Z axis


int stepper1Position, stepper2Position, stepper3Position, stepperZPosition;

const float theta1AngleToSteps = 41.2611111;
const float theta2AngleToSteps = 40.7111111;
const float phiAngleToSteps    = 40.7111111;
const float zDistanceToSteps   = 100;


float currentShoulderAngle = 0.0;
float currentElbowAngle = 0.0;
float currentWristAngle = 0.0;
float currentZHeight = 0.0;
int currentGripperPosition = 0;

// Communication variables
String inputBuffer = "";
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 1000;

bool isHomed = false;
bool emergencyStopActive = false;
bool motorsEnabled = false;
float currentSpeed = 500;  // Percentage
float currentAcceleration = 500;  // Percentage


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
  initializeSteppers();

  // Clear serial buffer
  while (Serial.available()) {
        Serial.read();
    }

  gripperServo.attach(11);
  gripperServo.write(90);

  Serial.println("SCARA_READY");
  Serial.println("INFO: SCARA Robot Controller v2.0 Initialized");
  //homeRobot();
}


void loop() {

  // Check emergency stop
  checkEmergencyStop();
    
  // Process serial commands
  processSerialCommands();
    
  // Send periodic heartbeat
  if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
        sendHeartbeat();
        lastHeartbeat = millis();
    }
    
}

void serialFlush() {
  while (Serial.available() > 0) {  //while there are characters in the serial buffer, because Serial.available is >0
    Serial.read();         // get one character
  }
}



// ===== Command Processing =====
void processSerialCommands() {
    while (Serial.available()) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                executeCommand(inputBuffer);
                inputBuffer = "";
            }
        } else {
            inputBuffer += c;
            if (inputBuffer.length() > 100) {
                inputBuffer = "";  // Prevent buffer overflow
            }
        }
    }
}

void executeCommand(String command) {
    command.trim();
    
    if (command == "PING") {
        Serial.println("PONG"); //conection
    }
    else if (command == "HOME") {
        homeRobot();
    }
    else if (command.startsWith("MOVE_JOINT")) {
        parseMoveJoint(command);
    }
    else if (command.startsWith("GRIPPER")) {
        parseGripper(command);
    }
    else if (command.startsWith("SET_MOTION")) {
        parseSetMotion(command); //speed and acceleration
    }
    else if (command == "GET_STATUS") {
        sendStatus();
    }
    else if (command == "GET_POSITION") {
        sendPosition();
    }
    else if (command == "EMERGENCY_STOP") {
        activateEmergencyStop();
    }
    else if (command == "RESET_EMERGENCY") {
        resetEmergencyStop();
    }
    // else if (command == "GET_ULTRASONIC") {
    //     sendUltrasonicReading();
    // }
    else {
        Serial.println("ERROR: Unknown command: " + command);
    }
}


// ===== Motion Settings =====
void initializeSteppers() {
    stepper1.setMaxSpeed(4000);
    stepper1.setAcceleration(2000);
    stepper2.setMaxSpeed(4000);
    stepper2.setAcceleration(2000);
    stepper3.setMaxSpeed(4000);
    stepper3.setAcceleration(2000);
    stepperZ.setMaxSpeed(4000);
    stepperZ.setAcceleration(2000);
}

void initializeSteppers2()
{
  stepper1.setSpeed(currentSpeed);
  stepper2.setSpeed(currentSpeed);
  stepper3.setSpeed(currentSpeed);
  stepperZ.setSpeed(currentSpeed);

  stepper1.setAcceleration(currentAcceleration);
  stepper2.setAcceleration(currentAcceleration);
  stepper3.setAcceleration(currentAcceleration);
  stepperZ.setAcceleration(currentAcceleration);

}
void parseSetMotion(String command) {
    float speed = parseValue(command, "SPEED");
    float accel = parseValue(command, "ACCEL");
    
    if (speed >= 500 && speed <= 4000) {
        currentSpeed = speed;
    }
    
    if (accel >= 500 && accel<= 4000) { // أو نطاق آخر للتسارع
        currentAcceleration = accel;
    }
    
    initializeSteppers2();
    Serial.println("OK: Motion settings updated");
}

// ===== Status and Feedback =====
void sendStatus() {
    String status = "STATUS:";
    status += "connected=1,";
    status += "homed=" + String(isHomed ? 1 : 0) + ",";
    status += "emergency=" + String(emergencyStopActive ? 1 : 0) + ",";
    //status += "motors=" + String(motorsEnabled ? 1 : 0) + ",";
    status += "speed=" + String(currentSpeed) + ",";
    status += "acceleration=" + String(currentAcceleration);
    
    Serial.println(status);
}


// ===== positions =====
void sendPosition() {
    String position = "POSITION:";
    position += "shoulder=" + String(currentShoulderAngle, 2) + ",";
    position += "elbow=" + String(currentElbowAngle, 2) + ",";
    position += "wrist=" + String(currentWristAngle, 2) + ",";
    position += "z=" + String(currentZHeight, 2) + ",";
    position += "gripper=" + String(currentGripperPosition);
    
    Serial.println(position);
}

void sendHeartbeat() {
    // Only send heartbeat if connected and no other communication
    if (millis() - lastHeartbeat > 5000) {
        Serial.println("HEARTBEAT");
    }
}

// ===== Safety Functions =====
bool checkEmergencyStop() {
    if (digitalRead(X_LIMIT_MIN) == LOW || digitalRead(Y_LIMIT_MIN) == LOW || digitalRead(GRIPPER_LIMIT) == LOW || digitalRead(Z_LIMIT_MAX) == LOW || digitalRead(Z_LIMIT_MIN) == LOW) {
        if (!emergencyStopActive) {
            activateEmergencyStop();
        }
        return true;
    }
    return false;
}



void activateEmergencyStop() {
    emergencyStopActive = true;
    stopAllMotors();
    Serial.println("EMERGENCY_STOP_ACTIVATED");
}

void resetEmergencyStop() {
    if (digitalRead(X_LIMIT_MIN) == HIGH || digitalRead(Y_LIMIT_MIN) == HIGH || digitalRead(GRIPPER_LIMIT) == HIGH || digitalRead(Z_LIMIT_MAX) == HIGH || digitalRead(Z_LIMIT_MIN) == HIGH) {
        emergencyStopActive = false;
        Serial.println("OK: Emergency stop reset");
    } else {
        Serial.println("ERROR: Emergency stop button still pressed");
    }
}

void stopAllMotors() {
    stepper1.stop();
    stepper2.stop();
    stepper3.stop();
    stepperZ.stop();
}

// ===== Movement Commands =====
void parseMoveJoint(String command) {
    if (!isHomed) {
        Serial.println("ERROR: Robot not homed");
        return;
    }
    
    if (emergencyStopActive) {
        Serial.println("ERROR: Emergency stop active");
        return;
    }
    
    // Parse joint angles from command
    float shoulder = parseValue(command, 'S');
    float elbow = parseValue(command, 'D');
    float wrist = parseValue(command, 'W');
    float z = parseValue(command, 'Z');
    
    // Convert to steps
    long shoulderSteps = shoulder*theta1AngleToSteps;
    long elbowSteps = elbow*theta2AngleToSteps;
    long wristSteps = wrist*phiAngleToSteps;
    long zSteps = -z*zDistanceToSteps;

    initializeSteppers2();
    
    // Set target positions
    stepper1.moveTo(shoulderSteps);
    stepper2.moveTo(elbowSteps);
    stepper3.moveTo(wristSteps);
    stepperZ.moveTo(zSteps);
    
    // Update current position tracking
    currentShoulderAngle = shoulder;
    currentElbowAngle = elbow;
    currentWristAngle = wrist;
    currentZHeight = z;
    
    // Wait for movement to complete
    while (stepper1.currentPosition() != shoulderSteps || stepper2.currentPosition() != elbowSteps || stepper3.currentPosition() != wristSteps || stepperZ.currentPosition() != zSteps) {
        checkEmergencyStop();
        if (emergencyStopActive) {
            stopAllMotors();
            Serial.println("ERROR: Emergency stop during movement");
            return;
        }
        
        stepper1.run();
        stepper2.run();
        stepper3.run();
        stepperZ.run();
    }
    
    Serial.println("OK: Joint movement completed");
}


// ===== Gripper Control =====
void parseGripper(String command) {
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex == -1) {
        Serial.println("ERROR: Invalid gripper command");
        return;
    }
    int position = command.substring(spaceIndex + 1).toInt();
    position = constrain(position, 0, 90);
    gripperServo.write(position);
    currentGripperPosition = position;
    delay(500);  // Allow time for servo to move
    Serial.println("OK: Gripper position set to " + String(position));
}


// ===== Utility Functions =====
float parseValue(String command, char identifier) {
    int index = command.indexOf(identifier);
    if (index == -1) return 0.0;
    
    int start = index + 1;
    int end = start;
    
    while (end < command.length() && 
           (isDigit(command[end]) || command[end] == '.' || command[end] == '-')) {
        end++;
    }
    
    return command.substring(start, end).toFloat();
}

float parseValue(String command, String identifier) {
    int index = command.indexOf(identifier);
    if (index == -1) return 0.0;
    
    int start = index + identifier.length();
    int end = start;
    
    while (end < command.length() && 
           (isDigit(command[end]) || command[end] == '.' || command[end] == '-')) {
        end++;
    }
    
    return command.substring(start, end).toFloat();
}


//HOMING PROCESS
void homeRobot() {
  delay(1000);
  homeStepper2();
  homeStepper1();
  homeStepper3();
  homeStepperZ();
  isHomed = true;
  Serial.println("oK");
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

}

//joint 3
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
}

void homeStepperZ() {
  stepperZ.setCurrentPosition(0);
  stepperZ.setSpeed(1100);

  while (digitalRead(Z_LIMIT_MIN) == 1) {
    stepperZ.runSpeed();
  }

  delay(20);
  stepperZ.setCurrentPosition(0); /// max distance

  stepperZ.moveTo(maxDistanceZ); // 55 mm

  while (stepperZ.currentPosition() != maxDistanceZ) {
    stepperZ.run();
  }

}





