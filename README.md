SCARA Robot (PRRR Configuration)
-----------------------------------------
🌟 Overview
----------------------------------------
This repository contains the design, build process, and control of a 4-DoF SCARA robot. The robot features a prismatic joint and three revolute joints (PRRR configuration) and is designed for precise pick-and-place tasks. It includes a functional GUI, real-time control, and an Arduino-based system for motor control.

<p align="center">
  <img src="https://github.com/user-attachments/assets/e19a59ea-998e-4cf3-8145-0a22f2858d83" alt="SCARA Robot 1" width="300" height="1000"/>
  &nbsp;&nbsp;&nbsp;
  <img src="https://github.com/user-attachments/assets/968f9d73-3855-4bc8-b43c-15e5222d4190" alt="SCARA Robot 2" width="300" height="350"/>
</p>

<p align="center"><strong>Figure 1: Final SCARA Robot</strong></p>

---
## 📖 1. Introduction  
This project focuses on building and analyzing a **SCARA (PRRR)** robot arm with **4 Degrees of Freedom (DoF)**. It combines both kinematic analysis and practical implementation, including calculations for forward kinematics, inverse kinematics, and Jacobian analysis. The project culminates in a demonstration where the robot performs precise pick-and-place tasks and can **hold objects weighing up to 1 kg**.

**This SCARA robot is developed for industrial pick-and-place applications.**  
The manipulator consists of:  
- **A prismatic joint** for vertical motion.  
- **Three revolute joints** for horizontal movements and rotation.  
- **A gripper** for handling square and round objects.  

The robot is controlled using an Arduino Uno and stepper motors through a CNC shield. The GUI simplifies user interaction, making the robot versatile for various tasks.  

---

## 🎯 2. Final Outcome  
The SCARA robot successfully performs pick-and-place tasks with smooth motion and precise control.  


https://github.com/user-attachments/assets/f581777a-0083-44fe-9ece-47c1c799fe9d

*Figure 2: Final Robot Outcome*  

------
--

## 🔌 4. Wiring  

<p align="center">
  <img src="https://github.com/ENG-ELSAYED-KANDIL/Scara-Robot/blob/main/Pictures/Wiring.png" width="50%">
</p>
<p align="center">
    <em>Figure 5: SCARA Robot main wiring diagram  </em>
</p>
<p align="center"> <em> Credit - how to mechatronics </em> </p>
 
The wiring connects stepper motors, the CNC shield, and the Arduino Uno. Here are the main considerations and steps followed:  

### **4.1 Motor Silence and Calibration**  
In wiring, I primarily focused on reducing motor noise. To achieve smooth motor operation, the current limit of the motor drivers was carefully adjusted. This calibration ensures precise motor activation. Setting an incorrect value can cause the motor drivers to overheat and potentially damage them. To further enhance motor precision, I adjusted the precision selector to 1/4 or 1/8 microstepping.

### **4.2 Connections and Testing**  
I connected the motor wiring as shown in the diagram and tested the connections with an oscilloscope and multimeter. Limit switches were placed at the corners of the joints' movement range to allow maximum rotational angles. After placing the limit switches, I assigned the pins for X+, Y+, Z+, and coolant functionalities.  

### **4.3 Servo Motor Powering**  
Since servo motors require a stable current, I initially tested them using the Arduino Uno with the CNC shield. However, the servo motor only received 3V from this setup, which was insufficient. To address this, I powered the servo motor separately using a 5V power supply.  

### **4.4 Power Supply Configuration**  
To power the entire system, which includes:  
- Four NEMA 17 stepper motors  
- Arduino Uno with CNC shield  
- Four motor drivers (DRV8825)  
- A servo motor  

I decided to use a 12V 10A power supply. A 12V 5A power supply would not have been sufficient to run all motors simultaneously.  

### **4.5 Cable Management**  
For clean and organized wiring, I used “Spiral Wire Wrapping Tube Cable Sleeves” and secured them with tie tags. This arrangement keeps the wires neat and minimizes clutter.  

### **4.6 Testing**  
To test all motors, I used simple Arduino code to verify their operation. This helped ensure that the motors, limit switches, and control pins were functioning as expected.  

### **4.7 Key Points**  
- **Shielded cables** were used for stepper motors to reduce electrical noise.  
- Correct polarity was ensured to avoid reversing motor directions.  
- The 12V DC power supply provided reliable power for the entire system.  
 

---

## 🏠 5. Homing Sequence
The homing sequence ensures that the robot starts from a known position:  

- The prismatic joint retracts to the lowest position.  
- Each revolute joint rotates to its home angle.  
- Limit switches provide feedback to confirm positions.  

As you can see in the video, the homing sequence begins with the gripper motor (joint 3). To determine the homing position, I used limit switches along with the following code:  
