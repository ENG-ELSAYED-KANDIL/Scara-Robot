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

## 📖 1. Introduction  
This project focuses on building and analyzing a **SCARA (PRRR)** robot arm with **4 Degrees of Freedom (DoF)**. It combines both kinematic analysis and practical implementation, including calculations for forward kinematics, inverse kinematics, and Jacobian analysis. The project culminates in a demonstration where the robot performs precise pick-and-place tasks and can **hold objects weighing up to 0.8 kg**.

**This SCARA robot is developed for industrial pick-and-place applications.**  
The manipulator consists of:  
- **A prismatic joint** for vertical motion.  
- **Three revolute joints** for horizontal movements and rotation.  
- **A gripper** for handling square and round objects.  

The robot is controlled using an Arduino Uno and stepper motors through a CNC shield. The GUI simplifies user interaction, making the robot versatile for various tasks.  

---
