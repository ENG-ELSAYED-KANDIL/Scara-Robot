import asyncio
import json
import logging
import math
import sqlite3
import threading
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
import queue

import cv2
import numpy as np
import serial
from flask import Flask, Response, jsonify, request, render_template_string
from flask_socketio import SocketIO, emit
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
import io
import base64

# Try to import numpy for kinematics calculations
try:
    import numpy as np

    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False
    print("Warning: NumPy not available. Some kinematics functions may not work properly.")


@dataclass
class RobotConfig:
    """Robot physical configuration"""
    arm1_length: float = 190.0
    arm2_length: float = 168.0
    max_z_height: float = 300.0  # Updated to 300mm
    steps_per_degree: float = 8.888
    steps_per_mm_z: float = 80.0

    @property
    def max_reach(self) -> float:
        return self.arm1_length + self.arm2_length

    @property
    def min_reach(self) -> float:
        return abs(self.arm1_length - self.arm2_length)


@dataclass
class MotionSettings:
    """Motion control settings"""
    max_speed: int = 80  # Percentage (0-100)
    acceleration: int = 50  # Percentage (0-100)
    jog_speed: int = 800
    approach_speed: int = 500
    safety_limits_enabled: bool = True

    # Actual speed/acceleration values for display
    @property
    def actual_speed(self) -> float:
        """Convert percentage to actual steps/second (0-4000)"""
        return (self.max_speed / 100.0) * 4000.0

    @property
    def actual_acceleration(self) -> float:
        """Convert percentage to actual steps/second² (0-4000)"""
        return (self.acceleration / 100.0) * 4000.0


@dataclass
class VisionSettings:
    """Computer vision settings"""
    camera_index: int = 0
    resolution: Tuple[int, int] = (640, 480)
    hsv_lower: List[int] = None
    hsv_upper: List[int] = None
    blur_kernel: int = 5
    min_contour_area: int = 500
    calibration_matrix: Optional[np.ndarray] = None

    def __post_init__(self):
        if self.hsv_lower is None:
            self.hsv_lower = [20, 100, 100]
        if self.hsv_upper is None:
            self.hsv_upper = [30, 255, 255]


@dataclass
class SystemState:
    """Current system state"""
    is_connected: bool = False
    is_homed: bool = False
    emergency_stop: bool = False
    motors_enabled: bool = False
    operation_mode: str = "manual"
    current_position: List[float] = None
    target_position: List[float] = None
    gripper_position: int = 0
    ultrasonic_distance: float = 0.0
    last_detection: Optional[Tuple[float, float]] = None
    system_errors: List[str] = None
    pick_and_place_enabled: bool = False
    is_executing_pickplace: bool = False
    is_executing_sequence: bool = False
    current_sequence_task: Optional[str] = None
    sequence_progress: Dict = None

    def __post_init__(self):
        if self.current_position is None:
            self.current_position = [0.0, 0.0, 0.0, 0.0]
        if self.target_position is None:
            self.target_position = [0.0, 0.0, 0.0, 0.0]
        if self.system_errors is None:
            self.system_errors = []
        if self.sequence_progress is None:
            self.sequence_progress = {
                'current_task': 0,
                'current_loop': 0,
                'total_loops': 0,
                'is_running': False,
                'is_paused': False
            }


@dataclass
class PickPlaceSettings:
    """Pick and place operation settings"""
    pick_z_height: float = 50.0
    drop_x: float = 20.0
    drop_y: float = 10.0
    drop_z: float = 20.0
    grip_position: int = 45  # Updated range 0-90
    open_position: int = 0
    pick_delay: float = 1.0
    drop_delay: float = 1.0
    approach_height: float = 80.0
    auto_mode: bool = False


class RobotController:
    """Main robot controller class with enhanced functionality"""

    def __init__(self, config: RobotConfig, port: str = '/dev/ttyUSB0', baudrate: int = 115200):
        self.config = config
        self.port = port
        self.baudrate = baudrate
        self.serial_connection: Optional[serial.Serial] = None
        self.command_queue = queue.Queue()
        self.response_queue = queue.Queue()
        self.is_running = False
        self.lock = threading.Lock()

        # Pick and place settings
        self.pickplace_settings = PickPlaceSettings()

        self.logger = logging.getLogger(__name__)

    async def connect(self) -> bool:
        """Establish connection to Arduino"""
        try:
            ports_to_try = ['/dev/ttyACM0', self.port, '/dev/ttyUSB0', '/dev/ttyUSB1', 'COM8', 'COM4', 'COM5']

            for port in ports_to_try:
                try:
                    self.logger.info(f"Attempting to connect to port: {port}")

                    self.serial_connection = serial.Serial(
                        port,
                        self.baudrate,
                        timeout=2.0,
                        write_timeout=2.0,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE
                    )

                    await asyncio.sleep(3)

                    self.serial_connection.reset_input_buffer()
                    self.serial_connection.reset_output_buffer()

                    for attempt in range(3):
                        self.logger.info(f"Connection attempt {attempt + 1}/3")

                        response = await self.send_command("GET_STATUS", timeout=3.0)

                        if response and ("STATUS:" in response or "SCARA_READY" in response or "OK" in response):
                            self.is_running = True
                            self.port = port
                            self.logger.info(f"Successfully connected to robot on port {port}")
                            return True

                        response = await self.send_command("PING", timeout=2.0)
                        if response:
                            self.is_running = True
                            self.port = port
                            self.logger.info(f"Connected to device on port {port} (responded to PING)")
                            return True

                        await asyncio.sleep(1)

                    if self.serial_connection and self.serial_connection.is_open:
                        self.serial_connection.close()

                except serial.SerialException as se:
                    self.logger.warning(f"Failed to connect to {port}: {se}")
                    if self.serial_connection and self.serial_connection.is_open:
                        self.serial_connection.close()
                    continue
                except Exception as e:
                    self.logger.warning(f"Unexpected error with port {port}: {e}")
                    if self.serial_connection and self.serial_connection.is_open:
                        self.serial_connection.close()
                    continue

            self.logger.error("Failed to connect to any serial port")

        except Exception as e:
            self.logger.error(f"Connection failed: {e}")

        return False

    async def disconnect(self):
        """Disconnect from Arduino"""
        self.is_running = False
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            self.logger.info("Disconnected from robot")

    async def send_command(self, command: str, timeout: float = 5.0) -> Optional[str]:
        """Send command to Arduino and wait for response"""
        if not self.serial_connection or not self.serial_connection.is_open:
            return None

        try:
            with self.lock:
                self.serial_connection.reset_input_buffer()

                self.serial_connection.write(f"{command}\n".encode('utf-8'))
                self.serial_connection.flush()

                start_time = time.time()
                while time.time() - start_time < timeout:
                    if self.serial_connection.in_waiting > 0:
                        try:
                            raw_data = self.serial_connection.readline()

                            try:
                                response = raw_data.decode('utf-8').strip()
                            except UnicodeDecodeError:
                                try:
                                    response = raw_data.decode('latin-1').strip()
                                    self.logger.warning(f"Used latin-1 encoding for response: {response}")
                                except UnicodeDecodeError:
                                    response = raw_data.decode('utf-8', errors='ignore').strip()
                                    self.logger.warning(f"Ignored decode errors for response: {response}")

                            if response:
                                self.logger.debug(f"Command: {command} -> Response: {response}")
                                return response

                        except Exception as decode_error:
                            self.logger.error(f"Response decode error: {decode_error}")
                            continue

                    await asyncio.sleep(0.01)

        except Exception as e:
            self.logger.error(f"Command send error: {e}")

        return None

    async def home_robot(self) -> bool:
        """Home the robot to its reference position"""
        self.logger.info("Starting homing sequence")

        response = await self.send_command("HOME", timeout=30.0)

        if response:
            self.logger.debug(f"Homing response: {response}")

            if any(keyword in response for keyword in ["OK", "HOMING COMPLETED", "HOME COMPLETE", "HOMED"]):
                self.logger.info("Homing completed successfully")
                return True
            elif "ERROR" in response:
                self.logger.error(f"Homing failed with error: {response}")
                return False
            elif "INFO" in response or "WARNING" in response:
                self.logger.info(f"Homing in progress: {response}")

                final_response = await self.send_command("GET_STATUS", timeout=5.0)
                if final_response and "homed=1" in final_response:
                    self.logger.info("Homing completed (confirmed via status)")
                    return True
                else:
                    self.logger.warning("Homing status unclear, but proceeding")
                    return True
            else:
                self.logger.warning(f"Unexpected homing response: {response}")
                return True
        else:
            self.logger.error("No response received from homing command")
            return False

    async def move_joint(self, shoulder: float, elbow: float, wrist: float, z: float) -> bool:
        """Move robot using joint coordinates with updated limits"""
        self.logger.info(f"Attempting joint movement: S={shoulder:.2f}°, E={elbow:.2f}°, W={wrist:.2f}°, Z={z:.2f}mm")

        # Updated validation ranges
        if not (0 <= shoulder <= 180):  # Updated: 0 to 180 degrees
            self.logger.error(f"Shoulder angle {shoulder}° out of range (0 to 180)")
            return False
        if not (-90 <= elbow <= 90):  # Same as before
            self.logger.error(f"Elbow angle {elbow}° out of range (-90 to 90)")
            return False
        if not (-90 <= wrist <= 90):  # Updated: -90 to 90 degrees
            self.logger.error(f"Wrist angle {wrist}° out of range (-90 to 90)")
            return False
        if not (0 <= z <= 300):  # Updated: 0 to 300 mm
            self.logger.error(f"Z height {z}mm out of range (0 to 300)")
            return False

        command = f"MOVE_JOINT S{shoulder:.2f} B{elbow:.2f} W{wrist:.2f} Z{z:.2f}"
        self.logger.debug(f"Sending command: {command}")

        response = await self.send_command(command, timeout=10.0)

        if response:
            self.logger.debug(f"Robot response: {response}")
            if "OK" in response:
                self.logger.info("Joint movement successful")
                return True
            elif "ERROR" in response:
                self.logger.error(f"Robot reported error: {response}")

                if "not homed" in response.lower():
                    self.logger.info("Robot needs to be homed first")

                return False
            else:
                self.logger.warning(f"Unexpected response: {response}")
                return True
        else:
            self.logger.error("No response received from robot")
            return False

    async def move_cartesian(self, x: float, y: float, z: float) -> bool:
        """Move robot using Cartesian coordinates with improved inverse kinematics"""
        self.logger.info(f"Attempting Cartesian movement to: X={x:.2f}mm, Y={y:.2f}mm, Z={z:.2f}mm")

        # Calculate inverse kinematics
        success, theta1, theta2, phi, final_z = self.inverse_kinematics(x, y, z)

        if not success:
            self.logger.error(f"Cannot reach position ({x:.2f}, {y:.2f}, {z:.2f})")
            return False

        # Verify the solution with forward kinematics
        verify_x, verify_y = self.forward_kinematics(theta1, theta2, phi)
        error = math.sqrt((x - verify_x) ** 2 + (y - verify_y) ** 2)

        if error > 4.0:  # Allow 4mm tolerance
            self.logger.warning(f"IK solution has {error:.4f}mm error")

        self.logger.info(
            f"IK Solution: Shoulder={theta1:.1f}°, Elbow={theta2:.1f}°, Wrist={phi:.1f}°, Error={error:.2f}mm")

        # Use the joint movement command with calculated angles
        return await self.move_joint(theta1, theta2, phi, final_z)

    async def control_gripper(self, position: int) -> bool:
        """Control gripper position with updated range (0-90 degrees)"""
        # Updated validation range
        if not (0 <= position <= 90):  # Updated: 0 to 90 degrees
            self.logger.error(f"Gripper position {position}° out of range (0 to 90)")
            return False

        command = f"GRIPPER {position}"
        response = await self.send_command(command)

        return response and "OK" in response

    async def set_motion_settings(self, speed: int, acceleration: int) -> bool:
        """Set motion speed and acceleration (percentages, converted to actual values)"""
        # Validate ranges (0-100 percentages)
        speed = max(0, min(100, speed))
        acceleration = max(0, min(100, acceleration))

        # Convert percentages to actual values for Arduino
        actual_speed = (speed / 100.0) * 4000.0
        actual_acceleration = (acceleration / 100.0) * 4000.0

        command = f"SET_MOTION SPEED{actual_speed:.0f} ACCEL{actual_acceleration:.0f}"
        response = await self.send_command(command)

        return response and "OK" in response

    # === IMPROVED INVERSE KINEMATICS IMPLEMENTATION ===

    def inverse_kinematics(self, x: float, y: float, z: float = None) -> Tuple[bool, float, float, float, float]:
        """
        Improved inverse kinematics for SCARA robot with corrected equations

        Args:
            x, y: Target Cartesian coordinates in mm
            z: Target Z height in mm (optional, defaults to current or 50mm)

        Returns:
            Tuple of (success, theta1, theta2, phi, z) where:
            - theta1: shoulder angle in degrees (0 to 180)
            - theta2: elbow angle in degrees (-90 to 90)
            - phi: gripper orientation angle in degrees (-90 to 90)
            - z: vertical position in mm (0 to 300)
        """
        try:
            # Use numpy if available, otherwise fall back to math
            if NUMPY_AVAILABLE:
                import numpy as np
                sqrt_func = np.sqrt
                clip_func = np.clip
                arccos_func = np.arccos
                arctan2_func = np.arctan2
                sin_func = np.sin
                cos_func = np.cos
                radians_func = np.radians
                degrees_func = np.degrees
            else:
                sqrt_func = math.sqrt

                def clip_func(val, min_val, max_val):
                    return max(min_val, min(max_val, val))

                arccos_func = math.acos
                arctan2_func = math.atan2
                sin_func = math.sin
                cos_func = math.cos
                radians_func = math.radians
                degrees_func = math.degrees

            # Robot arm lengths and limits
            L1 = self.config.arm1_length  # 190mm
            L2 = self.config.arm2_length  # 168mm
            MAX_REACH = L1 + L2  # 358mm
            MIN_REACH = abs(L1 - L2)  # 22mm

            # Set default Z if not provided
            if z is None:
                z = 50.0  # Default height

            # Validate Z height with new range
            if not (0 <= z <= self.config.max_z_height):
                self.logger.warning(f"Z height {z}mm out of range (0 to {self.config.max_z_height})")
                return False, 0.0, 0.0, 0.0, z

            # Check if point is reachable
            distance = sqrt_func(x * x + y * y)
            if distance > MAX_REACH or distance < MIN_REACH:
                self.logger.warning(f"Target position ({x}, {y}) is out of reachable workspace")
                return False, 0.0, 0.0, 0.0, z

            # Improved inverse kinematics calculation
            # Calculate elbow angle using law of cosines
            cos_theta2 = (distance * distance - L1 * L1 - L2 * L2) / (2 * L1 * L2)
            cos_theta2 = clip_func(cos_theta2, -1.0, 1.0)  # Ensure valid range for acos

            # Calculate two possible solutions for elbow
            theta2_1 = arccos_func(cos_theta2)  # Elbow up configuration
            theta2_2 = -arccos_func(cos_theta2)  # Elbow down configuration

            # Choose elbow configuration based on workspace
            theta2 = theta2_1 if theta2_1 <= math.pi / 2 else theta2_2

            # Calculate shoulder angle
            alpha = arctan2_func(y, x)
            beta = arctan2_func(L2 * sin_func(theta2), L1 + L2 * cos_func(theta2))
            theta1 = alpha - beta

            # Convert to degrees
            theta1_deg = degrees_func(theta1)
            theta2_deg = degrees_func(theta2)

            # Normalize shoulder angle to 0-180 range
            while theta1_deg < 0:
                theta1_deg += 360
            while theta1_deg > 180:
                theta1_deg -= 360

            # If still out of range, try the other elbow configuration
            if theta1_deg < 0 or theta1_deg > 180:
                theta2 = theta2_2 if theta2 == theta2_1 else theta2_1
                beta = arctan2_func(L2 * sin_func(theta2), L1 + L2 * cos_func(theta2))
                theta1 = alpha - beta
                theta1_deg = degrees_func(theta1)
                theta2_deg = degrees_func(theta2)

                # Normalize again
                while theta1_deg < 0:
                    theta1_deg += 360
                while theta1_deg > 180:
                    theta1_deg -= 360

            # Calculate wrist orientation (phi) to keep end-effector parallel to base
            # For a SCARA robot, phi should compensate for shoulder and elbow rotation
            phi = -(theta1_deg + theta2_deg)

            # Normalize phi to -90 to 90 range
            while phi > 90:
                phi -= 180
            while phi < -90:
                phi += 180

            # Round values for consistency
            theta1_deg = round(theta1_deg, 2)
            theta2_deg = round(theta2_deg, 2)
            phi = round(phi, 2)

            # Validate final angles are within joint limits
            if not (0 <= theta1_deg <= 180):
                self.logger.warning(f"Shoulder angle {theta1_deg}° out of range (0-180)")
                return False, 0.0, 0.0, 0.0, z

            if not (-90 <= theta2_deg <= 90):
                self.logger.warning(f"Elbow angle {theta2_deg}° out of range (-90-90)")
                return False, 0.0, 0.0, 0.0, z

            if not (-90 <= phi <= 90):
                self.logger.warning(f"Wrist angle {phi}° out of range (-90-90)")
                return False, 0.0, 0.0, 0.0, z

            self.logger.info(
                f"IK Solution: ({x:.1f}, {y:.1f}, {z:.1f}) -> Shoulder: {theta1_deg}°, Elbow: {theta2_deg}°, Wrist: {phi}°")
            return True, theta1_deg, theta2_deg, phi, z

        except Exception as e:
            self.logger.error(f"Inverse kinematics calculation error: {e}")
            return False, 0.0, 0.0, 0.0, z if z is not None else 50.0

    def forward_kinematics(self, shoulder_deg: float, elbow_deg: float, wrist_deg: float = 0.0) -> Tuple[float, float]:
        """
        Calculate forward kinematics for SCARA robot (verified and corrected)

        Args:
            shoulder_deg, elbow_deg, wrist_deg: Joint angles in degrees

        Returns:
            Tuple of (x, y) Cartesian coordinates in mm
        """
        try:
            # Use numpy if available, otherwise fall back to math
            if NUMPY_AVAILABLE:
                import numpy as np
                cos_func = np.cos
                sin_func = np.sin
                radians_func = np.radians
            else:
                cos_func = math.cos
                sin_func = math.sin
                radians_func = math.radians

            # Convert to radians
            theta1_rad = radians_func(shoulder_deg)
            theta2_rad = radians_func(elbow_deg)

            # Robot arm lengths
            L1 = self.config.arm1_length  # 190mm
            L2 = self.config.arm2_length  # 168mm

            # Calculate end effector position using improved SCARA forward kinematics
            x = L1 * cos_func(theta1_rad) + L2 * cos_func(theta1_rad + theta2_rad)
            y = L1 * sin_func(theta1_rad) + L2 * sin_func(theta1_rad + theta2_rad)

            return float(x), float(y)

        except Exception as e:
            self.logger.error(f"Forward kinematics calculation error: {e}")
            return 0.0, 0.0

    def is_position_reachable(self, x: float, y: float, z: float) -> bool:
        """
        Check if a given Cartesian position is reachable by the robot

        Args:
            x, y, z: Target position in mm

        Returns:
            bool: True if position is reachable, False otherwise
        """
        try:
            # Check Z limits
            if not (0 <= z <= self.config.max_z_height):
                return False

            # Check if XY position is within workspace
            distance = math.sqrt(x ** 2 + y ** 2)

            # Check against robot workspace limits
            max_reach = self.config.max_reach  # L1 + L2
            min_reach = self.config.min_reach  # |L1 - L2|

            if distance > max_reach or distance < min_reach:
                return False

            # Additional check: try inverse kinematics
            success, theta1, theta2, phi, _ = self.inverse_kinematics(x, y, z)

            # Verify angles are within joint limits
            if not success:
                return False

            if not (0 <= theta1 <= 180):
                return False
            if not (-90 <= theta2 <= 90):
                return False
            if not (-90 <= phi <= 90):
                return False

            return True

        except Exception as e:
            self.logger.error(f"Reachability check error: {e}")
            return False

    def validate_cartesian_position(self, x: float, y: float, z: float) -> bool:
        """Validate if Cartesian position is reachable (alias for is_position_reachable)"""
        return self.is_position_reachable(x, y, z)

    async def emergency_stop(self) -> bool:
        """Activate emergency stop"""
        response = await self.send_command("EMERGENCY_STOP")
        return response and "EMERGENCY_STOP_ACTIVATED" in response

    async def reset_emergency(self) -> bool:
        """Reset emergency stop"""
        response = await self.send_command("RESET_EMERGENCY")
        return response and "OK" in response

    async def get_status(self) -> Optional[Dict]:
        """Get current robot status"""
        response = await self.send_command("GET_STATUS")

        if response and "STATUS:" in response:
            try:
                status_data = response.replace("STATUS:", "")
                status_dict = {}

                for item in status_data.split(","):
                    if "=" in item:
                        key, value = item.split("=", 1)
                        status_dict[key.lower()] = value

                return status_dict
            except Exception as e:
                self.logger.error(f"Status parsing error: {e}")

        return None

    async def get_position(self) -> Optional[List[float]]:
        """Get current joint positions"""
        response = await self.send_command("GET_POSITION")

        if response and "POSITION:" in response:
            try:
                pos_data = response.replace("POSITION:", "")
                positions = []

                for item in pos_data.split(","):
                    if "=" in item:
                        _, value = item.split("=", 1)
                        positions.append(float(value))

                return positions
            except Exception as e:
                self.logger.error(f"Position parsing error: {e}")

        return None

    # === PICK AND PLACE FUNCTIONALITY (UPDATED WITH NEW LIMITS) ===

    async def pick_and_place(self, pick_x: float, pick_y: float, drop_x: Optional[float] = None,
                             drop_y: Optional[float] = None, drop_z: Optional[float] = None) -> bool:
        """
        Execute pick and place operation with updated position validation

        Args:
            pick_x, pick_y: Object pickup coordinates
            drop_x, drop_y, drop_z: Drop location (uses defaults if None)

        Returns:
            bool: True if operation successful
        """
        try:
            self.logger.info(f"Starting pick and place operation: pick({pick_x:.1f}, {pick_y:.1f})")

            # Use default drop location if not specified
            if drop_x is None:
                drop_x = self.pickplace_settings.drop_x
            if drop_y is None:
                drop_y = self.pickplace_settings.drop_y
            if drop_z is None:
                drop_z = self.pickplace_settings.drop_z

            # Validate coordinates using new reachability function
            if not self.is_position_reachable(pick_x, pick_y, self.pickplace_settings.pick_z_height):
                self.logger.error("Invalid pickup coordinates - position not reachable")
                return False

            if not self.is_position_reachable(drop_x, drop_y, drop_z):
                self.logger.error("Invalid drop coordinates - position not reachable")
                return False

            # Step 1: Approach above pickup position
            approach_z = max(self.pickplace_settings.pick_z_height + 30, self.pickplace_settings.approach_height)
            self.logger.info(f"Step 1: Moving to approach position ({pick_x:.1f}, {pick_y:.1f}, {approach_z:.1f})")

            success = await self.move_cartesian(pick_x, pick_y, approach_z)
            if not success:
                self.logger.error("Failed to move to approach position")
                return False

            await asyncio.sleep(0.5)  # Brief pause

            # Step 2: Move down to pickup position
            self.logger.info(f"Step 2: Moving down to pickup position (Z={self.pickplace_settings.pick_z_height:.1f})")
            success = await self.move_cartesian(pick_x, pick_y, self.pickplace_settings.pick_z_height)
            if not success:
                self.logger.error("Failed to move to pickup position")
                return False

            await asyncio.sleep(0.3)  # Brief pause

            # Step 3: Close gripper to pick up object
            self.logger.info(f"Step 3: Closing gripper to {self.pickplace_settings.grip_position}°")
            success = await self.control_gripper(self.pickplace_settings.grip_position)
            if not success:
                self.logger.error("Failed to close gripper")
                return False

            await asyncio.sleep(self.pickplace_settings.pick_delay)  # Wait for grip

            # Step 4: Lift object up
            self.logger.info(f"Step 4: Lifting object to approach height")
            success = await self.move_cartesian(pick_x, pick_y, approach_z)
            if not success:
                self.logger.error("Failed to lift object")
                return False

            await asyncio.sleep(0.5)  # Brief pause

            # Step 5: Move to drop location (approach height)
            drop_approach_z = max(drop_z + 30, self.pickplace_settings.approach_height)
            self.logger.info(
                f"Step 5: Moving to drop approach position ({drop_x:.1f}, {drop_y:.1f}, {drop_approach_z:.1f})")
            success = await self.move_cartesian(drop_x, drop_y, drop_approach_z)
            if not success:
                self.logger.error("Failed to move to drop approach position")
                return False

            await asyncio.sleep(0.5)  # Brief pause

            # Step 6: Move down to drop position
            self.logger.info(f"Step 6: Moving down to drop position (Z={drop_z:.1f})")
            success = await self.move_cartesian(drop_x, drop_y, drop_z)
            if not success:
                self.logger.error("Failed to move to drop position")
                return False

            await asyncio.sleep(0.3)  # Brief pause

            # Step 7: Open gripper to release object
            self.logger.info(f"Step 7: Opening gripper to {self.pickplace_settings.open_position}°")
            success = await self.control_gripper(self.pickplace_settings.open_position)
            if not success:
                self.logger.error("Failed to open gripper")
                return False

            await asyncio.sleep(self.pickplace_settings.drop_delay)  # Wait for release

            # Step 8: Lift up from drop position
            self.logger.info(f"Step 8: Lifting up from drop position")
            success = await self.move_cartesian(drop_x, drop_y, drop_approach_z)
            if not success:
                self.logger.error("Failed to lift from drop position")
                return False

            self.logger.info("Pick and place operation completed successfully!")
            return True

        except Exception as e:
            self.logger.error(f"Pick and place operation failed: {e}")
            return False

    def update_pickplace_settings(self, **kwargs):
        """Update pick and place settings with validation"""
        for key, value in kwargs.items():
            if hasattr(self.pickplace_settings, key):
                # Validate gripper positions
                if key in ['grip_position', 'open_position']:
                    value = max(0, min(90, int(value)))  # Clamp to 0-90 range
                setattr(self.pickplace_settings, key, value)
                self.logger.info(f"Updated {key} to {value}")

    async def safe_move_to_home_position(self) -> bool:
        """Move robot to a safe home position"""
        try:
            # Move to a safe neutral position within new shoulder range
            success = await self.move_cartesian(200, 0, 80)
            if success:
                await asyncio.sleep(0.5)
                # Open gripper
                await self.control_gripper(0)
            return success
        except Exception as e:
            self.logger.error(f"Failed to move to safe position: {e}")
            return False


class VisionSystem:
    """Enhanced computer vision system for object detection"""

    def __init__(self, settings: VisionSettings):
        self.settings = settings
        self.camera: Optional[cv2.VideoCapture] = None
        self.is_calibrated = False
        self.logger = logging.getLogger(__name__)

    def initialize_camera(self) -> bool:
        """Initialize camera"""
        try:
            self.camera = cv2.VideoCapture(self.settings.camera_index)

            if not self.camera.isOpened():
                raise Exception("Cannot open camera")

            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.settings.resolution[0])
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.settings.resolution[1])
            self.camera.set(cv2.CAP_PROP_FPS, 30)

            self.logger.info("Camera initialized successfully")
            return True

        except Exception as e:
            self.logger.error(f"Camera initialization failed: {e}")
            return False

    def release_camera(self):
        """Release camera resources"""
        if self.camera and self.camera.isOpened():
            self.camera.release()
            self.logger.info("Camera released")

    def capture_frame(self) -> Optional[np.ndarray]:
        """Capture a frame from camera"""
        if not self.camera or not self.camera.isOpened():
            return None

        ret, frame = self.camera.read()
        return frame if ret else None

    def detect_objects(self, frame: np.ndarray) -> Tuple[Optional[Tuple[float, float]], np.ndarray]:
        """Detect objects in frame using color-based detection"""
        if frame is None:
            return None, frame

        try:
            blurred = cv2.GaussianBlur(frame, (self.settings.blur_kernel, self.settings.blur_kernel), 0)

            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            lower = np.array(self.settings.hsv_lower)
            upper = np.array(self.settings.hsv_upper)
            mask = cv2.inRange(hsv, lower, upper)

            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)

                if cv2.contourArea(largest_contour) > self.settings.min_contour_area:
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
                        cv2.circle(frame, (cx, cy), 20, (0, 255, 255), 2)

                        world_coords = self.pixel_to_world(cx, cy)

                        if world_coords:
                            cv2.putText(frame, f"X: {world_coords[0]:.1f}mm, Y: {world_coords[1]:.1f}mm",
                                        (cx - 100, cy - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                        return world_coords, frame

            return None, frame

        except Exception as e:
            self.logger.error(f"Object detection error: {e}")
            return None, frame

    def pixel_to_world(self, px: int, py: int) -> Optional[Tuple[float, float]]:
        """Convert pixel coordinates to world coordinates"""
        if self.settings.calibration_matrix is not None:
            pixel_point = np.array([[px, py]], dtype=np.float32).reshape(-1, 1, 2)
            world_point = cv2.perspectiveTransform(pixel_point, self.settings.calibration_matrix)
            return float(world_point[0][0][0]), float(world_point[0][0][1])
        else:
            x_mm = (px + self.settings.resolution[0] / 2) * 0.5
            y_mm = 430 - (py * 0.5)
            return x_mm, y_mm

    def update_color_settings(self, hsv_lower: List[int], hsv_upper: List[int]):
        """Update color detection settings"""
        self.settings.hsv_lower = hsv_lower
        self.settings.hsv_upper = hsv_upper


class DataLogger:
    """Enhanced data logging and analytics system with task persistence"""

    def __init__(self, db_path: str = "scara_robot.db"):
        self.db_path = db_path
        self.logger = logging.getLogger(__name__)
        self.init_database()

    def init_database(self):
        """Initialize SQLite database with task and sequence tables"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            cursor.execute('''
                CREATE TABLE IF NOT EXISTS movements (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                    x REAL, y REAL, z REAL, w REAL,
                    mode TEXT,
                    duration REAL,
                    success BOOLEAN
                )
            ''')

            cursor.execute('''
                CREATE TABLE IF NOT EXISTS system_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                    event_type TEXT,
                    description TEXT,
                    data TEXT
                )
            ''')

            cursor.execute('''
                CREATE TABLE IF NOT EXISTS task_executions (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                    task_name TEXT,
                    duration REAL,
                    success BOOLEAN,
                    error_message TEXT
                )
            ''')

            cursor.execute('''
                CREATE TABLE IF NOT EXISTS pickplace_operations (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                    pick_x REAL, pick_y REAL, pick_z REAL,
                    drop_x REAL, drop_y REAL, drop_z REAL,
                    duration REAL,
                    success BOOLEAN,
                    error_message TEXT
                )
            ''')

            # NEW: Tasks table for persistent task storage
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS tasks (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT UNIQUE NOT NULL,
                    steps TEXT NOT NULL,
                    created DATETIME DEFAULT CURRENT_TIMESTAMP,
                    modified DATETIME DEFAULT CURRENT_TIMESTAMP,
                    executions INTEGER DEFAULT 0,
                    last_executed DATETIME
                )
            ''')

            # NEW: Sequences table for persistent sequence storage
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS sequences (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT UNIQUE NOT NULL,
                    tasks TEXT NOT NULL,
                    loop_count INTEGER DEFAULT 1,
                    created DATETIME DEFAULT CURRENT_TIMESTAMP,
                    modified DATETIME DEFAULT CURRENT_TIMESTAMP,
                    executions INTEGER DEFAULT 0,
                    last_executed DATETIME
                )
            ''')

            conn.commit()
            conn.close()

            self.logger.info("Database initialized successfully with task persistence")

        except Exception as e:
            self.logger.error(f"Database initialization failed: {e}")

    def log_movement(self, x: float, y: float, z: float, w: float,
                     mode: str, duration: float, success: bool):
        """Log robot movement"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            cursor.execute('''
                INSERT INTO movements (x, y, z, w, mode, duration, success)
                VALUES (?, ?, ?, ?, ?, ?, ?)
            ''', (x, y, z, w, mode, duration, success))

            conn.commit()
            conn.close()

        except Exception as e:
            self.logger.error(f"Movement logging failed: {e}")

    def log_event(self, event_type: str, description: str, data: Optional[Dict] = None):
        """Log system event"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            data_json = json.dumps(data) if data else None

            cursor.execute('''
                INSERT INTO system_events (event_type, description, data)
                VALUES (?, ?, ?)
            ''', (event_type, description, data_json))

            conn.commit()
            conn.close()

        except Exception as e:
            self.logger.error(f"Event logging failed: {e}")

    def log_pickplace_operation(self, pick_x: float, pick_y: float, pick_z: float,
                                drop_x: float, drop_y: float, drop_z: float,
                                duration: float, success: bool, error_message: str = None):
        """Log pick and place operation"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            cursor.execute('''
                INSERT INTO pickplace_operations
                (pick_x, pick_y, pick_z, drop_x, drop_y, drop_z, duration, success, error_message)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''', (pick_x, pick_y, pick_z, drop_x, drop_y, drop_z, duration, success, error_message))

            conn.commit()
            conn.close()

        except Exception as e:
            self.logger.error(f"Pick and place logging failed: {e}")

    # NEW: Task persistence methods
    def save_task(self, name: str, steps: List[Dict]) -> bool:
        """Save task to database"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            steps_json = json.dumps(steps)
            current_time = datetime.now().isoformat()

            cursor.execute('''
                INSERT OR REPLACE INTO tasks (name, steps, modified)
                VALUES (?, ?, ?)
            ''', (name, steps_json, current_time))

            conn.commit()
            conn.close()

            self.logger.info(f"Task '{name}' saved to database")
            return True

        except Exception as e:
            self.logger.error(f"Failed to save task '{name}': {e}")
            return False

    def load_task(self, name: str) -> Optional[Dict]:
        """Load specific task from database"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            cursor.execute('''
                SELECT name, steps, created, modified, executions, last_executed
                FROM tasks WHERE name = ?
            ''', (name,))

            result = cursor.fetchone()
            conn.close()

            if result:
                return {
                    'name': result[0],
                    'steps': json.loads(result[1]),
                    'created': result[2],
                    'modified': result[3],
                    'executions': result[4] or 0,
                    'last_executed': result[5]
                }

            return None

        except Exception as e:
            self.logger.error(f"Failed to load task '{name}': {e}")
            return None

    def load_all_tasks(self) -> Dict[str, Dict]:
        """Load all tasks from database"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            cursor.execute('''
                SELECT name, steps, created, modified, executions, last_executed
                FROM tasks ORDER BY created DESC
            ''')

            results = cursor.fetchall()
            conn.close()

            tasks = {}
            for result in results:
                try:
                    tasks[result[0]] = {
                        'name': result[0],
                        'steps': json.loads(result[1]),
                        'created': result[2],
                        'modified': result[3],
                        'executions': result[4] or 0,
                        'last_executed': result[5]
                    }
                except json.JSONDecodeError as e:
                    self.logger.error(f"Failed to parse steps for task '{result[0]}': {e}")
                    continue

            self.logger.info(f"Loaded {len(tasks)} tasks from database")
            return tasks

        except Exception as e:
            self.logger.error(f"Failed to load tasks: {e}")
            return {}

    def update_task_execution(self, name: str) -> bool:
        """Update task execution count and timestamp"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            current_time = datetime.now().isoformat()

            cursor.execute('''
                UPDATE tasks
                SET executions = executions + 1, last_executed = ?
                WHERE name = ?
            ''', (current_time, name))

            conn.commit()
            conn.close()

            return True

        except Exception as e:
            self.logger.error(f"Failed to update task execution for '{name}': {e}")
            return False

    def delete_task(self, name: str) -> bool:
        """Delete task from database"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            cursor.execute('DELETE FROM tasks WHERE name = ?', (name,))

            conn.commit()
            conn.close()

            self.logger.info(f"Task '{name}' deleted from database")
            return True

        except Exception as e:
            self.logger.error(f"Failed to delete task '{name}': {e}")
            return False

    # NEW: Sequence persistence methods
    def save_sequence(self, name: str, tasks: List[str], loop_count: int = 1) -> bool:
        """Save sequence to database"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            tasks_json = json.dumps(tasks)
            current_time = datetime.now().isoformat()

            cursor.execute('''
                INSERT OR REPLACE INTO sequences (name, tasks, loop_count, modified)
                VALUES (?, ?, ?, ?)
            ''', (name, tasks_json, loop_count, current_time))

            conn.commit()
            conn.close()

            self.logger.info(f"Sequence '{name}' saved to database")
            return True

        except Exception as e:
            self.logger.error(f"Failed to save sequence '{name}': {e}")
            return False

    def load_all_sequences(self) -> Dict[str, Dict]:
        """Load all sequences from database"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            cursor.execute('''
                SELECT name, tasks, loop_count, created, modified, executions, last_executed
                FROM sequences ORDER BY created DESC
            ''')

            results = cursor.fetchall()
            conn.close()

            sequences = {}
            for result in results:
                try:
                    sequences[result[0]] = {
                        'name': result[0],
                        'tasks': json.loads(result[1]),
                        'loop_count': result[2],
                        'created': result[3],
                        'modified': result[4],
                        'executions': result[5] or 0,
                        'last_executed': result[6]
                    }
                except json.JSONDecodeError as e:
                    self.logger.error(f"Failed to parse tasks for sequence '{result[0]}': {e}")
                    continue

            self.logger.info(f"Loaded {len(sequences)} sequences from database")
            return sequences

        except Exception as e:
            self.logger.error(f"Failed to load sequences: {e}")
            return {}

    def update_sequence_execution(self, name: str) -> bool:
        """Update sequence execution count and timestamp"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            current_time = datetime.now().isoformat()

            cursor.execute('''
                UPDATE sequences
                SET executions = executions + 1, last_executed = ?
                WHERE name = ?
            ''', (current_time, name))

            conn.commit()
            conn.close()

            return True

        except Exception as e:
            self.logger.error(f"Failed to update sequence execution for '{name}': {e}")
            return False

    def delete_sequence(self, name: str) -> bool:
        """Delete sequence from database"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            cursor.execute('DELETE FROM sequences WHERE name = ?', (name,))

            conn.commit()
            conn.close()

            self.logger.info(f"Sequence '{name}' deleted from database")
            return True

        except Exception as e:
            self.logger.error(f"Failed to delete sequence '{name}': {e}")
            return False

    def get_movement_history(self, limit: int = 100) -> List[Dict]:
        """Get recent movement history"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            cursor.execute('''
                SELECT timestamp, x, y, z, w, mode, duration, success
                FROM movements
                ORDER BY timestamp DESC
                LIMIT ?
            ''', (limit,))

            results = cursor.fetchall()
            conn.close()

            return [
                {
                    'timestamp': row[0],
                    'x': row[1], 'y': row[2], 'z': row[3], 'w': row[4],
                    'mode': row[5], 'duration': row[6], 'success': row[7]
                }
                for row in results
            ]

        except Exception as e:
            self.logger.error(f"Failed to get movement history: {e}")
            return []


class TaskManager:
    """Advanced task programming and execution system with persistence"""

    def __init__(self, robot_controller: RobotController, data_logger: DataLogger):
        self.robot = robot_controller
        self.data_logger = data_logger  # NEW: Add data logger reference
        self.tasks = {}
        self.current_task = None
        self.is_executing = False
        self.logger = logging.getLogger(__name__)

        # Task sequencing
        self.task_sequences = {}
        self.current_sequence = None
        self.is_executing_sequence = False
        self.sequence_thread = None
        self.sequence_stop_event = threading.Event()
        self.sequence_pause_event = threading.Event()

        # NEW: Load existing tasks and sequences from database on startup
        self.load_tasks_from_database()
        self.load_sequences_from_database()

    def load_tasks_from_database(self):
        """Load all tasks from database on startup"""
        try:
            self.tasks = self.data_logger.load_all_tasks()
            self.logger.info(f"Loaded {len(self.tasks)} tasks from database")
        except Exception as e:
            self.logger.error(f"Failed to load tasks from database: {e}")
            self.tasks = {}

    def load_sequences_from_database(self):
        """Load all sequences from database on startup"""
        try:
            self.task_sequences = self.data_logger.load_all_sequences()
            self.logger.info(f"Loaded {len(self.task_sequences)} sequences from database")
        except Exception as e:
            self.logger.error(f"Failed to load sequences from database: {e}")
            self.task_sequences = {}

    def create_task(self, name: str, steps: List[Dict]) -> bool:
        """Create a new task with immediate persistence"""
        try:
            validated_steps = []
            for step in steps:
                if self.validate_step(step):
                    validated_steps.append(step)
                else:
                    self.logger.error(f"Invalid step in task {name}: {step}")
                    return False

            # NEW: Save to database immediately
            success = self.data_logger.save_task(name, validated_steps)
            if not success:
                self.logger.error(f"Failed to save task '{name}' to database")
                return False

            # Update in-memory storage
            self.tasks[name] = {
                'name': name,
                'steps': validated_steps,
                'created': datetime.now().isoformat(),
                'executions': 0
            }

            self.logger.info(f"Task '{name}' created with {len(steps)} steps and saved to database")
            return True

        except Exception as e:
            self.logger.error(f"Task creation failed: {e}")
            return False

    def validate_step(self, step: Dict) -> bool:
        """Validate a task step with updated ranges"""
        required_fields = ['type']

        if not all(field in step for field in required_fields):
            return False

        step_type = step['type']

        if step_type == 'move_joint':
            if not all(key in step for key in ['shoulder', 'elbow', 'wrist', 'z']):
                return False
            # Validate ranges
            shoulder = step['shoulder']
            elbow = step['elbow']
            wrist = step['wrist']
            z = step['z']

            if not (0 <= shoulder <= 180):
                return False
            if not (-90 <= elbow <= 90):
                return False
            if not (-90 <= wrist <= 90):
                return False
            if not (0 <= z <= 300):
                return False

        elif step_type == 'move_cartesian':
            if not all(key in step for key in ['x', 'y', 'z']):
                return False
            # Check if position is reachable
            if not self.robot.is_position_reachable(step['x'], step['y'], step['z']):
                return False

        elif step_type == 'gripper':
            if 'position' not in step:
                return False
            if not (0 <= step['position'] <= 90):  # Updated range
                return False

        elif step_type == 'wait':
            return 'duration' in step and step['duration'] > 0

        elif step_type == 'pick_and_place':
            if not all(key in step for key in ['pick_x', 'pick_y']):
                return False
            # Check if pick position is reachable
            pick_z = step.get('pick_z', 50.0)
            if not self.robot.is_position_reachable(step['pick_x'], step['pick_y'], pick_z):
                return False

        return True

    async def execute_task(self, task_name: str) -> bool:
        """Execute a task with database logging"""
        if task_name not in self.tasks:
            self.logger.error(f"Task '{task_name}' not found")
            return False

        if self.is_executing:
            self.logger.error("Another task is already executing")
            return False

        self.is_executing = True
        self.current_task = task_name
        task = self.tasks[task_name]

        try:
            self.logger.info(f"Starting execution of task '{task_name}'")

            for i, step in enumerate(task['steps']):
                self.logger.info(f"Executing step {i + 1}/{len(task['steps'])}: {step['type']}")

                success = await self.execute_step(step)
                if not success:
                    self.logger.error(f"Step {i + 1} failed, aborting task")
                    return False

            # NEW: Update database execution count
            self.data_logger.update_task_execution(task_name)

            # Update in-memory count
            task['executions'] += 1
            task['last_executed'] = datetime.now().isoformat()

            self.logger.info(f"Task '{task_name}' completed successfully")
            return True

        except Exception as e:
            self.logger.error(f"Task execution failed: {e}")
            return False
        finally:
            self.is_executing = False
            self.current_task = None

    async def execute_step(self, step: Dict) -> bool:
        """Execute a single task step"""
        step_type = step['type']

        try:
            if step_type == 'move_joint':
                return await self.robot.move_joint(
                    step['shoulder'], step['elbow'], step['wrist'], step['z']
                )
            elif step_type == 'move_cartesian':
                return await self.robot.move_cartesian(step['x'], step['y'], step['z'])
            elif step_type == 'gripper':
                return await self.robot.control_gripper(step['position'])
            elif step_type == 'wait':
                await asyncio.sleep(step['duration'])
                return True
            elif step_type == 'pick_and_place':
                return await self.robot.pick_and_place(
                    step['pick_x'], step['pick_y'],
                    step.get('drop_x'), step.get('drop_y'), step.get('drop_z')
                )

            return False

        except Exception as e:
            self.logger.error(f"Step execution error: {e}")
            return False

    # === TASK SEQUENCING FUNCTIONALITY ===

    def create_sequence(self, name: str, task_names: List[str], loop_count: int = 1) -> bool:
        """Create a new task sequence with immediate persistence"""
        try:
            # Validate that all tasks exist
            for task_name in task_names:
                if task_name not in self.tasks:
                    self.logger.error(f"Task '{task_name}' not found for sequence")
                    return False

            # NEW: Save to database immediately
            success = self.data_logger.save_sequence(name, task_names, loop_count)
            if not success:
                self.logger.error(f"Failed to save sequence '{name}' to database")
                return False

            # Update in-memory storage
            self.task_sequences[name] = {
                'name': name,
                'tasks': task_names,
                'loop_count': loop_count,
                'created': datetime.now().isoformat(),
                'executions': 0,
                'last_executed': None
            }

            self.logger.info(
                f"Sequence '{name}' created with {len(task_names)} tasks, {loop_count} loops and saved to database")
            return True

        except Exception as e:
            self.logger.error(f"Sequence creation failed: {e}")
            return False

    def start_sequence(self, sequence_name: str, progress_callback=None) -> bool:
        """Start executing a task sequence"""
        if sequence_name not in self.task_sequences:
            self.logger.error(f"Sequence '{sequence_name}' not found")
            return False

        if self.is_executing_sequence:
            self.logger.error("Another sequence is already executing")
            return False

        self.current_sequence = sequence_name
        self.is_executing_sequence = True
        self.sequence_stop_event.clear()
        self.sequence_pause_event.clear()

        # Start sequence in background thread
        self.sequence_thread = threading.Thread(
            target=self._execute_sequence_background,
            args=(sequence_name, progress_callback),
            daemon=True
        )
        self.sequence_thread.start()
        return True

    def stop_sequence(self):
        """Stop the current sequence"""
        if self.is_executing_sequence:
            self.sequence_stop_event.set()
            self.logger.info("Sequence stop requested")

    def pause_sequence(self):
        """Pause the current sequence"""
        if self.is_executing_sequence:
            self.sequence_pause_event.set()
            self.logger.info("Sequence paused")

    def resume_sequence(self):
        """Resume the paused sequence"""
        if self.is_executing_sequence:
            self.sequence_pause_event.clear()
            self.logger.info("Sequence resumed")

    def _execute_sequence_background(self, sequence_name: str, progress_callback=None):
        """Execute sequence in background thread with database logging"""
        try:
            sequence = self.task_sequences[sequence_name]
            tasks = sequence['tasks']
            loop_count = sequence['loop_count']

            self.logger.info(f"Starting sequence '{sequence_name}' with {len(tasks)} tasks, {loop_count} loops")

            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            async def run_sequence():
                for current_loop in range(loop_count):
                    if self.sequence_stop_event.is_set():
                        self.logger.info("Sequence stopped by user")
                        break

                    self.logger.info(f"Starting loop {current_loop + 1}/{loop_count}")

                    for task_index, task_name in enumerate(tasks):
                        if self.sequence_stop_event.is_set():
                            self.logger.info("Sequence stopped by user")
                            break

                        # Handle pause
                        while self.sequence_pause_event.is_set():
                            if self.sequence_stop_event.is_set():
                                break
                            await asyncio.sleep(0.1)

                        if self.sequence_stop_event.is_set():
                            break

                        # Update progress
                        if progress_callback:
                            progress_callback({
                                'current_task': task_index,
                                'current_loop': current_loop,
                                'total_tasks': len(tasks),
                                'total_loops': loop_count,
                                'task_name': task_name,
                                'is_running': True,
                                'is_paused': self.sequence_pause_event.is_set()
                            })

                        # Execute task
                        self.logger.info(f"Executing task '{task_name}' (task {task_index + 1}/{len(tasks)})")
                        success = await self.execute_task(task_name)

                        if not success:
                            self.logger.error(f"Task '{task_name}' failed, stopping sequence")
                            break

                        # Small delay between tasks
                        await asyncio.sleep(0.5)

                    if self.sequence_stop_event.is_set():
                        break

                    # Delay between loops
                    if current_loop < loop_count - 1:
                        await asyncio.sleep(1.0)

                # NEW: Update database execution count
                self.data_logger.update_sequence_execution(sequence_name)

                # Update in-memory statistics
                sequence['executions'] += 1
                sequence['last_executed'] = datetime.now().isoformat()

                self.logger.info(f"Sequence '{sequence_name}' completed")

            # Run the async sequence
            loop.run_until_complete(run_sequence())

        except Exception as e:
            self.logger.error(f"Sequence execution error: {e}")
        finally:
            self.is_executing_sequence = False
            self.current_sequence = None
            loop.close()

            # Final progress update
            if progress_callback:
                progress_callback({
                    'current_task': 0,
                    'current_loop': 0,
                    'total_tasks': 0,
                    'total_loops': 0,
                    'task_name': '',
                    'is_running': False,
                    'is_paused': False
                })

    def get_sequence_status(self) -> Dict:
        """Get current sequence execution status"""
        return {
            'is_executing': self.is_executing_sequence,
            'current_sequence': self.current_sequence,
            'is_paused': self.sequence_pause_event.is_set() if self.is_executing_sequence else False
        }

    # NEW: Delete methods with database persistence
    def delete_task(self, task_name: str) -> bool:
        """Delete task from both memory and database"""
        try:
            # Remove from database
            success = self.data_logger.delete_task(task_name)
            if not success:
                self.logger.error(f"Failed to delete task '{task_name}' from database")
                return False

            # Remove from memory
            if task_name in self.tasks:
                del self.tasks[task_name]
                self.logger.info(f"Task '{task_name}' deleted from memory and database")
                return True
            else:
                self.logger.warning(f"Task '{task_name}' not found in memory")
                return True  # Still successful if removed from database

        except Exception as e:
            self.logger.error(f"Failed to delete task '{task_name}': {e}")
            return False

    def delete_sequence(self, sequence_name: str) -> bool:
        """Delete sequence from both memory and database"""
        try:
            # Remove from database
            success = self.data_logger.delete_sequence(sequence_name)
            if not success:
                self.logger.error(f"Failed to delete sequence '{sequence_name}' from database")
                return False

            # Remove from memory
            if sequence_name in self.task_sequences:
                del self.task_sequences[sequence_name]
                self.logger.info(f"Sequence '{sequence_name}' deleted from memory and database")
                return True
            else:
                self.logger.warning(f"Sequence '{sequence_name}' not found in memory")
                return True  # Still successful if removed from database

        except Exception as e:
            self.logger.error(f"Failed to delete sequence '{sequence_name}': {e}")
            return False


class SCARAWebApp:
    """Main Flask application with SocketIO"""

    def __init__(self):
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'scara_robot_secret_key'
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")

        self.config = RobotConfig()
        self.motion_settings = MotionSettings()
        self.vision_settings = VisionSettings()
        self.system_state = SystemState()

        self.robot = RobotController(self.config)
        self.vision = VisionSystem(self.vision_settings)

        # NEW: Pass data_logger to TaskManager
        self.data_logger = DataLogger()
        self.task_manager = TaskManager(self.robot, self.data_logger)

        # Pick and place state
        self.last_detection_time = 0
        self.detection_cooldown = 3.0  # Seconds between automatic pick operations

        # Auto mode integration
        self.auto_mode_active = False

        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('logs/scara_robot.log'),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)

        self.setup_routes()
        self.setup_socket_events()

        self.start_background_tasks()

    def setup_routes(self):
        """Setup Flask routes"""

        # Add CORS headers to all responses
        @self.app.after_request
        def after_request(response):
            response.headers.add('Access-Control-Allow-Origin', '*')
            response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
            response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
            return response

        # Add error handler
        @self.app.errorhandler(Exception)
        def handle_exception(e):
            self.logger.error(f"Unhandled exception: {str(e)}")
            return jsonify({
                'success': False,
                'error': f'Server error: {str(e)}'
            }), 500

        @self.app.route('/')
        def index():
            # Return the HTML template directly since we're not using external template files
            return self.get_html_template()

        @self.app.route('/api/ports')
        def list_serial_ports():
            """List available serial ports"""
            try:
                import serial.tools.list_ports
                ports = []

                for port in serial.tools.list_ports.comports():
                    ports.append({
                        'device': port.device,
                        'name': port.name,
                        'description': port.description,
                        'hwid': port.hwid,
                        'manufacturer': getattr(port, 'manufacturer', 'Unknown')
                    })

                return jsonify({'success': True, 'ports': ports})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/connect', methods=['POST'])
        def connect():
            try:
                data = request.get_json() or {}
                port = data.get('port', '/dev/ttyUSB0')
                self.robot.port = port

                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                success = loop.run_until_complete(self.robot.connect())
                loop.close()

                if success:
                    self.system_state.is_connected = True
                    self.data_logger.log_event("SYSTEM", "Robot connected")

                return jsonify({'success': success})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/disconnect', methods=['POST'])
        def disconnect():
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(self.robot.disconnect())
                loop.close()

                self.system_state.is_connected = False
                self.data_logger.log_event("SYSTEM", "Robot disconnected")
                return jsonify({'success': True})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/home', methods=['POST'])
        def home():
            try:
                self.logger.info("Homing request received from web interface")

                if not self.system_state.is_connected:
                    error_msg = "Robot not connected"
                    self.logger.warning(error_msg)
                    return jsonify({'success': False, 'error': error_msg})

                if self.system_state.emergency_stop:
                    error_msg = "Cannot home - emergency stop is active"
                    self.logger.error(error_msg)
                    return jsonify({'success': False, 'error': error_msg})

                self.logger.info("Initiating homing sequence...")

                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                start_time = time.time()

                success = loop.run_until_complete(self.robot.home_robot())

                loop.close()
                homing_time = time.time() - start_time

                if success:
                    self.system_state.is_homed = True
                    self.system_state.current_position = [0.0, 0.0, 0.0, 0.0]  # Reset to home position

                    self.data_logger.log_event("MOTION", f"Robot homed successfully in {homing_time:.2f}s")
                    self.logger.info(f"Homing completed successfully in {homing_time:.2f}s")

                    return jsonify({
                        'success': True,
                        'message': 'Robot homed successfully',
                        'time': homing_time
                    })
                else:
                    error_msg = "Homing sequence failed"
                    self.logger.error(error_msg)
                    self.data_logger.log_event("MOTION", f"Homing failed after {homing_time:.2f}s")

                    return jsonify({'success': False, 'error': error_msg, 'time': homing_time})

            except Exception as e:
                error_msg = f"Homing error: {str(e)}"
                self.logger.error(error_msg)
                return jsonify({'success': False, 'error': error_msg})

        @self.app.route('/api/settings/motion', methods=['POST'])
        def update_motion_settings():
            try:
                data = request.get_json()
                speed = data.get('speed', self.motion_settings.max_speed)
                acceleration = data.get('acceleration', self.motion_settings.acceleration)

                self.motion_settings.max_speed = speed
                self.motion_settings.acceleration = acceleration

                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                success = loop.run_until_complete(self.robot.set_motion_settings(speed, acceleration))
                loop.close()

                if success:
                    self.data_logger.log_event("SETTINGS",
                                               f"Motion settings updated: speed={speed}%, accel={acceleration}%")

                return jsonify({'success': success})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/settings/vision', methods=['POST'])
        def update_vision_settings():
            try:
                data = request.get_json()
                hsv_lower = data.get('hsv_lower', self.vision_settings.hsv_lower)
                hsv_upper = data.get('hsv_upper', self.vision_settings.hsv_upper)

                self.vision.update_color_settings(hsv_lower, hsv_upper)
                self.data_logger.log_event("SETTINGS", f"Vision settings updated")

                return jsonify({'success': True})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/move/joint', methods=['POST'])
        def move_joint():
            try:
                data = request.get_json()
                self.logger.info(f"Joint movement request received: {data}")

                required_fields = ['shoulder', 'elbow', 'wrist', 'z']
                for field in required_fields:
                    if field not in data:
                        error_msg = f"Missing required field: {field}"
                        self.logger.error(error_msg)
                        return jsonify({'success': False, 'error': error_msg})

                if not self.system_state.is_connected:
                    error_msg = "Robot not connected"
                    self.logger.warning(error_msg)
                    return jsonify({'success': False, 'error': error_msg})

                if self.system_state.emergency_stop:
                    error_msg = "Emergency stop is active"
                    self.logger.error(error_msg)
                    return jsonify({'success': False, 'error': error_msg})

                shoulder = float(data['shoulder'])
                elbow = float(data['elbow'])
                wrist = float(data['wrist'])
                z = float(data['z'])

                self.logger.info(f"Moving to joint position: S={shoulder}°, E={elbow}°, W={wrist}°, Z={z}mm")

                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                start_time = time.time()

                success = loop.run_until_complete(self.robot.move_joint(shoulder, elbow, wrist, z))

                loop.close()
                movement_time = time.time() - start_time

                if success:
                    self.logger.info(f"Joint movement completed successfully in {movement_time:.2f}s")

                    self.data_logger.log_movement(
                        data.get('x', 0), data.get('y', 0), z, wrist,
                        "manual_joint", movement_time, True
                    )

                    self.system_state.current_position = [shoulder, elbow, wrist, z]

                    return jsonify({'success': True, 'message': 'Joint movement completed', 'time': movement_time})
                else:
                    if not self.system_state.is_homed:
                        error_msg = "Robot needs to be homed first. Click the 'Home' button."
                        self.logger.warning(error_msg)
                        return jsonify({'success': False, 'error': error_msg, 'needs_homing': True})
                    else:
                        error_msg = "Joint movement failed - robot rejected command"
                        self.logger.error(error_msg)
                        return jsonify({'success': False, 'error': error_msg})

            except ValueError as ve:
                error_msg = f"Invalid numeric value in request: {ve}"
                self.logger.error(error_msg)
                return jsonify({'success': False, 'error': error_msg})
            except Exception as e:
                error_msg = f"Joint movement error: {str(e)}"
                self.logger.error(error_msg)
                return jsonify({'success': False, 'error': error_msg})

        @self.app.route('/api/move/cartesian', methods=['POST'])
        def move_cartesian():
            try:
                data = request.get_json()
                self.logger.info(f"Received Cartesian move command: {data}")
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                success = loop.run_until_complete(self.robot.move_cartesian(data['x'], data['y'], data['z']))
                loop.close()

                if success:
                    self.data_logger.log_movement(
                        data['x'], data['y'], data['z'], 0,
                        "manual", 0, True
                    )

                return jsonify({'success': success})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/gripper', methods=['POST'])
        def control_gripper():
            try:
                data = request.get_json()
                position = data.get('position', 0)

                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                success = loop.run_until_complete(self.robot.control_gripper(position))
                loop.close()

                if success:
                    self.system_state.gripper_position = position

                return jsonify({'success': success})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/emergency', methods=['POST'])
        def emergency_stop():
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                success = loop.run_until_complete(self.robot.emergency_stop())
                loop.close()

                if success:
                    self.system_state.emergency_stop = True
                    self.data_logger.log_event("SAFETY", "Emergency stop activated")

                return jsonify({'success': success})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/reset', methods=['POST'])
        def reset_emergency():
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                success = loop.run_until_complete(self.robot.reset_emergency())
                loop.close()

                if success:
                    self.system_state.emergency_stop = False
                    self.data_logger.log_event("SAFETY", "Emergency stop reset")

                return jsonify({'success': success})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/status')
        def get_status():
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                robot_status = loop.run_until_complete(self.robot.get_status())
                position = loop.run_until_complete(self.robot.get_position())
                loop.close()

                if position:
                    self.system_state.current_position = position

                return jsonify({
                    'system_state': asdict(self.system_state),
                    'robot_status': robot_status,
                    'position': position,
                    'motion_settings': asdict(self.motion_settings),
                    'vision_settings': asdict(self.vision_settings),
                    'pickplace_settings': asdict(self.robot.pickplace_settings)
                })
            except Exception as e:
                return jsonify({'error': str(e)})

        @self.app.route('/video_feed')
        def video_feed():
            return Response(
                self.generate_video_frames(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )

        # NEW: Enhanced task routes with persistence
        @self.app.route('/api/tasks', methods=['GET'])
        def get_tasks():
            """Get all tasks from memory (already loaded from database)"""
            return jsonify(self.task_manager.tasks)

        @self.app.route('/api/tasks', methods=['POST'])
        def create_task():
            """Create new task with immediate database persistence"""
            try:
                data = request.get_json()

                if not data:
                    return jsonify({'success': False, 'error': 'No data provided'})

                if 'name' not in data or 'steps' not in data:
                    return jsonify({'success': False, 'error': 'Missing required fields: name or steps'})

                task_name = data['name'].strip()
                if not task_name:
                    return jsonify({'success': False, 'error': 'Task name cannot be empty'})

                steps = data['steps']
                if not isinstance(steps, list) or len(steps) == 0:
                    return jsonify({'success': False, 'error': 'Steps must be a non-empty list'})

                self.logger.info(f"Creating task '{task_name}' with {len(steps)} steps")

                # This will now save to database immediately
                success = self.task_manager.create_task(task_name, steps)

                if success:
                    self.logger.info(f"Task '{task_name}' created and saved successfully")
                    return jsonify({'success': True, 'message': f'Task "{task_name}" created and saved'})
                else:
                    self.logger.error(f"Failed to create task '{task_name}'")
                    return jsonify({'success': False, 'error': 'Failed to create and save task'})

            except Exception as e:
                self.logger.error(f"Task creation error: {str(e)}")
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/tasks/<task_name>/execute', methods=['POST'])
        def execute_task(task_name):
            """Execute task with database logging"""
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                success = loop.run_until_complete(self.task_manager.execute_task(task_name))
                loop.close()
                return jsonify({'success': success})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        # NEW: Task deletion route
        @self.app.route('/api/tasks/<task_name>', methods=['DELETE'])
        def delete_task(task_name):
            """Delete task from both memory and database"""
            try:
                success = self.task_manager.delete_task(task_name)
                if success:
                    return jsonify({'success': True, 'message': f'Task "{task_name}" deleted'})
                else:
                    return jsonify({'success': False, 'error': 'Failed to delete task'})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/analytics/movements')
        def get_movement_analytics():
            try:
                history = self.data_logger.get_movement_history(200)
                return jsonify(history)
            except Exception as e:
                return jsonify({'error': str(e)})

        # === PICK AND PLACE ROUTES ===

        @self.app.route('/api/pick_and_place/toggle', methods=['POST'])
        def toggle_pick_and_place():
            try:
                data = request.get_json() or {}
                enable = data.get('enable', not self.system_state.pick_and_place_enabled)

                self.system_state.pick_and_place_enabled = enable

                if enable:
                    self.data_logger.log_event("PICKPLACE", "Pick and place mode enabled")
                    message = "Pick and place mode enabled"
                else:
                    self.data_logger.log_event("PICKPLACE", "Pick and place mode disabled")
                    message = "Pick and place mode disabled"

                return jsonify({'success': True, 'enabled': enable, 'message': message})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/pick_and_place/execute', methods=['POST'])
        def execute_pick_and_place():
            try:
                data = request.get_json()

                if not self.system_state.is_connected:
                    return jsonify({'success': False, 'error': 'Robot not connected'})

                if not self.system_state.is_homed:
                    return jsonify({'success': False, 'error': 'Robot not homed'})

                if self.system_state.is_executing_pickplace:
                    return jsonify({'success': False, 'error': 'Pick and place operation already in progress'})

                # Get coordinates
                pick_x = float(data.get('pick_x', 200))
                pick_y = float(data.get('pick_y', 0))
                drop_x = float(data.get('drop_x', 20))
                drop_y = float(data.get('drop_y', 10))
                drop_z = float(data.get('drop_z', 20))

                self.system_state.is_executing_pickplace = True

                def run_pickplace():
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                    start_time = time.time()

                    try:
                        success = loop.run_until_complete(
                            self.robot.pick_and_place(pick_x, pick_y, drop_x, drop_y, drop_z)
                        )
                        duration = time.time() - start_time

                        # Log the operation
                        self.data_logger.log_pickplace_operation(
                            pick_x, pick_y, self.robot.pickplace_settings.pick_z_height,
                            drop_x, drop_y, drop_z, duration, success
                        )

                        # Emit status update
                        self.socketio.emit('pickplace_complete', {
                            'success': success,
                            'duration': duration,
                            'pick_coords': [pick_x, pick_y],
                            'drop_coords': [drop_x, drop_y, drop_z]
                        })

                    except Exception as e:
                        self.logger.error(f"Pick and place execution error: {e}")
                        self.socketio.emit('pickplace_complete', {
                            'success': False,
                            'error': str(e)
                        })
                    finally:
                        self.system_state.is_executing_pickplace = False
                        loop.close()

                # Run in background thread
                thread = threading.Thread(target=run_pickplace)
                thread.daemon = True
                thread.start()

                return jsonify({'success': True, 'message': 'Pick and place operation started'})

            except Exception as e:
                self.system_state.is_executing_pickplace = False
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/pick_and_place/settings', methods=['GET', 'POST'])
        def pickplace_settings():
            if request.method == 'POST':
                try:
                    data = request.get_json()
                    self.robot.update_pickplace_settings(**data)
                    return jsonify({'success': True, 'message': 'Settings updated'})
                except Exception as e:
                    return jsonify({'success': False, 'error': str(e)})
            else:
                return jsonify(asdict(self.robot.pickplace_settings))

        @self.app.route('/api/pick_and_place/auto_pick', methods=['POST'])
        def auto_pick_detected_object():
            """Automatically pick up the last detected object"""
            try:
                if not self.system_state.is_connected or not self.system_state.is_homed:
                    return jsonify({'success': False, 'error': 'Robot not ready'})

                if not self.system_state.last_detection:
                    return jsonify({'success': False, 'error': 'No object detected'})

                if self.system_state.is_executing_pickplace:
                    return jsonify({'success': False, 'error': 'Pick and place already in progress'})

                pick_x, pick_y = self.system_state.last_detection

                self.system_state.is_executing_pickplace = True

                def run_auto_pickplace():
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                    start_time = time.time()

                    try:
                        success = loop.run_until_complete(self.robot.pick_and_place(pick_x, pick_y))
                        duration = time.time() - start_time

                        self.data_logger.log_pickplace_operation(
                            pick_x, pick_y, self.robot.pickplace_settings.pick_z_height,
                            self.robot.pickplace_settings.drop_x,
                            self.robot.pickplace_settings.drop_y,
                            self.robot.pickplace_settings.drop_z,
                            duration, success,
                            None if success else "Auto pick operation failed"
                        )

                        self.socketio.emit('pickplace_complete', {
                            'success': success,
                            'duration': duration,
                            'auto_mode': True,
                            'pick_coords': [pick_x, pick_y]
                        })

                    except Exception as e:
                        self.logger.error(f"Auto pick operation error: {e}")
                        self.socketio.emit('pickplace_complete', {
                            'success': False,
                            'error': str(e),
                            'auto_mode': True
                        })
                    finally:
                        self.system_state.is_executing_pickplace = False
                        loop.close()

                thread = threading.Thread(target=run_auto_pickplace)
                thread.daemon = True
                thread.start()

                return jsonify({'success': True, 'message': 'Auto pick operation started'})

            except Exception as e:
                self.system_state.is_executing_pickplace = False
                return jsonify({'success': False, 'error': str(e)})

        # === ENHANCED TASK SEQUENCING ROUTES ===

        @self.app.route('/api/sequences', methods=['GET'])
        def get_sequences():
            """Get all task sequences from memory (already loaded from database)"""
            return jsonify(self.task_manager.task_sequences)

        @self.app.route('/api/sequences', methods=['POST'])
        def create_sequence():
            """Create a new task sequence with immediate database persistence"""
            try:
                data = request.get_json()

                if not data:
                    return jsonify({'success': False, 'error': 'No data provided'})

                name = data.get('name', '').strip()
                tasks = data.get('tasks', [])
                loop_count = data.get('loop_count', 1)

                if not name:
                    return jsonify({'success': False, 'error': 'Sequence name is required'})

                if not tasks or not isinstance(tasks, list):
                    return jsonify({'success': False, 'error': 'At least one task is required'})

                self.logger.info(f"Creating sequence '{name}' with {len(tasks)} tasks, {loop_count} loops")

                # This will now save to database immediately
                success = self.task_manager.create_sequence(name, tasks, loop_count)

                if success:
                    self.logger.info(f"Sequence '{name}' created and saved successfully")
                    return jsonify({'success': True, 'message': f'Sequence "{name}" created and saved'})
                else:
                    self.logger.error(f"Failed to create sequence '{name}'")
                    return jsonify({'success': False, 'error': 'Failed to create and save sequence'})

            except Exception as e:
                self.logger.error(f"Sequence creation error: {str(e)}")
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/sequences/<sequence_name>/start', methods=['POST'])
        def start_sequence(sequence_name):
            """Start executing a task sequence"""
            try:
                if not self.system_state.is_connected or not self.system_state.is_homed:
                    return jsonify({'success': False, 'error': 'Robot not ready'})

                def progress_callback(progress):
                    self.system_state.sequence_progress = progress
                    self.system_state.is_executing_sequence = progress['is_running']
                    self.system_state.current_sequence_task = progress.get('task_name', '')
                    self.socketio.emit('sequence_progress', progress)

                success = self.task_manager.start_sequence(sequence_name, progress_callback)
                if success:
                    self.system_state.is_executing_sequence = True
                    return jsonify({'success': True, 'message': f'Sequence {sequence_name} started'})
                else:
                    return jsonify({'success': False, 'error': 'Failed to start sequence'})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/sequences/stop', methods=['POST'])
        def stop_sequence():
            """Stop the current sequence"""
            try:
                self.task_manager.stop_sequence()
                self.system_state.is_executing_sequence = False
                return jsonify({'success': True, 'message': 'Sequence stop requested'})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/sequences/pause', methods=['POST'])
        def pause_sequence():
            """Pause the current sequence"""
            try:
                self.task_manager.pause_sequence()
                return jsonify({'success': True, 'message': 'Sequence paused'})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/sequences/resume', methods=['POST'])
        def resume_sequence():
            """Resume the paused sequence"""
            try:
                self.task_manager.resume_sequence()
                return jsonify({'success': True, 'message': 'Sequence resumed'})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/sequences/<sequence_name>', methods=['DELETE'])
        def delete_sequence(sequence_name):
            """Delete a task sequence from both memory and database"""
            try:
                success = self.task_manager.delete_sequence(sequence_name)
                if success:
                    return jsonify({'success': True, 'message': f'Sequence "{sequence_name}" deleted'})
                else:
                    return jsonify({'success': False, 'error': 'Failed to delete sequence'})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/sequences/status')
        def get_sequence_status():
            """Get current sequence execution status"""
            try:
                status = self.task_manager.get_sequence_status()
                status.update({
                    'progress': self.system_state.sequence_progress
                })
                return jsonify(status)
            except Exception as e:
                return jsonify({'error': str(e)})

        # === INVERSE KINEMATICS TESTING ROUTE ===

        @self.app.route('/api/kinematics/inverse', methods=['POST'])
        def test_inverse_kinematics():
            """Test inverse kinematics calculation"""
            try:
                data = request.get_json()
                x = float(data.get('x', 200))
                y = float(data.get('y', 0))
                z = float(data.get('z', 50))

                success, theta1, theta2, phi, final_z = self.robot.inverse_kinematics(x, y, z)

                if success:
                    # Also calculate forward kinematics for verification
                    verify_x, verify_y = self.robot.forward_kinematics(theta1, theta2, phi)
                    error = math.sqrt((x - verify_x) ** 2 + (y - verify_y) ** 2)

                    return jsonify({
                        'success': True,
                        'target': {'x': x, 'y': y, 'z': z},
                        'solution': {
                            'theta1': theta1,
                            'theta2': theta2,
                            'phi': phi,
                            'z': final_z
                        },
                        'verification': {'x': verify_x, 'y': verify_y},
                        'error': error
                    })
                else:
                    return jsonify({
                        'success': False,
                        'error': f'Position ({x:.1f}, {y:.1f}, {z:.1f}) is unreachable',
                        'target': {'x': x, 'y': y, 'z': z}
                    })

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        @self.app.route('/api/kinematics/forward', methods=['POST'])
        def test_forward_kinematics():
            """Test forward kinematics calculation"""
            try:
                data = request.get_json()
                theta1 = float(data.get('theta1', 0))
                theta2 = float(data.get('theta2', 0))
                phi = float(data.get('phi', 0))

                x, y = self.robot.forward_kinematics(theta1, theta2, phi)

                return jsonify({
                    'success': True,
                    'joints': {'theta1': theta1, 'theta2': theta2, 'phi': phi},
                    'position': {'x': x, 'y': y}
                })

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        # === REACHABILITY CHECK ROUTE ===

        @self.app.route('/api/kinematics/reachable', methods=['POST'])
        def check_reachability():
            """Check if a position is reachable"""
            try:
                data = request.get_json()
                x = float(data.get('x', 200))
                y = float(data.get('y', 0))
                z = float(data.get('z', 50))

                is_reachable = self.robot.is_position_reachable(x, y, z)

                return jsonify({
                    'success': True,
                    'position': {'x': x, 'y': y, 'z': z},
                    'reachable': is_reachable,
                    'max_reach': self.robot.config.max_reach,
                    'min_reach': self.robot.config.min_reach,
                    'z_limits': {'min': 0, 'max': self.robot.config.max_z_height}
                })

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})

        # === ENHANCED MODE TOGGLE ===

        @self.app.route('/api/mode/toggle', methods=['POST', 'OPTIONS'])
        def toggle_mode():
            """Toggle between manual and auto mode"""
            # Handle CORS preflight request
            if request.method == 'OPTIONS':
                response = jsonify({'status': 'OK'})
                response.headers.add('Access-Control-Allow-Origin', '*')
                response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
                response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
                return response

            try:
                # Get JSON data safely
                data = request.get_json() if request.is_json else {}
                new_mode = data.get('mode') if data else None

                # Toggle mode logic
                if new_mode and new_mode in ['manual', 'auto']:
                    self.system_state.operation_mode = new_mode
                else:
                    # Toggle between current modes
                    self.system_state.operation_mode = 'auto' if self.system_state.operation_mode == 'manual' else 'manual'

                # Integrate with pick and place auto mode
                if self.system_state.operation_mode == 'auto':
                    self.auto_mode_active = True
                    # Enable pick and place mode automatically
                    self.system_state.pick_and_place_enabled = True
                    # Enable auto mode in pick and place settings
                    self.robot.pickplace_settings.auto_mode = True
                    message = "Auto mode enabled - will automatically pick and place detected objects"
                else:
                    self.auto_mode_active = False
                    # Disable auto mode in pick and place settings but keep pick and place enabled
                    self.robot.pickplace_settings.auto_mode = False
                    message = "Manual mode enabled - pick and place requires manual triggering"

                # Log the mode change
                self.data_logger.log_event("MODE", f"Mode switched to {self.system_state.operation_mode}")

                # Return success response
                response_data = {
                    'success': True,
                    'mode': self.system_state.operation_mode,
                    'message': message
                }

                response = jsonify(response_data)
                response.headers.add('Access-Control-Allow-Origin', '*')
                return response

            except Exception as e:
                self.logger.error(f"Mode toggle error: {str(e)}")
                error_response = {
                    'success': False,
                    'error': f"Mode toggle failed: {str(e)}"
                }
                response = jsonify(error_response)
                response.headers.add('Access-Control-Allow-Origin', '*')
                return response, 500

    def setup_socket_events(self):
        """Setup SocketIO events"""

        @self.socketio.on('connect')
        def handle_connect():
            self.logger.info("Client connected")
            emit('system_status', asdict(self.system_state))

        @self.socketio.on('disconnect')
        def handle_disconnect():
            self.logger.info("Client disconnected")

        @self.socketio.on('request_status')
        def handle_status_request():
            emit('system_status', asdict(self.system_state))

    def generate_video_frames(self):
        """Generate video frames for streaming"""
        while True:
            frame = self.vision.capture_frame()
            if frame is not None:
                detection, processed_frame = self.vision.detect_objects(frame)

                if detection:
                    self.system_state.last_detection = detection
                    current_time = time.time()

                    # Emit detection event
                    self.socketio.emit('object_detected', {
                        'x': detection[0],
                        'y': detection[1],
                        'timestamp': current_time
                    })

                    # Auto pick and place if enabled (either through auto mode or pick and place auto mode)
                    should_auto_pick = (
                            (self.auto_mode_active or self.robot.pickplace_settings.auto_mode) and
                            self.system_state.pick_and_place_enabled and
                            not self.system_state.is_executing_pickplace and
                            not self.system_state.is_executing_sequence and
                            self.system_state.is_connected and
                            self.system_state.is_homed and
                            current_time - self.last_detection_time > self.detection_cooldown
                    )

                    if should_auto_pick:
                        self.last_detection_time = current_time
                        auto_mode_type = "Main Auto Mode" if self.auto_mode_active else "Pick&Place Auto Mode"
                        self.logger.info(
                            f"Auto-triggering pick and place for object at ({detection[0]:.1f}, {detection[1]:.1f}) via {auto_mode_type}")

                        # Start auto pick and place in background
                        def auto_pick():
                            try:
                                self.system_state.is_executing_pickplace = True
                                loop = asyncio.new_event_loop()
                                asyncio.set_event_loop(loop)
                                start_time = time.time()

                                success = loop.run_until_complete(
                                    self.robot.pick_and_place(detection[0], detection[1])
                                )
                                duration = time.time() - start_time

                                self.data_logger.log_pickplace_operation(
                                    detection[0], detection[1], self.robot.pickplace_settings.pick_z_height,
                                    self.robot.pickplace_settings.drop_x,
                                    self.robot.pickplace_settings.drop_y,
                                    self.robot.pickplace_settings.drop_z,
                                    duration, success,
                                    None if success else "Auto pick failed"
                                )

                                self.socketio.emit('pickplace_complete', {
                                    'success': success,
                                    'duration': duration,
                                    'auto_triggered': True,
                                    'auto_mode_type': auto_mode_type,
                                    'pick_coords': detection
                                })

                            except Exception as e:
                                self.logger.error(f"Auto pick and place error: {e}")
                            finally:
                                self.system_state.is_executing_pickplace = False
                                loop.close()

                        thread = threading.Thread(target=auto_pick)
                        thread.daemon = True
                        thread.start()

                _, buffer = cv2.imencode('.jpg', processed_frame,
                                         [cv2.IMWRITE_JPEG_QUALITY, 85])

                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

            time.sleep(0.033)  # ~30 FPS

    def start_background_tasks(self):
        """Start background monitoring tasks"""

        def status_monitor():
            """Monitor robot status in background"""
            while True:
                try:
                    if self.system_state.is_connected:
                        loop = asyncio.new_event_loop()
                        asyncio.set_event_loop(loop)
                        status = loop.run_until_complete(self.robot.get_status())
                        position = loop.run_until_complete(self.robot.get_position())
                        loop.close()

                        if status:
                            self.system_state.emergency_stop = status.get('emergency', '0') == '1'
                            self.system_state.is_homed = status.get('homed', '0') == '1'
                            self.system_state.motors_enabled = status.get('motors', '0') == '1'

                        if position:
                            self.system_state.current_position = position

                        self.socketio.emit('status_update', asdict(self.system_state))

                except Exception as e:
                    self.logger.error(f"Status monitor error: {e}")

                time.sleep(1)

        status_thread = threading.Thread(target=status_monitor, daemon=True)
        status_thread.start()

        self.vision.initialize_camera()

    def get_html_template(self):
        """Return the Enhanced Professional HTML template"""
        return '''<!DOCTYPE html>
<html lang="en" class="h-full">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>SCARA PRO - Advanced Robotic Control System</title>

    <!-- External Libraries -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.7.2/socket.io.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
    <script src="https://cdn.tailwindcss.com"></script>

    <style>
        :root {
            --primary-gradient: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            --secondary-gradient: linear-gradient(135deg, #f093fb 0%, #f5576c 100%);
            --success-gradient: linear-gradient(135deg, #4facfe 0%, #00f2fe 100%);
            --warning-gradient: linear-gradient(135deg, #43e97b 0%, #38f9d7 100%);
            --danger-gradient: linear-gradient(135deg, #fa709a 0%, #fee140 100%);
            --dark-bg: #0f172a;
            --card-bg: rgba(255, 255, 255, 0.1);
            --glass-border: rgba(255, 255, 255, 0.2);
        }

        [data-theme="dark"] {
            --card-bg: rgba(15, 23, 42, 0.8);
            --glass-border: rgba(255, 255, 255, 0.1);
        }

        /* Enhanced Glassmorphism */
        .glass-card {
            background: var(--card-bg);
            backdrop-filter: blur(20px) saturate(180%);
            border: 1px solid var(--glass-border);
            box-shadow: 0 8px 32px rgba(31, 38, 135, 0.37);
            transition: all 0.3s ease;
        }

        .glass-card:hover {
            transform: translateY(-5px);
            box-shadow: 0 20px 50px rgba(31, 38, 135, 0.5);
        }

        /* Gradient Animations */
        .animated-gradient {
            background-size: 200% 200%;
            animation: gradientShift 3s ease infinite;
        }

        @keyframes gradientShift {
            0% { background-position: 0% 50%; }
            50% { background-position: 100% 50%; }
            100% { background-position: 0% 50%; }
        }

        /* Neon Effects */
        .neon-border {
            box-shadow: 0 0 20px rgba(102, 126, 234, 0.6);
            border: 2px solid rgba(102, 126, 234, 0.8);
        }

        .neon-text {
            text-shadow: 0 0 10px rgba(102, 126, 234, 0.8);
        }

        /* Enhanced Buttons */
        .btn-futuristic {
            position: relative;
            overflow: hidden;
            transition: all 0.3s ease;
        }

        .btn-futuristic::before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 100%;
            height: 100%;
            background: linear-gradient(90deg, transparent, rgba(255,255,255,0.2), transparent);
            transition: left 0.5s;
        }

        .btn-futuristic:hover::before {
            left: 100%;
        }

        /* Status Indicators */
        .status-pulse {
            animation: pulse 2s infinite;
        }

        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }

        /* Custom Scrollbar */
        ::-webkit-scrollbar {
            width: 8px;
        }

        ::-webkit-scrollbar-track {
            background: rgba(255, 255, 255, 0.1);
            border-radius: 10px;
        }

        ::-webkit-scrollbar-thumb {
            background: var(--primary-gradient);
            border-radius: 10px;
        }

        /* Enhanced 3D Workspace */
        .workspace-3d {
            position: relative;
            border-radius: 20px;
            overflow: hidden;
            background: var(--primary-gradient);
            box-shadow: inset 0 0 50px rgba(0,0,0,0.3);
        }

        /* Holographic Display */
        .holographic {
            background: linear-gradient(45deg,
                rgba(0,255,255,0.1) 0%,
                rgba(255,0,255,0.1) 25%,
                rgba(255,255,0,0.1) 50%,
                rgba(0,255,255,0.1) 75%,
                rgba(255,0,255,0.1) 100%);
            border: 1px solid rgba(0,255,255,0.3);
            box-shadow: 0 0 30px rgba(0,255,255,0.2);
        }

        /* Data Visualization */
        .data-card {
            background: linear-gradient(135deg,
                rgba(255,255,255,0.1) 0%,
                rgba(255,255,255,0.05) 100%);
            border-left: 4px solid #00f2fe;
        }

        /* Enhanced Navigation */
        .nav-item {
            position: relative;
            transition: all 0.3s ease;
        }

        .nav-item::after {
            content: '';
            position: absolute;
            bottom: 0;
            left: 50%;
            width: 0;
            height: 2px;
            background: var(--primary-gradient);
            transition: all 0.3s ease;
            transform: translateX(-50%);
        }

        .nav-item.active::after,
        .nav-item:hover::after {
            width: 100%;
        }

        /* Enhanced Forms */
        .futuristic-input {
            background: rgba(255, 255, 255, 0.05);
            border: 1px solid rgba(255, 255, 255, 0.2);
            backdrop-filter: blur(10px);
            transition: all 0.3s ease;
        }

        .futuristic-input:focus {
            background: rgba(255, 255, 255, 0.1);
            border-color: #667eea;
            box-shadow: 0 0 20px rgba(102, 126, 234, 0.3);
        }

        /* Enhanced Sliders */
        .futuristic-slider {
            -webkit-appearance: none;
            height: 8px;
            border-radius: 5px;
            background: rgba(255, 255, 255, 0.2);
            outline: none;
        }

        .futuristic-slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: var(--primary-gradient);
            cursor: pointer;
            box-shadow: 0 0 10px rgba(102, 126, 234, 0.5);
        }

        /* Responsive Design */
        @media (max-width: 768px) {
            .grid-responsive {
                grid-template-columns: 1fr !important;
            }
        }

        /* Enhanced Notifications */
        .notification-modern {
            background: var(--card-bg);
            backdrop-filter: blur(20px);
            border-left: 4px solid;
            border-radius: 12px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.3);
            transform: translateX(400px);
            animation: slideInNotification 0.5s ease forwards;
        }

        @keyframes slideInNotification {
            to { transform: translateX(0); }
        }

        /* Loading Animations */
        .loading-spinner {
            width: 40px;
            height: 40px;
            border: 4px solid rgba(255, 255, 255, 0.1);
            border-left: 4px solid #667eea;
            border-radius: 50%;
            animation: spin 1s linear infinite;
        }

        @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
        }

        /* Enhanced Charts */
        .chart-container {
            background: var(--card-bg);
            backdrop-filter: blur(20px);
            border-radius: 20px;
            padding: 20px;
        }

        /* Theme Toggle */
        .theme-toggle {
            position: fixed;
            top: 20px;
            right: 20px;
            width: 60px;
            height: 60px;
            border-radius: 50%;
            background: var(--primary-gradient);
            border: none;
            color: white;
            font-size: 24px;
            cursor: pointer;
            z-index: 1000;
            transition: all 0.3s ease;
            box-shadow: 0 5px 15px rgba(0,0,0,0.3);
        }

        .theme-toggle:hover {
            transform: scale(1.1) rotate(180deg);
        }

        /* Body and Background */
        body {
            background: var(--primary-gradient);
            min-height: 100vh;
            font-family: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
        }

        [data-theme="dark"] body {
            background: linear-gradient(135deg, #0f172a 0%, #1e293b 100%);
        }

        /* Enhanced Task Status */
        .task-saved-indicator {
            position: fixed;
            top: 20px;
            left: 50%;
            transform: translateX(-50%);
            background: rgba(34, 197, 94, 0.9);
            color: white;
            padding: 10px 20px;
            border-radius: 8px;
            z-index: 1000;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(34, 197, 94, 0.3);
            opacity: 0;
            transition: opacity 0.3s ease;
        }

        .task-saved-indicator.show {
            opacity: 1;
        }
    </style>
</head>

<body class="min-h-screen text-white overflow-x-hidden">
    <!-- Task Saved Indicator -->
    <div id="task-saved-indicator" class="task-saved-indicator">
        <i class="fas fa-check-circle mr-2"></i>
        <span id="task-saved-text">Task saved successfully!</span>
    </div>

    <!-- Theme Toggle -->
    <button class="theme-toggle" onclick="toggleTheme()" title="Toggle Theme">
        <i class="fas fa-moon" id="theme-icon"></i>
    </button>

    <!-- Main Container -->
    <div class="container mx-auto px-4 py-6 max-w-7xl">

        <!-- Header Section -->
        <header class="glass-card rounded-3xl p-6 mb-8 relative overflow-hidden">
            <div class="absolute inset-0 animated-gradient opacity-10" style="background: var(--primary-gradient);"></div>
            <div class="relative z-10 flex flex-col lg:flex-row items-center justify-between">
                <!-- Logo and Title -->
                <div class="flex items-center space-x-4 mb-4 lg:mb-0">
                    <div class="w-16 h-16 rounded-2xl bg-gradient-to-br from-blue-500 to-purple-600 flex items-center justify-center text-2xl">
                        <i class="fas fa-robot"></i>
                    </div>
                    <div>
                        <h1 class="text-3xl font-bold neon-text">SCARA PRO</h1>
                        <p class="text-blue-300 text-sm">Advanced Robotic Control System v2.2 - Enhanced Task Persistence</p>
                    </div>
                </div>

                <!-- Status Indicators -->
                <div class="flex flex-wrap gap-3">
                    <div id="connection-status" class="px-4 py-2 rounded-full text-sm font-semibold bg-red-500 status-pulse">
                        <i class="fas fa-wifi mr-2"></i>Disconnected
                    </div>
                    <div id="robot-status" class="px-4 py-2 rounded-full text-sm font-semibold bg-gray-500">
                        <i class="fas fa-robot mr-2"></i>Not Ready
                    </div>
                    <div id="mode-status" class="px-4 py-2 rounded-full text-sm font-semibold bg-blue-500">
                        <i class="fas fa-hand-paper mr-2"></i>Manual
                    </div>
                    <div id="pickplace-status" class="px-4 py-2 rounded-full text-sm font-semibold bg-gray-500">
                        <i class="fas fa-magic mr-2"></i>Pick&Place Off
                    </div>
                    <div id="task-persistence-status" class="px-4 py-2 rounded-full text-sm font-semibold bg-green-500">
                        <i class="fas fa-database mr-2"></i>DB Ready
                    </div>
                </div>
            </div>
        </header>

        <!-- Navigation Tabs -->
        <nav class="glass-card rounded-2xl p-2 mb-8">
            <div class="flex flex-wrap gap-2">
                <button class="nav-item active px-6 py-3 rounded-xl font-semibold transition-all" onclick="showTab('control')">
                    <i class="fas fa-gamepad mr-2"></i>Control
                </button>
                <button class="nav-item px-6 py-3 rounded-xl font-semibold transition-all" onclick="showTab('vision')">
                    <i class="fas fa-eye mr-2"></i>Vision
                </button>
                <button class="nav-item px-6 py-3 rounded-xl font-semibold transition-all" onclick="showTab('tasks')">
                    <i class="fas fa-tasks mr-2"></i>Tasks
                </button>
                <button class="nav-item px-6 py-3 rounded-xl font-semibold transition-all" onclick="showTab('analytics')">
                    <i class="fas fa-chart-line mr-2"></i>Analytics
                </button>
                <button class="nav-item px-6 py-3 rounded-xl font-semibold transition-all" onclick="showTab('settings')">
                    <i class="fas fa-cog mr-2"></i>Settings
                </button>
            </div>
        </nav>

        <!-- Control Tab -->
        <div id="tab-control" class="tab-content">
            <div class="grid grid-cols-1 lg:grid-cols-2 gap-8 mb-8">

                <!-- Camera Feed -->
                <div class="glass-card rounded-3xl p-6 holographic">
                    <div class="flex items-center justify-between mb-6">
                        <h2 class="text-2xl font-bold">
                            <i class="fas fa-video mr-3 text-cyan-400"></i>Live Camera Feed
                        </h2>
                        <div class="flex gap-2">
                            <button class="btn-futuristic px-4 py-2 bg-green-500 rounded-lg text-sm" onclick="connectRobot()">
                                <i class="fas fa-plug mr-1"></i>Connect
                            </button>
                            <button class="btn-futuristic px-4 py-2 bg-blue-500 rounded-lg text-sm" onclick="homeRobot()">
                                <i class="fas fa-home mr-1"></i>Home
                            </button>
                        </div>
                    </div>

                    <div class="relative rounded-2xl overflow-hidden bg-black mb-4">
                        <img id="video-feed" src="/video_feed" alt="Robot Camera Feed" class="w-full h-auto">
                        <div class="absolute top-4 left-4 bg-black bg-opacity-50 px-3 py-1 rounded-lg text-sm" id="detection-info">
                            No objects detected
                        </div>
                    </div>

                    <!-- Quick Actions -->
                    <div class="grid grid-cols-2 gap-3">
                        <button class="btn-futuristic px-4 py-3 bg-purple-500 rounded-xl" onclick="toggleMode()">
                            <i class="fas fa-exchange-alt mr-2"></i>Toggle Mode
                        </button>
                        <button class="btn-futuristic px-4 py-3 bg-orange-500 rounded-xl" onclick="testConnection()">
                            <i class="fas fa-stethoscope mr-2"></i>Test
                        </button>
                    </div>

                    <!-- Robot Specs -->
                    <div class="mt-6 p-4 bg-gradient-to-r from-blue-500/20 to-purple-500/20 rounded-xl">
                        <h3 class="text-lg font-semibold mb-3">
                            <i class="fas fa-info-circle mr-2 text-blue-400"></i>Robot Specifications
                        </h3>
                        <div class="grid grid-cols-2 gap-4 text-sm">
                            <div>
                                <div class="text-gray-300">Max Reach</div>
                                <div class="font-bold text-cyan-400">358mm</div>
                            </div>
                            <div>
                                <div class="text-gray-300">Min Reach</div>
                                <div class="font-bold text-cyan-400">22mm</div>
                            </div>
                            <div>
                                <div class="text-gray-300">Z Range</div>
                                <div class="font-bold text-cyan-400">0-300mm</div>
                            </div>
                            <div>
                                <div class="text-gray-300">DOF</div>
                                <div class="font-bold text-cyan-400">4-Axis</div>
                            </div>
                        </div>
                    </div>
                </div>

                <!-- Robot Control Panel -->
                <div class="glass-card rounded-3xl p-6">
                    <h2 class="text-2xl font-bold mb-6">
                        <i class="fas fa-gamepad mr-3 text-green-400"></i>Robot Control
                    </h2>

                    <!-- Current Position Display -->
                    <div class="grid grid-cols-2 lg:grid-cols-4 gap-4 mb-8">
                        <div class="data-card p-4 rounded-xl text-center">
                            <div class="text-2xl font-bold text-cyan-400" id="pos-shoulder">0.0°</div>
                            <div class="text-sm text-gray-300">Shoulder</div>
                        </div>
                        <div class="data-card p-4 rounded-xl text-center">
                            <div class="text-2xl font-bold text-green-400" id="pos-elbow">0.0°</div>
                            <div class="text-sm text-gray-300">Elbow</div>
                        </div>
                        <div class="data-card p-4 rounded-xl text-center">
                            <div class="text-2xl font-bold text-yellow-400" id="pos-wrist">0.0°</div>
                            <div class="text-sm text-gray-300">Wrist</div>
                        </div>
                        <div class="data-card p-4 rounded-xl text-center">
                            <div class="text-2xl font-bold text-purple-400" id="pos-z">0.0mm</div>
                            <div class="text-sm text-gray-300">Height</div>
                        </div>
                    </div>

                    <!-- Control Tabs -->
                    <div class="flex gap-2 mb-6">
                        <button class="control-tab active px-4 py-2 rounded-lg bg-blue-500" onclick="showControlTab('joint')">
                            <i class="fas fa-sliders-h mr-1"></i>Joint
                        </button>
                        <button class="control-tab px-4 py-2 rounded-lg bg-gray-600" onclick="showControlTab('cartesian')">
                            <i class="fas fa-crosshairs mr-1"></i>Cartesian
                        </button>
                        <button class="control-tab px-4 py-2 rounded-lg bg-gray-600" onclick="showControlTab('gripper')">
                            <i class="fas fa-hand-rock mr-1"></i>Gripper
                        </button>
                    </div>

                    <!-- Joint Control -->
                    <div id="control-joint" class="control-content">
                        <div class="space-y-6">
                            <!-- Shoulder -->
                            <div>
                                <label class="block text-sm font-medium mb-2">Shoulder (0° - 180°)</label>
                                <input type="range" class="futuristic-slider w-full" id="shoulder-slider"
                                       min="0" max="180" value="90" oninput="updateJointDisplay('shoulder', this.value)">
                                <div class="text-center mt-2 text-cyan-400 font-bold" id="shoulder-value">90°</div>
                            </div>

                            <!-- Elbow -->
                            <div>
                                <label class="block text-sm font-medium mb-2">Elbow (-90° - 90°)</label>
                                <input type="range" class="futuristic-slider w-full" id="elbow-slider"
                                       min="-90" max="90" value="0" oninput="updateJointDisplay('elbow', this.value)">
                                <div class="text-center mt-2 text-green-400 font-bold" id="elbow-value">0°</div>
                            </div>

                            <!-- Wrist -->
                            <div>
                                <label class="block text-sm font-medium mb-2">Wrist (-90° - 90°)</label>
                                <input type="range" class="futuristic-slider w-full" id="wrist-slider"
                                       min="-90" max="90" value="0" oninput="updateJointDisplay('wrist', this.value)">
                                <div class="text-center mt-2 text-yellow-400 font-bold" id="wrist-value">0°</div>
                            </div>

                            <!-- Z Height -->
                            <div>
                                <label class="block text-sm font-medium mb-2">Z Height (0 - 300mm)</label>
                                <input type="range" class="futuristic-slider w-full" id="z-slider"
                                       min="0" max="300" value="50" oninput="updateJointDisplay('z', this.value)">
                                <div class="text-center mt-2 text-purple-400 font-bold" id="z-value">50mm</div>
                            </div>

                            <button class="btn-futuristic w-full py-4 bg-gradient-to-r from-blue-500 to-purple-600 rounded-xl text-lg font-semibold" onclick="moveJoint()">
                                <i class="fas fa-arrows-alt mr-2"></i>Move to Position
                            </button>
                        </div>
                    </div>

                    <!-- Cartesian Control -->
                    <div id="control-cartesian" class="control-content hidden">
                        <div class="space-y-6">
                            <div class="grid grid-cols-3 gap-4">
                                <div>
                                    <label class="block text-sm font-medium mb-2">X (mm)</label>
                                    <input type="number" class="futuristic-input w-full px-3 py-2 rounded-lg text-white"
                                           id="x-input" value="200" min="0" max="400" step="1">
                                </div>
                                <div>
                                    <label class="block text-sm font-medium mb-2">Y (mm)</label>
                                    <input type="number" class="futuristic-input w-full px-3 py-2 rounded-lg text-white"
                                           id="y-input" value="0" min="-200" max="200" step="1">
                                </div>
                                <div>
                                    <label class="block text-sm font-medium mb-2">Z (mm)</label>
                                    <input type="number" class="futuristic-input w-full px-3 py-2 rounded-lg text-white"
                                           id="z-input" value="50" min="0" max="300" step="1">
                                </div>
                            </div>

                            <div class="grid grid-cols-2 gap-4">
                                <button class="btn-futuristic py-3 bg-gradient-to-r from-green-500 to-blue-500 rounded-xl" onclick="moveCartesian()">
                                    <i class="fas fa-location-arrow mr-2"></i>Move to XYZ
                                </button>
                                <button class="btn-futuristic py-3 bg-gradient-to-r from-yellow-500 to-orange-500 rounded-xl" onclick="checkReachability()">
                                    <i class="fas fa-check-circle mr-2"></i>Check Reach
                                </button>
                            </div>

                            <div class="grid grid-cols-2 gap-4">
                                <button class="btn-futuristic py-2 bg-blue-600 rounded-lg text-sm" onclick="testInverseKinematics()">
                                    <i class="fas fa-calculator mr-1"></i>Test IK
                                </button>
                                <button class="btn-futuristic py-2 bg-purple-600 rounded-lg text-sm" onclick="testForwardKinematics()">
                                    <i class="fas fa-arrows-alt mr-1"></i>Test FK
                                </button>
                            </div>
                        </div>
                    </div>

                    <!-- Gripper Control -->
                    <div id="control-gripper" class="control-content hidden">
                        <div class="space-y-6">
                            <div class="grid grid-cols-2 gap-4">
                                <button class="btn-futuristic py-4 bg-gradient-to-r from-green-500 to-teal-500 rounded-xl" onclick="controlGripper(0)">
                                    <i class="fas fa-hand-paper mr-2"></i>Open
                                </button>
                                <button class="btn-futuristic py-4 bg-gradient-to-r from-red-500 to-pink-500 rounded-xl" onclick="controlGripper(90)">
                                    <i class="fas fa-hand-rock mr-2"></i>Close
                                </button>
                            </div>

                            <div>
                                <label class="block text-sm font-medium mb-2">Gripper Position (0° - 90°)</label>
                                <input type="range" class="futuristic-slider w-full" id="gripper-slider"
                                       min="0" max="90" value="0" oninput="updateGripperDisplay(this.value)">
                                <div class="text-center mt-2 text-orange-400 font-bold" id="gripper-value">0° (Open)</div>
                            </div>

                            <button class="btn-futuristic w-full py-3 bg-gradient-to-r from-orange-500 to-red-500 rounded-xl" onclick="setGripperPosition()">
                                <i class="fas fa-hand-point-right mr-2"></i>Set Position
                            </button>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Pick & Place Panel -->
            <div class="glass-card rounded-3xl p-6 mb-8 border-l-4 border-purple-500">
                <div class="flex items-center justify-between mb-6">
                    <h2 class="text-2xl font-bold">
                        <i class="fas fa-magic mr-3 text-purple-400"></i>Pick & Place Control
                    </h2>
                    <div id="pickplace-status-display" class="px-4 py-2 rounded-full text-sm font-semibold bg-gray-600">
                        <i class="fas fa-circle mr-2"></i>Pick & Place: Disabled
                    </div>
                </div>

                <!-- Coordinates Display -->
                <div class="grid grid-cols-2 lg:grid-cols-5 gap-4 mb-6">
                    <div class="data-card p-4 rounded-xl text-center border-l-4 border-purple-500">
                        <div class="text-xl font-bold text-purple-400" id="pickup-x">--</div>
                        <div class="text-sm text-gray-300">Pickup X</div>
                    </div>
                    <div class="data-card p-4 rounded-xl text-center border-l-4 border-purple-500">
                        <div class="text-xl font-bold text-purple-400" id="pickup-y">--</div>
                        <div class="text-sm text-gray-300">Pickup Y</div>
                    </div>
                    <div class="data-card p-4 rounded-xl text-center border-l-4 border-cyan-500">
                        <div class="text-xl font-bold text-cyan-400" id="drop-x">20</div>
                        <div class="text-sm text-gray-300">Drop X</div>
                    </div>
                    <div class="data-card p-4 rounded-xl text-center border-l-4 border-cyan-500">
                        <div class="text-xl font-bold text-cyan-400" id="drop-y">10</div>
                        <div class="text-sm text-gray-300">Drop Y</div>
                    </div>
                    <div class="data-card p-4 rounded-xl text-center border-l-4 border-cyan-500">
                        <div class="text-xl font-bold text-cyan-400" id="drop-z">20</div>
                        <div class="text-sm text-gray-300">Drop Z</div>
                    </div>
                </div>

                <!-- Pick & Place Actions -->
                <div class="grid grid-cols-2 lg:grid-cols-4 gap-4">
                    <button class="btn-futuristic py-3 bg-gradient-to-r from-purple-500 to-pink-500 rounded-xl" onclick="togglePickAndPlace()">
                        <i class="fas fa-power-off mr-2"></i>Toggle
                    </button>
                    <button class="btn-futuristic py-3 bg-gradient-to-r from-green-500 to-teal-500 rounded-xl" onclick="executePickAndPlace()">
                        <i class="fas fa-play mr-2"></i>Execute
                    </button>
                    <button class="btn-futuristic py-3 bg-gradient-to-r from-yellow-500 to-orange-500 rounded-xl" onclick="autoPickDetected()">
                        <i class="fas fa-crosshairs mr-2"></i>Pick Detected
                    </button>
                    <button class="btn-futuristic py-3 bg-gradient-to-r from-blue-500 to-purple-500 rounded-xl" onclick="openPickPlaceSettings()">
                        <i class="fas fa-cog mr-2"></i>Settings
                    </button>
                </div>
            </div>

            <!-- 3D Workspace Visualization -->
            <div class="glass-card rounded-3xl p-6 workspace-3d">
                <div class="flex items-center justify-between mb-6">
                    <h2 class="text-2xl font-bold">
                        <i class="fas fa-cube mr-3 text-cyan-400"></i>3D Robot Workspace
                    </h2>
                    <div class="flex gap-2">
                        <button class="px-3 py-1 bg-blue-500 rounded-lg text-sm camera-btn" onclick="setCameraView('perspective')">3D</button>
                        <button class="px-3 py-1 bg-gray-600 rounded-lg text-sm camera-btn" onclick="setCameraView('top')">Top</button>
                        <button class="px-3 py-1 bg-gray-600 rounded-lg text-sm camera-btn" onclick="setCameraView('side')">Side</button>
                        <button class="px-3 py-1 bg-gray-600 rounded-lg text-sm camera-btn" onclick="resetCameraView()">Reset</button>
                    </div>
                </div>

                <div id="robot-3d-container" class="h-96 rounded-2xl bg-gradient-to-br from-gray-900 to-black cursor-crosshair"
                     onclick="handleWorkspaceClick(event)" title="Click to set target position">
                </div>

                <!-- Workspace Metrics -->
                <div class="grid grid-cols-2 lg:grid-cols-6 gap-4 mt-6">
                    <div class="text-center">
                        <div class="text-lg font-bold text-cyan-400" id="workspace-x">200.0</div>
                        <div class="text-xs text-gray-300">X Position (mm)</div>
                    </div>
                    <div class="text-center">
                        <div class="text-lg font-bold text-green-400" id="workspace-y">0.0</div>
                        <div class="text-xs text-gray-300">Y Position (mm)</div>
                    </div>
                    <div class="text-center">
                        <div class="text-lg font-bold text-yellow-400" id="workspace-z">50.0</div>
                        <div class="text-xs text-gray-300">Z Position (mm)</div>
                    </div>
                    <div class="text-center">
                        <div class="text-lg font-bold text-purple-400" id="workspace-reach">
                            <i class="fas fa-check-circle"></i>
                        </div>
                        <div class="text-xs text-gray-300">Reachable</div>
                    </div>
                    <div class="text-center">
                        <div class="text-lg font-bold text-orange-400" id="cycle-time">2.3s</div>
                        <div class="text-xs text-gray-300">Est. Cycle Time</div>
                    </div>
                    <div class="text-center">
                        <div class="text-lg font-bold text-red-400" id="joint-health">
                            <i class="fas fa-heart"></i>
                        </div>
                        <div class="text-xs text-gray-300">Joint Health</div>
                    </div>
                </div>
            </div>
        </div>

        <!-- Vision Tab -->
        <div id="tab-vision" class="tab-content hidden">
            <div class="grid grid-cols-1 lg:grid-cols-2 gap-8">
                <!-- Vision Settings -->
                <div class="glass-card rounded-3xl p-6">
                    <h2 class="text-2xl font-bold mb-6">
                        <i class="fas fa-palette mr-3 text-pink-400"></i>Vision Settings
                    </h2>

                    <!-- HSV Color Range Controls -->
                    <div class="space-y-6">
                        <div>
                            <h3 class="text-lg font-semibold mb-4">Lower HSV Range</h3>
                            <div class="grid grid-cols-3 gap-4">
                                <div>
                                    <label class="block text-sm mb-2">Hue</label>
                                    <input type="range" class="futuristic-slider w-full" id="hue-lower-slider"
                                           min="0" max="179" value="20" oninput="updateColorDisplay()">
                                    <div class="text-center mt-1 text-sm" id="hue-lower-value">20</div>
                                </div>
                                <div>
                                    <label class="block text-sm mb-2">Saturation</label>
                                    <input type="range" class="futuristic-slider w-full" id="sat-lower-slider"
                                           min="0" max="255" value="100" oninput="updateColorDisplay()">
                                    <div class="text-center mt-1 text-sm" id="sat-lower-value">100</div>
                                </div>
                                <div>
                                    <label class="block text-sm mb-2">Value</label>
                                    <input type="range" class="futuristic-slider w-full" id="val-lower-slider"
                                           min="0" max="255" value="100" oninput="updateColorDisplay()">
                                    <div class="text-center mt-1 text-sm" id="val-lower-value">100</div>
                                </div>
                            </div>
                        </div>

                        <div>
                            <h3 class="text-lg font-semibold mb-4">Upper HSV Range</h3>
                            <div class="grid grid-cols-3 gap-4">
                                <div>
                                    <label class="block text-sm mb-2">Hue</label>
                                    <input type="range" class="futuristic-slider w-full" id="hue-upper-slider"
                                           min="0" max="179" value="30" oninput="updateColorDisplay()">
                                    <div class="text-center mt-1 text-sm" id="hue-upper-value">30</div>
                                </div>
                                <div>
                                    <label class="block text-sm mb-2">Saturation</label>
                                    <input type="range" class="futuristic-slider w-full" id="sat-upper-slider"
                                           min="0" max="255" value="255" oninput="updateColorDisplay()">
                                    <div class="text-center mt-1 text-sm" id="sat-upper-value">255</div>
                                </div>
                                <div>
                                    <label class="block text-sm mb-2">Value</label>
                                    <input type="range" class="futuristic-slider w-full" id="val-upper-slider"
                                           min="0" max="255" value="255" oninput="updateColorDisplay()">
                                    <div class="text-center mt-1 text-sm" id="val-upper-value">255</div>
                                </div>
                            </div>
                        </div>

                        <!-- Color Preview -->
                        <div class="flex items-center gap-4">
                            <span class="text-sm">Color Preview:</span>
                            <div class="w-16 h-16 rounded-full border-4 border-white/30" id="color-preview"
                                 style="background: hsl(25, 80%, 50%);"></div>
                        </div>

                        <!-- Color Presets -->
                        <div class="grid grid-cols-3 gap-4">
                            <button class="btn-futuristic py-3 bg-red-500 rounded-xl" onclick="setPresetColor('red')">
                                <i class="fas fa-circle mr-2"></i>Red
                            </button>
                            <button class="btn-futuristic py-3 bg-green-500 rounded-xl" onclick="setPresetColor('green')">
                                <i class="fas fa-circle mr-2"></i>Green
                            </button>
                            <button class="btn-futuristic py-3 bg-blue-500 rounded-xl" onclick="setPresetColor('blue')">
                                <i class="fas fa-circle mr-2"></i>Blue
                            </button>
                        </div>

                        <button class="btn-futuristic w-full py-3 bg-gradient-to-r from-pink-500 to-purple-600 rounded-xl" onclick="updateColorSettings()">
                            <i class="fas fa-save mr-2"></i>Apply Color Settings
                        </button>
                    </div>
                </div>

                <!-- Detection Info -->
                <div class="glass-card rounded-3xl p-6">
                    <h2 class="text-2xl font-bold mb-6">
                        <i class="fas fa-crosshairs mr-3 text-yellow-400"></i>Detection Info
                    </h2>

                    <!-- Current Detection Range -->
                    <div class="bg-gradient-to-r from-yellow-500/20 to-orange-500/20 rounded-xl p-4 mb-6">
                        <h3 class="text-lg font-semibold mb-3">Current Detection Range</h3>
                        <div class="space-y-2 text-sm">
                            <div class="flex justify-between">
                                <span>Hue:</span>
                                <span id="detection-hue-range" class="font-mono">20-30</span>
                            </div>
                            <div class="flex justify-between">
                                <span>Saturation:</span>
                                <span id="detection-sat-range" class="font-mono">100-255</span>
                            </div>
                            <div class="flex justify-between">
                                <span>Value:</span>
                                <span id="detection-val-range" class="font-mono">100-255</span>
                            </div>
                        </div>
                    </div>

                    <!-- Detection Statistics -->
                    <div class="space-y-4">
                        <div class="data-card p-4 rounded-xl">
                            <div class="text-lg font-bold text-green-400">0</div>
                            <div class="text-sm text-gray-300">Objects Detected Today</div>
                        </div>
                        <div class="data-card p-4 rounded-xl">
                            <div class="text-lg font-bold text-blue-400">0</div>
                            <div class="text-sm text-gray-300">Successful Picks</div>
                        </div>
                        <div class="data-card p-4 rounded-xl">
                            <div class="text-lg font-bold text-yellow-400">0%</div>
                            <div class="text-sm text-gray-300">Success Rate</div>
                        </div>
                    </div>

                    <!-- Reset Button -->
                    <button class="btn-futuristic w-full py-3 bg-gradient-to-r from-gray-600 to-gray-700 rounded-xl mt-6" onclick="resetToDefaults()">
                        <i class="fas fa-undo mr-2"></i>Reset to Defaults
                    </button>
                </div>
            </div>
        </div>

        <!-- Tasks Tab with Enhanced Persistence -->
        <div id="tab-tasks" class="tab-content hidden">
            <div class="grid grid-cols-1 lg:grid-cols-2 gap-8">

                <!-- Task Programming -->
                <div class="glass-card rounded-3xl p-6">
                    <div class="flex items-center justify-between mb-6">
                        <h2 class="text-2xl font-bold">
                            <i class="fas fa-tasks mr-3 text-green-400"></i>Task Programming
                            <span class="text-sm text-green-400 ml-2" id="task-count-indicator">(0 saved)</span>
                        </h2>
                        <div class="flex gap-2">
                            <button class="btn-futuristic px-4 py-2 bg-green-500 rounded-xl" onclick="openTaskModal()">
                                <i class="fas fa-plus mr-2"></i>New Task
                            </button>
                            <button class="btn-futuristic px-3 py-2 bg-blue-500 rounded-lg text-sm" onclick="refreshTasks()">
                                <i class="fas fa-sync-alt mr-1"></i>Refresh
                            </button>
                        </div>
                    </div>

                    <!-- Persistence Status -->
                    <div class="bg-gradient-to-r from-green-500/20 to-blue-500/20 rounded-xl p-4 mb-6">
                        <div class="flex items-center gap-3">
                            <i class="fas fa-database text-green-400 text-xl"></i>
                            <div>
                                <div class="font-semibold text-green-400">Enhanced Task Persistence</div>
                                <div class="text-sm text-gray-300">Tasks are automatically saved to database and persist across sessions</div>
                            </div>
                        </div>
                    </div>

                    <div id="task-list" class="space-y-3">
                        <!-- Tasks will be populated here -->
                        <div class="text-center text-gray-400 py-8" id="no-tasks-message">
                            No tasks created yet. Click "New Task" to start.
                        </div>
                    </div>
                </div>

                <!-- Task Sequencing -->
                <div class="glass-card rounded-3xl p-6">
                    <div class="flex items-center justify-between mb-6">
                        <h2 class="text-2xl font-bold">
                            <i class="fas fa-list-ol mr-3 text-purple-400"></i>Task Sequencing
                            <span class="text-sm text-purple-400 ml-2" id="sequence-count-indicator">(0 saved)</span>
                        </h2>
                        <div class="flex gap-2">
                            <button class="btn-futuristic px-4 py-2 bg-purple-500 rounded-xl" onclick="openSequenceModal()">
                                <i class="fas fa-plus mr-2"></i>New Sequence
                            </button>
                            <button class="btn-futuristic px-3 py-2 bg-blue-500 rounded-lg text-sm" onclick="refreshSequences()">
                                <i class="fas fa-sync-alt mr-1"></i>Refresh
                            </button>
                        </div>
                    </div>

                    <!-- Sequence Progress -->
                    <div id="sequence-progress" class="hidden mb-6">
                        <div class="bg-gradient-to-r from-purple-500/20 to-blue-500/20 rounded-xl p-4">
                            <div class="flex items-center justify-between mb-3">
                                <strong id="current-sequence-name">Sequence Name</strong>
                                <div id="sequence-status-badge" class="px-3 py-1 bg-green-500 rounded-full text-sm">
                                    <i class="fas fa-play mr-1"></i>Running
                                </div>
                            </div>
                            <div class="mb-3">
                                <div class="flex justify-between text-sm mb-2">
                                    <span>Task: <strong id="current-task-name">Task Name</strong></span>
                                    <span>Loop: <strong id="current-loop-display">1/5</strong></span>
                                </div>
                                <div class="bg-gray-700 rounded-full h-3 overflow-hidden">
                                    <div id="sequence-progress-bar" class="bg-gradient-to-r from-green-400 to-blue-500 h-full transition-all duration-300" style="width: 0%;"></div>
                                </div>
                            </div>
                            <div class="flex gap-2 justify-center">
                                <button class="btn-futuristic px-4 py-2 bg-yellow-500 rounded-lg" onclick="pauseSequence()" id="pause-btn">
                                    <i class="fas fa-pause mr-1"></i>Pause
                                </button>
                                <button class="btn-futuristic px-4 py-2 bg-blue-500 rounded-lg hidden" onclick="resumeSequence()" id="resume-btn">
                                    <i class="fas fa-play mr-1"></i>Resume
                                </button>
                                <button class="btn-futuristic px-4 py-2 bg-red-500 rounded-lg" onclick="stopSequence()">
                                    <i class="fas fa-stop mr-1"></i>Stop
                                </button>
                            </div>
                        </div>
                    </div>

                    <div id="sequence-list" class="space-y-3">
                        <!-- Sequences will be populated here -->
                        <div class="text-center text-gray-400 py-8" id="no-sequences-message">
                            No sequences created yet. Click "New Sequence" to start.
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <!-- Analytics Tab -->
        <div id="tab-analytics" class="tab-content hidden">
            <div class="grid grid-cols-1 lg:grid-cols-2 gap-8 mb-8">

                <!-- Performance Metrics -->
                <div class="glass-card rounded-3xl p-6">
                    <div class="flex items-center justify-between mb-6">
                        <h2 class="text-2xl font-bold">
                            <i class="fas fa-tachometer-alt mr-3 text-blue-400"></i>Performance Metrics
                        </h2>
                        <button class="btn-futuristic px-4 py-2 bg-blue-500 rounded-xl" onclick="refreshCharts()">
                            <i class="fas fa-sync-alt mr-2"></i>Refresh
                        </button>
                    </div>

                    <div class="grid grid-cols-2 gap-4 mb-6">
                        <div class="data-card p-4 rounded-xl text-center">
                            <div class="text-2xl font-bold text-green-400" id="total-movements">0</div>
                            <div class="text-sm text-gray-300">Total Movements</div>
                        </div>
                        <div class="data-card p-4 rounded-xl text-center">
                            <div class="text-2xl font-bold text-blue-400" id="avg-cycle-time">0.0s</div>
                            <div class="text-sm text-gray-300">Avg Cycle Time</div>
                        </div>
                        <div class="data-card p-4 rounded-xl text-center">
                            <div class="text-2xl font-bold text-yellow-400" id="success-rate">100%</div>
                            <div class="text-sm text-gray-300">Success Rate</div>
                        </div>
                        <div class="data-card p-4 rounded-xl text-center">
                            <div class="text-2xl font-bold text-purple-400" id="uptime">0h</div>
                            <div class="text-sm text-gray-300">System Uptime</div>
                        </div>
                    </div>
                </div>

                <!-- System Status -->
                <div class="glass-card rounded-3xl p-6">
                    <h2 class="text-2xl font-bold mb-6">
                        <i class="fas fa-info-circle mr-3 text-orange-400"></i>System Status
                    </h2>

                    <div class="space-y-4">
                        <div class="flex items-center justify-between p-3 bg-gradient-to-r from-green-500/20 to-teal-500/20 rounded-xl">
                            <span class="flex items-center">
                                <i class="fas fa-microchip mr-3 text-green-400"></i>CPU Usage
                            </span>
                            <span class="font-bold">15%</span>
                        </div>
                        <div class="flex items-center justify-between p-3 bg-gradient-to-r from-blue-500/20 to-purple-500/20 rounded-xl">
                            <span class="flex items-center">
                                <i class="fas fa-memory mr-3 text-blue-400"></i>Memory Usage
                            </span>
                            <span class="font-bold">45%</span>
                        </div>
                        <div class="flex items-center justify-between p-3 bg-gradient-to-r from-yellow-500/20 to-orange-500/20 rounded-xl">
                            <span class="flex items-center">
                                <i class="fas fa-thermometer-half mr-3 text-yellow-400"></i>Temperature
                            </span>
                            <span class="font-bold">42°C</span>
                        </div>
                        <div class="flex items-center justify-between p-3 bg-gradient-to-r from-purple-500/20 to-pink-500/20 rounded-xl">
                            <span class="flex items-center">
                                <i class="fas fa-bolt mr-3 text-purple-400"></i>Power Consumption
                            </span>
                            <span class="font-bold">125W</span>
                        </div>
                        <div class="flex items-center justify-between p-3 bg-gradient-to-r from-green-500/20 to-cyan-500/20 rounded-xl">
                            <span class="flex items-center">
                                <i class="fas fa-database mr-3 text-green-400"></i>Database Status
                            </span>
                            <span class="font-bold text-green-400">Operational</span>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Movement Analytics Chart -->
            <div class="glass-card rounded-3xl p-6 chart-container">
                <h2 class="text-2xl font-bold mb-6">
                    <i class="fas fa-chart-line mr-3 text-cyan-400"></i>Movement Analytics
                </h2>
                <div class="h-96">
                    <canvas id="movementChart"></canvas>
                </div>
            </div>
        </div>

        <!-- Settings Tab -->
        <div id="tab-settings" class="tab-content hidden">
            <div class="grid grid-cols-1 lg:grid-cols-2 gap-8">

                <!-- Motion Settings -->
                <div class="glass-card rounded-3xl p-6">
                    <h2 class="text-2xl font-bold mb-6">
                        <i class="fas fa-cog mr-3 text-blue-400"></i>Motion Settings
                    </h2>

                    <div class="space-y-6">
                        <!-- Speed Setting -->
                        <div>
                            <label class="block text-sm font-medium mb-2">Motor Speed (%)</label>
                            <div class="flex gap-4 items-center">
                                <input type="range" class="futuristic-slider flex-1" id="speed-slider"
                                       min="0" max="100" value="80" step="1" oninput="updateSpeedDisplay(this.value)">
                                <input type="number" class="futuristic-input w-20 px-3 py-1 rounded-lg text-white text-center"
                                       id="speed-input" min="0" max="100" value="80" step="1" oninput="updateSpeedFromInput(this.value)">
                            </div>
                            <div class="text-center mt-2">
                                <span class="text-cyan-400 font-bold" id="speed-value">80%</span>
                                <span class="text-gray-400 text-sm ml-2" id="actual-speed">(3200 steps/s)</span>
                            </div>
                        </div>

                        <!-- Acceleration Setting -->
                        <div>
                            <label class="block text-sm font-medium mb-2">Acceleration (%)</label>
                            <div class="flex gap-4 items-center">
                                <input type="range" class="futuristic-slider flex-1" id="acceleration-slider"
                                       min="0" max="100" value="50" step="1" oninput="updateAccelDisplay(this.value)">
                                <input type="number" class="futuristic-input w-20 px-3 py-1 rounded-lg text-white text-center"
                                       id="acceleration-input" min="0" max="100" value="50" step="1" oninput="updateAccelFromInput(this.value)">
                            </div>
                            <div class="text-center mt-2">
                                <span class="text-green-400 font-bold" id="acceleration-value">50%</span>
                                <span class="text-gray-400 text-sm ml-2" id="actual-accel">(2000 steps/s²)</span>
                            </div>
                        </div>

                        <button class="btn-futuristic w-full py-3 bg-gradient-to-r from-blue-500 to-purple-600 rounded-xl" onclick="updateMotionSettings()">
                            <i class="fas fa-save mr-2"></i>Apply Motion Settings
                        </button>
                    </div>
                </div>

                <!-- System Information -->
                <div class="glass-card rounded-3xl p-6">
                    <h2 class="text-2xl font-bold mb-6">
                        <i class="fas fa-info-circle mr-3 text-green-400"></i>System Information
                    </h2>

                    <div class="space-y-4">
                        <div class="bg-gradient-to-r from-blue-500/20 to-purple-500/20 rounded-xl p-4">
                            <h3 class="text-lg font-semibold mb-3">Current Settings</h3>
                            <div class="grid grid-cols-2 gap-4 text-sm">
                                <div>
                                    <div class="text-gray-300">Speed</div>
                                    <div class="font-bold text-cyan-400" id="current-speed">80%</div>
                                </div>
                                <div>
                                    <div class="text-gray-300">Acceleration</div>
                                    <div class="font-bold text-green-400" id="current-accel">50%</div>
                                </div>
                            </div>
                        </div>

                        <div class="bg-gradient-to-r from-green-500/20 to-teal-500/20 rounded-xl p-4">
                            <h3 class="text-lg font-semibold mb-3">Robot Limits</h3>
                            <div class="space-y-2 text-sm">
                                <div class="flex justify-between">
                                    <span>Shoulder:</span>
                                    <span class="font-mono">0° to 180°</span>
                                </div>
                                <div class="flex justify-between">
                                    <span>Elbow:</span>
                                    <span class="font-mono">-90° to 90°</span>
                                </div>
                                <div class="flex justify-between">
                                    <span>Wrist:</span>
                                    <span class="font-mono">-90° to 90°</span>
                                </div>
                                <div class="flex justify-between">
                                    <span>Gripper:</span>
                                    <span class="font-mono">0° to 90°</span>
                                </div>
                                <div class="flex justify-between">
                                    <span>Z-Axis:</span>
                                    <span class="font-mono">0mm to 300mm</span>
                                </div>
                            </div>
                        </div>

                        <div class="bg-gradient-to-r from-yellow-500/20 to-orange-500/20 rounded-xl p-4">
                            <h3 class="text-lg font-semibold mb-3">Version Info</h3>
                            <div class="space-y-1 text-sm">
                                <div>System: SCARA PRO v2.2</div>
                                <div>Feature: Enhanced Task Persistence</div>
                                <div>Database: SQLite with Auto-Save</div>
                                <div>Last Update: December 2024</div>
                            </div>
                        </div>

                        <div class="bg-gradient-to-r from-purple-500/20 to-pink-500/20 rounded-xl p-4">
                            <h3 class="text-lg font-semibold mb-3">Persistence Status</h3>
                            <div class="space-y-2 text-sm">
                                <div class="flex items-center justify-between">
                                    <span>Tasks Saved:</span>
                                    <span class="font-bold text-green-400" id="db-task-count">0</span>
                                </div>
                                <div class="flex items-center justify-between">
                                    <span>Sequences Saved:</span>
                                    <span class="font-bold text-green-400" id="db-sequence-count">0</span>
                                </div>
                                <div class="flex items-center justify-between">
                                    <span>Auto-Save:</span>
                                    <span class="font-bold text-green-400">Enabled</span>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <!-- Emergency Stop Button -->
    <button class="fixed bottom-8 right-8 w-20 h-20 bg-gradient-to-r from-red-500 to-pink-600 rounded-full text-white text-2xl shadow-2xl hover:scale-110 transition-all z-50" onclick="emergencyStop()" title="Emergency Stop">
        <i class="fas fa-stop"></i>
    </button>

    <!-- Modals -->

    <!-- Task Creation Modal -->
    <div class="modal fixed inset-0 bg-black bg-opacity-50 backdrop-blur-sm z-50 hidden" id="task-modal">
        <div class="flex items-center justify-center min-h-screen p-4">
            <div class="glass-card rounded-3xl p-8 max-w-2xl w-full max-h-screen overflow-y-auto">
                <div class="flex items-center justify-between mb-6">
                    <h3 class="text-2xl font-bold">
                        <i class="fas fa-plus-circle mr-3 text-green-400"></i>Create New Task
                        <span class="text-sm text-green-400 block">Automatically saved to database</span>
                    </h3>
                    <button class="text-2xl hover:text-red-400 transition-colors" onclick="closeTaskModal()">
                        <i class="fas fa-times"></i>
                    </button>
                </div>

                <div class="space-y-6">
                    <div>
                        <label class="block text-sm font-medium mb-2">Task Name</label>
                        <input type="text" class="futuristic-input w-full px-4 py-3 rounded-xl text-white"
                               id="task-name" placeholder="Enter task name" maxlength="50">
                        <div class="text-xs text-gray-400 mt-1">Task will be saved immediately upon creation</div>
                    </div>

                    <div id="task-steps">
                        <h4 class="text-lg font-semibold mb-4">Task Steps:</h4>
                    </div>

                    <div class="flex flex-wrap gap-3">
                        <button class="btn-futuristic px-4 py-2 bg-blue-500 rounded-xl" onclick="addTaskStep('move')">
                            <i class="fas fa-arrows-alt mr-2"></i>Add Move
                        </button>
                        <button class="btn-futuristic px-4 py-2 bg-orange-500 rounded-xl" onclick="addTaskStep('gripper')">
                            <i class="fas fa-hand-rock mr-2"></i>Add Gripper
                        </button>
                        <button class="btn-futuristic px-4 py-2 bg-yellow-500 rounded-xl" onclick="addTaskStep('wait')">
                            <i class="fas fa-clock mr-2"></i>Add Wait
                        </button>
                        <button class="btn-futuristic px-4 py-2 bg-purple-500 rounded-xl" onclick="addTaskStep('pickplace')">
                            <i class="fas fa-magic mr-2"></i>Add Pick&Place
                        </button>
                    </div>

                    <div class="flex gap-4 justify-end">
                        <button class="btn-futuristic px-6 py-3 bg-gray-600 rounded-xl" onclick="closeTaskModal()">Cancel</button>
                        <button class="btn-futuristic px-6 py-3 bg-gradient-to-r from-green-500 to-teal-500 rounded-xl" onclick="saveTask()">
                            <i class="fas fa-save mr-2"></i>Save Task to Database
                        </button>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <!-- Sequence Creation Modal -->
    <div class="modal fixed inset-0 bg-black bg-opacity-50 backdrop-blur-sm z-50 hidden" id="sequence-modal">
        <div class="flex items-center justify-center min-h-screen p-4">
            <div class="glass-card rounded-3xl p-8 max-w-2xl w-full max-h-screen overflow-y-auto">
                <div class="flex items-center justify-between mb-6">
                    <h3 class="text-2xl font-bold">
                        <i class="fas fa-list-ol mr-3 text-purple-400"></i>Create Task Sequence
                        <span class="text-sm text-purple-400 block">Automatically saved to database</span>
                    </h3>
                    <button class="text-2xl hover:text-red-400 transition-colors" onclick="closeSequenceModal()">
                        <i class="fas fa-times"></i>
                    </button>
                </div>

                <div class="space-y-6">
                    <div>
                        <label class="block text-sm font-medium mb-2">Sequence Name</label>
                        <input type="text" class="futuristic-input w-full px-4 py-3 rounded-xl text-white"
                               id="sequence-name" placeholder="Enter sequence name" maxlength="50">
                        <div class="text-xs text-gray-400 mt-1">Sequence will be saved immediately upon creation</div>
                    </div>

                    <div>
                        <label class="block text-sm font-medium mb-2">Loop Count</label>
                        <div class="flex gap-4 items-center">
                            <input type="number" class="futuristic-input w-32 px-4 py-3 rounded-xl text-white"
                                   id="loop-count" value="1" min="1" max="1000">
                            <label class="flex items-center gap-2">
                                <input type="checkbox" id="infinite-loop" class="w-4 h-4">
                                <span>Infinite Loop</span>
                            </label>
                        </div>
                    </div>

                    <div>
                        <label class="block text-sm font-medium mb-2">Available Tasks</label>
                        <div id="available-tasks" class="max-h-48 overflow-y-auto bg-gray-800/50 rounded-xl p-4">
                            <!-- Available tasks will be populated here -->
                        </div>
                    </div>

                    <div>
                        <label class="block text-sm font-medium mb-2">Selected Tasks</label>
                        <div id="selected-tasks" class="min-h-32 bg-gray-800/50 rounded-xl p-4">
                            <div class="text-center text-gray-400 py-8">
                                Select tasks from above to add to sequence
                            </div>
                        </div>
                    </div>

                    <div class="flex gap-4 justify-end">
                        <button class="btn-futuristic px-6 py-3 bg-gray-600 rounded-xl" onclick="closeSequenceModal()">Cancel</button>
                        <button class="btn-futuristic px-6 py-3 bg-gradient-to-r from-purple-500 to-pink-500 rounded-xl" onclick="saveSequence()">
                            <i class="fas fa-save mr-2"></i>Save Sequence to Database
                        </button>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <!-- Pick & Place Settings Modal -->
    <div class="modal fixed inset-0 bg-black bg-opacity-50 backdrop-blur-sm z-50 hidden" id="pickplace-settings-modal">
        <div class="flex items-center justify-center min-h-screen p-4">
            <div class="glass-card rounded-3xl p-8 max-w-xl w-full">
                <div class="flex items-center justify-between mb-6">
                    <h3 class="text-2xl font-bold">
                        <i class="fas fa-magic mr-3 text-purple-400"></i>Pick & Place Settings
                    </h3>
                    <button class="text-2xl hover:text-red-400 transition-colors" onclick="closePickPlaceSettings()">
                        <i class="fas fa-times"></i>
                    </button>
                </div>

                <div class="space-y-6">
                    <div>
                        <label class="block text-sm font-medium mb-2">Pick Z Height (mm)</label>
                        <input type="number" class="futuristic-input w-full px-4 py-3 rounded-xl text-white"
                               id="pick-z-height" value="50" min="0" max="300" step="1">
                    </div>

                    <div>
                        <label class="block text-sm font-medium mb-2">Drop Location</label>
                        <div class="grid grid-cols-3 gap-4">
                            <input type="number" class="futuristic-input px-4 py-3 rounded-xl text-white"
                                   id="drop-x-setting" placeholder="X" value="20" step="1">
                            <input type="number" class="futuristic-input px-4 py-3 rounded-xl text-white"
                                   id="drop-y-setting" placeholder="Y" value="10" step="1">
                            <input type="number" class="futuristic-input px-4 py-3 rounded-xl text-white"
                                   id="drop-z-setting" placeholder="Z" value="20" step="1">
                        </div>
                    </div>

                    <div>
                        <label class="block text-sm font-medium mb-2">Gripper Positions (0-90°)</label>
                        <div class="grid grid-cols-2 gap-4">
                            <div>
                                <label class="block text-xs text-gray-400 mb-1">Grip Position (°)</label>
                                <input type="number" class="futuristic-input w-full px-4 py-3 rounded-xl text-white"
                                       id="grip-position" value="45" min="0" max="90" step="1">
                            </div>
                            <div>
                                <label class="block text-xs text-gray-400 mb-1">Open Position (°)</label>
                                <input type="number" class="futuristic-input w-full px-4 py-3 rounded-xl text-white"
                                       id="open-position" value="0" min="0" max="90" step="1">
                            </div>
                        </div>
                    </div>

                    <div>
                        <label class="block text-sm font-medium mb-2">Timing Settings</label>
                        <div class="grid grid-cols-2 gap-4">
                            <div>
                                <label class="block text-xs text-gray-400 mb-1">Pick Delay (s)</label>
                                <input type="number" class="futuristic-input w-full px-4 py-3 rounded-xl text-white"
                                       id="pick-delay" value="1.0" min="0.1" max="5" step="0.1">
                            </div>
                            <div>
                                <label class="block text-xs text-gray-400 mb-1">Drop Delay (s)</label>
                                <input type="number" class="futuristic-input w-full px-4 py-3 rounded-xl text-white"
                                       id="drop-delay" value="1.0" min="0.1" max="5" step="0.1">
                            </div>
                        </div>
                    </div>

                    <div>
                        <label class="block text-sm font-medium mb-2">Approach Height (mm)</label>
                        <input type="number" class="futuristic-input w-full px-4 py-3 rounded-xl text-white"
                               id="approach-height" value="80" min="50" max="300" step="1">
                    </div>

                    <div>
                        <label class="flex items-center gap-3">
                            <input type="checkbox" id="auto-mode-checkbox" class="w-4 h-4">
                            <span>Auto Mode (automatically pick detected objects)</span>
                        </label>
                    </div>

                    <div class="flex gap-4 justify-end">
                        <button class="btn-futuristic px-6 py-3 bg-gray-600 rounded-xl" onclick="closePickPlaceSettings()">Cancel</button>
                        <button class="btn-futuristic px-6 py-3 bg-gradient-to-r from-purple-500 to-pink-500 rounded-xl" onclick="savePickPlaceSettings()">
                            <i class="fas fa-save mr-2"></i>Save Settings
                        </button>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <!-- Notification Container -->
    <div id="notification-container" class="fixed top-24 right-6 z-50 space-y-4">
        <!-- Notifications will be added here -->
    </div>

    <script>
        // Global variables
        let socket;
        let movementChart;
        let robot3D = null;
        let scene, camera, renderer;
        let currentTask = [];
        let isConnected = false;
        let currentMode = 'manual';
        let pickPlaceEnabled = false;
        let lastDetectedCoords = null;
        let selectedTasks = [];
        let currentSequenceProgress = null;

        // NEW: Enhanced task persistence variables
        let totalTasksCount = 0;
        let totalSequencesCount = 0;
        let taskPersistenceEnabled = true;

        // 3D Workspace Variables
        let reachabilityVisible = false;
        let trajectoryVisible = false;
        let gridVisible = false;
        let currentCameraView = 'perspective';
        let reachabilityMesh = null;
        let gridMesh = null;
        let trajectoryPoints = [];
        let workspaceGroup = null;

        // Theme Management
        let currentTheme = localStorage.getItem('theme') || 'dark';

        // Initialize application with enhanced error handling
        document.addEventListener('DOMContentLoaded', function() {
            debugLog('Application starting with enhanced task persistence...');

            try {
                initializeTheme();
                debugLog('Theme initialized');

                initializeSocketIO();
                debugLog('SocketIO initialized');

                initializeCharts();
                debugLog('Charts initialized');

                initialize3DVisualization();
                debugLog('3D visualization initialized');

                initializeSettingsDisplays();
                debugLog('Settings displays initialized');

                loadPickPlaceSettings();
                debugLog('Pick & place settings loading started');

                // Add event listener for infinite loop checkbox
                const infiniteLoopCheckbox = document.getElementById('infinite-loop');
                if (infiniteLoopCheckbox) {
                    infiniteLoopCheckbox.addEventListener('change', function() {
                        const loopCountInput = document.getElementById('loop-count');
                        if (this.checked) {
                            loopCountInput.disabled = true;
                            loopCountInput.style.opacity = '0.5';
                        } else {
                            loopCountInput.disabled = false;
                            loopCountInput.style.opacity = '1';
                        }
                    });
                    debugLog('Infinite loop checkbox event listener added');
                }

                // NEW: Delay task/sequence loading to ensure DOM is ready
                setTimeout(() => {
                    loadTasksWithPersistence();
                    debugLog('Enhanced task loading started');

                    loadSequencesWithPersistence();
                    debugLog('Enhanced sequence loading started');
                }, 500); // 500ms delay to ensure DOM is fully ready

                // Update displays periodically
                setInterval(updateDisplays, 1000);
                setInterval(updateWorkspaceMetrics, 500);
                setInterval(updatePersistenceStatus, 5000); // NEW: Update persistence status
                debugLog('Periodic update intervals started');

                // Show initialization complete notification
                setTimeout(() => {
                    showNotification('🚀 SCARA PRO v2.2 initialized with Enhanced Task Persistence!', 'success');
                    showTaskSavedIndicator('System initialized with database persistence enabled');
                    debugLog('Application initialization completed');
                }, 1500); // Increased delay

            } catch (error) {
                debugLog('Application initialization error:', error);
                console.error('Failed to initialize application:', error);
                showNotification('Failed to initialize application: ' + error.message, 'error');
            }
        });

        // === ENHANCED TASK PERSISTENCE FUNCTIONS ===

        function showTaskSavedIndicator(message) {
            try {
                const indicator = document.getElementById('task-saved-indicator');
                const textElement = document.getElementById('task-saved-text');

                if (!indicator || !textElement) {
                    debugLog('Task saved indicator elements not found');
                    return;
                }

                textElement.textContent = message;
                indicator.classList.add('show');

                setTimeout(() => {
                    indicator.classList.remove('show');
                }, 3000);
            } catch (error) {
                debugLog('Error showing task saved indicator:', error);
            }
        }

        async function loadTasksWithPersistence() {
            try {
                debugLog('Loading tasks with enhanced persistence...');
                const response = await fetch('/api/tasks');
                const tasks = await response.json();

                window.allTasks = tasks;
                totalTasksCount = Object.keys(tasks).length;

                debugLog(`Loaded ${totalTasksCount} tasks from persistent storage`);

                const taskList = document.getElementById('task-list');
                const noTasksMessage = document.getElementById('no-tasks-message');
                const taskCountIndicator = document.getElementById('task-count-indicator');

                if (!taskList) {
                    debugLog('Task list element not found');
                    return;
                }

                taskList.innerHTML = '';

                if (totalTasksCount === 0) {
                    if (noTasksMessage) {
                        noTasksMessage.style.display = 'block';
                    }
                } else {
                    if (noTasksMessage) {
                        noTasksMessage.style.display = 'none';
                    }

                    Object.values(tasks).forEach(task => {
                        const taskDiv = document.createElement('div');
                        taskDiv.className = 'bg-gray-700/50 rounded-xl p-4 border-l-4 border-green-500 hover:bg-gray-600/50 transition-all relative';

                        // Enhanced task display with persistence info
                        const lastExecuted = task.last_executed ? new Date(task.last_executed).toLocaleString() : 'Never';
                        const created = task.created ? new Date(task.created).toLocaleString() : 'Unknown';

                        taskDiv.innerHTML = `
                            <div class="flex items-center justify-between">
                                <div class="flex-1">
                                    <div class="flex items-center gap-2 mb-2">
                                        <div class="font-bold text-green-400">${task.name}</div>
                                        <div class="px-2 py-1 bg-green-500/20 rounded-full text-xs text-green-400">
                                            <i class="fas fa-database mr-1"></i>Saved
                                        </div>
                                    </div>
                                    <div class="text-sm text-gray-300 mb-2">
                                        ${task.steps.length} steps | Executed ${task.executions || 0} times
                                    </div>
                                    <div class="text-xs text-gray-400">
                                        <div>Created: ${created}</div>
                                        <div>Last run: ${lastExecuted}</div>
                                    </div>
                                </div>
                                <div class="flex gap-2">
                                    <button class="btn-futuristic px-4 py-2 bg-green-500 rounded-xl" onclick="executeTask('${task.name}')">
                                        <i class="fas fa-play mr-1"></i>Run
                                    </button>
                                    <button class="btn-futuristic px-3 py-2 bg-red-500 rounded-xl" onclick="deleteTaskWithConfirmation('${task.name}')">
                                        <i class="fas fa-trash"></i>
                                    </button>
                                </div>
                            </div>
                        `;
                        taskList.appendChild(taskDiv);
                    });
                }

                // Update task count indicator
                if (taskCountIndicator) {
                    taskCountIndicator.textContent = `(${totalTasksCount} saved)`;
                }

                updatePersistenceStatus();

                if (document.getElementById('sequence-modal').classList.contains('hidden') === false) {
                    populateAvailableTasks();
                }

                debugLog(`Successfully loaded and displayed ${totalTasksCount} persistent tasks`);

            } catch (error) {
                console.error('Failed to load tasks with persistence:', error);
                showNotification('Failed to load saved tasks: ' + error.message, 'error');
            }
        }

        async function loadSequencesWithPersistence() {
            try {
                debugLog('Loading sequences with enhanced persistence...');
                const response = await fetch('/api/sequences');
                const sequences = await response.json();

                totalSequencesCount = Object.keys(sequences).length;

                debugLog(`Loaded ${totalSequencesCount} sequences from persistent storage`);

                const sequenceList = document.getElementById('sequence-list');
                const noSequencesMessage = document.getElementById('no-sequences-message');
                const sequenceCountIndicator = document.getElementById('sequence-count-indicator');

                if (!sequenceList) {
                    debugLog('Sequence list element not found');
                    return;
                }

                sequenceList.innerHTML = '';

                if (totalSequencesCount === 0) {
                    if (noSequencesMessage) {
                        noSequencesMessage.style.display = 'block';
                    }
                } else {
                    if (noSequencesMessage) {
                        noSequencesMessage.style.display = 'none';
                    }

                    Object.values(sequences).forEach(sequence => {
                        const sequenceDiv = document.createElement('div');
                        sequenceDiv.className = 'bg-gray-700/50 rounded-xl p-4 border-l-4 border-purple-500 hover:bg-gray-600/50 transition-all';

                        const loopText = sequence.loop_count > 1000 ? 'Infinite' : sequence.loop_count;
                        const lastExecuted = sequence.last_executed ? new Date(sequence.last_executed).toLocaleString() : 'Never';
                        const created = sequence.created ? new Date(sequence.created).toLocaleString() : 'Unknown';

                        sequenceDiv.innerHTML = `
                            <div class="flex items-center justify-between">
                                <div class="flex-1">
                                    <div class="flex items-center gap-2 mb-2">
                                        <div class="font-bold text-purple-400">${sequence.name}</div>
                                        <div class="px-2 py-1 bg-purple-500/20 rounded-full text-xs text-purple-400">
                                            <i class="fas fa-database mr-1"></i>Saved
                                        </div>
                                    </div>
                                    <div class="text-sm text-gray-300 mb-2">
                                        ${sequence.tasks.length} tasks | ${loopText} loops | Executed ${sequence.executions || 0} times
                                    </div>
                                    <div class="text-xs text-gray-400 mb-2">Tasks: ${sequence.tasks.join(', ')}</div>
                                    <div class="text-xs text-gray-400">
                                        <div>Created: ${created}</div>
                                        <div>Last run: ${lastExecuted}</div>
                                    </div>
                                </div>
                                <div class="flex gap-2">
                                    <button class="btn-futuristic px-4 py-2 bg-purple-500 rounded-xl" onclick="startSequence('${sequence.name}')">
                                        <i class="fas fa-play mr-1"></i>Start
                                    </button>
                                    <button class="btn-futuristic px-3 py-2 bg-red-500 rounded-xl" onclick="deleteSequenceWithConfirmation('${sequence.name}')">
                                        <i class="fas fa-trash"></i>
                                    </button>
                                </div>
                            </div>
                        `;
                        sequenceList.appendChild(sequenceDiv);
                    });
                }

                // Update sequence count indicator
                if (sequenceCountIndicator) {
                    sequenceCountIndicator.textContent = `(${totalSequencesCount} saved)`;
                }

                updatePersistenceStatus();

                debugLog(`Successfully loaded and displayed ${totalSequencesCount} persistent sequences`);

            } catch (error) {
                console.error('Failed to load sequences with persistence:', error);
                showNotification('Failed to load saved sequences: ' + error.message, 'error');
            }
        }

        async function refreshTasks() {
            showNotification('Refreshing tasks from database...', 'info');
            await loadTasksWithPersistence();
            showNotification('Tasks refreshed successfully', 'success');
        }

        async function refreshSequences() {
            showNotification('Refreshing sequences from database...', 'info');
            await loadSequencesWithPersistence();
            showNotification('Sequences refreshed successfully', 'success');
        }

        function updatePersistenceStatus() {
            try {
                // Update database status indicators
                const dbTaskCount = document.getElementById('db-task-count');
                const dbSequenceCount = document.getElementById('db-sequence-count');

                if (dbTaskCount) dbTaskCount.textContent = totalTasksCount;
                if (dbSequenceCount) dbSequenceCount.textContent = totalSequencesCount;

                debugLog(`Persistence status updated: ${totalTasksCount} tasks, ${totalSequencesCount} sequences`);
            } catch (error) {
                debugLog('Error updating persistence status:', error);
            }
        }

        async function deleteTaskWithConfirmation(taskName) {
            if (confirm(`Are you sure you want to permanently delete task "${taskName}"? This will remove it from the database.`)) {
                try {
                    showNotification('Deleting task from database...', 'info');

                    const response = await fetch(`/api/tasks/${taskName}`, {
                        method: 'DELETE'
                    });

                    const data = await response.json();

                    if (data.success) {
                        showNotification(`Task "${taskName}" deleted successfully`, 'success');
                        showTaskSavedIndicator(`Task "${taskName}" permanently deleted from database`);
                        await loadTasksWithPersistence(); // Refresh the list
                    } else {
                        showNotification('Failed to delete task: ' + (data.error || ''), 'error');
                    }
                } catch (error) {
                    showNotification('Delete task error: ' + error.message, 'error');
                }
            }
        }

        async function deleteSequenceWithConfirmation(sequenceName) {
            if (confirm(`Are you sure you want to permanently delete sequence "${sequenceName}"? This will remove it from the database.`)) {
                try {
                    showNotification('Deleting sequence from database...', 'info');

                    const response = await fetch(`/api/sequences/${sequenceName}`, {
                        method: 'DELETE'
                    });

                    const data = await response.json();

                    if (data.success) {
                        showNotification(`Sequence "${sequenceName}" deleted successfully`, 'success');
                        showTaskSavedIndicator(`Sequence "${sequenceName}" permanently deleted from database`);
                        await loadSequencesWithPersistence(); // Refresh the list
                    } else {
                        showNotification('Failed to delete sequence: ' + (data.error || ''), 'error');
                    }
                } catch (error) {
                    showNotification('Delete sequence error: ' + error.message, 'error');
                }
            }
        }

        // === THEME MANAGEMENT ===

        function initializeTheme() {
            document.documentElement.setAttribute('data-theme', currentTheme);
            updateThemeIcon();
        }

        function toggleTheme() {
            currentTheme = currentTheme === 'light' ? 'dark' : 'light';
            document.documentElement.setAttribute('data-theme', currentTheme);
            localStorage.setItem('theme', currentTheme);
            updateThemeIcon();

            // Add a subtle animation to indicate theme change
            document.body.style.transform = 'scale(0.98)';
            setTimeout(() => {
                document.body.style.transform = 'scale(1)';
            }, 150);

            showNotification(`Switched to ${currentTheme} theme`, 'info');
        }

        function updateThemeIcon() {
            const icon = document.getElementById('theme-icon');
            icon.className = currentTheme === 'light' ? 'fas fa-moon' : 'fas fa-sun';
        }

        // === TAB NAVIGATION ===

        function showTab(tabName) {
            // Hide all tabs
            document.querySelectorAll('.tab-content').forEach(tab => {
                tab.classList.add('hidden');
            });

            // Remove active class from all nav items
            document.querySelectorAll('.nav-item').forEach(item => {
                item.classList.remove('active');
            });

            // Show selected tab
            document.getElementById(`tab-${tabName}`).classList.remove('hidden');

            // Add active class to clicked nav item
            event.target.closest('.nav-item').classList.add('active');
        }

        // === CONTROL TAB NAVIGATION ===

        function showControlTab(tabName) {
            // Hide all control contents
            document.querySelectorAll('.control-content').forEach(content => {
                content.classList.add('hidden');
            });

            // Remove active class from all control tabs
            document.querySelectorAll('.control-tab').forEach(tab => {
                tab.classList.remove('active', 'bg-blue-500');
                tab.classList.add('bg-gray-600');
            });

            // Show selected control content
            document.getElementById(`control-${tabName}`).classList.remove('hidden');

            // Add active class to clicked control tab
            event.target.closest('.control-tab').classList.add('active', 'bg-blue-500');
            event.target.closest('.control-tab').classList.remove('bg-gray-600');
        }

        // Initialize settings displays
        function initializeSettingsDisplays() {
            updateSpeedDisplay(80);
            updateAccelDisplay(50);
            updateColorDisplay();
        }

        // === ENHANCED TASK CREATION WITH PERSISTENCE ===

        async function saveTask() {
            try {
                const taskName = document.getElementById('task-name').value.trim();
                if (!taskName) {
                    showNotification('Please enter a task name', 'warning');
                    return;
                }

                const steps = [];
                const stepElements = document.querySelectorAll('[class*="bg-gray-700"]') ||
                                   document.querySelectorAll('.task-step-item') ||
                                   document.querySelectorAll('#task-steps > div > div');

                console.log('Found', stepElements.length, 'step elements');

                stepElements.forEach((stepEl, index) => {
                    console.log('Processing step', index, stepEl);
                    const stepData = {};

                    // Check for move step
                    const xInput = stepEl.querySelector('.step-x');
                    const yInput = stepEl.querySelector('.step-y');
                    const zInput = stepEl.querySelector('.step-z');

                    if (xInput && yInput && zInput) {
                        stepData.type = 'move_cartesian';
                        stepData.x = parseFloat(xInput.value) || 0;
                        stepData.y = parseFloat(yInput.value) || 0;
                        stepData.z = parseFloat(zInput.value) || 0;
                        console.log('Found move step:', stepData);
                    }
                    // Check for gripper step
                    else if (stepEl.querySelector('.step-gripper')) {
                        stepData.type = 'gripper';
                        stepData.position = parseInt(stepEl.querySelector('.step-gripper').value) || 0;
                        console.log('Found gripper step:', stepData);
                    }
                    // Check for wait step
                    else if (stepEl.querySelector('.step-duration')) {
                        stepData.type = 'wait';
                        stepData.duration = parseFloat(stepEl.querySelector('.step-duration').value) || 1;
                        console.log('Found wait step:', stepData);
                    }
                    // Check for pick and place step
                    else if (stepEl.querySelector('.step-pick-x')) {
                        stepData.type = 'pick_and_place';
                        stepData.pick_x = parseFloat(stepEl.querySelector('.step-pick-x').value) || 0;
                        stepData.pick_y = parseFloat(stepEl.querySelector('.step-pick-y').value) || 0;
                        stepData.drop_x = parseFloat(stepEl.querySelector('.step-drop-x').value) || 0;
                        stepData.drop_y = parseFloat(stepEl.querySelector('.step-drop-y').value) || 0;
                        stepData.drop_z = parseFloat(stepEl.querySelector('.step-drop-z').value) || 0;
                        console.log('Found pick&place step:', stepData);
                    }

                    // Only add step if it has a valid type
                    if (stepData.type) {
                        steps.push(stepData);
                    }
                });

                console.log('Total steps to save:', steps);

                if (steps.length === 0) {
                    showNotification('Please add at least one step to the task', 'warning');
                    return;
                }

                showNotification('Saving task to database...', 'info');
                showTaskSavedIndicator('Saving task to persistent storage...');

                const response = await fetch('/api/tasks', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                        'Accept': 'application/json'
                    },
                    body: JSON.stringify({
                        name: taskName,
                        steps: steps
                    })
                });

                if (!response.ok) {
                    throw new Error(`HTTP ${response.status}: ${response.statusText}`);
                }

                const data = await response.json();

                if (data.success) {
                    showNotification('Task saved to database successfully!', 'success');
                    showTaskSavedIndicator(`Task "${taskName}" saved to database with ${steps.length} steps`);
                    closeTaskModal();
                    await loadTasksWithPersistence(); // Refresh with persistence
                } else {
                    showNotification('Failed to save task to database: ' + (data.error || 'Unknown error'), 'error');
                }
            } catch (error) {
                console.error('Save task error:', error);
                showNotification('Save error: ' + error.message, 'error');
            }
        }

        async function saveSequence() {
            const sequenceName = document.getElementById('sequence-name').value.trim();
            const infiniteLoop = document.getElementById('infinite-loop').checked;
            const loopCount = infiniteLoop ? 999999 : parseInt(document.getElementById('loop-count').value);

            if (!sequenceName) {
                showNotification('Please enter a sequence name', 'warning');
                return;
            }

            if (selectedTasks.length === 0) {
                showNotification('Please select at least one task', 'warning');
                return;
            }

            try {
                showNotification('Saving sequence to database...', 'info');
                showTaskSavedIndicator('Saving sequence to persistent storage...');

                const response = await fetch('/api/sequences', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        name: sequenceName,
                        tasks: selectedTasks,
                        loop_count: loopCount
                    })
                });

                const data = await response.json();
                if (data.success) {
                    showNotification('Sequence saved to database successfully!', 'success');
                    showTaskSavedIndicator(`Sequence "${sequenceName}" saved to database with ${selectedTasks.length} tasks`);
                    closeSequenceModal();
                    await loadSequencesWithPersistence(); // Refresh with persistence
                } else {
                    showNotification('Failed to save sequence to database: ' + (data.error || ''), 'error');
                }
            } catch (error) {
                showNotification('Sequence creation error: ' + error.message, 'error');
            }
        }

        // === UPDATED SPEED/ACCELERATION DISPLAY FUNCTIONS ===

        function updateSpeedDisplay(value) {
            document.getElementById('speed-value').textContent = value + '%';
            document.getElementById('current-speed').textContent = value + '%';
            document.getElementById('speed-input').value = value;
            document.getElementById('speed-slider').value = value;

            // Calculate and display actual speed
            const actualSpeed = Math.round((value / 100) * 4000);
            document.getElementById('actual-speed').textContent = `(${actualSpeed} steps/s)`;
        }

        function updateAccelDisplay(value) {
            document.getElementById('acceleration-value').textContent = value + '%';
            document.getElementById('current-accel').textContent = value + '%';
            document.getElementById('acceleration-input').value = value;
            document.getElementById('acceleration-slider').value = value;

            // Calculate and display actual acceleration
            const actualAccel = Math.round((value / 100) * 4000);
            document.getElementById('actual-accel').textContent = `(${actualAccel} steps/s²)`;
        }

        function updateSpeedFromInput(value) {
            updateSpeedDisplay(value);
        }

        function updateAccelFromInput(value) {
            updateAccelDisplay(value);
        }

        function updateColorDisplay() {
            const hLower = parseInt(document.getElementById('hue-lower-slider').value);
            const sLower = parseInt(document.getElementById('sat-lower-slider').value);
            const vLower = parseInt(document.getElementById('val-lower-slider').value);
            const hUpper = parseInt(document.getElementById('hue-upper-slider').value);
            const sUpper = parseInt(document.getElementById('sat-upper-slider').value);
            const vUpper = parseInt(document.getElementById('val-upper-slider').value);

            // Update display values
            document.getElementById('hue-lower-value').textContent = hLower;
            document.getElementById('sat-lower-value').textContent = sLower;
            document.getElementById('val-lower-value').textContent = vLower;
            document.getElementById('hue-upper-value').textContent = hUpper;
            document.getElementById('sat-upper-value').textContent = sUpper;
            document.getElementById('val-upper-value').textContent = vUpper;

            // Update ranges display
            document.getElementById('detection-hue-range').textContent = `${hLower}-${hUpper}`;
            document.getElementById('detection-sat-range').textContent = `${sLower}-${sUpper}`;
            document.getElementById('detection-val-range').textContent = `${vLower}-${vUpper}`;

            // Update color preview (approximate HSV to RGB conversion for preview)
            const avgHue = (hLower + hUpper) / 2;
            const avgSat = (sLower + sUpper) / 2;
            const avgVal = (vLower + vUpper) / 2;

            const hslColor = `hsl(${avgHue * 2}, ${(avgSat / 255) * 100}%, ${(avgVal / 255) * 50}%)`;
            document.getElementById('color-preview').style.background = hslColor;
        }

        async function updateMotionSettings() {
            if (!isConnected) {
                showNotification('Robot not connected', 'warning');
                return;
            }

            const speed = parseInt(document.getElementById('speed-input').value) || 80;
            const acceleration = parseInt(document.getElementById('acceleration-input').value) || 50;

            // Validate ranges (0-100%)
            const validSpeed = Math.max(0, Math.min(100, speed));
            const validAccel = Math.max(0, Math.min(100, acceleration));

            if (speed !== validSpeed || acceleration !== validAccel) {
                showNotification('Values clamped to valid range (0-100%)', 'warning');
                updateSpeedDisplay(validSpeed);
                updateAccelDisplay(validAccel);
            }

            try {
                const response = await fetch('/api/settings/motion', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ speed: validSpeed, acceleration: validAccel })
                });

                const data = await response.json();
                if (data.success) {
                    const actualSpeed = Math.round((validSpeed / 100) * 4000);
                    const actualAccel = Math.round((validAccel / 100) * 4000);
                    showNotification(`Motion settings updated: ${validSpeed}% (${actualSpeed} steps/s), ${validAccel}% (${actualAccel} steps/s²)`, 'success');
                } else {
                    showNotification('Failed to update motion settings: ' + (data.error || ''), 'error');
                }
            } catch (error) {
                showNotification('Settings update error: ' + error.message, 'error');
            }
        }

        async function updateColorSettings() {
            const hLower = parseInt(document.getElementById('hue-lower-slider').value);
            const sLower = parseInt(document.getElementById('sat-lower-slider').value);
            const vLower = parseInt(document.getElementById('val-lower-slider').value);
            const hUpper = parseInt(document.getElementById('hue-upper-slider').value);
            const sUpper = parseInt(document.getElementById('sat-upper-slider').value);
            const vUpper = parseInt(document.getElementById('val-upper-slider').value);

            try {
                const response = await fetch('/api/settings/vision', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        hsv_lower: [hLower, sLower, vLower],
                        hsv_upper: [hUpper, sUpper, vUpper]
                    })
                });

                const data = await response.json();
                if (data.success) {
                    showNotification('Color detection settings updated', 'success');
                } else {
                    showNotification('Failed to update color settings: ' + (data.error || ''), 'error');
                }
            } catch (error) {
                showNotification('Color settings error: ' + error.message, 'error');
            }
        }

        function setPresetColor(color) {
            let hLower, sLower, vLower, hUpper, sUpper, vUpper;

            switch(color) {
                case 'red':
                    hLower = 0; sLower = 120; vLower = 70;
                    hUpper = 10; sUpper = 255; vUpper = 255;
                    break;
                case 'green':
                    hLower = 40; sLower = 40; vLower = 40;
                    hUpper = 80; sUpper = 255; vUpper = 255;
                    break;
                case 'blue':
                    hLower = 100; sLower = 150; vLower = 0;
                    hUpper = 130; sUpper = 255; vUpper = 255;
                    break;
            }

            document.getElementById('hue-lower-slider').value = hLower;
            document.getElementById('sat-lower-slider').value = sLower;
            document.getElementById('val-lower-slider').value = vLower;
            document.getElementById('hue-upper-slider').value = hUpper;
            document.getElementById('sat-upper-slider').value = sUpper;
            document.getElementById('val-upper-slider').value = vUpper;

            updateColorDisplay();
            showNotification(`${color.charAt(0).toUpperCase() + color.slice(1)} preset applied`, 'info');
        }

        function resetToDefaults() {
            // Reset motion settings
            updateSpeedDisplay(80);
            updateAccelDisplay(50);

            // Reset color settings to yellow/orange detection
            document.getElementById('hue-lower-slider').value = 20;
            document.getElementById('sat-lower-slider').value = 100;
            document.getElementById('val-lower-slider').value = 100;
            document.getElementById('hue-upper-slider').value = 30;
            document.getElementById('sat-upper-slider').value = 255;
            document.getElementById('val-upper-slider').value = 255;

            updateColorDisplay();
            showNotification('Settings reset to defaults', 'info');
        }

        // === PICK AND PLACE FUNCTIONS ===

        async function togglePickAndPlace() {
            try {
                const response = await fetch('/api/pick_and_place/toggle', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ enable: !pickPlaceEnabled })
                });

                const data = await response.json();
                if (data.success) {
                    pickPlaceEnabled = data.enabled;
                    updatePickPlaceStatus();
                    showNotification(data.message, 'pickplace');
                } else {
                    showNotification('Failed to toggle pick and place: ' + (data.error || ''), 'error');
                }
            } catch (error) {
                showNotification('Pick and place toggle error: ' + error.message, 'error');
            }
        }

        async function executePickAndPlace() {
            if (!isConnected) {
                showNotification('Robot not connected', 'warning');
                return;
            }

            const pickX = parseFloat(document.getElementById('x-input').value);
            const pickY = parseFloat(document.getElementById('y-input').value);
            const dropX = parseFloat(document.getElementById('drop-x').textContent);
            const dropY = parseFloat(document.getElementById('drop-y').textContent);
            const dropZ = parseFloat(document.getElementById('drop-z').textContent);

            try {
                showNotification('Starting pick and place operation...', 'info');

                const response = await fetch('/api/pick_and_place/execute', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        pick_x: pickX,
                        pick_y: pickY,
                        drop_x: dropX,
                        drop_y: dropY,
                        drop_z: dropZ
                    })
                });

                const data = await response.json();
                if (data.success) {
                    showNotification(data.message, 'pickplace');
                } else {
                    showNotification('Pick and place failed: ' + (data.error || ''), 'error');
                }
            } catch (error) {
                showNotification('Pick and place error: ' + error.message, 'error');
            }
        }

        async function autoPickDetected() {
            if (!lastDetectedCoords) {
                showNotification('No object detected', 'warning');
                return;
            }

            if (!isConnected) {
                showNotification('Robot not connected', 'warning');
                return;
            }

            try {
                showNotification('Auto-picking detected object...', 'info');

                const response = await fetch('/api/pick_and_place/auto_pick', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' }
                });

                const data = await response.json();
                if (data.success) {
                    showNotification(data.message, 'pickplace');
                } else {
                    showNotification('Auto pick failed: ' + (data.error || ''), 'error');
                }
            } catch (error) {
                showNotification('Auto pick error: ' + error.message, 'error');
            }
        }

        function openPickPlaceSettings() {
            loadPickPlaceSettings();
            document.getElementById('pickplace-settings-modal').classList.remove('hidden');
        }

        function closePickPlaceSettings() {
            document.getElementById('pickplace-settings-modal').classList.add('hidden');
        }

        async function loadPickPlaceSettings() {
            try {
                const response = await fetch('/api/pick_and_place/settings');
                const settings = await response.json();

                document.getElementById('pick-z-height').value = settings.pick_z_height || 50;
                document.getElementById('drop-x-setting').value = settings.drop_x || 20;
                document.getElementById('drop-y-setting').value = settings.drop_y || 10;
                document.getElementById('drop-z-setting').value = settings.drop_z || 20;
                document.getElementById('grip-position').value = settings.grip_position || 45;
                document.getElementById('open-position').value = settings.open_position || 0;
                document.getElementById('pick-delay').value = settings.pick_delay || 1.0;
                document.getElementById('drop-delay').value = settings.drop_delay || 1.0;
                document.getElementById('approach-height').value = settings.approach_height || 80;
                document.getElementById('auto-mode-checkbox').checked = settings.auto_mode || false;

                // Update display
                document.getElementById('drop-x').textContent = settings.drop_x || 20;
                document.getElementById('drop-y').textContent = settings.drop_y || 10;
                document.getElementById('drop-z').textContent = settings.drop_z || 20;

            } catch (error) {
                console.error('Failed to load pick and place settings:', error);
            }
        }

        async function savePickPlaceSettings() {
            try {
                const settings = {
                    pick_z_height: parseFloat(document.getElementById('pick-z-height').value),
                    drop_x: parseFloat(document.getElementById('drop-x-setting').value),
                    drop_y: parseFloat(document.getElementById('drop-y-setting').value),
                    drop_z: parseFloat(document.getElementById('drop-z-setting').value),
                    grip_position: parseInt(document.getElementById('grip-position').value),
                    open_position: parseInt(document.getElementById('open-position').value),
                    pick_delay: parseFloat(document.getElementById('pick-delay').value),
                    drop_delay: parseFloat(document.getElementById('drop-delay').value),
                    approach_height: parseFloat(document.getElementById('approach-height').value),
                    auto_mode: document.getElementById('auto-mode-checkbox').checked
                };

                const response = await fetch('/api/pick_and_place/settings', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(settings)
                });

                const data = await response.json();
                if (data.success) {
                    showNotification('Pick and place settings saved', 'success');

                    // Update display
                    document.getElementById('drop-x').textContent = settings.drop_x;
                    document.getElementById('drop-y').textContent = settings.drop_y;
                    document.getElementById('drop-z').textContent = settings.drop_z;

                    closePickPlaceSettings();
                } else {
                    showNotification('Failed to save settings: ' + (data.error || ''), 'error');
                }
            } catch (error) {
                showNotification('Settings save error: ' + error.message, 'error');
            }
        }

        function updatePickPlaceStatus() {
            const statusElement = document.getElementById('pickplace-status');
            const displayElement = document.getElementById('pickplace-status-display');

            if (pickPlaceEnabled) {
                statusElement.className = 'px-4 py-2 rounded-full text-sm font-semibold bg-purple-500';
                statusElement.innerHTML = '<i class="fas fa-magic mr-2"></i>Pick&Place On';
                displayElement.className = 'px-4 py-2 rounded-full text-sm font-semibold bg-purple-500';
                displayElement.innerHTML = '<i class="fas fa-circle mr-2"></i>Pick & Place: Enabled';
            } else {
                statusElement.className = 'px-4 py-2 rounded-full text-sm font-semibold bg-gray-500';
                statusElement.innerHTML = '<i class="fas fa-magic mr-2"></i>Pick&Place Off';
                displayElement.className = 'px-4 py-2 rounded-full text-sm font-semibold bg-gray-600';
                displayElement.innerHTML = '<i class="fas fa-circle mr-2"></i>Pick & Place: Disabled';
            }
        }

        // === ENHANCED REACHABILITY CHECK FUNCTION ===

        async function checkReachability() {
            const x = parseFloat(document.getElementById('x-input').value);
            const y = parseFloat(document.getElementById('y-input').value);
            const z = parseFloat(document.getElementById('z-input').value);

            try {
                showNotification('Checking position reachability...', 'info');

                const response = await fetch('/api/kinematics/reachable', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ x, y, z })
                });

                const data = await response.json();
                if (data.success) {
                    if (data.reachable) {
                        showNotification(
                            `Position (${x}, ${y}, ${z}) is REACHABLE ✓`,
                            'success'
                        );
                    } else {
                        showNotification(
                            `Position (${x}, ${y}, ${z}) is NOT reachable ✗ - Check limits: Max reach=${data.max_reach}mm, Min reach=${data.min_reach}mm, Z: ${data.z_limits.min}-${data.z_limits.max}mm`,
                            'warning'
                        );
                    }
                } else {
                    showNotification('Reachability check failed: ' + (data.error || 'Unknown error'), 'error');
                }
            } catch (error) {
                showNotification('Reachability check error: ' + error.message, 'error');
            }
        }

        // Enhanced 3D Visualization initialization
        function initialize3DVisualization() {
            const container = document.getElementById('robot-3d-container');
            if (!container) return;

            scene = new THREE.Scene();
            camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
            renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });

            renderer.setSize(container.clientWidth, container.clientHeight);
            renderer.setClearColor(0x000000, 0.0);
            renderer.shadowMap.enabled = true;
            renderer.shadowMap.type = THREE.PCFSoftShadowMap;
            container.appendChild(renderer.domElement);

            // Enhanced lighting setup
            const ambientLight = new THREE.AmbientLight(0x404040, 0.4);
            scene.add(ambientLight);

            const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
            directionalLight.position.set(50, 50, 50);
            directionalLight.castShadow = true;
            directionalLight.shadow.mapSize.width = 2048;
            directionalLight.shadow.mapSize.height = 2048;
            scene.add(directionalLight);

            // Add point lights for better illumination
            const pointLight1 = new THREE.PointLight(0x00aaff, 0.3, 200);
            pointLight1.position.set(100, 100, 0);
            scene.add(pointLight1);

            const pointLight2 = new THREE.PointLight(0xff00aa, 0.3, 200);
            pointLight2.position.set(-100, 100, 0);
            scene.add(pointLight2);

            // Create enhanced robot structure
            robot3D = createEnhancedRobotModel(scene);

            // Create workspace group for better organization
            workspaceGroup = new THREE.Group();
            scene.add(workspaceGroup);

            // Set initial camera position
            camera.position.set(100, 100, 100);
            camera.lookAt(0, 0, 0);

            // Add mouse controls for camera interaction
            let isDragging = false;
            let previousMousePosition = { x: 0, y: 0 };

            container.addEventListener('mousedown', function(e) {
                if (e.ctrlKey) {
                    isDragging = true;
                    previousMousePosition = { x: e.clientX, y: e.clientY };
                }
            });

            container.addEventListener('mousemove', function(e) {
                if (isDragging && e.ctrlKey) {
                    const deltaMove = {
                        x: e.clientX - previousMousePosition.x,
                        y: e.clientY - previousMousePosition.y
                    };

                    const spherical = new THREE.Spherical();
                    spherical.setFromVector3(camera.position);
                    spherical.theta -= deltaMove.x * 0.01;
                    spherical.phi += deltaMove.y * 0.01;
                    spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, spherical.phi));

                    camera.position.setFromSpherical(spherical);
                    camera.lookAt(0, 0, 0);

                    previousMousePosition = { x: e.clientX, y: e.clientY };
                }
            });

            container.addEventListener('mouseup', function() {
                isDragging = false;
            });

            // Mouse wheel zoom
            container.addEventListener('wheel', function(e) {
                e.preventDefault();
                const zoomSpeed = 0.1;
                const direction = e.deltaY > 0 ? 1 : -1;

                camera.position.multiplyScalar(1 + direction * zoomSpeed);

                const distance = camera.position.length();
                if (distance < 50) camera.position.normalize().multiplyScalar(50);
                if (distance > 500) camera.position.normalize().multiplyScalar(500);
            });

            // Animation loop
            function animate() {
                requestAnimationFrame(animate);
                updateEnhancedRobot3D();
                renderer.render(scene, camera);
            }
            animate();

            // Handle window resize
            window.addEventListener('resize', function() {
                camera.aspect = container.clientWidth / container.clientHeight;
                camera.updateProjectionMatrix();
                renderer.setSize(container.clientWidth, container.clientHeight);
            });

            // Show initial help
            setTimeout(() => {
                showNotification('💡 Click to move robot, Ctrl+drag to rotate view, scroll to zoom', 'info');
            }, 2000);
        }

        // Create enhanced 3D robot model
        function createEnhancedRobotModel(scene) {
            const robot = {
                base: null,
                shoulder: null,
                elbow: null,
                wrist: null,
                gripper: null
            };

            // Enhanced materials
            const baseMaterial = new THREE.MeshPhongMaterial({
                color: 0x444444,
                shininess: 30,
                specular: 0x111111
            });

            const armMaterial = new THREE.MeshPhongMaterial({
                color: 0x0066cc,
                shininess: 100,
                specular: 0x222244
            });

            const jointMaterial = new THREE.MeshPhongMaterial({
                color: 0x00aa00,
                shininess: 80,
                specular: 0x224422
            });

            // Base
            const baseGeometry = new THREE.CylinderGeometry(15, 20, 10, 16);
            robot.base = new THREE.Mesh(baseGeometry, baseMaterial);
            robot.base.position.y = 5;
            robot.base.castShadow = true;
            robot.base.receiveShadow = true;
            scene.add(robot.base);

            // Shoulder arm
            const shoulderGeometry = new THREE.BoxGeometry(4, 4, 60);
            robot.shoulder = new THREE.Mesh(shoulderGeometry, armMaterial);
            robot.shoulder.position.set(0, 15, 30);
            robot.shoulder.castShadow = true;
            scene.add(robot.shoulder);

            // Elbow arm
            const elbowGeometry = new THREE.BoxGeometry(4, 4, 50);
            robot.elbow = new THREE.Mesh(elbowGeometry, jointMaterial);
            robot.elbow.position.set(0, 15, 85);
            robot.elbow.castShadow = true;
            scene.add(robot.elbow);

            // Wrist
            const wristGeometry = new THREE.CylinderGeometry(3, 3, 8, 8);
            const wristMaterial = new THREE.MeshPhongMaterial({
                color: 0xaa0000,
                shininess: 120,
                specular: 0x442222
            });
            robot.wrist = new THREE.Mesh(wristGeometry, wristMaterial);
            robot.wrist.position.set(0, 15, 115);
            robot.wrist.castShadow = true;
            scene.add(robot.wrist);

            // Gripper
            const gripperGeometry = new THREE.BoxGeometry(8, 2, 6);
            const gripperMaterial = new THREE.MeshPhongMaterial({
                color: 0xffaa00,
                shininess: 90,
                specular: 0x444422
            });
            robot.gripper = new THREE.Mesh(gripperGeometry, gripperMaterial);
            robot.gripper.position.set(0, 15, 125);
            robot.gripper.castShadow = true;
            scene.add(robot.gripper);

            // Add coordinate axes
            const axesHelper = new THREE.AxesHelper(30);
            scene.add(axesHelper);

            return robot;
        }

        // Enhanced robot 3D update
        function updateEnhancedRobot3D() {
            if (!robot3D) return;

            const shoulderAngle = parseFloat(document.getElementById('shoulder-slider').value) * Math.PI / 180;
            const elbowAngle = parseFloat(document.getElementById('elbow-slider').value) * Math.PI / 180;
            const wristAngle = parseFloat(document.getElementById('wrist-slider').value) * Math.PI / 180;

            // Smooth interpolation for robot joint rotations
            if (robot3D.shoulder) {
                robot3D.shoulder.rotation.y = THREE.MathUtils.lerp(robot3D.shoulder.rotation.y, shoulderAngle, 0.1);
            }
            if (robot3D.elbow) {
                robot3D.elbow.rotation.y = THREE.MathUtils.lerp(robot3D.elbow.rotation.y, shoulderAngle + elbowAngle, 0.1);
            }
            if (robot3D.wrist) {
                robot3D.wrist.rotation.y = THREE.MathUtils.lerp(robot3D.wrist.rotation.y, shoulderAngle + elbowAngle, 0.1);
                robot3D.wrist.rotation.z = THREE.MathUtils.lerp(robot3D.wrist.rotation.z, wristAngle, 0.1);
            }
            if (robot3D.gripper) {
                const gripperPos = parseFloat(document.getElementById('gripper-slider').value) / 90;
                robot3D.gripper.scale.x = THREE.MathUtils.lerp(robot3D.gripper.scale.x, 1 - gripperPos * 0.3, 0.1);
            }
        }

        // === 3D WORKSPACE INTERACTION ===

        function handleWorkspaceClick(event) {
            if (!isConnected) {
                showNotification('Robot not connected', 'warning');
                return;
            }

            const container = document.getElementById('robot-3d-container');
            const rect = container.getBoundingClientRect();
            const x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
            const y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

            // Convert screen coordinates to world coordinates
            const vector = new THREE.Vector3(x, y, 0.5);
            vector.unproject(camera);
            const dir = vector.sub(camera.position).normalize();
            const distance = -camera.position.z / dir.z;
            const pos = camera.position.clone().add(dir.multiplyScalar(distance));

            // Convert to robot workspace coordinates
            const robotX = pos.x * 2;
            const robotY = pos.y * 2;
            const robotZ = parseFloat(document.getElementById('z-input').value) || 50;

            // Update input fields
            document.getElementById('x-input').value = robotX.toFixed(1);
            document.getElementById('y-input').value = robotY.toFixed(1);

            showNotification(`Target: (${robotX.toFixed(1)}, ${robotY.toFixed(1)}, ${robotZ})`, 'info');

            // Auto-move option
            if (event.shiftKey) {
                moveCartesian();
            }
        }

        function setCameraView(view) {
            currentCameraView = view;

            // Update active button
            document.querySelectorAll('.camera-btn').forEach(btn => btn.classList.remove('bg-blue-500'));
            document.querySelectorAll('.camera-btn').forEach(btn => btn.classList.add('bg-gray-600'));
            event.target.classList.add('bg-blue-500');
            event.target.classList.remove('bg-gray-600');

            // Animate camera to new position
            animateCameraToView(view);
            showNotification(`Switched to ${view} view`, 'info');
        }

        function resetCameraView() {
            camera.position.set(100, 100, 100);
            camera.lookAt(0, 0, 0);
            currentCameraView = 'perspective';

            document.querySelectorAll('.camera-btn').forEach(btn => btn.classList.remove('bg-blue-500'));
            document.querySelectorAll('.camera-btn').forEach(btn => btn.classList.add('bg-gray-600'));
            document.querySelector('.camera-btn').classList.add('bg-blue-500');

            showNotification('Camera view reset', 'info');
        }

        function animateCameraToView(view) {
            let targetPosition, targetLookAt;

            switch(view) {
                case 'top':
                    targetPosition = new THREE.Vector3(0, 200, 0);
                    targetLookAt = new THREE.Vector3(0, 0, 0);
                    break;
                case 'side':
                    targetPosition = new THREE.Vector3(200, 50, 0);
                    targetLookAt = new THREE.Vector3(0, 0, 0);
                    break;
                case 'front':
                    targetPosition = new THREE.Vector3(0, 50, 200);
                    targetLookAt = new THREE.Vector3(0, 0, 0);
                    break;
                default:
                    targetPosition = new THREE.Vector3(100, 100, 100);
                    targetLookAt = new THREE.Vector3(0, 0, 0);
            }

            const startPosition = camera.position.clone();
            const startTime = Date.now();
            const duration = 1000;

            function animateCamera() {
                const elapsed = Date.now() - startTime;
                const progress = Math.min(elapsed / duration, 1);
                const eased = 1 - Math.pow(1 - progress, 3);

                camera.position.lerpVectors(startPosition, targetPosition, eased);
                camera.lookAt(targetLookAt);

                if (progress < 1) {
                    requestAnimationFrame(animateCamera);
                }
            }

            animateCamera();
        }

        function updateWorkspaceMetrics() {
            const x = parseFloat(document.getElementById('x-input').value) || 200;
            const y = parseFloat(document.getElementById('y-input').value) || 0;
            const z = parseFloat(document.getElementById('z-input').value) || 50;

            document.getElementById('workspace-x').textContent = x.toFixed(1);
            document.getElementById('workspace-y').textContent = y.toFixed(1);
            document.getElementById('workspace-z').textContent = z.toFixed(1);

            checkWorkspaceReachability(x, y, z);
            updateCycleTimeEstimate(x, y, z);
            updateJointHealthIndicator();
        }

        async function checkWorkspaceReachability(x, y, z) {
            try {
                const response = await fetch('/api/kinematics/reachable', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ x, y, z })
                });

                const data = await response.json();
                const reachIcon = document.getElementById('workspace-reach');

                if (data.success && data.reachable) {
                    reachIcon.innerHTML = '<i class="fas fa-check-circle text-green-400"></i>';
                } else {
                    reachIcon.innerHTML = '<i class="fas fa-times-circle text-red-400"></i>';
                }
            } catch (error) {
                console.error('Reachability check failed:', error);
            }
        }

        function updateCycleTimeEstimate(x, y, z) {
            const currentX = parseFloat(document.getElementById('pos-x')?.textContent) || 200;
            const currentY = parseFloat(document.getElementById('pos-y')?.textContent) || 0;
            const currentZ = parseFloat(document.getElementById('pos-z')?.textContent) || 50;

            const distance = Math.sqrt(
                Math.pow(x - currentX, 2) +
                Math.pow(y - currentY, 2) +
                Math.pow(z - currentZ, 2)
            );

            const estimatedTime = (distance / 100) + 0.5;
            document.getElementById('cycle-time').textContent = estimatedTime.toFixed(1) + 's';
        }

        function updateJointHealthIndicator() {
            const healthStatus = Math.random() > 0.1 ? 'good' : 'warning';
            const healthIcon = document.getElementById('joint-health');

            if (healthStatus === 'good') {
                healthIcon.innerHTML = '<i class="fas fa-heart text-green-400"></i>';
            } else {
                healthIcon.innerHTML = '<i class="fas fa-heart text-yellow-400"></i>';
            }
        }

        // === TASK MANAGEMENT FUNCTIONS ===

        function openTaskModal() {
            currentTask = [];
            document.getElementById('task-name').value = '';
            document.getElementById('task-steps').innerHTML = '<h4 class="text-lg font-semibold mb-4">Task Steps:</h4>';
            document.getElementById('task-modal').classList.remove('hidden');
        }

        function closeTaskModal() {
            document.getElementById('task-modal').classList.add('hidden');
        }

        function addTaskStep(type) {
            const stepDiv = document.createElement('div');
            stepDiv.className = 'task-step-item bg-gray-700/50 rounded-xl p-4 mb-3 border-l-4 border-blue-500';

            let stepContent = '';
            if (type === 'move') {
                stepContent = `
                    <div class="flex items-center justify-between">
                        <div class="flex-1">
                            <strong class="text-blue-400">Move Step:</strong>
                            <div class="grid grid-cols-3 gap-2 mt-2">
                                <input type="number" value="200" placeholder="X" class="futuristic-input px-2 py-1 rounded text-white text-sm step-x">
                                <input type="number" value="0" placeholder="Y" class="futuristic-input px-2 py-1 rounded text-white text-sm step-y">
                                <input type="number" value="50" placeholder="Z" min="0" max="300" class="futuristic-input px-2 py-1 rounded text-white text-sm step-z">
                            </div>
                        </div>
                        <button type="button" class="btn-futuristic px-3 py-1 bg-red-500 rounded-lg ml-4" onclick="removeTaskStep(this)">
                            <i class="fas fa-trash"></i>
                        </button>
                    </div>
                `;
            } else if (type === 'gripper') {
                stepContent = `
                    <div class="flex items-center justify-between">
                        <div>
                            <strong class="text-orange-400">Gripper Step:</strong>
                            <select class="futuristic-input px-2 py-1 rounded text-white text-sm ml-2 step-gripper">
                                <option value="0">Open (0°)</option>
                                <option value="45">Partial (45°)</option>
                                <option value="90">Close (90°)</option>
                            </select>
                        </div>
                        <button type="button" class="btn-futuristic px-3 py-1 bg-red-500 rounded-lg" onclick="removeTaskStep(this)">
                            <i class="fas fa-trash"></i>
                        </button>
                    </div>
                `;
            } else if (type === 'wait') {
                stepContent = `
                    <div class="flex items-center justify-between">
                        <div>
                            <strong class="text-yellow-400">Wait Step:</strong>
                            <input type="number" value="1" step="0.1" placeholder="Duration" class="futuristic-input px-2 py-1 rounded text-white text-sm ml-2 step-duration">
                            <span class="text-sm text-gray-400 ml-1">seconds</span>
                        </div>
                        <button type="button" class="btn-futuristic px-3 py-1 bg-red-500 rounded-lg" onclick="removeTaskStep(this)">
                            <i class="fas fa-trash"></i>
                        </button>
                    </div>
                `;
            } else if (type === 'pickplace') {
                stepContent = `
                    <div class="flex items-center justify-between">
                        <div class="flex-1">
                            <strong class="text-purple-400">Pick & Place Step:</strong>
                            <div class="grid grid-cols-2 gap-2 mt-2">
                                <div>
                                    <label class="text-xs text-gray-400">Pick Position</label>
                                    <div class="flex gap-1">
                                        <input type="number" value="200" placeholder="X" class="futuristic-input px-2 py-1 rounded text-white text-sm step-pick-x">
                                        <input type="number" value="0" placeholder="Y" class="futuristic-input px-2 py-1 rounded text-white text-sm step-pick-y">
                                    </div>
                                </div>
                                <div>
                                    <label class="text-xs text-gray-400">Drop Position</label>
                                    <div class="flex gap-1">
                                        <input type="number" value="20" placeholder="X" class="futuristic-input px-2 py-1 rounded text-white text-sm step-drop-x">
                                        <input type="number" value="10" placeholder="Y" class="futuristic-input px-2 py-1 rounded text-white text-sm step-drop-y">
                                        <input type="number" value="20" placeholder="Z" class="futuristic-input px-2 py-1 rounded text-white text-sm step-drop-z">
                                    </div>
                                </div>
                            </div>
                        </div>
                        <button type="button" class="btn-futuristic px-3 py-1 bg-red-500 rounded-lg ml-4" onclick="removeTaskStep(this)">
                            <i class="fas fa-trash"></i>
                        </button>
                    </div>
                `;
            }

            stepDiv.innerHTML = stepContent;
            document.getElementById('task-steps').appendChild(stepDiv);

            // Show notification for successful step addition
            showNotification(`${type.charAt(0).toUpperCase() + type.slice(1)} step added`, 'success');
        }

        function removeTaskStep(button) {
            // Find the parent task step element
            const stepElement = button.closest('[class*="bg-gray-700"]') || button.closest('.task-step-item');
            if (stepElement) {
                stepElement.remove();
            } else {
                // Fallback: remove parent element
                button.parentElement.parentElement.remove();
            }
        }

        async function executeTask(taskName) {
            if (!isConnected) {
                showNotification('Robot not connected', 'warning');
                return;
            }

            try {
                showNotification(`Executing task: ${taskName}`, 'info');
                const response = await fetch(`/api/tasks/${taskName}/execute`, { method: 'POST' });
                const data = await response.json();

                if (data.success) {
                    showNotification(`Task "${taskName}" completed successfully`, 'success');
                    showTaskSavedIndicator(`Task "${taskName}" execution logged to database`);
                    await loadTasksWithPersistence(); // Refresh to show updated execution count
                } else {
                    showNotification(`Task execution failed: ` + (data.error || ''), 'error');
                }
            } catch (error) {
                showNotification('Execution error: ' + error.message, 'error');
            }
        }

        // === SEQUENCE MANAGEMENT ===

        function openSequenceModal() {
            populateAvailableTasks();
            selectedTasks = [];
            updateSelectedTasksDisplay();
            document.getElementById('sequence-name').value = '';
            document.getElementById('loop-count').value = 1;
            document.getElementById('infinite-loop').checked = false;
            document.getElementById('sequence-modal').classList.remove('hidden');
        }

        function closeSequenceModal() {
            document.getElementById('sequence-modal').classList.add('hidden');
        }

        function populateAvailableTasks() {
            const container = document.getElementById('available-tasks');
            container.innerHTML = '';

            Object.values(window.allTasks || {}).forEach(task => {
                const taskDiv = document.createElement('div');
                taskDiv.className = 'bg-gray-600/50 rounded-lg p-3 mb-2 hover:bg-gray-500/50 transition-all';
                taskDiv.innerHTML = `
                    <div class="flex items-center justify-between">
                        <div>
                            <div class="font-semibold text-white">${task.name}</div>
                            <div class="text-sm text-gray-300">${task.steps.length} steps</div>
                        </div>
                        <button class="btn-futuristic px-3 py-2 bg-blue-500 rounded-lg text-sm" onclick="addTaskToSequence('${task.name}')">
                            <i class="fas fa-plus mr-1"></i>Add
                        </button>
                    </div>
                `;
                container.appendChild(taskDiv);
            });
        }

        function addTaskToSequence(taskName) {
            if (!selectedTasks.includes(taskName)) {
                selectedTasks.push(taskName);
                updateSelectedTasksDisplay();
            }
        }

        function removeTaskFromSequence(taskName) {
            const index = selectedTasks.indexOf(taskName);
            if (index > -1) {
                selectedTasks.splice(index, 1);
                updateSelectedTasksDisplay();
            }
        }

        function moveTaskUp(taskName) {
            const index = selectedTasks.indexOf(taskName);
            if (index > 0) {
                [selectedTasks[index], selectedTasks[index - 1]] = [selectedTasks[index - 1], selectedTasks[index]];
                updateSelectedTasksDisplay();
            }
        }

        function moveTaskDown(taskName) {
            const index = selectedTasks.indexOf(taskName);
            if (index < selectedTasks.length - 1) {
                [selectedTasks[index], selectedTasks[index + 1]] = [selectedTasks[index + 1], selectedTasks[index]];
                updateSelectedTasksDisplay();
            }
        }

        function updateSelectedTasksDisplay() {
            const container = document.getElementById('selected-tasks');

            if (selectedTasks.length === 0) {
                container.innerHTML = `
                    <div class="text-center text-gray-400 py-8">
                        Select tasks from above to add to sequence
                    </div>
                `;
                return;
            }

            container.innerHTML = '';
            selectedTasks.forEach((taskName, index) => {
                const taskDiv = document.createElement('div');
                taskDiv.className = 'bg-gray-600/50 rounded-lg p-3 mb-2 border-l-4 border-purple-500';
                taskDiv.innerHTML = `
                    <div class="flex items-center justify-between">
                        <div class="font-semibold text-white">${index + 1}. ${taskName}</div>
                        <div class="flex gap-2">
                            <button class="btn-futuristic px-2 py-1 bg-blue-500 rounded text-sm" onclick="moveTaskUp('${taskName}')" ${index === 0 ? 'disabled' : ''}>
                                <i class="fas fa-arrow-up"></i>
                            </button>
                            <button class="btn-futuristic px-2 py-1 bg-blue-500 rounded text-sm" onclick="moveTaskDown('${taskName}')" ${index === selectedTasks.length - 1 ? 'disabled' : ''}>
                                <i class="fas fa-arrow-down"></i>
                            </button>
                            <button class="btn-futuristic px-2 py-1 bg-red-500 rounded text-sm" onclick="removeTaskFromSequence('${taskName}')">
                                <i class="fas fa-trash"></i>
                            </button>
                        </div>
                    </div>
                `;
                container.appendChild(taskDiv);
            });
        }

        async function startSequence(sequenceName) {
            if (!isConnected) {
                showNotification('Robot not connected', 'warning');
                return;
            }

            try {
                showNotification(`Starting sequence: ${sequenceName}`, 'info');
                const response = await fetch(`/api/sequences/${sequenceName}/start`, { method: 'POST' });
                const data = await response.json();

                if (data.success) {
                    document.getElementById('current-sequence-name').textContent = sequenceName;
                    document.getElementById('sequence-progress').classList.remove('hidden');
                    showNotification(data.message, 'success');
                    showTaskSavedIndicator(`Sequence "${sequenceName}" started - progress will be logged`);
                } else {
                    showNotification('Failed to start sequence: ' + (data.error || ''), 'error');
                }
            } catch (error) {
                showNotification('Sequence start error: ' + error.message, 'error');
            }
        }

        async function stopSequence() {
            try {
                const response = await fetch('/api/sequences/stop', { method: 'POST' });
                const data = await response.json();

                if (data.success) {
                    document.getElementById('sequence-progress').classList.add('hidden');
                    showNotification(data.message, 'warning');
                    showTaskSavedIndicator('Sequence stopped - execution data saved to database');
                    await loadSequencesWithPersistence(); // Refresh to show updated execution count
                } else {
                    showNotification('Failed to stop sequence: ' + (data.error || ''), 'error');
                }
            } catch (error) {
                showNotification('Sequence stop error: ' + error.message, 'error');
            }
        }

        async function pauseSequence() {
            try {
                const response = await fetch('/api/sequences/pause', { method: 'POST' });
                const data = await response.json();

                if (data.success) {
                    document.getElementById('pause-btn').classList.add('hidden');
                    document.getElementById('resume-btn').classList.remove('hidden');
                    updateSequenceStatusBadge('paused');
                    showNotification(data.message, 'warning');
                } else {
                    showNotification('Failed to pause sequence: ' + (data.error || ''), 'error');
                }
            } catch (error) {
                showNotification('Sequence pause error: ' + error.message, 'error');
            }
        }

        async function resumeSequence() {
            try {
                const response = await fetch('/api/sequences/resume', { method: 'POST' });
                const data = await response.json();

                if (data.success) {
                    document.getElementById('pause-btn').classList.remove('hidden');
                    document.getElementById('resume-btn').classList.add('hidden');
                    updateSequenceStatusBadge('running');
                    showNotification(data.message, 'info');
                } else {
                    showNotification('Failed to resume sequence: ' + (data.error || ''), 'error');
                }
            } catch (error) {
                showNotification('Sequence resume error: ' + error.message, 'error');
            }
        }

        function updateSequenceProgress(progress) {
            if (!progress || !progress.is_running) {
                document.getElementById('sequence-progress').classList.add('hidden');
                return;
            }

            document.getElementById('current-task-name').textContent = progress.task_name || 'Unknown';
            document.getElementById('current-loop-display').textContent = `${progress.current_loop + 1}/${progress.total_loops}`;

            const taskProgress = progress.total_tasks > 0 ? (progress.current_task / progress.total_tasks) : 0;
            const loopProgress = progress.total_loops > 0 ? (progress.current_loop / progress.total_loops) : 0;
            const overallProgress = ((loopProgress + (taskProgress / progress.total_loops)) * 100);

            document.getElementById('sequence-progress-bar').style.width = overallProgress + '%';

            if (progress.is_paused) {
                updateSequenceStatusBadge('paused');
            } else {
                updateSequenceStatusBadge('running');
            }
        }

        function updateSequenceStatusBadge(status) {
            const badge = document.getElementById('sequence-status-badge');
            if (status === 'running') {
                badge.className = 'px-3 py-1 bg-green-500 rounded-full text-sm';
                badge.innerHTML = '<i class="fas fa-play mr-1"></i>Running';
            } else if (status === 'paused') {
                badge.className = 'px-3 py-1 bg-yellow-500 rounded-full text-sm';
                badge.innerHTML = '<i class="fas fa-pause mr-1"></i>Paused';
            }
        }

        // Socket.IO initialization
        function initializeSocketIO() {
            socket = io();

            socket.on('connect', function() {
                console.log('Connected to server');
                showNotification('Connected to server', 'success');
            });

            socket.on('disconnect', function() {
                console.log('Disconnected from server');
                showNotification('Disconnected from server', 'error');
                updateConnectionStatus(false);
            });

            socket.on('system_status', function(data) {
                updateSystemStatus(data);
            });

            socket.on('object_detected', function(data) {
                lastDetectedCoords = [data.x, data.y];
                updateDetectionInfo(data.x, data.y);

                document.getElementById('pickup-x').textContent = data.x.toFixed(1);
                document.getElementById('pickup-y').textContent = data.y.toFixed(1);
            });

            socket.on('status_update', function(data) {
                updateSystemStatus(data);
            });

            socket.on('pickplace_complete', function(data) {
                if (data.success) {
                    const duration = data.duration ? ` in ${data.duration.toFixed(2)}s` : '';
                    let mode = '';
                    if (data.auto_triggered) {
                        mode = data.auto_mode_type ? `${data.auto_mode_type} ` : 'Auto ';
                    }
                    showNotification(`${mode}Pick & Place completed${duration}`, 'pickplace');
                    showTaskSavedIndicator('Pick & place operation data saved to database');
                } else {
                    showNotification(`Pick & Place failed: ${data.error || 'Unknown error'}`, 'error');
                }
            });

            socket.on('sequence_progress', function(data) {
                updateSequenceProgress(data);
            });
        }

        // Chart initialization
        function initializeCharts() {
            const ctx = document.getElementById('movementChart').getContext('2d');
            movementChart = new Chart(ctx, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [{
                        label: 'X Position',
                        data: [],
                        borderColor: '#00f2fe',
                        backgroundColor: 'rgba(0, 242, 254, 0.1)',
                        tension: 0.4,
                        fill: true
                    }, {
                        label: 'Y Position',
                        data: [],
                        borderColor: '#4facfe',
                        backgroundColor: 'rgba(79, 172, 254, 0.1)',
                        tension: 0.4,
                        fill: true
                    }, {
                        label: 'Z Position',
                        data: [],
                        borderColor: '#43e97b',
                        backgroundColor: 'rgba(67, 233, 123, 0.1)',
                        tension: 0.4,
                        fill: true
                    }]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    plugins: {
                        legend: {
                            labels: {
                                color: '#ffffff'
                            }
                        }
                    },
                    scales: {
                        x: {
                            ticks: {
                                color: '#ffffff'
                            },
                            grid: {
                                color: 'rgba(255, 255, 255, 0.1)'
                            }
                        },
                        y: {
                            ticks: {
                                color: '#ffffff'
                            },
                            grid: {
                                color: 'rgba(255, 255, 255, 0.1)'
                            }
                        }
                    }
                }
            });
        }

        // Robot control functions
        async function connectRobot() {
            try {
                showNotification('Searching for serial ports...', 'info');

                const portsResponse = await fetch('/api/ports');
                const portsData = await portsResponse.json();

                if (portsData.success && portsData.ports.length > 0) {
                    console.log('Available serial ports:', portsData.ports);
                    showNotification(`Found ${portsData.ports.length} serial port(s). Attempting connection...`, 'info');
                } else {
                    showNotification('No serial ports found. Check Arduino connection.', 'warning');
                }

                const response = await fetch('/api/connect', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ port: '/dev/ttyACM0' })
                });

                const data = await response.json();
                if (data.success) {
                    showNotification('Robot connected successfully', 'success');
                    updateConnectionStatus(true);
                } else {
                    showNotification('Failed to connect to robot. Check Arduino connection.', 'error');
                }
            } catch (error) {
                showNotification('Connection error: ' + error.message, 'error');
            }
        }

        async function homeRobot() {
            if (!isConnected) {
                showNotification('Robot not connected', 'warning');
                return;
            }

            try {
                showNotification('Starting homing sequence...', 'info');

                const response = await fetch('/api/home', { method: 'POST' });
                const data = await response.json();

                if (data.success) {
                    const timeStr = data.time ? ` in ${data.time.toFixed(2)}s` : '';
                    showNotification(`Robot homed successfully${timeStr}`, 'success');
                    resetSliders();
                    setTimeout(() => {
                        showNotification('Robot ready for movement commands', 'info');
                    }, 1000);
                } else {
                    const errorMsg = data.error || 'Homing failed';
                    showNotification(`Homing failed: ${errorMsg}`, 'error');
                }
            } catch (error) {
                showNotification('Homing communication error: ' + error.message, 'error');
            }
        }

        async function moveJoint() {
            if (!isConnected) {
                showNotification('Robot not connected', 'warning');
                return;
            }

            const shoulder = parseFloat(document.getElementById('shoulder-slider').value);
            const elbow = parseFloat(document.getElementById('elbow-slider').value);
            const wrist = parseFloat(document.getElementById('wrist-slider').value);
            const z = parseFloat(document.getElementById('z-slider').value);

            if (shoulder < 0 || shoulder > 180) {
                showNotification('Shoulder angle must be between 0° and 180°', 'error');
                return;
            }
            if (elbow < -90 || elbow > 90) {
                showNotification('Elbow angle must be between -90° and 90°', 'error');
                return;
            }
            if (wrist < -90 || wrist > 90) {
                showNotification('Wrist angle must be between -90° and 90°', 'error');
                return;
            }
            if (z < 0 || z > 300) {
                showNotification('Z height must be between 0mm and 300mm', 'error');
                return;
            }

            try {
                showNotification('Sending joint movement command...', 'info');

                const response = await fetch('/api/move/joint', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ shoulder, elbow, wrist, z })
                });

                const data = await response.json();

                if (data.success) {
                    const timeStr = data.time ? ` in ${data.time.toFixed(2)}s` : '';
                    showNotification(`Joint movement completed${timeStr}`, 'success');
                } else {
                    const errorMsg = data.error || 'Unknown error';

                    if (data.needs_homing || errorMsg.includes('homed')) {
                        showNotification('Robot needs homing first!', 'warning');
                        setTimeout(() => {
                            showNotification('Click the "Home" button to initialize the robot', 'info');
                        }, 1500);
                    } else {
                        showNotification(`Joint movement failed: ${errorMsg}`, 'error');
                    }
                }
            } catch (error) {
                showNotification('Movement error: ' + error.message, 'error');
            }
        }

        async function moveCartesian() {
            if (!isConnected) {
                showNotification('Robot not connected', 'warning');
                return;
            }

            const x = parseFloat(document.getElementById('x-input').value);
            const y = parseFloat(document.getElementById('y-input').value);
            const z = parseFloat(document.getElementById('z-input').value);

            try {
                showNotification(`Moving to Cartesian position (${x}, ${y}, ${z}) using IK...`, 'info');

                const response = await fetch('/api/move/cartesian', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ x, y, z })
                });

                const data = await response.json();
                if (data.success) {
                    showNotification(`Cartesian movement completed`, 'success');
                } else {
                    showNotification('Cartesian movement failed: ' + (data.error || ''), 'error');
                }
            } catch (error) {
                showNotification('Movement error: ' + error.message, 'error');
            }
        }

        async function testInverseKinematics() {
            const x = parseFloat(document.getElementById('x-input').value);
            const y = parseFloat(document.getElementById('y-input').value);
            const z = parseFloat(document.getElementById('z-input').value);

            try {
                showNotification('Testing inverse kinematics...', 'info');

                const response = await fetch('/api/kinematics/inverse', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ x, y, z })
                });

                const data = await response.json();
                if (data.success) {
                    const solution = data.solution;
                    const error = data.error;

                    showNotification(
                        `IK Solution: θ1=${solution.theta1}°, θ2=${solution.theta2}°, φ=${solution.phi}°, Error=${error.toFixed(2)}mm`,
                        'success'
                    );

                    document.getElementById('shoulder-slider').value = solution.theta1;
                    document.getElementById('elbow-slider').value = solution.theta2;
                    document.getElementById('wrist-slider').value = solution.phi;
                    updateJointDisplay('shoulder', solution.theta1);
                    updateJointDisplay('elbow', solution.theta2);
                    updateJointDisplay('wrist', solution.phi);

                } else {
                    showNotification('IK Test: ' + (data.error || 'Position unreachable'), 'warning');
                }
            } catch (error) {
                showNotification('IK test error: ' + error.message, 'error');
            }
        }

        async function testForwardKinematics() {
            const theta1 = parseFloat(document.getElementById('shoulder-slider').value);
            const theta2 = parseFloat(document.getElementById('elbow-slider').value);
            const phi = parseFloat(document.getElementById('wrist-slider').value);

            try {
                showNotification('Testing forward kinematics...', 'info');

                const response = await fetch('/api/kinematics/forward', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ theta1, theta2, phi })
                });

                const data = await response.json();
                if (data.success) {
                    const position = data.position;

                    showNotification(
                        `FK Result: X=${position.x.toFixed(1)}mm, Y=${position.y.toFixed(1)}mm`,
                        'success'
                    );

                    document.getElementById('x-input').value = position.x.toFixed(1);
                    document.getElementById('y-input').value = position.y.toFixed(1);

                } else {
                    showNotification('FK Test failed: ' + (data.error || 'Unknown error'), 'error');
                }
            } catch (error) {
                showNotification('FK test error: ' + error.message, 'error');
            }
        }

        async function controlGripper(position) {
            if (!isConnected) {
                showNotification('Robot not connected', 'warning');
                return;
            }

            try {
                const response = await fetch('/api/gripper', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ position })
                });

                const data = await response.json();
                if (data.success) {
                    document.getElementById('gripper-slider').value = position;
                    updateGripperDisplay(position);
                    showNotification(`Gripper ${position === 0 ? 'opened' : 'closed'}`, 'success');
                } else {
                    showNotification('Gripper control failed: ' + (data.error || ''), 'error');
                }
            } catch (error) {
                showNotification('Gripper error: ' + error.message, 'error');
            }
        }

        async function setGripperPosition() {
            const position = parseInt(document.getElementById('gripper-slider').value);
            await controlGripper(position);
        }

        async function emergencyStop() {
            if (confirm('Are you sure you want to activate emergency stop?')) {
                try {
                    const response = await fetch('/api/emergency', { method: 'POST' });
                    const data = await response.json();

                    if (data.success) {
                        showNotification('EMERGENCY STOP ACTIVATED', 'error');
                    }
                } catch (error) {
                    showNotification('Emergency stop error: ' + error.message, 'error');
                }
            }
        }

        // Enhanced logging for debugging
        function debugLog(message, data = null) {
            console.log(`[SCARA DEBUG] ${message}`, data);
        }

        async function toggleMode() {
            try {
                debugLog('Toggle mode started');
                showNotification('Switching mode...', 'info');

                const requestBody = {
                    timestamp: new Date().toISOString()
                };

                debugLog('Sending toggle mode request', requestBody);

                const response = await fetch('/api/mode/toggle', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                        'Accept': 'application/json'
                    },
                    body: JSON.stringify(requestBody)
                });

                debugLog('Toggle mode response status:', response.status);

                if (!response.ok) {
                    const errorText = await response.text();
                    debugLog('Toggle mode error response:', errorText);
                    throw new Error(`HTTP ${response.status}: ${response.statusText}. Response: ${errorText}`);
                }

                const data = await response.json();
                debugLog('Toggle mode success response:', data);

                if (data.success) {
                    currentMode = data.mode;
                    updateModeDisplay();
                    showNotification(data.message, 'success');
                    updatePickPlaceStatus();
                    debugLog('Mode successfully changed to:', currentMode);
                } else {
                    debugLog('Toggle mode failed:', data.error);
                    showNotification('Failed to toggle mode: ' + (data.error || 'Unknown error'), 'error');
                }
            } catch (error) {
                debugLog('Toggle mode exception:', error);
                console.error('Mode toggle error:', error);
                showNotification('Mode toggle error: ' + error.message, 'error');

                // Try to provide more helpful error information
                if (error.message.includes('400')) {
                    showNotification('Server rejected the request. Check server logs for details.', 'warning');
                } else if (error.message.includes('500')) {
                    showNotification('Server internal error. Check server status.', 'error');
                } else if (error.message.includes('NetworkError') || error.message.includes('fetch')) {
                    showNotification('Network connection error. Check server connection.', 'error');
                }
            }
        }

        // Add server health check function (internal use)
        async function checkServerHealth() {
            try {
                const response = await fetch('/api/status', {
                    method: 'GET',
                    headers: { 'Accept': 'application/json' }
                });

                if (response.ok) {
                    debugLog('Server health check passed');
                    return true;
                } else {
                    debugLog('Server health check failed:', response.status);
                    return false;
                }
            } catch (error) {
                debugLog('Server health check error:', error);
                return false;
            }
        }

        // Enhanced test connection function
        async function testConnection() {
            try {
                showNotification('Testing server connection...', 'info');

                // Test basic server connection
                const healthCheck = await checkServerHealth();
                if (!healthCheck) {
                    showNotification('Server connection failed', 'error');
                    return;
                }

                // Test robot connection if server is healthy
                if (!isConnected) {
                    showNotification('Server OK, but robot not connected', 'warning');
                    return;
                }

                // Test robot communication
                const pingResponse = await fetch('/api/status');
                const statusData = await pingResponse.json();

                if (statusData.error) {
                    showNotification('Robot communication failed: ' + statusData.error, 'error');
                } else {
                    showNotification('All systems OK: Server ✓ Robot ✓ Database ✓', 'success');
                    debugLog('System status check passed:', statusData);
                }
            } catch (error) {
                debugLog('Connection test error:', error);
                showNotification('Connection test failed: ' + error.message, 'error');
            }
        }

        // Display update functions
        function updateJointDisplay(joint, value) {
            const unit = joint === 'z' ? 'mm' : '°';
            document.getElementById(`${joint}-value`).textContent = `${value}${unit}`;
        }

        function updateGripperDisplay(value) {
            const status = value == 0 ? 'Open' : value == 90 ? 'Closed' : 'Partial';
            document.getElementById('gripper-value').textContent = `${value}° (${status})`;
        }

        function updateSystemStatus(data) {
            if (data.is_connected !== undefined) {
                isConnected = data.is_connected;
                updateConnectionStatus(data.is_connected);
            }

            if (data.pick_and_place_enabled !== undefined) {
                pickPlaceEnabled = data.pick_and_place_enabled;
                updatePickPlaceStatus();
            }

            if (data.current_position) {
                document.getElementById('pos-shoulder').textContent = data.current_position[0].toFixed(1) + '°';
                document.getElementById('pos-elbow').textContent = data.current_position[1].toFixed(1) + '°';
                document.getElementById('pos-wrist').textContent = data.current_position[2].toFixed(1) + '°';
                document.getElementById('pos-z').textContent = data.current_position[3].toFixed(1) + 'mm';
            }

            if (data.emergency_stop !== undefined) {
                updateEmergencyStatus(data.emergency_stop);
            }
        }

        function updateConnectionStatus(connected) {
            const statusElement = document.getElementById('connection-status');
            const robotStatus = document.getElementById('robot-status');

            if (connected) {
                statusElement.className = 'px-4 py-2 rounded-full text-sm font-semibold bg-green-500';
                statusElement.innerHTML = '<i class="fas fa-wifi mr-2"></i>Connected';
                robotStatus.className = 'px-4 py-2 rounded-full text-sm font-semibold bg-green-500';
                robotStatus.innerHTML = '<i class="fas fa-robot mr-2"></i>Ready';
            } else {
                statusElement.className = 'px-4 py-2 rounded-full text-sm font-semibold bg-red-500 status-pulse';
                statusElement.innerHTML = '<i class="fas fa-wifi mr-2"></i>Disconnected';
                robotStatus.className = 'px-4 py-2 rounded-full text-sm font-semibold bg-gray-500';
                robotStatus.innerHTML = '<i class="fas fa-robot mr-2"></i>Not Ready';
            }
        }

        function updateEmergencyStatus(emergency) {
            const statusElement = document.getElementById('robot-status');
            if (emergency) {
                statusElement.className = 'px-4 py-2 rounded-full text-sm font-semibold bg-red-500 status-pulse';
                statusElement.innerHTML = '<i class="fas fa-exclamation-triangle mr-2"></i>EMERGENCY';
            }
        }

        function updateModeDisplay() {
            const modeElement = document.getElementById('mode-status');
            const icon = currentMode === 'manual' ? 'fa-hand-paper' : 'fa-robot';
            const modeText = currentMode.charAt(0).toUpperCase() + currentMode.slice(1);

            if (currentMode === 'auto') {
                modeElement.className = 'px-4 py-2 rounded-full text-sm font-semibold bg-purple-500';
            } else {
                modeElement.className = 'px-4 py-2 rounded-full text-sm font-semibold bg-blue-500';
            }

            modeElement.innerHTML = `<i class="fas ${icon} mr-2"></i>${modeText}`;
        }

        function updateDetectionInfo(x, y) {
            const detectionElement = document.getElementById('detection-info');
            detectionElement.textContent = `Object detected at (${x.toFixed(1)}, ${y.toFixed(1)})`;
        }

        function resetSliders() {
            document.getElementById('shoulder-slider').value = 90;
            document.getElementById('elbow-slider').value = 0;
            document.getElementById('wrist-slider').value = 0;
            document.getElementById('z-slider').value = 50;
            updateJointDisplay('shoulder', 90);
            updateJointDisplay('elbow', 0);
            updateJointDisplay('wrist', 0);
            updateJointDisplay('z', 50);
        }

        // Chart functions
        async function refreshCharts() {
            try {
                const response = await fetch('/api/analytics/movements');
                const data = await response.json();

                if (Array.isArray(data) && data.length > 0) {
                    const labels = data.map(d => new Date(d.timestamp).toLocaleTimeString());
                    const xData = data.map(d => d.x);
                    const yData = data.map(d => d.y);
                    const zData = data.map(d => d.z);

                    movementChart.data.labels = labels;
                    movementChart.data.datasets[0].data = xData;
                    movementChart.data.datasets[1].data = yData;
                    movementChart.data.datasets[2].data = zData;
                    movementChart.update();

                    showNotification('Charts updated', 'success');
                }
            } catch (error) {
                showNotification('Failed to refresh charts', 'error');
            }
        }

        // Enhanced Notification System
        function showNotification(message, type) {
            const container = document.getElementById('notification-container');
            const notification = document.createElement('div');

            const colors = {
                success: 'border-green-400 bg-green-500/20 text-green-100',
                error: 'border-red-400 bg-red-500/20 text-red-100',
                warning: 'border-yellow-400 bg-yellow-500/20 text-yellow-100',
                info: 'border-blue-400 bg-blue-500/20 text-blue-100',
                pickplace: 'border-purple-400 bg-purple-500/20 text-purple-100'
            };

            const icons = {
                success: 'fa-check-circle',
                error: 'fa-exclamation-circle',
                warning: 'fa-exclamation-triangle',
                info: 'fa-info-circle',
                pickplace: 'fa-magic'
            };

            notification.className = `notification-modern p-4 rounded-xl ${colors[type] || colors.info} max-w-sm`;
            notification.innerHTML = `
                <div class="flex items-start gap-3">
                    <i class="fas ${icons[type] || icons.info} text-lg mt-0.5"></i>
                    <div class="flex-1">
                        <div class="font-medium">${message}</div>
                    </div>
                    <button onclick="this.parentElement.parentElement.remove()" class="text-lg hover:opacity-70">
                        <i class="fas fa-times"></i>
                    </button>
                </div>
            `;

            container.appendChild(notification);

            // Auto remove after 5 seconds
            setTimeout(() => {
                if (notification.parentElement) {
                    notification.style.transform = 'translateX(400px)';
                    setTimeout(() => notification.remove(), 300);
                }
            }, 5000);
        }

        function updateDisplays() {
            if (socket && socket.connected) {
                socket.emit('request_status');
            }
        }

        // Enhanced keyboard shortcuts
        document.addEventListener('keydown', function(event) {
            if (event.ctrlKey) {
                switch(event.key) {
                    case 'h':
                        event.preventDefault();
                        homeRobot();
                        break;
                    case 'e':
                        event.preventDefault();
                        emergencyStop();
                        break;
                    case 'm':
                        event.preventDefault();
                        toggleMode();
                        break;
                    case 'p':
                        event.preventDefault();
                        togglePickAndPlace();
                        break;
                    case 's':
                        event.preventDefault();
                        openSequenceModal();
                        break;
                    case 't':
                        event.preventDefault();
                        openTaskModal();
                        break;
                    case 'r':
                        event.preventDefault();
                        checkReachability();
                        break;
                    case 'd':
                        event.preventDefault();
                        toggleTheme();
                        break;
                }
            } else if (event.key === 'Escape') {
                // Close any open modals
                document.querySelectorAll('.modal').forEach(modal => {
                    modal.classList.add('hidden');
                });
            } else if (event.key === 'F1') {
                event.preventDefault();
                showKeyboardShortcuts();
            }
        });

        function showKeyboardShortcuts() {
            const shortcuts = `
🎛️ SCARA PRO v2.2 - Keyboard Shortcuts

📋 Basic Controls:
• Ctrl+H - Home Robot
• Ctrl+E - Emergency Stop
• Ctrl+M - Toggle Mode
• Ctrl+P - Toggle Pick & Place
• Ctrl+R - Check Reachability
• Ctrl+D - Toggle Dark/Light Theme

🎯 Programming:
• Ctrl+T - New Task (Auto-Saved)
• Ctrl+S - New Sequence (Auto-Saved)
• Escape - Close Modals

🖱️ Mouse Controls:
• Click 3D View - Set target position
• Shift+Click - Move to position
• Ctrl+Drag - Rotate camera
• Scroll - Zoom in/out

🗄️ Enhanced Features:
• All tasks auto-save to database
• Tasks persist across sessions
• Real-time execution logging

💡 Press F1 to show this help again
            `;

            showNotification(shortcuts, 'info');
            console.log('%cSCARA PRO v2.2 - Enhanced Task Persistence - Keyboard Shortcuts', 'font-size: 16px; font-weight: bold; color: #667eea;');
            console.log(shortcuts);
        }
    </script>
</body>
</html>'''

    def run(self, host='0.0.0.0', port=5000, debug=False):
        """Run the Flask application"""
        try:
            self.logger.info(f"Starting Enhanced SCARA Robot Control System v2.2 on {host}:{port}")
            self.logger.info("Key improvements:")
            self.logger.info("- Enhanced Task Persistence with SQLite Database")
            self.logger.info("- Auto-save functionality for tasks and sequences")
            self.logger.info("- Tasks and sequences persist across sessions")
            self.logger.info("- Real-time execution tracking and logging")
            self.logger.info("- Professional modern UI with task status indicators")

            # Create logs directory
            Path('logs').mkdir(exist_ok=True)
            self.socketio.run(self.app, host=host, port=port, debug=debug, allow_unsafe_werkzeug=True)
        except KeyboardInterrupt:
            self.logger.info("Shutting down...")
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup resources"""
        self.vision.release_camera()
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.robot.disconnect())
        loop.close()
        self.logger.info("Cleanup completed")


# ===== Main Application Entry Point =====

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Enhanced SCARA Robot Control System v2.2 with Task Persistence')
    parser.add_argument('--host', default='0.0.0.0', help='Host address')
    parser.add_argument('--port', type=int, default=5000, help='Port number')
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    parser.add_argument('--robot-port', default='/dev/ttyUSB0', help='Robot serial port')

    args = parser.parse_args()

    # Ensure logs directory exists
    Path('logs').mkdir(exist_ok=True)

    app = SCARAWebApp()
    app.robot.port = args.robot_port
    app.run(host=args.host, port=args.port, debug=args.debug)