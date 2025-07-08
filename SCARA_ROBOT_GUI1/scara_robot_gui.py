import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                            QPushButton, QWidget, QLabel, QSlider, QLineEdit,
                            QGroupBox, QStatusBar, QMessageBox, QFrame, QGridLayout)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QIntValidator, QIcon
import serial
import math

class ScaraRobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.L0= 150.8
        self.L1 = 190  # mm
        self.L2 = 168  # mm
        self.theta1 = 0
        self.theta2 = 0
        self.phi = 0
        self.z = 100
        self.x = 0  # mm
        self.y = 0
        self.gripper_value = 90
        self.speed = 500
        self.acceleration = 500
        self.save_status = 0
        self.run_status = 0
        self.positions = []
        self.serial_port = None
        
        self.init_ui()
        self.init_serial()
        
    def init_ui(self):
        self.setWindowTitle('SCARA Robot Control')
        self.setGeometry(100, 100, 1000, 850)
        
        # Apply modern stylesheet
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2c3e50;
                font-family: 'Segoe UI', Arial, sans-serif;
            }
            
            QGroupBox {
                background-color: #34495e;
                border: 2px solid #3498db;
                border-radius: 8px;
                margin-top: 20px;
                padding: 15px 10px 15px 10px;
                font-size: 16px;
                font-weight: bold;
                color: #ecf0f1;
            }
            
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                color: #3498db;
            }
            
            QLabel {
                color: #ecf0f1;
                font-size: 14px;
            }
            
            QPushButton {
                background-color: #3498db;
                color: white;
                border: none;
                border-radius: 5px;
                padding: 8px 15px;
                font-size: 14px;
                min-width: 100px;
            }
            
            QPushButton:hover {
                background-color: #2980b9;
            }
            
            QPushButton:pressed {
                background-color: #1c6ea4;
            }
            
            QPushButton#runButton {
                background-color: #27ae60;
            }
            
            QPushButton#runButton:hover {
                background-color: #219653;
            }
            
            QPushButton#runButton:checked {
                background-color: #c0392b;
            }
            
            QSlider::groove:horizontal {
                height: 8px;
                background: #95a5a6;
                border-radius: 4px;
            }
            
            QSlider::handle:horizontal {
                width: 20px;
                height: 20px;
                margin: -6px 0;
                background: #3498db;
                border-radius: 10px;
            }
            
            QSlider::sub-page:horizontal {
                background: #1abc9c;
                border-radius: 4px;
            }
            
            QLineEdit {
                background-color: #ecf0f1;
                border: 1px solid #3498db;
                border-radius: 4px;
                padding: 6px;
                font-size: 14px;
                color: #2c3e50;
            }
            
            QStatusBar {
                background-color: #3498db;
                color: white;
                font-size: 12px;
            }
            
            QFrame#divider {
                background-color: #3498db;
                border: none;
            }
            
            .title {
                font-size: 32px;
                font-weight: bold;
                color: #1abc9c;
                padding: 10px 0;
            }
            
            .valueDisplay {
                font-size: 20px;
                font-weight: bold;
                color: #1abc9c;
                min-width: 50px;
                text-align: center;
            }
            
            .positionDisplay {
                font-size: 20px;
                color: #f1c40f;
                font-weight: bold;
                background-color: rgba(0, 0, 0, 0.2);
                padding: 8px;
                border-radius: 5px;
            }
            
            .jogButton {
                min-width: 60px;
                font-weight: bold;
                font-size: 16px;
            }
        """)
        
        # Main Widget
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(20, 15, 20, 15)
        main_layout.setSpacing(15)
        main_widget.setLayout(main_layout)
        
        # Title
        title = QLabel("SCARA ROBOT CONTROL SYSTEM")
        title.setStyleSheet("font-size: 32px; font-weight: bold; color: #1abc9c; padding: 10px 0;")
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)

        # Main Content
        content_layout = QHBoxLayout()
        content_layout.setSpacing(20)
        
        # Left Column - Joint Controls
        left_column = QVBoxLayout()
        left_column.setSpacing(12)  # Reduced spacing between widgets
        left_column.setContentsMargins(5, 5, 5, 5)  # Tighter margins

        # Joint Controls Group
        joints_group = QGroupBox("JOINT CONTROLS")
        joints_group.setMinimumHeight(750)
        joints_group.setStyleSheet("""
            QGroupBox {
                font-size: 14px;
                font-weight: bold;
                border: 2px solid #3498db;
                border-radius: 5px;
                margin-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)

        joints_layout = QVBoxLayout()
        joints_layout.setSpacing(10)  # Reduced spacing between joint controls
        joints_layout.setContentsMargins(10, 15, 10, 10)  # Balanced internal padding

        # Create joint controls with consistent styling
        j1_group = self.create_joint_control("J1", -90, 266, 0, "Base Rotation")
        j2_group = self.create_joint_control("J2", -150, 150, 0, "Arm Rotation") 
        j3_group = self.create_joint_control("J3", -162, 162, 0, "Wrist Rotation")
        z_group = self.create_joint_control("Z", 0, 150, 100, "Vertical Movement")

        # Add joint controls with stretch for better spacing
        joints_layout.addWidget(j1_group)
        joints_layout.addWidget(j2_group)
        joints_layout.addWidget(j3_group)
        joints_layout.addWidget(z_group)
        joints_layout.addStretch()  # Pushes controls upward

        joints_group.setLayout(joints_layout)
        left_column.addWidget(joints_group)
        # Speed and Acceleration
        sa_group = QGroupBox("MOTION PARAMETERS")
        sa_layout = QVBoxLayout()
        sa_layout.setSpacing(12)
        
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("Speed:"))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(500, 4000)
        self.speed_slider.setValue(500)
        speed_layout.addWidget(self.speed_slider)
        self.speed_label = QLabel("500")
        self.speed_label.setStyleSheet("font-size: 20px; font-weight: bold; color: #1abc9c; min-width: 50px; text-align: center;")
        speed_layout.addWidget(self.speed_label)
        
        accel_layout = QHBoxLayout()
        accel_layout.addWidget(QLabel("Acceleration:"))
        self.accel_slider = QSlider(Qt.Horizontal)
        self.accel_slider.setRange(500, 4000)
        self.accel_slider.setValue(500)
        accel_layout.addWidget(self.accel_slider)
        self.accel_label = QLabel("500")
        self.accel_label.setStyleSheet("font-size: 20px; font-weight: bold; color: #1abc9c; min-width: 50px; text-align: center;")
        accel_layout.addWidget(self.accel_label)
        
        update_btn = QPushButton("Apply Parameters")
        update_btn.clicked.connect(self.update_sa)
        
        sa_layout.addLayout(speed_layout)
        sa_layout.addLayout(accel_layout)
        sa_layout.addWidget(update_btn, alignment=Qt.AlignCenter)
        sa_group.setLayout(sa_layout)
        left_column.addWidget(sa_group)
        
        content_layout.addLayout(left_column)
        
        # Vertical Divider
        divider = QFrame()
        divider.setFrameShape(QFrame.VLine)
        divider.setFrameShadow(QFrame.Sunken)
        divider.setLineWidth(2)
        content_layout.addWidget(divider)
        
        # Right Column - Position Controls
        right_column = QVBoxLayout()
        right_column.setSpacing(15)
        
        # Position Control Group
        pos_group = QGroupBox("POSITION CONTROL")
        pos_layout = QVBoxLayout()
        pos_layout.setSpacing(15)
        
        # Coordinate Input
        coord_layout = QGridLayout()
        coord_layout.setHorizontalSpacing(10)
        coord_layout.setVerticalSpacing(8)
        
        coord_layout.addWidget(QLabel("X Coordinate:"), 0, 0)
        self.x_input = QLineEdit(str(self.x))
        self.x_input.setFixedWidth(100)
        coord_layout.addWidget(self.x_input, 0, 1)
        coord_layout.addWidget(QLabel("mm"), 0, 2)
        
        coord_layout.addWidget(QLabel("Y Coordinate:"), 1, 0)
        self.y_input = QLineEdit(str(self.y))
        self.y_input.setFixedWidth(100)
        coord_layout.addWidget(self.y_input, 1, 1)
        coord_layout.addWidget(QLabel("mm"), 1, 2)
        
        coord_layout.addWidget(QLabel("Z Height:"), 2, 0)
        self.z_input = QLineEdit(str(self.z))
        self.z_input.setFixedWidth(100)
        coord_layout.addWidget(self.z_input, 2, 1)
        coord_layout.addWidget(QLabel("mm"), 2, 2)
        
        move_btn = QPushButton("Move to Position")
        move_btn.clicked.connect(self.move_to_position)
        coord_layout.addWidget(move_btn, 3, 0, 1, 3)
        
        pos_layout.addLayout(coord_layout)
        
        # Current Position Display 
        pos_display = QLabel(f"Current Position: X={self.x}  Y={self.y}  Z={self.z}")
        pos_display.setStyleSheet("""
            QLabel {
                font-size: 18px;
                color: #f1c40f;
                font-weight: bold;
                background-color: rgba(0, 0, 0, 0.2);
                padding: 6px;
                border-radius: 5px;
                min-width: 300px;
            }
        """)
        pos_display.setAlignment(Qt.AlignCenter)
        pos_layout.addWidget(pos_display)
        self.pos_display = pos_display

        pos_group.setLayout(pos_layout)
        right_column.addWidget(pos_group)
        
        # Gripper Control
        grip_group = QGroupBox("GRIPPER CONTROL")
        grip_group.setMaximumHeight(150)
        grip_layout = QVBoxLayout()
        grip_layout.setSpacing(5)
        grip_layout.setContentsMargins(8, 8, 8, 8)

        # تسمية Gripper Position
        grip_label = QLabel("Gripper Position:")
        grip_label.setStyleSheet("QLabel { font-size: 14px; }")  # تصحيح صيغة الـ stylesheet
        grip_layout.addWidget(grip_label)

        # شريط التمرير
        slider_layout = QHBoxLayout()
        slider_layout.setSpacing(5)

        closed_label = QLabel("Closed")
        closed_label.setStyleSheet("QLabel { font-size: 12px; }")  # تصحيح صيغة الـ stylesheet
        slider_layout.addWidget(closed_label)

        self.grip_slider = QSlider(Qt.Horizontal)
        self.grip_slider.setRange(0, 90)
        self.grip_slider.setValue(90)
        self.grip_slider.setFixedHeight(20)
        self.grip_slider.valueChanged.connect(self.update_gripper)
        slider_layout.addWidget(self.grip_slider)

        open_label = QLabel("Open")
        open_label.setStyleSheet("QLabel { font-size: 12px; }")  # تصحيح صيغة الـ stylesheet
        slider_layout.addWidget(open_label)

        grip_layout.addLayout(slider_layout)

        # قيمة النسبة المئوية
        grip_value_layout = QHBoxLayout()
        self.grip_value_label = QLabel("90%")
        self.grip_value_label.setStyleSheet("""
            QLabel {
                font-size: 16px;
                font-weight: bold; 
                color: #1abc9c; 
                min-width: 40px;
                text-align: center;
            }
        """)  # تصحيح صيغة الـ stylesheet
        grip_value_layout.addStretch()
        grip_value_layout.addWidget(self.grip_value_label)
        grip_value_layout.addStretch()
        grip_layout.addLayout(grip_value_layout)

        grip_group.setLayout(grip_layout)
        right_column.addWidget(grip_group)
        
        # Program Control
        prog_group = QGroupBox("PROGRAMMING")
        prog_layout = QVBoxLayout()
        prog_layout.setSpacing(15)
        
        save_btn = QPushButton("Save Current Position")
        save_btn.clicked.connect(self.save_position)
        
        self.run_btn = QPushButton("Run Program")
        self.run_btn.setCheckable(True)
        self.run_btn.clicked.connect(self.toggle_run)
        
        prog_layout.addWidget(save_btn)
        prog_layout.addWidget(self.run_btn)
        
        # Last Saved Position
        last_pos = QLabel("Last saved position: None")
        last_pos.setWordWrap(True)
        last_pos.setStyleSheet("font-size: 13px; color: #bdc3c7;")
        prog_layout.addWidget(last_pos)
        self.last_pos_label = last_pos
        
        prog_group.setLayout(prog_layout)
        right_column.addWidget(prog_group)
        
        content_layout.addLayout(right_column)
        main_layout.addLayout(content_layout)
        
        # Status Bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("System Ready - Connected to COM15")
        
        # Timer for periodic updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)
        
        # Connect slider signals
        self.speed_slider.valueChanged.connect(lambda val: self.speed_label.setText(str(val)))
        self.accel_slider.valueChanged.connect(lambda val: self.accel_label.setText(str(val)))
        self.grip_slider.valueChanged.connect(lambda val: self.grip_value_label.setText(f"{val}%"))

    def create_joint_control(self, name, min_val, max_val, init_val, description):
        
        group = QGroupBox(description)
        layout = QVBoxLayout()
        layout.setSpacing(8)
        
        # Value display
        value_layout = QHBoxLayout()
        value_layout.addStretch()
        value_label = QLabel(str(init_val))
        value_label.setStyleSheet("font-size: 20px; font-weight: bold; color: #1abc9c; min-width: 50px; text-align: center;")
        value_layout.addWidget(value_label)
        value_layout.addStretch()
        layout.addLayout(value_layout)
        
        # Slider
        slider = QSlider(Qt.Horizontal)
        slider.setRange(min_val, max_val)
        slider.setValue(init_val)
        setattr(self, f"{name.lower()}_slider", slider)
        layout.addWidget(slider)
        
        # Jog Controls
        jog_layout = QHBoxLayout()
        
        jog_minus = QPushButton("◀")
        jog_minus.setStyleSheet("min-width: 60px; font-weight: bold; font-size: 16px;")
        jog_minus.setFixedWidth(50)
        
        jog_value = QLineEdit("1")
        jog_value.setFixedWidth(50)
        jog_value.setAlignment(Qt.AlignCenter)
        jog_value.setValidator(QIntValidator(1, 20))
        
        jog_plus = QPushButton("▶")
        jog_plus.setStyleSheet("min-width: 60px; font-weight: bold; font-size: 16px;")
        jog_plus.setFixedWidth(50)
        
        jog_minus.clicked.connect(lambda: self.jog(name, -int(float(jog_value.text() or 1))))
        jog_plus.clicked.connect(lambda: self.jog(name, int(float(jog_value.text() or 1))))
        
        jog_layout.addWidget(jog_minus)
        jog_layout.addWidget(jog_value)
        jog_layout.addWidget(jog_plus)
        layout.addLayout(jog_layout)
        
        group.setLayout(layout)
        
        # Connect slider changes
        slider.valueChanged.connect(lambda val: self.update_joint_value(name.lower(), val, value_label))
        slider.valueChanged.connect(lambda val: self.joint_changed(name.lower(), val))
        
        return group

    def jog(self, joint, value):
        slider = getattr(self, f"{joint.lower()}_slider")
        current = slider.value()
        slider.setValue(int(round(current + value)))
        self.update_joint_value(joint.lower(), slider.value())

    def update_joint_value(self, joint, value, label=None):
        if label:
            label.setText(str(value))
        
        if joint == 'j1':
            self.theta1 = value
        elif joint == 'j2':
            self.theta2 = value
        elif joint == 'j3':
            self.phi = value
        elif joint == 'z':
            self.z = value
    
    def init_serial(self):
        try:
            print("Attempting to connect to COM15...")
            self.serial_port = serial.Serial('COM15', 9600, timeout=1)
            self.status_bar.showMessage("Connected to COM15")
        except serial.SerialException as e:
            self.status_bar.showMessage(f"Serial error: {str(e)}")
          #  QMessageBox.warning(self, "Serial Error", f"Could not open serial port: {str(e)}")
            msg = QMessageBox(self)
            msg.setIcon(QMessageBox.Warning)
            msg.setWindowTitle("Error")
            msg.setText( "Serial Error : Could not open serial port")
            msg.setStyleSheet("QLabel { color: black; }")  # هنا بنخلي الخط أسود
            msg.exec_()

    
    def joint_changed(self, joint, value):
        # Update the internal variables
        if joint == 'j1':
            self.theta1 = value
        elif joint == 'j2':
            self.theta2 = value
        elif joint == 'j3':
            self.phi = value
        elif joint == 'z':
            self.z = value
        
        self.forward_kinematics()
        self.send_data()

    def forward_kinematics(self):
        theta1_rad = math.radians(self.theta1)
        theta2_rad = math.radians(self.theta2)
        
        self.x = round(self.L1 * math.cos(theta1_rad) + self.L2 * math.cos(theta1_rad + theta2_rad))
        self.y = round(self.L1 * math.sin(theta1_rad) + self.L2 * math.sin(theta1_rad + theta2_rad))
        
        self.pos_display.setText(f"Current Position: X={self.x}  Y={self.y}  Z={self.z}")
    
    def inverse_kinematics(self, x, y):
        try:
            x = float(x)
            y = float(y)
        except ValueError:
            return False
        
        #y= y- self.L0  # Adjust y coordinate based on L0 offset

        # حساب قيمة الكسر
        numerator = (x**2 + y**2 - self.L1**2 - self.L2**2)
        denominator = 2 * self.L1 * self.L2
        fraction = numerator / denominator

        # التحقق من إن القيمة داخل النطاق [-1, 1]
        if fraction < -1 or fraction > 1:
            msg = QMessageBox(self)
            msg.setIcon(QMessageBox.Warning)
            msg.setWindowTitle("Error")
            msg.setText("Target position is out of reachable area!")
            msg.setStyleSheet("QLabel { color: black; }")  # هنا بنخلي الخط أسود
            msg.exec_()

            return False

        self.theta2 = math.acos(fraction)

        if x < 0 and y < 0:
            self.theta2 = -self.theta2

        self.theta1 = math.atan(x / y) - math.atan((self.L2 * math.sin(self.theta2)) / (self.L1 + self.L2 * math.cos(self.theta2)))

        self.theta2 = -math.degrees(self.theta2)
        self.theta1 = math.degrees(self.theta1)

        # Angle adjustments based on quadrant
        if x >= 0 and y >= 0:  # 1st quadrant
            self.theta1 = 90 - self.theta1
        elif x < 0 and y > 0:  # 2nd quadrant
            self.theta1 = 90 - self.theta1
        elif x < 0 and y < 0:  # 3rd quadrant
            self.theta1 = 270 - self.theta1
            self.phi = 270 - self.theta1 - self.theta2
            self.phi = -self.phi
        elif x > 0 and y < 0:  # 4th quadrant
            self.theta1 = -90 - self.theta1
        elif x < 0 and y == 0:
            self.theta1 = 270 + self.theta1

        # Calculate phi angle for gripper orientation
        self.phi = 90 + self.theta1 + self.theta2
        self.phi = -self.phi

        if x < 0 and y < 0:  # 3rd quadrant
            self.phi = 270 - self.theta1 - self.theta2

        if abs(self.phi) > 165:
            self.phi = 180 + self.phi

        self.theta1 = round(self.theta1)
        self.theta2 = round(self.theta2)
        self.phi = round(self.phi)

        # Update sliders
        self.j1_slider.setValue(self.theta1)
        self.j2_slider.setValue(self.theta2)
        self.j3_slider.setValue(self.phi)

        return True


    def move_to_position(self):
        x = self.x_input.text()
        y = self.y_input.text()
        z = self.z_input.text()
        
        if self.inverse_kinematics(x, y):
            try:
                self.z = int(z)
                self.z_slider.setValue(self.z)
                self.send_data()
            except ValueError:
                QMessageBox.warning(self, "Error", "Invalid Z value")
    
    def update_gripper(self, value):
        self.gripper_value = value   # Adjust range as needed
        self.grip_value_label.setText(f"{value}%")
        self.send_data()
    
    def save_position(self):
        pos_str = (f"J1={self.theta1}; J2={self.theta2}; "
                  f"J3={self.phi}; Z={self.z}")
        self.positions.append(pos_str)
        self.last_pos_label.setText(f"Last saved position: {pos_str}")
        self.save_status = 1
        self.send_data()
        self.save_status = 0
    
    def toggle_run(self):
        if self.run_status == 0:
            self.run_status = 1
            self.run_btn.setText("STOP")
            self.run_btn.setStyleSheet("background-color: #c0392b; color: white;")
        else:
            self.run_status = 0
            self.run_btn.setText("RUN PROGRAM")
            self.run_btn.setStyleSheet("background-color: #27ae60; color: white;")
        
        self.send_data()
    
    def update_sa(self):
        self.speed = self.speed_slider.value()
        self.acceleration = self.accel_slider.value()
        self.send_data()
    
    def send_data(self):
        if self.serial_port and self.serial_port.is_open:
            data = (f"{self.save_status},{self.run_status},"
                   f"{self.theta1},{self.theta2},{self.phi},{self.z},"
                   f"{self.gripper_value},{self.speed},{self.acceleration}")
            
            print(f"Sending to Arduino: {data}")
            self.serial_port.write(data.encode())
            self.status_bar.showMessage(f"Sent: {data}")
        else:
            # For testing without serial connection
            data = (f"{self.save_status},{self.run_status},"
                   f"{self.theta1},{self.theta2},{self.phi},{self.z},"
                   f"{self.gripper_value},{self.speed},{self.acceleration}")
            self.status_bar.showMessage(f"Simulated: {data}")
    
    def update_ui(self):
        # Update any dynamic UI elements here
        pass
    
    def closeEvent(self, event):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = ScaraRobotGUI()
    gui.show()
    sys.exit(app.exec_())