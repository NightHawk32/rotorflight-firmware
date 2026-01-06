#!/usr/bin/env python3
"""
XACT Servo Programming GUI
Connects to Rotorflight firmware via MSP protocol to configure XACT servos
"""

import sys
import struct
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QComboBox, QPushButton, 
                             QSpinBox, QGroupBox, QGridLayout, QTextEdit,
                             QMessageBox, QTabWidget, QScrollArea)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont

# MSP Protocol Constants
MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_SET_XACT_SCAN = 161
MSP_XACT_PARAMS = 162
MSP_SET_XACT_PARAMS = 163

# XACT Field IDs
XACT_FIELD_PHYSICAL_ID = 0x00
XACT_FIELD_APP_ID_BASE = 0x01
XACT_FIELD_DATA_RATE = 0x02
XACT_FIELD_RANGE = 0x04
XACT_FIELD_DIRECTION = 0x05
XACT_FIELD_PULSE_TYPE = 0x06
XACT_FIELD_CHANNEL = 0x07
XACT_FIELD_CENTER = 0x08
XACT_FIELD_P1 = 0x11
XACT_FIELD_P2 = 0x12
XACT_FIELD_D1 = 0x13
XACT_FIELD_TB = 0x15
XACT_FIELD_POT_GAP = 0x21

# Field names and descriptions
XACT_FIELDS = {
    XACT_FIELD_PHYSICAL_ID: ("Physical ID", 0, 255),
    XACT_FIELD_APP_ID_BASE: ("App ID Offset", 0, 255),
    XACT_FIELD_DATA_RATE: ("Data Rate (ms)", 0, 65535),  # 16-bit value
    XACT_FIELD_RANGE: ("Range", 0, 2),  # 0=120°, 1=90°, 2=180°
    XACT_FIELD_DIRECTION: ("Direction", 0, 1),  # 0=CW, 1=CCW
    XACT_FIELD_PULSE_TYPE: ("Pulse Type", 0, 1),  # 0=1500us, 1=760us
    XACT_FIELD_CHANNEL: ("Channel", 0, 255),
    XACT_FIELD_CENTER: ("Center", 0, 255),
    XACT_FIELD_P1: ("P1 Gain", 0, 255),
    XACT_FIELD_P2: ("P2 Gain", 0, 255),
    XACT_FIELD_D1: ("D1 Gain", 0, 255),
    XACT_FIELD_TB: ("TB", 0, 255),
    XACT_FIELD_POT_GAP: ("POT Gap", 0, 255),
}

RANGE_OPTIONS = ["120°", "90°", "180°"]
DIRECTION_OPTIONS = ["Clockwise", "Counter-Clockwise"]
PULSE_TYPE_OPTIONS = ["1500us", "760us"]


class MSPProtocol:
    """MSP Protocol handler for serial communication"""
    
    def __init__(self):
        self.serial = None
        
    def connect(self, port, baudrate=115200):
        """Connect to serial port"""
        try:
            self.serial = serial.Serial(port, baudrate, timeout=1)
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from serial port"""
        if self.serial and self.serial.is_open:
            self.serial.close()
    
    def is_connected(self):
        """Check if connected"""
        return self.serial and self.serial.is_open
    
    def _calculate_crc(self, data):
        """Calculate MSP CRC"""
        crc = 0
        for byte in data:
            crc ^= byte
        return crc
    
    def send_command(self, cmd, payload=b''):
        """Send MSP command"""
        if not self.is_connected():
            return False
        
        # MSP v1 format: $M< size cmd payload crc
        size = len(payload)
        header = b'$M<'
        message = struct.pack('<BB', size, cmd) + payload
        crc = self._calculate_crc(message)
        
        packet = header + message + struct.pack('<B', crc)
        self.serial.write(packet)
        return True
    
    def read_response(self, timeout=1.0):
        """Read MSP response"""
        if not self.is_connected():
            return None
        
        self.serial.timeout = timeout
        
        # Look for header
        while True:
            byte = self.serial.read(1)
            if not byte:
                return None
            if byte == b'$':
                break
        
        # Read M and direction
        header = self.serial.read(2)
        if len(header) != 2 or header[0:1] != b'M':
            return None
        
        direction = header[1:2]
        if direction not in [b'>', b'!']:
            return None
        
        # Read size and command
        size_cmd = self.serial.read(2)
        if len(size_cmd) != 2:
            return None
        
        size, cmd = struct.unpack('<BB', size_cmd)
        
        # Read payload
        payload = self.serial.read(size) if size > 0 else b''
        if len(payload) != size:
            return None
        
        # Read CRC
        crc_byte = self.serial.read(1)
        if not crc_byte:
            return None
        
        # Verify CRC
        expected_crc = self._calculate_crc(size_cmd + payload)
        actual_crc = struct.unpack('<B', crc_byte)[0]
        
        if expected_crc != actual_crc:
            print(f"CRC mismatch: expected {expected_crc}, got {actual_crc}")
            return None
        
        return {
            'cmd': cmd,
            'payload': payload,
            'error': direction == b'!'
        }
    
    def send_and_receive(self, cmd, payload=b'', timeout=1.0):
        """Send command and wait for response"""
        if not self.send_command(cmd, payload):
            return None
        return self.read_response(timeout)


class XACTServoGUI(QMainWindow):
    """Main GUI window for XACT servo programming"""
    
    def __init__(self):
        super().__init__()
        self.msp = MSPProtocol()
        self.current_servo_phy_id = None
        self.servo_app_id = 0x6800  # Default app ID for XACT servos
        
        self.init_ui()
        self.refresh_ports()
        
        # Auto-refresh timer
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.refresh_ports)
        self.refresh_timer.start(2000)  # Refresh every 2 seconds
    
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("XACT Servo Programming Tool")
        self.setGeometry(100, 100, 900, 700)
        
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        
        # Connection group
        conn_group = self.create_connection_group()
        layout.addWidget(conn_group)
        
        # Servo control group
        servo_group = self.create_servo_control_group()
        layout.addWidget(servo_group)
        
        # Parameters tabs
        self.param_tabs = QTabWidget()
        self.param_tabs.setEnabled(False)
        
        # Basic parameters tab
        basic_tab = self.create_basic_params_tab()
        self.param_tabs.addTab(basic_tab, "Basic Settings")
        
        # Advanced parameters tab
        advanced_tab = self.create_advanced_params_tab()
        self.param_tabs.addTab(advanced_tab, "Advanced Settings")
        
        # Swashplate setup tab
        swashplate_tab = self.create_swashplate_tab()
        self.param_tabs.addTab(swashplate_tab, "Swashplate Setup")
        
        layout.addWidget(self.param_tabs)
        
        # Log area
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
        
        self.log("Application started. Please connect to flight controller.")
    
    def create_connection_group(self):
        """Create connection controls"""
        group = QGroupBox("Connection")
        layout = QHBoxLayout()
        
        layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        layout.addWidget(self.port_combo)
        
        layout.addWidget(QLabel("Baudrate:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "57600", "38400", "19200"])
        layout.addWidget(self.baud_combo)
        
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_btn)
        
        layout.addStretch()
        group.setLayout(layout)
        return group
    
    def create_servo_control_group(self):
        """Create servo control buttons"""
        group = QGroupBox("Servo Control")
        layout = QHBoxLayout()
        
        self.scan_btn = QPushButton("Scan for Servos")
        self.scan_btn.clicked.connect(self.scan_servos)
        self.scan_btn.setEnabled(False)
        layout.addWidget(self.scan_btn)
        
        self.read_btn = QPushButton("Read Parameters")
        self.read_btn.clicked.connect(self.read_servo_params)
        self.read_btn.setEnabled(False)
        layout.addWidget(self.read_btn)
        
        self.write_btn = QPushButton("Write All Parameters")
        self.write_btn.clicked.connect(self.write_all_params)
        self.write_btn.setEnabled(False)
        layout.addWidget(self.write_btn)
        
        self.servo_status_label = QLabel("No servo detected")
        layout.addWidget(self.servo_status_label)
        
        layout.addStretch()
        group.setLayout(layout)
        return group
    
    def create_basic_params_tab(self):
        """Create basic parameters tab"""
        widget = QWidget()
        layout = QGridLayout()
        
        self.basic_spinboxes = {}
        
        row = 0
        # Physical ID
        layout.addWidget(QLabel("Physical ID:"), row, 0)
        self.physical_id_spin = QSpinBox()
        self.physical_id_spin.setRange(0, 255)
        self.physical_id_spin.setPrefix("0x")
        self.physical_id_spin.setDisplayIntegerBase(16)
        layout.addWidget(self.physical_id_spin, row, 1)
        
        row += 1
        # App ID Offset
        layout.addWidget(QLabel("App ID Offset:"), row, 0)
        self.app_id_offset_spin = QSpinBox()
        self.app_id_offset_spin.setRange(0, 255)
        layout.addWidget(self.app_id_offset_spin, row, 1)
        
        row += 1
        # Range
        layout.addWidget(QLabel("Range:"), row, 0)
        self.range_combo = QComboBox()
        self.range_combo.addItems(RANGE_OPTIONS)
        layout.addWidget(self.range_combo, row, 1)
        
        row += 1
        # Direction
        layout.addWidget(QLabel("Direction:"), row, 0)
        self.direction_combo = QComboBox()
        self.direction_combo.addItems(DIRECTION_OPTIONS)
        layout.addWidget(self.direction_combo, row, 1)
        
        row += 1
        # Pulse Type
        layout.addWidget(QLabel("Pulse Type:"), row, 0)
        self.pulse_combo = QComboBox()
        self.pulse_combo.addItems(PULSE_TYPE_OPTIONS)
        layout.addWidget(self.pulse_combo, row, 1)
        
        row += 1
        # Data Rate
        layout.addWidget(QLabel("Data Rate (ms):"), row, 0)
        self.data_rate_spin = QSpinBox()
        self.data_rate_spin.setRange(0, 65535)  # 16-bit value
        layout.addWidget(self.data_rate_spin, row, 1)
        
        row += 1
        # Channel
        layout.addWidget(QLabel("Channel:"), row, 0)
        self.channel_spin = QSpinBox()
        self.channel_spin.setRange(0, 255)
        layout.addWidget(self.channel_spin, row, 1)
        
        row += 1
        # Center
        layout.addWidget(QLabel("Center:"), row, 0)
        self.center_spin = QSpinBox()
        self.center_spin.setRange(0, 255)
        layout.addWidget(self.center_spin, row, 1)
        
        layout.setRowStretch(row + 1, 1)
        widget.setLayout(layout)
        return widget
    
    def create_advanced_params_tab(self):
        """Create advanced parameters tab"""
        widget = QWidget()
        layout = QGridLayout()
        
        row = 0
        # P1 Gain
        layout.addWidget(QLabel("P1 Gain:"), row, 0)
        self.p1_spin = QSpinBox()
        self.p1_spin.setRange(0, 255)
        layout.addWidget(self.p1_spin, row, 1)
        
        row += 1
        # P2 Gain
        layout.addWidget(QLabel("P2 Gain:"), row, 0)
        self.p2_spin = QSpinBox()
        self.p2_spin.setRange(0, 255)
        layout.addWidget(self.p2_spin, row, 1)
        
        row += 1
        # D1 Gain
        layout.addWidget(QLabel("D1 Gain:"), row, 0)
        self.d1_spin = QSpinBox()
        self.d1_spin.setRange(0, 255)
        layout.addWidget(self.d1_spin, row, 1)
        
        row += 1
        # TB
        layout.addWidget(QLabel("TB:"), row, 0)
        self.tb_spin = QSpinBox()
        self.tb_spin.setRange(0, 255)
        layout.addWidget(self.tb_spin, row, 1)
        
        row += 1
        # POT Gap
        layout.addWidget(QLabel("POT Gap:"), row, 0)
        self.pot_gap_spin = QSpinBox()
        self.pot_gap_spin.setRange(0, 255)
        layout.addWidget(self.pot_gap_spin, row, 1)
        
        layout.setRowStretch(row + 1, 1)
        widget.setLayout(layout)
        return widget
    
    def create_swashplate_tab(self):
        """Create swashplate setup tab with 4 servos"""
        widget = QWidget()
        main_layout = QVBoxLayout()
        
        # Title
        title = QLabel("Swashplate Servo Configuration")
        title_font = QFont()
        title_font.setPointSize(12)
        title_font.setBold(True)
        title.setFont(title_font)
        main_layout.addWidget(title)
        
        # Instructions
        instructions = QLabel(
            "Click 'Program' to scan for a servo and configure it for the corresponding swashplate position.\n"
            "Each servo will be assigned its channel and mapped to the correct Physical ID and App ID Offset."
        )
        instructions.setWordWrap(True)
        main_layout.addWidget(instructions)
        
        # Servo grid
        servo_grid = QGridLayout()
        servo_grid.setSpacing(20)
        
        # Servo configurations: (servo_num, physical_id, app_id_offset)
        servo_configs = [
            (1, 0x0C, 0x01),
            (2, 0x09, 0x02),
            (3, 0x18, 0x03),
            (4, 0x10, 0x04)
        ]
        
        self.swashplate_servos = {}
        
        for idx, (servo_num, phy_id, app_id_offset) in enumerate(servo_configs):
            row = idx // 2
            col = (idx % 2) * 3
            
            # Servo group box
            servo_group = QGroupBox(f"Servo {servo_num}")
            servo_layout = QVBoxLayout()
            
            # Servo illustration (simple text representation)
            servo_visual = QLabel("🔧")
            servo_visual.setAlignment(Qt.AlignCenter)
            servo_visual_font = QFont()
            servo_visual_font.setPointSize(48)
            servo_visual.setFont(servo_visual_font)
            servo_layout.addWidget(servo_visual)
            
            # Servo number label
            servo_label = QLabel(f"Servo {servo_num}")
            servo_label.setAlignment(Qt.AlignCenter)
            servo_label_font = QFont()
            servo_label_font.setPointSize(14)
            servo_label_font.setBold(True)
            servo_label.setFont(servo_label_font)
            servo_layout.addWidget(servo_label)
            
            # Configuration info
            config_info = QLabel(
                f"Physical ID: 0x{phy_id:02X}\n"
                f"App ID Offset: 0x{app_id_offset:02X}\n"
                f"Channel: {servo_num - 1}"
            )
            config_info.setAlignment(Qt.AlignCenter)
            servo_layout.addWidget(config_info)
            
            # Status label
            status_label = QLabel("Not configured")
            status_label.setAlignment(Qt.AlignCenter)
            status_label.setStyleSheet("color: gray;")
            servo_layout.addWidget(status_label)
            
            # Program button
            program_btn = QPushButton(f"Program Servo {servo_num}")
            program_btn.setMinimumHeight(40)
            program_btn.clicked.connect(
                lambda checked, sn=servo_num, pid=phy_id, aid=app_id_offset, sl=status_label:
                self.program_swashplate_servo(sn, pid, aid, sl)
            )
            servo_layout.addWidget(program_btn)
            
            servo_group.setLayout(servo_layout)
            servo_grid.addWidget(servo_group, row, col, 1, 3)
            
            # Store references
            self.swashplate_servos[servo_num] = {
                'status_label': status_label,
                'button': program_btn,
                'physical_id': phy_id,
                'app_id_offset': app_id_offset
            }
        
        main_layout.addLayout(servo_grid)
        main_layout.addStretch()
        
        widget.setLayout(main_layout)
        return widget
    
    def program_swashplate_servo(self, servo_num, physical_id, app_id_offset, status_label):
        """Program a swashplate servo with predefined settings"""
        if not self.msp.is_connected():
            QMessageBox.warning(self, "Error", "Not connected to flight controller")
            return
        
        channel = servo_num - 1  # Servo 1 -> Channel 0, etc.
        
        self.log(f"Programming Servo {servo_num}...")
        self.log(f"  Target Physical ID: 0x{physical_id:02X}")
        self.log(f"  Target App ID Offset: 0x{app_id_offset:02X}")
        self.log(f"  Target Channel: {channel}")
        
        status_label.setText("Scanning...")
        status_label.setStyleSheet("color: orange;")
        QApplication.processEvents()
        
        # Send scan command
        response = self.msp.send_and_receive(MSP_SET_XACT_SCAN)
        if not response or response['error']:
            self.log(f"✗ Failed to send scan command for Servo {servo_num}")
            status_label.setText("Scan failed")
            status_label.setStyleSheet("color: red;")
            return
        
        self.log(f"Scan command sent, waiting 2 seconds...")
        
        # Wait for scan to complete
        QTimer.singleShot(2000, lambda: self.complete_swashplate_programming(
            servo_num, physical_id, app_id_offset, channel, status_label
        ))
    
    def complete_swashplate_programming(self, servo_num, physical_id, app_id_offset, channel, status_label):
        """Complete the programming after scan"""
        # Get discovered servo
        response = self.msp.send_and_receive(MSP_XACT_PARAMS)
        
        if not response or response['error']:
            self.log(f"✗ Failed to get servo parameters for Servo {servo_num}")
            status_label.setText("No servo found")
            status_label.setStyleSheet("color: red;")
            return
        
        payload = response['payload']
        if len(payload) < 13:
            self.log(f"✗ No servo discovered for Servo {servo_num}")
            status_label.setText("No servo found")
            status_label.setStyleSheet("color: red;")
            return
        
        # Parse current parameters
        current_phy_id = payload[0]
        
        if current_phy_id == 0:
            self.log(f"✗ No servo discovered for Servo {servo_num}")
            status_label.setText("No servo found")
            status_label.setStyleSheet("color: red;")
            return
        
        self.log(f"Found servo with Physical ID: 0x{current_phy_id:02X}")
        
        # Parse all current parameters
        params = {
            'physicalId': payload[0],
            'appIdOffset': payload[1],
            'dataRate': struct.unpack('<H', payload[2:4])[0],
            'range': payload[4],
            'direction': payload[5],
            'pulseType': payload[6],
            'channel': payload[7],
            'center': payload[8],
            'p1': payload[9],
            'p2': payload[10],
            'd1': payload[11],
            'tb': payload[12],
            'potGap': payload[13]
        }
        
        # Build payload with new settings
        new_payload = struct.pack('<BBHBBBBBBBBBB',
            physical_id,  # New physical ID
            app_id_offset,  # New app ID offset
            params['dataRate'],  # Keep existing data rate
            params['range'],  # Keep existing range
            params['direction'],  # Keep existing direction
            params['pulseType'],  # Keep existing pulse type
            channel,  # Set channel to servo_num - 1
            params['center'],  # Keep existing center
            params['p1'],  # Keep existing p1
            params['p2'],  # Keep existing p2
            params['d1'],  # Keep existing d1
            params['tb'],  # Keep existing tb
            params['potGap']  # Keep existing potGap
        )
        
        # Write parameters
        response = self.msp.send_and_receive(MSP_SET_XACT_PARAMS, new_payload)
        
        if response and not response['error']:
            self.log(f"✓ Successfully programmed Servo {servo_num}")
            self.log(f"  Physical ID: 0x{physical_id:02X}")
            self.log(f"  App ID Offset: 0x{app_id_offset:02X}")
            self.log(f"  Channel: {channel}")
            status_label.setText("✓ Configured")
            status_label.setStyleSheet("color: green;")
        else:
            self.log(f"✗ Failed to write parameters for Servo {servo_num}")
            status_label.setText("Write failed")
            status_label.setStyleSheet("color: red;")
    
    def refresh_ports(self):
        """Refresh available serial ports"""
        current = self.port_combo.currentText()
        self.port_combo.clear()
        
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}")
        
        # Restore selection if possible
        index = self.port_combo.findText(current, Qt.MatchStartsWith)
        if index >= 0:
            self.port_combo.setCurrentIndex(index)
    
    def toggle_connection(self):
        """Connect or disconnect from flight controller"""
        if self.msp.is_connected():
            self.msp.disconnect()
            self.connect_btn.setText("Connect")
            self.scan_btn.setEnabled(False)
            self.read_btn.setEnabled(False)
            self.param_tabs.setEnabled(False)
            self.log("Disconnected")
        else:
            port_text = self.port_combo.currentText()
            if not port_text:
                QMessageBox.warning(self, "Error", "No port selected")
                return
            
            port = port_text.split(' - ')[0]
            baudrate = int(self.baud_combo.currentText())
            
            if self.msp.connect(port, baudrate):
                self.connect_btn.setText("Disconnect")
                self.scan_btn.setEnabled(True)
                self.log(f"Connected to {port} at {baudrate} baud")
                
                # Try to get FC info
                self.get_fc_info()
            else:
                QMessageBox.critical(self, "Error", f"Failed to connect to {port}")
    
    def get_fc_info(self):
        """Get flight controller information"""
        response = self.msp.send_and_receive(MSP_FC_VARIANT)
        if response and not response['error']:
            variant = response['payload'].decode('ascii', errors='ignore')
            self.log(f"Flight Controller: {variant}")
    
    def scan_servos(self):
        """Scan for XACT servos"""
        self.log("Starting servo scan...")
        
        # Send scan command (no response expected)
        response = self.msp.send_and_receive(MSP_SET_XACT_SCAN)
        if not response or response['error']:
            self.log("Failed to send scan command")
            return
        
        self.log("Scan command sent, waiting 2 seconds for discovery...")
        
        # Wait 2 seconds for scan to complete, then query results
        QTimer.singleShot(2000, self.get_discovered_servos)
    
    def get_discovered_servos(self):
        """Get parameters of the first discovered servo"""
        response = self.msp.send_and_receive(MSP_XACT_PARAMS)
        
        if not response or response['error']:
            self.log("Failed to get servo parameters")
            self.servo_status_label.setText("No servo detected")
            return
        
        payload = response['payload']
        if len(payload) < 13:
            self.log("No servos discovered or invalid response")
            self.servo_status_label.setText("No servo detected")
            return
        
        # Parse all parameters (dataRate is now 16-bit)
        params = {
            'physicalId': payload[0],
            'appIdOffset': payload[1],
            'dataRate': struct.unpack('<H', payload[2:4])[0],  # 16-bit value
            'range': payload[4],
            'direction': payload[5],
            'pulseType': payload[6],
            'channel': payload[7],
            'center': payload[8],
            'p1': payload[9],
            'p2': payload[10],
            'd1': payload[11],
            'tb': payload[12],
            'potGap': payload[13]
        }
        
        self.current_servo_phy_id = params['physicalId']
        
        if self.current_servo_phy_id == 0:
            self.log("No servo discovered")
            self.servo_status_label.setText("No servo detected")
            return
        
        self.log(f"Discovered servo with Physical ID: 0x{self.current_servo_phy_id:02X}")
        self.servo_status_label.setText(f"Servo 0x{self.current_servo_phy_id:02X} detected")
        
        # Update GUI with parameters
        self.update_gui_from_params(params)
        
        # Enable controls
        self.read_btn.setEnabled(True)
        self.write_btn.setEnabled(True)
        self.param_tabs.setEnabled(True)
    
    def update_gui_from_params(self, params):
        """Update GUI controls with parameter values"""
        self.physical_id_spin.setValue(params['physicalId'])
        self.app_id_offset_spin.setValue(params['appIdOffset'])
        self.range_combo.setCurrentIndex(params['range'])
        self.direction_combo.setCurrentIndex(params['direction'])
        self.pulse_combo.setCurrentIndex(params['pulseType'])
        self.data_rate_spin.setValue(params['dataRate'])
        self.channel_spin.setValue(params['channel'])
        self.center_spin.setValue(params['center'])
        self.p1_spin.setValue(params['p1'])
        self.p2_spin.setValue(params['p2'])
        self.d1_spin.setValue(params['d1'])
        self.tb_spin.setValue(params['tb'])
        self.pot_gap_spin.setValue(params['potGap'])
        
        self.log("Parameters loaded into GUI")
    
    def read_servo_params(self):
        """Read all parameters from the first discovered servo"""
        if self.current_servo_phy_id is None:
            QMessageBox.warning(self, "Error", "No servo detected. Please scan first.")
            return
        
        self.log(f"Reading parameters from servo 0x{self.current_servo_phy_id:02X}...")
        self.get_discovered_servos()
    
    def write_all_params(self):
        """Write all parameters to the servo (only changed ones will be written)"""
        if self.current_servo_phy_id is None:
            QMessageBox.warning(self, "Error", "No servo detected. Please scan first.")
            return
        
        self.log(f"Writing all parameters to servo 0x{self.current_servo_phy_id:02X}...")
        
        # Build payload with all parameters
        # Format: phyID (1), appId (2), then all 13 parameters
        payload = struct.pack('<BBHBBBBBBBBBB',
            self.physical_id_spin.value(),  # physicalId (editable)
            self.app_id_offset_spin.value(),  # appIdOffset (editable)
            self.data_rate_spin.value(),  # dataRate (16-bit)
            self.range_combo.currentIndex(),  # range
            self.direction_combo.currentIndex(),  # direction
            self.pulse_combo.currentIndex(),  # pulseType
            self.channel_spin.value(),  # channel
            self.center_spin.value(),  # center
            self.p1_spin.value(),  # p1
            self.p2_spin.value(),  # p2
            self.d1_spin.value(),  # d1
            self.tb_spin.value(),  # tb
            self.pot_gap_spin.value()  # potGap
        )
        
        response = self.msp.send_and_receive(MSP_SET_XACT_PARAMS, payload)
        
        if response and not response['error']:
            self.log("✓ Successfully wrote all parameters (only changed values were written)")
            # Update current servo ID if physical ID was changed
            new_phy_id = self.physical_id_spin.value()
            if new_phy_id != self.current_servo_phy_id:
                self.log(f"Physical ID changed from 0x{self.current_servo_phy_id:02X} to 0x{new_phy_id:02X}")
                self.current_servo_phy_id = new_phy_id
                self.servo_status_label.setText(f"Servo 0x{self.current_servo_phy_id:02X} detected")
        else:
            self.log("✗ Failed to write parameters")
            QMessageBox.warning(self, "Error", "Failed to write parameters")
    
    def log(self, message):
        """Add message to log"""
        self.log_text.append(message)
        # Auto-scroll to bottom
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def closeEvent(self, event):
        """Handle window close"""
        if self.msp.is_connected():
            self.msp.disconnect()
        event.accept()


def main():
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    # Create and show main window
    window = XACTServoGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()