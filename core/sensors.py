#!/usr/bin/env python3
"""
Sensor Management: GPS, Compass (BNO055), Ultrasonic, HC-SR04
COPIED FROM WORKING CODE - Handles all sensor reading and calibration
"""

import time
import smbus
import serial
import sys
import os
sys.path.append('/home/pi/Working Code/Updated FULL Code Standalone')
from config import *
from gpiozero import DistanceSensor


class CompassSensor:
    """BNO055 9-DOF IMU - Direct register access - FROM WORKING CODE"""

    # Calibration data file path
    CALIBRATION_FILE = '/home/pi/Working Code/Updated FULL Code Standalone/data/bno055_calibration.json'

    def __init__(self, i2c_bus=I2C_BUS, addr=BNO055_ADDR):
        self.i2c = smbus.SMBus(i2c_bus)
        self.addr = addr
        self.initialized = False
        self.calibration = {'sys': 0, 'gyro': 0, 'accel': 0, 'mag': 0}
        self.init_compass()
        self._load_calibration()

    def init_compass(self):
        """Initialize BNO055 using direct register writes - FROM WORKING CODE"""
        try:
            print("[Compass] Initializing BNO055...")

            # Verify chip ID
            chip_id = self.i2c.read_byte_data(self.addr, BNO055_CHIP_ID)
            if chip_id != 0xA0:
                raise Exception(f"Wrong chip ID: 0x{chip_id:02x} (expected 0xa0)")

            # Set to config mode
            self.i2c.write_byte_data(self.addr, BNO055_OPR_MODE, OPERATION_MODE_CONFIG)
            time.sleep(0.03)

            # Select page 0
            self.i2c.write_byte_data(self.addr, BNO055_PAGE_ID, 0x00)
            time.sleep(0.01)

            # Set power mode to normal
            self.i2c.write_byte_data(self.addr, BNO055_PWR_MODE, 0x00)
            time.sleep(0.01)

            # Set to NDOF mode (fusion of all sensors)
            self.i2c.write_byte_data(self.addr, BNO055_OPR_MODE, OPERATION_MODE_NDOF)
            time.sleep(0.03)

            self.initialized = True
            print("[Compass] ✓ BNO055 initialized - Move in figure-8 for calibration")

        except Exception as e:
            print(f"[Compass] ✗ Initialization failed: {e}")
            self.initialized = False

    def read_heading(self):
        """Read compass heading in degrees (0-360) - FROM WORKING CODE"""
        if not self.initialized:
            return None

        try:
            # Read heading from Euler angle registers
            lsb = self.i2c.read_byte_data(self.addr, BNO055_EULER_H_LSB)
            msb = self.i2c.read_byte_data(self.addr, BNO055_EULER_H_MSB)
            raw = (msb << 8) | lsb
            heading = raw / 16.0

            # Apply mounting offset correction
            # This corrects for BNO055 not being aligned with robot's front
            heading = (heading + COMPASS_OFFSET_DEG) % 360

            # Read calibration status
            cal_stat = self.i2c.read_byte_data(self.addr, BNO055_CALIB_STAT)
            self.calibration = {
                'sys': (cal_stat >> 6) & 0x03,
                'gyro': (cal_stat >> 4) & 0x03,
                'accel': (cal_stat >> 2) & 0x03,
                'mag': cal_stat & 0x03
            }

            return heading

        except Exception as e:
            print(f"[Compass] Read error: {e}")
            return None

    def get_calibration_status(self):
        """Get calibration quality (0-3 for each sensor)"""
        return self.calibration

    def _load_calibration(self):
        """Load saved calibration offsets from file"""
        import json
        import os

        if not self.initialized:
            return

        if not os.path.exists(self.CALIBRATION_FILE):
            print("[Compass] No saved calibration found")
            return

        try:
            with open(self.CALIBRATION_FILE, 'r') as f:
                data = json.load(f)

            offsets = data.get('offsets', [])
            if len(offsets) != 22:
                print("[Compass] Invalid calibration data")
                return

            # Switch to config mode to write offsets
            self.i2c.write_byte_data(self.addr, BNO055_OPR_MODE, OPERATION_MODE_CONFIG)
            time.sleep(0.03)

            # Write calibration offsets (registers 0x55-0x6A)
            for i, val in enumerate(offsets):
                self.i2c.write_byte_data(self.addr, 0x55 + i, val)

            # Switch back to NDOF mode
            self.i2c.write_byte_data(self.addr, BNO055_OPR_MODE, OPERATION_MODE_NDOF)
            time.sleep(0.03)

            print(f"[Compass] ✓ Loaded saved calibration from {data.get('saved_at', 'unknown')}")

        except Exception as e:
            print(f"[Compass] Failed to load calibration: {e}")

    def save_calibration(self):
        """Save current calibration offsets to file (call when fully calibrated)"""
        import json
        import os
        from datetime import datetime

        if not self.initialized:
            return False

        # Check if fully calibrated
        if self.calibration.get('sys', 0) < 3 or self.calibration.get('mag', 0) < 3:
            print(f"[Compass] Not fully calibrated yet: {self.calibration}")
            return False

        try:
            # Switch to config mode to read offsets
            self.i2c.write_byte_data(self.addr, BNO055_OPR_MODE, OPERATION_MODE_CONFIG)
            time.sleep(0.03)

            # Read calibration offsets (registers 0x55-0x6A, 22 bytes)
            offsets = []
            for i in range(22):
                val = self.i2c.read_byte_data(self.addr, 0x55 + i)
                offsets.append(val)

            # Switch back to NDOF mode
            self.i2c.write_byte_data(self.addr, BNO055_OPR_MODE, OPERATION_MODE_NDOF)
            time.sleep(0.03)

            # Save to file
            os.makedirs(os.path.dirname(self.CALIBRATION_FILE), exist_ok=True)
            data = {
                'offsets': offsets,
                'calibration_status': self.calibration,
                'saved_at': datetime.now().isoformat()
            }
            with open(self.CALIBRATION_FILE, 'w') as f:
                json.dump(data, f, indent=2)

            print(f"[Compass] ✓ Calibration saved to {self.CALIBRATION_FILE}")
            return True

        except Exception as e:
            print(f"[Compass] Failed to save calibration: {e}")
            return False


class GPSSensor:
    """NEO-8M GPS Module - NMEA sentence parsing - FROM WORKING CODE"""

    def __init__(self, port=GPS_PORT, baudrate=GPS_BAUD):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.satellites = 0
        self.hdop = 99.0
        self.fix_quality = 0
        self.last_known_position = None
        self.init_gps()

    def init_gps(self):
        """Initialize GPS serial connection"""
        try:
            print(f"[GPS] Initializing on {self.port}...")
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            print("[GPS] ✓ GPS initialized")
        except Exception as e:
            print(f"[GPS] ✗ Initialization failed: {e}")
            self.serial = None

    def read_position(self):
        """
        Read GPS position and quality metrics - FROM WORKING CODE
        Returns: {'latitude': float, 'longitude': float} or None
        """
        if not self.serial:
            return None

        try:
            line = self.serial.readline().decode('ascii', errors='replace')

            # Parse GPGGA for satellites, HDOP, fix quality (quality metrics ONLY)
            if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                parts = line.split(',')
                if len(parts) > 8:
                    try:
                        self.fix_quality = int(parts[6]) if parts[6] else 0
                        self.satellites = int(parts[7]) if parts[7] else 0
                        self.hdop = float(parts[8]) if parts[8] else 99.0
                    except:
                        pass

            # Parse GNGLL for position (MATCHES WORKING CODE EXACTLY)
            if line.startswith('$GNGLL'):
                parts = line.split(',')
                if len(parts) > 5 and parts[1] and parts[3]:
                    # Convert NMEA format to decimal degrees
                    lat = float(parts[1][:2]) + float(parts[1][2:]) / 60.0
                    if parts[2] == 'S':
                        lat = -lat
                    lon = float(parts[3][:3]) + float(parts[3][3:]) / 60.0
                    if parts[4] == 'W':
                        lon = -lon

                    # Log GPS position (matches working script)
                    print(f"[GPS] {lat:.6f}, {lon:.6f} | Sats: {self.satellites}, HDOP: {self.hdop:.1f}, Fix: {self.fix_quality}")

                    self.last_known_position = {'latitude': lat, 'longitude': lon}

            # Return the last known position if fix quality is good
            if self.fix_quality > 0:
                return self.last_known_position

        except Exception as e:
            print(f"[GPS] Error: {e}")

        return None

    def get_quality_metrics(self):
        """Get GPS signal quality"""
        return {
            'satellites': self.satellites,
            'hdop': self.hdop,
            'fix_quality': self.fix_quality
        }


class UltrasonicSensor:
    """Ultrasonic distance sensor - via Raspbot motor controller I2C - FROM WORKING CODE"""

    def __init__(self, motor_controller):
        self.car = motor_controller
        self.enabled = False
        self.init_ultrasonic()

    def init_ultrasonic(self):
        """Enable ultrasonic sensor"""
        try:
            print("[Ultrasonic] Initializing...")
            self.car.Ctrl_Ulatist_Switch(1)
            time.sleep(0.1)
            self.enabled = True
            print("[Ultrasonic] ✓ Sensor enabled")
        except Exception as e:
            print(f"[Ultrasonic] ✗ Initialization failed: {e}")

    def read_distance(self):
        """Read distance in millimeters - FROM WORKING CODE"""
        if not self.enabled:
            return None

        try:
            diss_H = self.car.read_data_array(0x1b, 1)[0]
            diss_L = self.car.read_data_array(0x1a, 1)[0]
            distance = (diss_H << 8) | diss_L
            return distance
        except Exception as e:
            return None


class HCSR04Sensor:
    """HC-SR04 Ultrasonic distance sensor - via GPIO pins (OPTIONAL - can run without)"""

    def __init__(self, trigger_pin=HCSR04_TRIGGER_PIN, echo_pin=HCSR04_ECHO_PIN):
        """
        Initialize HC-SR04 sensor
        Args:
            trigger_pin: GPIO pin for TRIG (default GPIO 5 = Pin 29)
            echo_pin: GPIO pin for ECHO (default GPIO 6 = Pin 31)
        """
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.sensor = None
        self.enabled = False

        # Check config - only try to init if HCSR04_ENABLED is True
        if not HCSR04_ENABLED:
            print("[HC-SR04] Disabled in config (HCSR04_ENABLED=False)")
            print("[HC-SR04] Set HCSR04_ENABLED=True in config.py when sensor is connected")
            return

        self.init_sensor()

    def init_sensor(self):
        """Initialize HC-SR04 sensor using gpiozero with timeout protection"""
        print(f"[HC-SR04] Initializing on GPIO {self.trigger_pin} (TRIG), GPIO {self.echo_pin} (ECHO)...")

        try:
            # Use shorter timeout to prevent hanging if sensor not powered properly
            self.sensor = DistanceSensor(
                echo=self.echo_pin,
                trigger=self.trigger_pin,
                max_distance=4.0,       # 4m max range (HC-SR04 spec)
                queue_len=3,            # Average 3 readings for stability
                threshold_distance=0.3  # 30cm threshold
            )
            self.enabled = True
            print("[HC-SR04] ✓ Sensor initialized")

            # Quick non-blocking test
            try:
                import threading
                result = [None]
                def quick_read():
                    try:
                        result[0] = self.sensor.distance
                    except:
                        pass
                t = threading.Thread(target=quick_read)
                t.start()
                t.join(timeout=0.5)  # 500ms timeout for test read
                if result[0] is not None and result[0] < 1.9:
                    print(f"[HC-SR04]   Test reading: {result[0]*100:.1f}cm")
                else:
                    print("[HC-SR04]   No echo in test (check 3.3V power or use 5V)")
            except:
                pass

        except Exception as e:
            print(f"[HC-SR04] ✗ Initialization failed: {e}")
            print("[HC-SR04]   Check wiring: TRIG->GPIO5, ECHO->GPIO6")
            print("[HC-SR04]   Note: HC-SR04 needs 5V. Try powering from 5V pin.")
            self.sensor = None
            self.enabled = False

    def read_distance(self):
        """Read distance in centimeters with timeout (returns None if no echo)"""
        if not self.enabled or not self.sensor:
            return None

        try:
            import threading
            result = [None]

            def do_read():
                try:
                    result[0] = self.sensor.distance
                except:
                    pass

            t = threading.Thread(target=do_read)
            t.start()
            t.join(timeout=0.05)  # 50ms timeout per reading (allows ~20 reads/sec)

            if t.is_alive():
                # Timed out - sensor not responding
                return None

            distance_m = result[0]
            # gpiozero returns max_distance when no echo received
            if distance_m is None or distance_m >= 3.9:  # Near max = no echo
                return None
            distance_cm = distance_m * 100
            return round(distance_cm, 1)
        except:
            return None

    def read_distance_mm(self):
        """Read distance in millimeters (for compatibility)"""
        distance_cm = self.read_distance()
        if distance_cm is not None:
            return int(distance_cm * 10)
        return None

    def is_tree_detected(self):
        """
        Check if object is within detection range (single reading)
        Robot is moving, so single detection is enough to trigger stop + verify
        """
        dist = self.read_distance()
        if dist is None:
            return False
        return HCSR04_TREE_DETECT_MIN_CM < dist < HCSR04_TREE_DETECT_MAX_CM

    def get_detection_distance(self):
        """Get distance if object detected, else None"""
        dist = self.read_distance()
        if dist is not None and HCSR04_TREE_DETECT_MIN_CM < dist < HCSR04_TREE_DETECT_MAX_CM:
            return dist
        return None

    def cleanup(self):
        """Cleanup GPIO resources"""
        if self.sensor:
            try:
                self.sensor.close()
            except:
                pass


class SensorManager:
    """
    Unified sensor management - FROM WORKING CODE
    Coordinates GPS, Compass, Ultrasonic, and HC-SR04 sensors
    """

    def __init__(self, motor_controller):
        print("\n=== Initializing Sensors ===")

        self.compass = CompassSensor()
        self.gps = GPSSensor()
        self.ultrasonic = UltrasonicSensor(motor_controller)
        self.hcsr04 = HCSR04Sensor(trigger_pin=HCSR04_TRIGGER_PIN, echo_pin=HCSR04_ECHO_PIN)

        # Latest readings
        self.last_heading = None
        self.last_position = None
        self.last_distance = None              # Front ultrasonic (mm)
        self.last_front_distance_mm = None     # Alias for obstacle_avoider compatibility
        self.last_hcsr04_distance = None       # Side HC-SR04 (cm)

        # Tree detection state
        self._tree_trigger_active = False

        print("=== Sensors Ready ===\n")

    def update_all(self):
        """Update all sensor readings"""
        self.last_heading = self.compass.read_heading()
        self.last_position = self.gps.read_position()
        self.last_distance = self.ultrasonic.read_distance()
        self.last_front_distance_mm = self.last_distance  # Alias for obstacle_avoider
        self.last_hcsr04_distance = self.hcsr04.read_distance()

    def get_status(self):
        """Get current status of all sensors"""
        return {
            'compass_heading': self.last_heading,
            'compass_calibration': self.compass.get_calibration_status(),
            'gps_position': self.last_position,
            'gps_quality': self.gps.get_quality_metrics(),
            'ultrasonic_distance_mm': self.last_distance,
            'hcsr04_distance_cm': self.last_hcsr04_distance
        }

    def is_obstacle_ahead(self):
        """Check for obstacle in front"""
        if self.last_distance is None:
            return False
        return self.last_distance < OBSTACLE_STOP_DISTANCE_MM

    def is_tree_on_side(self):
        """Check for object on left side (single detection triggers stop + verify)"""
        return self.hcsr04.is_tree_detected()

    def confirm_tree_on_side(self):
        """
        Check if tree detected on side and not already triggered.
        Used by row_navigator to avoid multiple triggers for same tree.
        """
        if self._tree_trigger_active:
            return False  # Already handling a tree

        if self.hcsr04.is_tree_detected():
            self._tree_trigger_active = True
            return True
        return False

    def reset_tree_trigger(self):
        """Reset tree trigger after handling - allows next tree detection"""
        self._tree_trigger_active = False

    def get_side_detection_distance(self):
        """Get distance to detected object on side, or None"""
        return self.hcsr04.get_detection_distance()

    def is_ready_for_navigation(self):
        """Check if sensors are ready for autonomous navigation"""
        gps_ok = self.last_position is not None
        compass_ok = self.last_heading is not None
        return gps_ok and compass_ok

    def read_hcsr04(self):
        """
        Direct HC-SR04 read - bypasses caching for fastest response.
        Use this for watch mode and time-critical detection.
        Returns distance in cm or None.
        """
        return self.hcsr04.read_distance()

    def cleanup(self):
        """Cleanup sensor resources"""
        if self.hcsr04:
            self.hcsr04.cleanup()
