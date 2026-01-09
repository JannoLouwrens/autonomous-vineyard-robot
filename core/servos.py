#!/usr/bin/env python3
"""
Servo Control - Camera pan/tilt for tree inspection
"""

import time
import sys
sys.path.append('/home/pi/Working Code/Updated FULL Code Standalone')
from config import *


class ServoController:
    """
    Controls 2 servos for camera pan/tilt
    Servo 1: Pan (left/right) - 0 to 180 degrees
    Servo 2: Tilt (up/down) - 0 to 100 degrees (hardware limit)
    """

    def __init__(self, raspbot):
        print("[Servos] Initializing...")
        self.car = raspbot
        self.pan = SERVO_CENTER
        self.tilt = SERVO_TILT_STRAIGHT
        self.center()
        print("[Servos] Ready - centered")

    def set_pan(self, angle):
        """Set pan angle (0-180), inverted for correct left/right"""
        angle = max(SERVO_PAN_MIN, min(SERVO_PAN_MAX, angle))
        # Invert so higher values = right
        inverted = SERVO_PAN_MAX - angle + SERVO_PAN_MIN
        self.car.Ctrl_Servo(1, int(inverted))
        self.pan = angle

    def set_tilt(self, angle):
        """Set tilt angle (0-100)"""
        angle = max(SERVO_TILT_MIN, min(SERVO_TILT_MAX, angle))
        self.car.Ctrl_Servo(2, int(angle))
        self.tilt = angle

    def set_both(self, pan, tilt):
        """Set both servos"""
        self.set_pan(pan)
        self.set_tilt(tilt)

    def center(self):
        """Center both servos"""
        self.set_both(SERVO_CENTER, SERVO_TILT_STRAIGHT)

    def look_left(self):
        """Look left at tree row"""
        self.set_pan(SERVO_LOOK_LEFT)

    def look_right(self):
        """Look right"""
        self.set_pan(SERVO_LOOK_RIGHT)

    def look_ground(self):
        """Look down at ground for thermal scan"""
        self.set_tilt(SERVO_TILT_GROUND)

    def look_canopy(self):
        """Look up at canopy"""
        self.set_tilt(SERVO_TILT_CANOPY)

    def look_straight(self):
        """Look straight ahead"""
        self.set_tilt(SERVO_TILT_STRAIGHT)

    def get_position(self):
        return {'pan': self.pan, 'tilt': self.tilt}

    def pan_scan(self, start=SERVO_PAN_MIN, end=SERVO_PAN_MAX, step=10, delay=0.2):
        """Generator for pan scanning"""
        for angle in range(start, end + 1, step):
            self.set_pan(angle)
            time.sleep(delay)
            yield angle

    def tilt_scan(self, start=SERVO_TILT_MIN, end=SERVO_TILT_MAX, step=5, delay=0.3):
        """Generator for tilt scanning"""
        for angle in range(start, end + 1, step):
            self.set_tilt(angle)
            time.sleep(delay)
            yield angle
