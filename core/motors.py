#!/usr/bin/env python3
"""
Motor Control - 4-wheel mecanum drive with obstacle avoidance
"""

import time
import smbus
import sys
sys.path.append('/home/pi/Working Code/Updated FULL Code Standalone')
from config import *


class Raspbot:
    """Low-level I2C motor driver"""

    def __init__(self):
        self._device = smbus.SMBus(I2C_BUS)
        self._addr = MOTOR_DRIVER_ADDR

    def write_array(self, reg, data):
        try:
            self._device.write_i2c_block_data(self._addr, reg, data)
        except:
            print('[Raspbot] I2C write error')

    def read_data_array(self, reg, length):
        try:
            return self._device.read_i2c_block_data(self._addr, reg, length)
        except:
            print('[Raspbot] I2C read error')
            return [0] * length

    def Ctrl_Muto(self, motor_id, speed):
        """Control motor with speed -255 to +255"""
        speed = max(-255, min(255, speed))
        direction = 1 if speed < 0 else 0
        self.write_array(0x01, [motor_id, direction, abs(speed)])

    def Ctrl_Servo(self, servo_id, angle):
        """Control servo (1 or 2) with angle 0-180"""
        angle = max(0, min(180, angle))
        if servo_id == 2 and angle > 100:
            angle = 100  # Hardware limit
        self.write_array(0x02, [servo_id, angle])

    def Ctrl_Ulatist_Switch(self, state):
        """Enable/disable front ultrasonic"""
        self.write_array(0x07, [1 if state else 0])

    def Car_Stop(self):
        """Stop all motors"""
        for i in range(4):
            self.Ctrl_Muto(i, 0)


class MotorController:
    """
    High-level motor controller with obstacle avoidance
    Motors: 0=LF, 1=L, 2=RF, 3=R
    """

    def __init__(self):
        print("[Motors] Initializing...")
        self.car = Raspbot()
        self.current_speeds = [0, 0, 0, 0]
        self.obstacle_callback = None
        print("[Motors] Ready")

    def set_motors(self, lf, l, rf, r):
        """Set all 4 motor speeds (-255 to 255)"""
        self.car.Ctrl_Muto(0, lf)
        self.car.Ctrl_Muto(1, l)
        self.car.Ctrl_Muto(2, rf)
        self.car.Ctrl_Muto(3, r)
        self.current_speeds = [lf, l, rf, r]

    def forward(self, speed):
        self.set_motors(speed, speed, speed, speed)

    def backward(self, speed):
        self.set_motors(-speed, -speed, -speed, -speed)

    def turn_left(self, speed):
        """Rotate left in place"""
        self.set_motors(-speed, -speed, speed, speed)

    def turn_right(self, speed):
        """Rotate right in place"""
        self.set_motors(speed, speed, -speed, -speed)

    def strafe_left(self, speed):
        """Mecanum strafe left"""
        self.set_motors(-speed, speed, speed, -speed)

    def strafe_right(self, speed):
        """Mecanum strafe right"""
        self.set_motors(speed, -speed, -speed, speed)

    def curve_left(self, speed, ratio=0.5):
        """Move forward while curving left"""
        left_speed = int(speed * ratio)
        self.set_motors(left_speed, left_speed, speed, speed)

    def curve_right(self, speed, ratio=0.5):
        """Move forward while curving right"""
        right_speed = int(speed * ratio)
        self.set_motors(speed, speed, right_speed, right_speed)

    def stop(self):
        self.set_motors(0, 0, 0, 0)
        self.car.Car_Stop()

    def get_speeds(self):
        return self.current_speeds
