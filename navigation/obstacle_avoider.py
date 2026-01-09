#!/usr/bin/env python3
"""
Obstacle Avoider - Smart obstacle avoidance while staying on course
"""

import time
from math import radians, cos, sin
import sys
sys.path.append('/home/pi/Working Code/Updated FULL Code Standalone')
from config import *


class ObstacleAvoider:
    """
    Handles obstacle avoidance while maintaining course
    Uses front ultrasonic to detect and navigate around obstacles
    """

    def __init__(self, motors, sensors):
        self.motors = motors
        self.sensors = sensors
        self.avoiding = False
        self.original_heading = None
        print("[Obstacle Avoider] Initialized")

    def check_and_avoid(self):
        """
        Check for obstacle and avoid if needed
        Returns: True if had to avoid, False if path clear
        """
        distance = self.sensors.last_front_distance_mm
        if distance is None:
            return False

        # Emergency stop
        if distance < OBSTACLE_STOP_DISTANCE_MM:
            self.motors.stop()
            print(f"[Obstacle] EMERGENCY STOP - {distance}mm")
            return True

        # Need to avoid
        if distance < OBSTACLE_AVOID_DISTANCE_MM:
            print(f"[Obstacle] Avoiding obstacle at {distance}mm")
            self._avoid_obstacle()
            return True

        # Slow down zone
        if distance < OBSTACLE_SLOW_DISTANCE_MM:
            # Reduce speed but continue
            return False

        return False

    def _avoid_obstacle(self):
        """
        Execute obstacle avoidance maneuver
        Strategy: Turn right, move forward, turn left to resume course
        """
        self.avoiding = True
        self.original_heading = self.sensors.last_heading

        # Step 1: Stop
        self.motors.stop()
        time.sleep(0.2)

        # Step 2: Turn right to avoid (away from tree row on left)
        print("[Obstacle] Turning right...")
        self._turn_degrees(OBSTACLE_AVOID_TURN_ANGLE)

        # Step 3: Move forward past obstacle
        print("[Obstacle] Moving past...")
        self.motors.forward(NAV_BASE_SPEED)
        time.sleep(OBSTACLE_AVOID_FORWARD_TIME)

        # Check if still blocked
        self.sensors.update_all()
        if self.sensors.is_obstacle_ahead():
            # Still blocked, turn more
            self.motors.stop()
            self._turn_degrees(OBSTACLE_AVOID_TURN_ANGLE)
            self.motors.forward(NAV_BASE_SPEED)
            time.sleep(OBSTACLE_AVOID_FORWARD_TIME)

        # Step 4: Turn left to resume original direction
        print("[Obstacle] Resuming course...")
        self.motors.stop()
        self._turn_degrees(-OBSTACLE_AVOID_TURN_ANGLE)

        # Step 5: Move forward to get back on line
        self.motors.forward(NAV_BASE_SPEED)
        time.sleep(OBSTACLE_AVOID_FORWARD_TIME * 0.5)

        # Step 6: Turn left again to face original direction
        self.motors.stop()
        self._return_to_heading()

        self.avoiding = False
        print("[Obstacle] Avoidance complete")

    def _turn_degrees(self, degrees):
        """
        Turn by specified degrees (positive = right, negative = left)
        """
        if degrees > 0:
            self.motors.turn_right(NAV_TURN_SPEED)
        else:
            self.motors.turn_left(NAV_TURN_SPEED)

        # Approximate turn time (calibrate for your robot)
        turn_time = abs(degrees) / 90.0 * 0.8  # ~0.8 seconds for 90 degrees
        time.sleep(turn_time)
        self.motors.stop()
        time.sleep(0.1)

    def _return_to_heading(self):
        """
        Return to original heading using compass
        """
        if self.original_heading is None:
            return

        max_attempts = 10
        for _ in range(max_attempts):
            self.sensors.update_all()
            current = self.sensors.last_heading
            if current is None:
                break

            error = self.original_heading - current
            # Normalize to -180 to 180
            while error > 180:
                error -= 360
            while error < -180:
                error += 360

            if abs(error) < NAV_HEADING_TOLERANCE_DEG:
                break

            if error > 0:
                self.motors.turn_right(NAV_TURN_SPEED // 2)
            else:
                self.motors.turn_left(NAV_TURN_SPEED // 2)

            time.sleep(0.1)
            self.motors.stop()
            time.sleep(0.05)

        self.motors.stop()

    def is_path_clear(self, min_distance_mm=OBSTACLE_SLOW_DISTANCE_MM):
        """Check if path is clear ahead"""
        distance = self.sensors.last_front_distance_mm
        return distance is None or distance > min_distance_mm
