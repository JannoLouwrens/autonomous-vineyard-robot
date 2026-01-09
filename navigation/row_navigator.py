#!/usr/bin/env python3
"""
Row Navigator - Navigate along tree row from START to END GPS
"""

import time
from math import radians, cos, sin, asin, sqrt, degrees, atan2
import numpy as np
import sys
sys.path.append('/home/pi/Working Code/Updated FULL Code Standalone')
from config import *


class RowNavigator:
    """
    Navigates robot along tree row using:
    1. GPS for start/end waypoints
    2. Compass for heading
    3. Row detector for visual guidance (tree wall detection)
    4. Obstacle avoider for safety
    """

    def __init__(self, motors, sensors, servos, row_detector, obstacle_avoider, camera=None):
        self.motors = motors
        self.sensors = sensors
        self.servos = servos
        self.row_detector = row_detector
        self.obstacle_avoider = obstacle_avoider
        self.camera = camera

        self.start_gps = None
        self.end_gps = None
        self.target_heading = None

        self.navigating = False
        self.paused = False

        # Visual guidance settings
        self.use_visual_guidance = True  # Enable row wall detection
        self.visual_weight = 0.7  # How much to trust visual vs compass (0-1)
        self.last_row_detection = None
        self.visual_fail_count = 0
        self.max_visual_fails = 10  # Fall back to compass after this many failures

        print("[Row Navigator] Initialized")
        print(f"  Visual guidance: {'ENABLED' if camera else 'DISABLED (no camera)'}")

    def set_row(self, start_gps, end_gps):
        """
        Set row start and end points
        Args:
            start_gps: {'latitude': float, 'longitude': float}
            end_gps: {'latitude': float, 'longitude': float}
        """
        self.start_gps = start_gps
        self.end_gps = end_gps

        # Calculate target heading from start to end
        self.target_heading = self._calculate_bearing(start_gps, end_gps)

        print(f"[Row Navigator] Row set:")
        print(f"  Start: {start_gps}")
        print(f"  End: {end_gps}")
        print(f"  Target heading: {self.target_heading:.1f}°")

    def navigate_to_start(self, timeout=120):
        """
        Navigate to row start point
        Returns: {'success': bool, 'reason': str}
        """
        if self.start_gps is None:
            return {'success': False, 'reason': 'no_start_set'}

        print("[Row Navigator] Navigating to row start...")
        return self._navigate_to_gps(self.start_gps, timeout)

    def orient_to_start(self):
        """
        Turn to face the start waypoint from current position
        Returns: {'success': bool, 'heading': float}
        """
        if self.start_gps is None:
            return {'success': False, 'reason': 'no_start_set'}

        self.sensors.update_all()
        if self.sensors.last_position is None:
            return {'success': False, 'reason': 'no_gps_fix'}

        # Calculate bearing from current position to start
        bearing_to_start = self._calculate_bearing(
            self.sensors.last_position,
            self.start_gps
        )

        print(f"[Row Navigator] Orienting to START waypoint (bearing {bearing_to_start:.1f}°)...")
        return self._orient_to_heading(bearing_to_start)

    def orient_to_end(self):
        """
        Turn to face the end of the row (using pre-calculated start→end bearing)
        Returns: {'success': bool, 'heading': float}
        """
        if self.target_heading is None:
            return {'success': False, 'reason': 'no_heading_set'}

        print(f"[Row Navigator] Orienting to END waypoint (bearing {self.target_heading:.1f}°)...")
        return self._orient_to_heading(self.target_heading)

    def _orient_to_heading(self, target_heading):
        """
        Turn to face a specific heading using proportional control
        Args:
            target_heading: Target compass heading (0-360°)
        Returns: {'success': bool, 'heading': float}
        """
        timeout = 30  # seconds
        start_time = time.time()

        while time.time() - start_time < timeout:
            self.sensors.update_all()
            current = self.sensors.last_heading

            if current is None:
                time.sleep(0.1)
                continue

            # Calculate error with wrap-around
            error = target_heading - current
            while error > 180:
                error -= 360
            while error < -180:
                error += 360

            abs_error = abs(error)

            # Check if we're close enough
            if abs_error < NAV_HEADING_TOLERANCE_DEG:
                self.motors.stop()
                print(f"[Row Navigator] Oriented - heading: {current:.1f}° (error: {error:.1f}°)")
                return {'success': True, 'heading': current}

            # Proportional speed: faster when far, slower when close
            if abs_error > 45:
                turn_speed = NAV_TURN_SPEED  # Full speed for large errors
                turn_time = 0.4
            elif abs_error > 20:
                turn_speed = int(NAV_TURN_SPEED * 0.75)  # 75% speed
                turn_time = 0.3
            else:
                turn_speed = int(NAV_TURN_SPEED * 0.5)  # 50% speed for fine adjustment
                turn_time = 0.2

            # Turn in correct direction
            if error > 0:
                self.motors.turn_right(turn_speed)
            else:
                self.motors.turn_left(turn_speed)

            time.sleep(turn_time)
            self.motors.stop()
            time.sleep(0.15)  # Pause to let compass settle and momentum stop

            print(f"[Orient] Current: {current:.1f}° Target: {target_heading:.1f}° Error: {error:.1f}°")

        self.motors.stop()
        return {'success': False, 'reason': 'timeout'}

    def navigate_row(self, tree_callback=None, progress_callback=None):
        """
        Navigate along the row, calling tree_callback when tree detected
        Maintains target distance from tree row using strafe corrections
        Args:
            tree_callback: function() called when tree detected on side
            progress_callback: function(distance_remaining) called periodically
        Returns: {'success': bool, 'trees_found': int, 'reason': str}
        """
        if self.start_gps is None or self.end_gps is None:
            return {'success': False, 'reason': 'row_not_set', 'trees_found': 0}

        print("[Row Navigator] Starting row navigation...")
        print(f"  Target distance from row: {ROW_TARGET_DISTANCE_CM}cm (+/- {ROW_DISTANCE_TOLERANCE_CM}cm)")
        self.navigating = True
        self.paused = False
        trees_found = 0
        last_tree_scan_time = 0  # Prevent rapid re-scans of same tree

        # Center servos to look forward
        self.servos.center()

        while self.navigating:
            # Update sensors
            self.sensors.update_all()

            # Check if reached end
            distance = self._distance_to_gps(self.end_gps)
            if distance < NAV_WAYPOINT_RADIUS_M:
                self.motors.stop()
                print(f"[Row Navigator] Reached end of row!")
                return {'success': True, 'trees_found': trees_found, 'distance': distance}

            # Progress callback
            if progress_callback:
                progress_callback(distance)

            # Check for obstacles
            if self.obstacle_avoider.check_and_avoid():
                continue  # Obstacle handled, continue loop

            # Get current distance to tree row (HC-SR04)
            side_distance = self.sensors.last_hcsr04_distance

            # Check if tree detected (within detection range)
            tree_detected = (side_distance is not None and
                           HCSR04_TREE_DETECT_MIN_CM < side_distance < HCSR04_TREE_DETECT_MAX_CM)

            if tree_detected:
                current_time = time.time()

                # Only scan if enough time passed since last scan (prevent re-scanning same tree)
                if current_time - last_tree_scan_time > 3.0:
                    print(f"[Row Navigator] Tree detected at {side_distance}cm!")
                    self.motors.stop()

                    # Move closer to tree for accurate scan if needed
                    if side_distance > ROW_SCAN_DISTANCE_CM:
                        print(f"[Row Navigator] Moving closer for scan ({side_distance}cm -> {ROW_SCAN_DISTANCE_CM}cm)...")
                        self._strafe_to_distance(ROW_SCAN_DISTANCE_CM)

                    trees_found += 1
                    last_tree_scan_time = current_time

                    if tree_callback:
                        # Callback handles tree inspection (YOLO verify, thermal, canopy photo)
                        tree_callback()

                    # After scan, move back to target row distance
                    print(f"[Row Navigator] Returning to row distance ({ROW_TARGET_DISTANCE_CM}cm)...")
                    self._strafe_to_distance(ROW_TARGET_DISTANCE_CM)

                    # Reset trigger and continue
                    self.sensors.reset_tree_trigger()
                    time.sleep(0.3)
                    continue

            # Maintain row distance while moving forward
            self._navigate_with_row_distance()

            time.sleep(1.0 / MAIN_LOOP_RATE_HZ)

        self.motors.stop()
        return {'success': False, 'reason': 'stopped', 'trees_found': trees_found}

    def _strafe_to_distance(self, target_cm, timeout=5.0):
        """
        Strafe left/right until HC-SR04 reads target distance
        Args:
            target_cm: Target distance from tree row
            timeout: Max time to spend strafing
        """
        start_time = time.time()
        tolerance = 10  # cm tolerance for "close enough"

        while time.time() - start_time < timeout:
            self.sensors.update_all()
            current = self.sensors.last_hcsr04_distance

            if current is None:
                # No reading, stop and wait
                self.motors.stop()
                time.sleep(0.1)
                continue

            error = current - target_cm

            if abs(error) <= tolerance:
                # Close enough
                self.motors.stop()
                print(f"[Row Navigator] Reached target distance: {current}cm")
                return True

            if error > 0:
                # Too far from tree, strafe LEFT (toward tree)
                self.motors.strafe_left(ROW_STRAFE_SPEED)
            else:
                # Too close to tree, strafe RIGHT (away from tree)
                self.motors.strafe_right(ROW_STRAFE_SPEED)

            time.sleep(0.05)

        self.motors.stop()
        print(f"[Row Navigator] Strafe timeout - current distance: {self.sensors.last_hcsr04_distance}cm")
        return False

    def _navigate_with_row_distance(self):
        """
        Move forward while maintaining target distance from tree row
        Combines forward movement with strafe corrections
        """
        if self.paused:
            self.motors.stop()
            return

        self.sensors.update_all()
        side_distance = self.sensors.last_hcsr04_distance
        current_heading = self.sensors.last_heading

        # Calculate row distance correction
        strafe_correction = 0.0
        if side_distance is not None and HCSR04_TREE_DETECT_MIN_CM < side_distance < HCSR04_TREE_DETECT_MAX_CM:
            distance_error = side_distance - ROW_TARGET_DISTANCE_CM

            if abs(distance_error) > ROW_DISTANCE_TOLERANCE_CM:
                # Need strafe correction: positive = too far = strafe left
                strafe_correction = np.clip(distance_error / 50.0, -1.0, 1.0)

        # Get heading/visual correction (same as before)
        heading_correction = 0.0

        # Try visual guidance
        if self.use_visual_guidance and self.camera and self.row_detector:
            visual_result = self._get_visual_guidance()
            if visual_result['valid']:
                heading_correction = visual_result['correction']

        # Fall back to compass if no visual
        if heading_correction == 0.0 and current_heading is not None and self.target_heading is not None:
            heading_error = self.target_heading - current_heading
            while heading_error > 180:
                heading_error -= 360
            while heading_error < -180:
                heading_error += 360
            heading_correction = np.clip(heading_error / 45.0, -1.0, 1.0)

        # Apply combined movement (forward + strafe + steering)
        self._apply_combined_movement(heading_correction, strafe_correction)

    def _navigate_step(self):
        """
        Single navigation step - maintain heading and course
        Uses HYBRID approach:
        1. Visual row detection (tree wall) for precise centering
        2. Compass heading as fallback/supplement
        """
        if self.paused:
            self.motors.stop()
            return

        # Get sensor data
        self.sensors.update_all()
        current_heading = self.sensors.last_heading

        # Try visual guidance first (if camera available)
        visual_correction = 0.0
        visual_valid = False

        if self.use_visual_guidance and self.camera and self.row_detector:
            visual_result = self._get_visual_guidance()
            if visual_result['valid']:
                visual_correction = visual_result['correction']
                visual_valid = True
                self.visual_fail_count = 0
            else:
                self.visual_fail_count += 1

        # Get compass-based correction
        compass_correction = 0.0
        compass_valid = False

        if current_heading is not None and self.target_heading is not None:
            heading_error = self.target_heading - current_heading
            while heading_error > 180:
                heading_error -= 360
            while heading_error < -180:
                heading_error += 360
            # Normalize to -1 to +1
            compass_correction = np.clip(heading_error / 45.0, -1.0, 1.0)
            compass_valid = True

        # Combine corrections (hybrid approach)
        if visual_valid and compass_valid:
            # Blend both: visual for centering, compass for direction
            correction = (self.visual_weight * visual_correction +
                         (1 - self.visual_weight) * compass_correction)
        elif visual_valid:
            # Visual only
            correction = visual_correction
        elif compass_valid:
            # Compass only (fallback)
            correction = compass_correction
        else:
            # No guidance, go slow and straight
            self.motors.forward(NAV_BASE_SPEED // 2)
            return

        # Apply steering based on combined correction
        self._apply_steering(correction)

    def _get_visual_guidance(self):
        """
        Get steering correction from visual row detection
        Returns: {'valid': bool, 'correction': float (-1 to +1), 'details': dict}
        """
        try:
            # Capture frame
            frame = self.camera.capture_csi_frame()
            if frame is None:
                return {'valid': False, 'reason': 'no_frame'}

            # Detect row lines and vanishing point
            result = self.row_detector.detect_rows(frame)
            self.last_row_detection = result

            if not result['found']:
                return {'valid': False, 'reason': result.get('reason', 'not_found')}

            # Get steering correction from row detector
            # offset_from_center: positive = vanishing point right of center = turn right
            correction = self.row_detector.get_steering_correction(frame)

            return {
                'valid': True,
                'correction': correction,
                'details': {
                    'vanishing_point': result['vanishing_point'],
                    'row_angle': result['row_angle'],
                    'offset': result['offset_from_center'],
                    'line_count': result['line_count']
                }
            }

        except Exception as e:
            return {'valid': False, 'reason': str(e)}

    def _apply_steering(self, correction):
        """
        Apply steering based on correction value
        Args:
            correction: -1 (full left) to +1 (full right)
        """
        # Dead zone for going straight
        if abs(correction) < 0.1:
            self.motors.forward(NAV_BASE_SPEED)
        elif correction > 0:
            # Need to turn right
            if correction > 0.5:
                # Sharp turn
                self.motors.turn_right(NAV_TURN_SPEED)
            else:
                # Gentle curve
                curve_factor = 1.0 - (correction * 0.8)  # 0.6 to 1.0
                self.motors.curve_right(NAV_BASE_SPEED, curve_factor)
        else:
            # Need to turn left
            if correction < -0.5:
                # Sharp turn
                self.motors.turn_left(NAV_TURN_SPEED)
            else:
                # Gentle curve
                curve_factor = 1.0 - (abs(correction) * 0.8)
                self.motors.curve_left(NAV_BASE_SPEED, curve_factor)

    def _apply_combined_movement(self, heading_correction, strafe_correction):
        """
        Apply combined forward + strafe + steering movement
        Uses mecanum wheel capabilities for diagonal movement
        Args:
            heading_correction: -1 to +1 (left/right steering)
            strafe_correction: -1 to +1 (negative=right/away, positive=left/toward tree)
        """
        # Base forward speed
        base = NAV_BASE_SPEED

        # Calculate individual wheel speeds for mecanum drive
        # Forward: all wheels same direction
        # Strafe left: LF-, LR+, RF+, RR-
        # Strafe right: LF+, LR-, RF-, RR+
        # Turn left: left wheels back, right wheels forward
        # Turn right: left wheels forward, right wheels back

        # Start with forward motion
        lf = base
        lr = base
        rf = base
        rr = base

        # Add strafe component (mecanum diagonal)
        strafe_amount = int(strafe_correction * ROW_STRAFE_SPEED)
        lf -= strafe_amount  # Strafe left = LF slower
        lr += strafe_amount  # Strafe left = LR faster
        rf += strafe_amount  # Strafe left = RF faster
        rr -= strafe_amount  # Strafe left = RR slower

        # Add steering component
        steer_amount = int(heading_correction * NAV_TURN_SPEED * 0.5)
        lf += steer_amount  # Turn right = left wheels faster
        lr += steer_amount
        rf -= steer_amount  # Turn right = right wheels slower
        rr -= steer_amount

        # Clamp values to valid range
        lf = int(np.clip(lf, -255, 255))
        lr = int(np.clip(lr, -255, 255))
        rf = int(np.clip(rf, -255, 255))
        rr = int(np.clip(rr, -255, 255))

        # Apply to motors
        self.motors.set_motors(lf, lr, rf, rr)

    def test_row_detection(self):
        """
        Test row detection and return results
        Returns: dict with detection results and visualization
        """
        if not self.camera:
            return {'success': False, 'error': 'no_camera'}

        if not self.row_detector:
            return {'success': False, 'error': 'no_row_detector'}

        # Make sure looking forward
        self.servos.center()
        time.sleep(0.3)

        # Capture and detect
        frame = self.camera.capture_csi_frame()
        if frame is None:
            return {'success': False, 'error': 'capture_failed'}

        result = self.row_detector.detect_rows(frame)

        # Get visualization
        annotated = self.row_detector.visualize(frame)

        # Save debug image
        if annotated is not None:
            import cv2
            debug_path = '/tmp/row_detection_test.jpg'
            cv2.imwrite(debug_path, annotated)
            result['debug_image'] = debug_path

        if result['found']:
            correction = self.row_detector.get_steering_correction(frame)
            result['steering_correction'] = correction
            result['recommendation'] = (
                'GO STRAIGHT' if abs(correction) < 0.1 else
                f'TURN {"RIGHT" if correction > 0 else "LEFT"} ({abs(correction):.2f})'
            )

        return result

    def _navigate_to_gps(self, target_gps, timeout):
        """Navigate to specific GPS point"""
        start_time = time.time()

        while time.time() - start_time < timeout:
            self.sensors.update_all()

            if not self.sensors.is_ready_for_navigation():
                time.sleep(0.5)
                continue

            # Check distance
            distance = self._distance_to_gps(target_gps)
            if distance < NAV_WAYPOINT_RADIUS_M:
                self.motors.stop()
                return {'success': True, 'distance': distance}

            # Calculate bearing
            bearing = self._calculate_bearing(
                self.sensors.last_position,
                target_gps
            )

            # Heading error
            error = bearing - self.sensors.last_heading
            while error > 180:
                error -= 360
            while error < -180:
                error += 360

            # Check obstacles
            if self.obstacle_avoider.check_and_avoid():
                continue

            # Navigate
            if abs(error) < NAV_HEADING_TOLERANCE_DEG:
                self.motors.forward(NAV_BASE_SPEED)
            elif error > 0:
                self.motors.turn_right(NAV_TURN_SPEED)
            else:
                self.motors.turn_left(NAV_TURN_SPEED)

            time.sleep(0.1)

        self.motors.stop()
        return {'success': False, 'reason': 'timeout'}

    def _calculate_bearing(self, from_gps, to_gps):
        """Calculate bearing between two GPS points"""
        lat1 = radians(from_gps['latitude'])
        lat2 = radians(to_gps['latitude'])
        dlon = radians(to_gps['longitude'] - from_gps['longitude'])

        x = sin(dlon) * cos(lat2)
        y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)

        bearing = (degrees(atan2(x, y)) + 360) % 360
        return bearing

    def _distance_to_gps(self, target_gps):
        """Calculate distance to GPS point in meters"""
        if self.sensors.last_position is None:
            return float('inf')

        R = 6371000  # Earth radius in meters
        lat1 = radians(self.sensors.last_position['latitude'])
        lat2 = radians(target_gps['latitude'])
        dlat = lat2 - lat1
        dlon = radians(target_gps['longitude'] - self.sensors.last_position['longitude'])

        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
        return R * 2 * asin(sqrt(a))

    def pause(self):
        """Pause navigation"""
        self.paused = True
        self.motors.stop()

    def resume(self):
        """Resume navigation"""
        self.paused = False

    def stop(self):
        """Stop navigation completely"""
        self.navigating = False
        self.motors.stop()

    def get_status(self):
        """Get navigation status"""
        distance = None
        if self.end_gps and self.sensors.last_position:
            distance = self._distance_to_gps(self.end_gps)

        return {
            'navigating': self.navigating,
            'paused': self.paused,
            'target_heading': self.target_heading,
            'distance_remaining': distance,
            'start_gps': self.start_gps,
            'end_gps': self.end_gps
        }
