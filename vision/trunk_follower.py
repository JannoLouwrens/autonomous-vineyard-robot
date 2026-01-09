#!/usr/bin/env python3
"""
Trunk Follower - Follow tree trunk up/down with servo
"""

import time
import cv2
import numpy as np
import sys
sys.path.append('/home/pi/Working Code/Updated FULL Code Standalone')
from config import *


class TrunkFollower:
    """
    Follows tree trunk vertically using servo tilt
    Used to find trunk base (for thermal) and canopy (for photo)
    """

    def __init__(self, camera, servos, yolo_classifier=None):
        self.camera = camera
        self.servos = servos
        self.yolo = yolo_classifier

        # HSV range for brown trunk detection (backup if YOLO fails)
        self.trunk_hsv_lower = np.array([10, 50, 20])
        self.trunk_hsv_upper = np.array([30, 200, 200])

        print("[Trunk Follower] Initialized")

    def detect_trunk_color(self, frame):
        """
        Detect trunk using HSV color (brown)
        Returns: {'found': bool, 'center_x': int, 'center_y': int, 'bbox': tuple}
        """
        if frame is None:
            return {'found': False}

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.trunk_hsv_lower, self.trunk_hsv_upper)

        # Morphology cleanup
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return {'found': False}

        # Find largest vertical contour
        best = None
        best_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 200:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            aspect = h / w if w > 0 else 0

            # Trunk should be vertical (aspect > 1.5)
            if aspect >= 1.5 and area > best_area:
                best_area = area
                best = (x, y, w, h)

        if best:
            x, y, w, h = best
            return {
                'found': True,
                'center_x': x + w // 2,
                'center_y': y + h // 2,
                'bbox': best,
                'area': best_area
            }

        return {'found': False}

    def detect_trunk(self, frame):
        """
        Detect trunk using YOLO if available, else color
        """
        if self.yolo and self.yolo.is_available():
            result = self.yolo.verify_tree(frame)
            if result['verified']:
                det = result['detection']
                x, y, w, h = det['bbox']
                return {
                    'found': True,
                    'center_x': det['center'][0],
                    'center_y': det['center'][1],
                    'bbox': det['bbox'],
                    'method': 'yolo',
                    'confidence': det['confidence']
                }

        # Fallback to color detection
        result = self.detect_trunk_color(frame)
        if result['found']:
            result['method'] = 'color'
        return result

    def follow_down_to_base(self):
        """
        Follow trunk down until we see ground/base
        Returns: {'success': bool, 'tilt': final tilt angle}
        """
        print("[Trunk Follower] Following trunk down to base...")

        # Start looking straight ahead
        self.servos.set_tilt(SERVO_TILT_STRAIGHT)
        time.sleep(0.3)

        # Initial detection
        frame = self.camera.capture_csi_frame()
        detection = self.detect_trunk(frame)

        if not detection['found']:
            return {'success': False, 'reason': 'no_initial_trunk'}

        last_x = detection['center_x']
        lost_count = 0
        final_tilt = SERVO_TILT_STRAIGHT

        # Tilt down incrementally
        for tilt in range(SERVO_TILT_STRAIGHT - TRUNK_FOLLOW_STEP, SERVO_TILT_GROUND - 1, -TRUNK_FOLLOW_STEP):
            self.servos.set_tilt(tilt)
            time.sleep(TRUNK_FOLLOW_DELAY)

            frame = self.camera.capture_csi_frame()
            if frame is None:
                continue

            detection = self.detect_trunk(frame)

            if detection['found']:
                # Check if it's the same trunk (x position similar)
                if abs(detection['center_x'] - last_x) < frame.shape[1] // 4:
                    last_x = detection['center_x']
                    final_tilt = tilt
                    lost_count = 0
                else:
                    # Different trunk, stop
                    break
            else:
                lost_count += 1
                if lost_count >= TRUNK_LOST_THRESHOLD:
                    # Lost trunk, use last good position
                    break

        # Set to final good position
        self.servos.set_tilt(final_tilt)
        print(f"[Trunk Follower] Base found at tilt={final_tilt}")

        return {'success': True, 'tilt': final_tilt}

    def follow_up_to_canopy(self):
        """
        Follow trunk up until we see canopy/leaves
        Returns: {'success': bool, 'tilt': final tilt angle, 'green_pct': float}
        """
        print("[Trunk Follower] Following trunk up to canopy...")

        # Start from current position or straight
        current_tilt = self.servos.tilt
        if current_tilt < SERVO_TILT_STRAIGHT:
            self.servos.set_tilt(SERVO_TILT_STRAIGHT)
            current_tilt = SERVO_TILT_STRAIGHT
            time.sleep(0.3)

        best_tilt = current_tilt
        best_green = 0

        # Tilt up incrementally
        for tilt in range(current_tilt + TRUNK_FOLLOW_STEP, SERVO_TILT_CANOPY + 1, TRUNK_FOLLOW_STEP):
            self.servos.set_tilt(tilt)
            time.sleep(TRUNK_FOLLOW_DELAY)

            frame = self.camera.capture_csi_frame()
            if frame is None:
                continue

            green_pct = self._detect_green_percentage(frame)

            if green_pct > best_green:
                best_green = green_pct
                best_tilt = tilt

            # If we have good canopy coverage, stop
            if green_pct > CANOPY_MIN_PERCENTAGE:
                break

        self.servos.set_tilt(best_tilt)
        print(f"[Trunk Follower] Canopy found at tilt={best_tilt}, green={best_green:.1f}%")

        return {
            'success': best_green > CANOPY_MIN_PERCENTAGE / 2,
            'tilt': best_tilt,
            'green_percentage': best_green
        }

    def _detect_green_percentage(self, frame):
        """Calculate percentage of frame that is green (leaves)"""
        if frame is None:
            return 0

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array(CANOPY_HSV_LOWER)
        upper = np.array(CANOPY_HSV_UPPER)
        mask = cv2.inRange(hsv, lower, upper)

        green_pixels = np.count_nonzero(mask)
        total_pixels = mask.shape[0] * mask.shape[1]

        return (green_pixels / total_pixels) * 100

    def center_on_trunk(self):
        """
        Adjust pan to center trunk in frame
        Returns: pan adjustment needed (negative = pan left)
        """
        frame = self.camera.capture_csi_frame()
        if frame is None:
            return 0

        detection = self.detect_trunk(frame)
        if not detection['found']:
            return 0

        frame_center = frame.shape[1] // 2
        trunk_x = detection['center_x']
        offset = trunk_x - frame_center

        # Normalize to -1 to +1
        return offset / frame_center
