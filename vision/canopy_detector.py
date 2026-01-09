#!/usr/bin/env python3
"""
Canopy Detector - Detect green leaf canopy
"""

import cv2
import numpy as np
import sys
sys.path.append('/home/pi/Working Code/Updated FULL Code Standalone')
from config import *


class CanopyDetector:
    """
    Detects green canopy/leaves using HSV color space
    """

    def __init__(self):
        self.lower_hsv = np.array(CANOPY_HSV_LOWER)
        self.upper_hsv = np.array(CANOPY_HSV_UPPER)
        print("[Canopy Detector] Initialized")

    def detect(self, frame):
        """
        Detect green canopy in frame
        Returns: {
            'found': bool,
            'green_percentage': float,
            'center_x': int,
            'center_y': int,
            'bbox': (x, y, w, h),
            'contour_count': int
        }
        """
        if frame is None:
            return {'found': False, 'green_percentage': 0}

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)

        # Cleanup
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Calculate green percentage
        green_pixels = np.count_nonzero(mask)
        total_pixels = mask.shape[0] * mask.shape[1]
        green_pct = (green_pixels / total_pixels) * 100

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter by minimum area
        valid_contours = [c for c in contours if cv2.contourArea(c) >= CANOPY_MIN_AREA]

        if not valid_contours or green_pct < CANOPY_MIN_PERCENTAGE:
            return {
                'found': False,
                'green_percentage': green_pct,
                'contour_count': len(valid_contours)
            }

        # Find largest contour
        largest = max(valid_contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)

        # Calculate centroid
        M = cv2.moments(largest)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx, cy = x + w // 2, y + h // 2

        return {
            'found': True,
            'green_percentage': green_pct,
            'center_x': cx,
            'center_y': cy,
            'bbox': (x, y, w, h),
            'contour_count': len(valid_contours),
            'area': cv2.contourArea(largest)
        }

    def visualize(self, frame):
        """Draw canopy detection visualization"""
        if frame is None:
            return None

        annotated = frame.copy()
        result = self.detect(frame)

        if result['found']:
            x, y, w, h = result['bbox']
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(annotated, (result['center_x'], result['center_y']), 10, (0, 255, 0), -1)

        # Show percentage
        color = (0, 255, 0) if result['found'] else (0, 0, 255)
        cv2.putText(annotated, f"Green: {result['green_percentage']:.1f}%",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        return annotated

    def get_mask(self, frame):
        """Get green mask for debugging"""
        if frame is None:
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask
