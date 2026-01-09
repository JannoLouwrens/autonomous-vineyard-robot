#!/usr/bin/env python3
"""
Row Detector - Detects tree row "wall" using vanishing point
Uses Canny edge detection + Hough lines to find row direction
"""

import cv2
import numpy as np
import sys
sys.path.append('/home/pi/Working Code/Updated FULL Code Standalone')
from config import *


class RowDetector:
    """
    Detects tree/vine row using computer vision
    Finds vanishing point where parallel row lines converge
    Used to keep robot oriented along the row
    """

    def __init__(self):
        print("[Row Detector] Initialized")
        self.last_vanishing_point = None
        self.last_row_lines = []

    def detect_rows(self, frame):
        """
        Detect row lines and vanishing point
        Args:
            frame: BGR image from camera
        Returns:
            dict: {
                'found': bool,
                'vanishing_point': (x, y) or None,
                'lines': list of line segments,
                'row_angle': float (degrees from vertical),
                'offset_from_center': float (pixels, + = right of center)
            }
        """
        if frame is None:
            return {'found': False}

        height, width = frame.shape[:2]
        center_x = width // 2

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Canny edge detection
        edges = cv2.Canny(
            blurred,
            ROW_DETECT_CANNY_LOW,
            ROW_DETECT_CANNY_HIGH
        )

        # Hough line detection
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=ROW_DETECT_HOUGH_THRESHOLD,
            minLineLength=ROW_DETECT_MIN_LINE_LENGTH,
            maxLineGap=ROW_DETECT_MAX_LINE_GAP
        )

        if lines is None or len(lines) < 2:
            return {'found': False, 'reason': 'insufficient_lines'}

        # Filter lines by angle (keep near-vertical lines)
        vertical_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]

            # Calculate angle from vertical
            if x2 - x1 == 0:
                angle = 90
            else:
                angle = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1)))

            # Keep lines within angle range (near vertical)
            if ROW_DETECT_ANGLE_MIN <= angle <= ROW_DETECT_ANGLE_MAX:
                length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                vertical_lines.append({
                    'coords': (x1, y1, x2, y2),
                    'angle': angle,
                    'length': length,
                    'x_center': (x1 + x2) / 2
                })

        if len(vertical_lines) < 2:
            return {'found': False, 'reason': 'no_vertical_lines'}

        # Find vanishing point using line intersections
        vanishing_point = self._find_vanishing_point(vertical_lines, width, height)

        if vanishing_point is None:
            return {'found': False, 'reason': 'no_vanishing_point'}

        # Calculate row angle (direction to vanishing point from bottom center)
        bottom_center = (center_x, height)
        vp_x, vp_y = vanishing_point

        row_angle = np.degrees(np.arctan2(vp_x - center_x, height - vp_y))
        offset = vp_x - center_x

        self.last_vanishing_point = vanishing_point
        self.last_row_lines = vertical_lines

        return {
            'found': True,
            'vanishing_point': vanishing_point,
            'lines': vertical_lines,
            'row_angle': row_angle,
            'offset_from_center': offset,
            'line_count': len(vertical_lines)
        }

    def _find_vanishing_point(self, lines, width, height):
        """
        Find vanishing point as intersection of multiple lines
        Uses weighted averaging of intersection points
        """
        if len(lines) < 2:
            return None

        intersections = []
        weights = []

        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                line1 = lines[i]['coords']
                line2 = lines[j]['coords']

                # Find intersection
                intersection = self._line_intersection(line1, line2)

                if intersection:
                    ix, iy = intersection

                    # Only keep intersections above horizon (upper half of image)
                    # and within reasonable bounds
                    if 0 < iy < height * 0.7 and -width < ix < 2 * width:
                        # Weight by combined line length (longer lines = more reliable)
                        weight = lines[i]['length'] + lines[j]['length']
                        intersections.append((ix, iy))
                        weights.append(weight)

        if not intersections:
            return None

        # Weighted average of intersections
        total_weight = sum(weights)
        if total_weight == 0:
            return None

        vp_x = sum(p[0] * w for p, w in zip(intersections, weights)) / total_weight
        vp_y = sum(p[1] * w for p, w in zip(intersections, weights)) / total_weight

        return (int(vp_x), int(vp_y))

    def _line_intersection(self, line1, line2):
        """Calculate intersection point of two line segments"""
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2

        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-10:
            return None  # Parallel lines

        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom

        ix = x1 + t * (x2 - x1)
        iy = y1 + t * (y2 - y1)

        return (ix, iy)

    def get_steering_correction(self, frame):
        """
        Get steering correction to stay on row
        Returns: float (-1 to +1, negative = turn left)
        """
        result = self.detect_rows(frame)
        if not result['found']:
            return 0.0

        # Use offset from center, normalized
        offset = result['offset_from_center']
        width = frame.shape[1]

        # Normalize to -1 to +1 range
        correction = np.clip(offset / (width / 2), -1.0, 1.0)

        return correction

    def visualize(self, frame):
        """
        Draw detection visualization on frame
        Returns: annotated frame
        """
        if frame is None:
            return None

        annotated = frame.copy()
        height, width = frame.shape[:2]

        result = self.detect_rows(frame)

        if result['found']:
            # Draw detected lines
            for line in result['lines']:
                x1, y1, x2, y2 = line['coords']
                cv2.line(annotated, (int(x1), int(y1)), (int(x2), int(y2)),
                         (0, 255, 0), 2)

            # Draw vanishing point
            vp = result['vanishing_point']
            cv2.circle(annotated, vp, 15, (0, 0, 255), 3)
            cv2.drawMarker(annotated, vp, (0, 0, 255), cv2.MARKER_CROSS, 30, 3)

            # Draw guide line from bottom center to vanishing point
            bottom_center = (width // 2, height)
            cv2.line(annotated, bottom_center, vp, (255, 0, 0), 2)

            # Draw info
            cv2.putText(annotated, f"Row Angle: {result['row_angle']:.1f}deg",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(annotated, f"Offset: {result['offset_from_center']:.0f}px",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(annotated, f"Lines: {result['line_count']}",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        else:
            cv2.putText(annotated, "NO ROW DETECTED",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(annotated, f"Reason: {result.get('reason', 'unknown')}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Draw center line
        cv2.line(annotated, (width // 2, 0), (width // 2, height),
                 (128, 128, 128), 1)

        return annotated
