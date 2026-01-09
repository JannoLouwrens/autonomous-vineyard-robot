#!/usr/bin/env python3
"""
Tree Scanner - Complete tree inspection sequence
"""

import time
import os
import sys
sys.path.append('/home/pi/Working Code/Updated FULL Code Standalone')
from config import *


class TreeScanner:
    """
    Handles complete tree inspection sequence:
    1. Look left at detected tree
    2. YOLO verify it's correct tree type
    3. Follow trunk down to base
    4. Thermal scan for water
    5. Follow trunk up to canopy
    6. Take canopy photo
    """

    def __init__(self, camera, servos, yolo_classifier, trunk_follower, water_detector):
        self.camera = camera
        self.servos = servos
        self.yolo = yolo_classifier
        self.trunk_follower = trunk_follower
        self.water_detector = water_detector

        self.scan_count = 0

        # Create canopy images directory
        os.makedirs(CANOPY_IMAGES_DIR, exist_ok=True)

        print("[Tree Scanner] Initialized")

    def scan_tree(self):
        """
        Complete tree inspection sequence
        Returns: {
            'success': bool,
            'verified': bool,
            'water_result': dict,
            'canopy_image': str (path),
            'reason': str (if failed)
        }
        """
        self.scan_count += 1
        print(f"\n{'='*50}")
        print(f"[Tree Scanner] Starting scan #{self.scan_count}")
        print(f"{'='*50}")

        result = {
            'scan_id': self.scan_count,
            'success': False,
            'verified': False,
            'water_result': None,
            'canopy_image': None
        }

        # Step 1: Look left at tree
        print("[Step 1] Looking left at tree...")
        self.servos.look_left()
        self.servos.look_straight()
        time.sleep(0.5)

        # Step 2: YOLO verify tree type
        print("[Step 2] Verifying tree type...")
        frame = self.camera.capture_csi_frame()
        if frame is None:
            result['reason'] = 'camera_failed'
            return result

        # YOLO verification is MANDATORY - HC-SR04 can detect posts, poles, etc.
        if not self.yolo or not self.yolo.is_available():
            print("[Tree Scanner] ERROR: YOLO not available - cannot verify tree")
            result['reason'] = 'yolo_unavailable'
            self.servos.center()
            return result

        verify_result = self.yolo.verify_tree(frame)
        if not verify_result['verified']:
            print(f"[Tree Scanner] Not a valid tree (probably post/pole) - skipping")
            result['reason'] = 'yolo_rejected'
            self.servos.center()
            return result

        result['verified'] = True
        print(f"[Tree Scanner] Tree verified: {verify_result['detection']['class']} ({verify_result['confidence']:.2f})")

        # Step 3: Follow trunk down to base
        print("[Step 3] Following trunk down to base...")
        base_result = self.trunk_follower.follow_down_to_base()
        if not base_result['success']:
            print(f"[Tree Scanner] Failed to find base: {base_result.get('reason', 'unknown')}")
            result['reason'] = 'base_not_found'
            self.servos.center()
            return result

        # Step 4: Thermal water scan
        print("[Step 4] Scanning ground for water...")
        water_result = self.water_detector.analyze_ground(
            self.servos.pan,
            self.servos.tilt
        )
        result['water_result'] = water_result

        if water_result.get('has_water'):
            print(f"[Tree Scanner] WATER DETECTED - {water_result['water_percentage']:.1f}%")
        else:
            print(f"[Tree Scanner] DRY SOIL - {water_result.get('water_percentage', 0):.1f}%")

        # Step 5: Follow trunk up to canopy
        print("[Step 5] Following trunk up to canopy...")
        canopy_result = self.trunk_follower.follow_up_to_canopy()

        # Step 6: Take canopy photo
        print("[Step 6] Capturing canopy image...")
        timestamp = int(time.time())
        image_path = f"{CANOPY_IMAGES_DIR}/tree_{self.scan_count}_{timestamp}.jpg"

        if self.camera.save_image(image_path, use_thermal=False):
            result['canopy_image'] = image_path
            print(f"[Tree Scanner] Saved canopy image: {image_path}")
        else:
            print("[Tree Scanner] Failed to save canopy image")

        # Return to center
        self.servos.center()

        result['success'] = True
        print(f"\n[Tree Scanner] Scan #{self.scan_count} complete!")
        print(f"  Water: {'YES' if water_result.get('has_water') else 'NO'}")
        print(f"  Confidence: {water_result.get('confidence', 'N/A')}")
        print(f"{'='*50}\n")

        return result

    def quick_verify(self):
        """
        Quick check if looking at a tree (for testing)
        Returns: bool
        """
        self.servos.look_left()
        self.servos.look_straight()
        time.sleep(0.3)

        frame = self.camera.capture_csi_frame()
        if frame is None:
            self.servos.center()
            return False

        # YOLO verification is MANDATORY
        if not self.yolo or not self.yolo.is_available():
            print("[Tree Scanner] ERROR: YOLO not available for verification")
            self.servos.center()
            return False

        result = self.yolo.verify_tree(frame)
        self.servos.center()
        return result['verified']

    def test_thermal(self):
        """Test thermal water detection at current position"""
        print("[Tree Scanner] Testing thermal...")
        return self.water_detector.analyze_ground(
            self.servos.pan,
            self.servos.tilt
        )

    def test_canopy(self):
        """Test canopy detection"""
        print("[Tree Scanner] Testing canopy detection...")
        self.servos.look_canopy()
        time.sleep(0.3)

        result = self.trunk_follower.follow_up_to_canopy()
        self.servos.center()
        return result

    def get_stats(self):
        """Get scanning statistics"""
        return {
            'total_scans': self.scan_count,
            'yolo_available': self.yolo.is_available() if self.yolo else False
        }
