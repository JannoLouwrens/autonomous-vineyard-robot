#!/usr/bin/env python3
"""
Vineyard Robot - Main Controller
Autonomous tree row navigation and inspection
"""

import time
import json
import signal
import sys
import os
import threading

# Add paths for imports - insert at beginning to take priority
# Order matters: Updated FULL first (for vision, core, etc), then Working Code (for senxor only)
sys.path.insert(0, '/home/pi/Working Code')  # For senxor library
sys.path.insert(0, '/home/pi/Working Code/Updated FULL Code Standalone')  # Must be first!

from config import *

# Core modules
from core.motors import MotorController, Raspbot
from core.servos import ServoController
from core.sensors import SensorManager
from core.cameras import CameraManager

# Vision modules
from vision.row_detector import RowDetector
from vision.yolo_classifier import YOLOClassifier
from vision.trunk_follower import TrunkFollower
from vision.canopy_detector import CanopyDetector

# Navigation modules
from navigation.row_navigator import RowNavigator
from navigation.tree_scanner import TreeScanner
from navigation.obstacle_avoider import ObstacleAvoider

# Analysis
from analysis.water_detector import WaterDetector

# Network
from network.command_server import CommandServer
from network.stream_server import StreamServer


class VineyardRobot:
    """
    Main robot controller
    Handles all subsystems and command processing
    """

    def __init__(self):
        print("\n" + "="*60)
        print("VINEYARD ROBOT STARTING")
        print(f"Tree Type: {TREE_TYPE}")
        print("="*60 + "\n")

        # Create data directories
        os.makedirs(DATA_DIR, exist_ok=True)
        os.makedirs(CANOPY_IMAGES_DIR, exist_ok=True)

        # Initialize hardware
        self.raspbot = Raspbot()
        self.motors = MotorController()
        self.servos = ServoController(self.raspbot)
        self.sensors = SensorManager(self.raspbot)
        self.camera = CameraManager()

        # Initialize vision
        self.row_detector = RowDetector()
        self.yolo = YOLOClassifier()
        self.canopy_detector = CanopyDetector()
        self.trunk_follower = TrunkFollower(self.camera, self.servos, self.yolo)

        # Initialize analysis
        self.water_detector = WaterDetector(self.camera, self.servos)

        # Initialize navigation
        self.obstacle_avoider = ObstacleAvoider(self.motors, self.sensors)
        self.row_navigator = RowNavigator(
            self.motors, self.sensors, self.servos,
            self.row_detector, self.obstacle_avoider,
            camera=self.camera  # Pass camera for visual row guidance
        )
        self.tree_scanner = TreeScanner(
            self.camera, self.servos, self.yolo,
            self.trunk_follower, self.water_detector
        )

        # Network
        self.command_server = CommandServer(self)
        self.stream_server = StreamServer(self.camera, self.sensors, yolo=self.yolo)

        # State
        self.running = True
        self.rows = self._load_rows()
        self.inspection_log = []

        # Start sensor update thread
        self.sensor_thread = threading.Thread(target=self._sensor_loop, daemon=True)
        self.sensor_thread.start()

        print("\n" + "="*60)
        print("ROBOT READY")
        print("="*60 + "\n")

    def _load_rows(self):
        """Load saved rows from file"""
        if os.path.exists(ROWS_FILE):
            try:
                with open(ROWS_FILE, 'r') as f:
                    return json.load(f)
            except:
                pass
        return []

    def _save_rows(self):
        """Save rows to file"""
        with open(ROWS_FILE, 'w') as f:
            json.dump(self.rows, f, indent=2)

    def _sensor_loop(self):
        """Background sensor updates"""
        while self.running:
            self.sensors.update_all()
            time.sleep(1.0 / SENSOR_UPDATE_RATE_HZ)

    def start(self):
        """Start all servers"""
        self.command_server.start()
        self.stream_server.start()
        print("\n[Robot] Servers started - ready for commands")

    def process_command(self, cmd):
        """
        Process command string from client
        Returns: response string
        """
        parts = cmd.strip().split()
        if not parts:
            return "ERROR: Empty command"

        command = parts[0].lower()
        args = parts[1:]

        try:
            # === MOTOR COMMANDS ===
            if command == 'forward':
                speed = int(args[0]) if args else NAV_BASE_SPEED
                self.motors.forward(speed)
                return f"OK: Moving forward at {speed}"

            elif command == 'backward':
                speed = int(args[0]) if args else NAV_BASE_SPEED
                self.motors.backward(speed)
                return f"OK: Moving backward at {speed}"

            elif command == 'left':
                speed = int(args[0]) if args else NAV_TURN_SPEED
                self.motors.turn_left(speed)
                return f"OK: Turning left at {speed}"

            elif command == 'right':
                speed = int(args[0]) if args else NAV_TURN_SPEED
                self.motors.turn_right(speed)
                return f"OK: Turning right at {speed}"

            elif command == 'stop':
                self.motors.stop()
                return "OK: Stopped"

            # === SERVO COMMANDS ===
            elif command == 'servo':
                if len(args) >= 2:
                    pan = int(args[0])
                    tilt = int(args[1])
                    self.servos.set_both(pan, tilt)
                    return f"OK: Servo set to pan={pan}, tilt={tilt}"
                return "ERROR: servo <pan> <tilt>"

            elif command == 'center':
                self.servos.center()
                return "OK: Servos centered"

            elif command == 'look_left':
                self.servos.look_left()
                return "OK: Looking left"

            elif command == 'look_ground':
                self.servos.look_ground()
                return "OK: Looking at ground"

            elif command == 'look_canopy':
                self.servos.look_canopy()
                return "OK: Looking at canopy"

            # === CAMERA COMMANDS ===
            elif command == 'camera':
                if args:
                    cam_type = args[0].lower()
                    if cam_type in ['csi', 'thermal']:
                        self.camera.switch_camera(cam_type)
                        self.stream_server.set_camera(cam_type)
                        return f"OK: Switched to {cam_type} camera"
                    return "ERROR: camera <csi|thermal>"
                return f"OK: Active camera: {self.camera.active_camera}"

            elif command == 'yolo_overlay':
                if args:
                    if args[0].lower() in ['on', '1', 'true']:
                        self.stream_server.set_yolo_overlay(True)
                        return "OK: YOLO overlay ENABLED - boxes will appear on video"
                    elif args[0].lower() in ['off', '0', 'false']:
                        self.stream_server.set_yolo_overlay(False)
                        return "OK: YOLO overlay DISABLED"
                # Toggle if no argument
                current = self.stream_server.yolo_overlay
                self.stream_server.set_yolo_overlay(not current)
                status = "ENABLED" if not current else "DISABLED"
                return f"OK: YOLO overlay {status}"

            # === ADDITIONAL MOTOR COMMANDS ===
            elif command == 'strafe_left':
                speed = int(args[0]) if args else NAV_BASE_SPEED
                self.motors.strafe_left(speed)
                return f"OK: Strafing left at {speed}"

            elif command == 'strafe_right':
                speed = int(args[0]) if args else NAV_BASE_SPEED
                self.motors.strafe_right(speed)
                return f"OK: Strafing right at {speed}"

            elif command == 'move':
                if len(args) >= 4:
                    lf, l, rf, r = int(args[0]), int(args[1]), int(args[2]), int(args[3])
                    self.motors.set_motors(lf, l, rf, r)
                    return f"OK: Motors set to LF={lf}, L={l}, RF={rf}, R={r}"
                return "ERROR: move <lf> <l> <rf> <r>"

            # === SENSOR COMMANDS ===
            elif command == 'sensors':
                status = self.sensors.get_status()
                return json.dumps(status, indent=2)

            elif command == 'distance':
                front = self.sensors.last_distance
                side = self.sensors.last_hcsr04_distance
                return f"Front: {front}mm, Side: {side}cm"

            elif command == 'gps':
                pos = self.sensors.last_position
                if pos:
                    return f"GPS: {pos['latitude']:.6f}, {pos['longitude']:.6f}"
                return "GPS: No fix"

            elif command == 'heading':
                h = self.sensors.last_heading
                return f"Heading: {h:.1f}°" if h else "Heading: No data"

            # === ROW COMMANDS ===
            elif command == 'set_row_start':
                if self.sensors.last_position:
                    self._temp_row_start = self.sensors.last_position.copy()
                    return f"OK: Row start set to {self._temp_row_start}"
                return "ERROR: No GPS fix"

            elif command == 'set_row_end':
                if self.sensors.last_position:
                    self._temp_row_end = self.sensors.last_position.copy()
                    return f"OK: Row end set to {self._temp_row_end}"
                return "ERROR: No GPS fix"

            elif command == 'save_row':
                if hasattr(self, '_temp_row_start') and hasattr(self, '_temp_row_end'):
                    row = {
                        'id': len(self.rows) + 1,
                        'start': self._temp_row_start,
                        'end': self._temp_row_end
                    }
                    self.rows.append(row)
                    self._save_rows()
                    return f"OK: Row {row['id']} saved"
                return "ERROR: Set start and end first"

            elif command == 'list_rows':
                if not self.rows:
                    return "No rows saved"
                result = "Saved rows:\n"
                for row in self.rows:
                    result += f"  Row {row['id']}: {row['start']} -> {row['end']}\n"
                return result

            # === NAVIGATION COMMANDS ===
            elif command == 'navigate_row':
                row_id = int(args[0]) if args else 1
                row = next((r for r in self.rows if r['id'] == row_id), None)
                if not row:
                    return f"ERROR: Row {row_id} not found"

                # Set row and start navigation
                self.row_navigator.set_row(row['start'], row['end'])

                def tree_callback():
                    result = self.tree_scanner.scan_tree()
                    self.inspection_log.append(result)

                # Run in thread
                def run_nav():
                    result = self.row_navigator.navigate_to_start()
                    if result['success']:
                        self.row_navigator.orient_to_end()
                        self.row_navigator.navigate_row(tree_callback=tree_callback)

                threading.Thread(target=run_nav, daemon=True).start()
                return f"OK: Starting navigation of row {row_id}"

            elif command == 'stop_nav':
                self.row_navigator.stop()
                return "OK: Navigation stopped"

            elif command == 'pause_nav':
                self.row_navigator.pause()
                return "OK: Navigation paused"

            elif command == 'resume_nav':
                self.row_navigator.resume()
                return "OK: Navigation resumed"

            # === TEST COMMANDS ===
            elif command == 'test_row_detect':
                frame = self.camera.capture_csi_frame()
                result = self.row_detector.detect_rows(frame)
                return json.dumps(result, indent=2)

            elif command == 'test_yolo':
                try:
                    self.servos.look_left()
                    self.servos.look_straight()
                    time.sleep(0.3)
                    frame = self.camera.capture_csi_frame()
                    if frame is None:
                        self.servos.center()
                        return "ERROR: Camera capture failed"
                    if self.yolo is None:
                        self.servos.center()
                        return "ERROR: YOLO not initialized"
                    if not self.yolo.is_available():
                        self.servos.center()
                        return f"ERROR: YOLO model not loaded from {self.yolo.model_path}"
                    result = self.yolo.verify_tree(frame)
                    self.servos.center()
                    return json.dumps(result, indent=2)
                except Exception as e:
                    self.servos.center()
                    return f"ERROR: YOLO test failed - {str(e)}"

            elif command == 'test_trunk_down':
                result = self.trunk_follower.follow_down_to_base()
                return json.dumps(result, indent=2)

            elif command == 'test_trunk_up':
                result = self.trunk_follower.follow_up_to_canopy()
                return json.dumps(result, indent=2)

            elif command == 'test_thermal':
                result = self.water_detector.analyze_ground()
                return json.dumps(result, indent=2)

            elif command == 'test_scan':
                result = self.tree_scanner.scan_tree()
                return json.dumps(result, indent=2)

            elif command == 'test_hcsr04':
                dist = self.sensors.last_hcsr04_distance
                tree = self.sensors.is_tree_on_side()
                return f"HC-SR04: {dist}cm, Tree detected: {tree}"

            # HC-SR04 Watch Mode - continuous monitoring for tree detection testing
            elif command == 'hcsr04_watch_start':
                if hasattr(self, '_hcsr04_watching') and self._hcsr04_watching:
                    return "WATCH_ALREADY_ACTIVE"

                self._hcsr04_watching = True
                self._hcsr04_watch_status = "WATCHING"
                self._hcsr04_last_distance = None
                self._hcsr04_tree_count = 0

                def watch_loop():
                    print("[HC-SR04 Watch] Started - waiting for trees...")
                    print(f"  Detection range: {HCSR04_TREE_DETECT_MIN_CM}-{HCSR04_TREE_DETECT_MAX_CM}cm")
                    last_tree_time = 0

                    while self._hcsr04_watching:
                        # Read HC-SR04 directly for fastest response
                        dist = self.sensors.read_hcsr04()
                        self._hcsr04_last_distance = dist

                        if dist is None:
                            self._hcsr04_watch_status = "NO_SIGNAL"
                        elif dist < HCSR04_TREE_DETECT_MIN_CM:
                            self._hcsr04_watch_status = f"TOO_CLOSE:{int(dist)}"
                        elif dist > HCSR04_TREE_DETECT_MAX_CM:
                            self._hcsr04_watch_status = f"NO_TREE:{int(dist)}"
                        else:
                            # Tree detected!
                            current_time = time.time()
                            if current_time - last_tree_time > 1.5:  # Debounce 1.5s for new tree
                                self._hcsr04_tree_count += 1
                                last_tree_time = current_time
                                print(f"[HC-SR04 Watch] TREE #{self._hcsr04_tree_count} DETECTED at {dist:.1f}cm!")
                            self._hcsr04_watch_status = f"TREE_DETECTED:{int(dist)}"

                        time.sleep(0.02)  # 50Hz - as fast as HC-SR04 can reliably read

                    print("[HC-SR04 Watch] Stopped")

                threading.Thread(target=watch_loop, daemon=True).start()
                return "OK: HC-SR04 watch mode started"

            elif command == 'hcsr04_watch_stop':
                self._hcsr04_watching = False
                return f"OK: Watch stopped. Trees detected: {getattr(self, '_hcsr04_tree_count', 0)}"

            elif command == 'hcsr04_watch_status':
                if not getattr(self, '_hcsr04_watching', False):
                    return "WATCH_INACTIVE"
                status = getattr(self, '_hcsr04_watch_status', 'UNKNOWN')
                count = getattr(self, '_hcsr04_tree_count', 0)
                return f"{status}|COUNT:{count}"

            elif command == 'test_row_detect':
                # Test row wall detection (vanishing point)
                result = self.row_navigator.test_row_detection()
                if result.get('found'):
                    vp = result.get('vanishing_point', (0, 0))
                    angle = result.get('row_angle', 0)
                    offset = result.get('offset_from_center', 0)
                    lines = result.get('line_count', 0)
                    correction = result.get('steering_correction', 0)
                    rec = result.get('recommendation', 'UNKNOWN')
                    return (f"ROW DETECTED!\n"
                            f"  Vanishing point: {vp}\n"
                            f"  Row angle: {angle:.1f}°\n"
                            f"  Offset: {offset:.0f}px\n"
                            f"  Lines found: {lines}\n"
                            f"  Steering: {correction:.2f}\n"
                            f"  Action: {rec}\n"
                            f"  Debug image: {result.get('debug_image', 'N/A')}")
                else:
                    return f"NO ROW DETECTED: {result.get('reason', result.get('error', 'unknown'))}"

            # Step 7: Test row wall detection (visual guidance)
            elif command == 'step7_row_wall':
                def run_step7():
                    print("[Step 7] Testing row wall detection...")
                    self.servos.center()
                    time.sleep(0.5)

                    # Run detection multiple times
                    for i in range(5):
                        result = self.row_navigator.test_row_detection()
                        if result.get('found'):
                            print(f"[Step 7] Detection {i+1}: FOUND")
                            print(f"         Vanishing point: {result.get('vanishing_point')}")
                            print(f"         Offset: {result.get('offset_from_center', 0):.0f}px")
                            print(f"         Steering: {result.get('steering_correction', 0):.2f}")
                            print(f"         Recommendation: {result.get('recommendation')}")
                        else:
                            print(f"[Step 7] Detection {i+1}: NOT FOUND - {result.get('reason', 'unknown')}")
                        time.sleep(0.5)

                    print("[Step 7] Row detection test complete")

                threading.Thread(target=run_step7, daemon=True).start()
                return "OK: Step 7 - Testing row wall detection (5 samples)..."

            # === MANUAL STEP COMMANDS (for testing each step individually) ===
            # Full flow: orient_to_start → goto_start → orient_to_end → navigate_row (find trees)

            # Step 0: Orient to start waypoint (turn to face start before driving)
            elif command == 'step0_orient_to_start':
                row_id = int(args[0]) if args else 1
                row = next((r for r in self.rows if r['id'] == row_id), None)
                if not row:
                    return f"ERROR: Row {row_id} not found. Use set_row_start, set_row_end, save_row first."

                self.row_navigator.set_row(row['start'], row['end'])

                def run_step0():
                    result = self.row_navigator.orient_to_start()
                    print(f"[Step 0] Orient to start result: {result}")

                threading.Thread(target=run_step0, daemon=True).start()
                return f"OK: Step 0 - Orienting to face start waypoint of row {row_id}..."

            # Step 1: Go to start waypoint (drives with dynamic orientation)
            elif command == 'step1_goto_start':
                row_id = int(args[0]) if args else 1
                row = next((r for r in self.rows if r['id'] == row_id), None)
                if not row:
                    return f"ERROR: Row {row_id} not found. Use set_row_start, set_row_end, save_row first."

                self.row_navigator.set_row(row['start'], row['end'])

                def run_step1():
                    result = self.row_navigator.navigate_to_start()
                    print(f"[Step 1] Navigate to start result: {result}")

                threading.Thread(target=run_step1, daemon=True).start()
                return f"OK: Step 1 - Navigating to start of row {row_id}..."

            # Step 2: Orient to end waypoint (turn to face row direction)
            elif command == 'step2_orient':
                if not self.row_navigator.target_heading:
                    return "ERROR: Run step1 first or set row with set_row_start/end"

                def run_step2():
                    result = self.row_navigator.orient_to_end()
                    print(f"[Step 2] Orient to end result: {result}")

                threading.Thread(target=run_step2, daemon=True).start()
                return f"OK: Step 2 - Orienting to end waypoint (heading {self.row_navigator.target_heading:.1f}°)..."

            # Step 3: Go straight until HC-SR04 detects tree, stop, turn servo to side
            elif command == 'step3_find_tree':
                def run_step3():
                    print("[Step 3] Moving forward, watching for tree on HC-SR04...")
                    self.servos.center()  # Start looking forward

                    max_time = 60  # 60 second timeout
                    start_time = time.time()

                    while time.time() - start_time < max_time:
                        self.sensors.update_all()

                        # Check front obstacle first
                        front_dist = self.sensors.last_distance
                        if front_dist and front_dist < OBSTACLE_STOP_DISTANCE_MM:
                            self.motors.stop()
                            print(f"[Step 3] OBSTACLE! Front distance: {front_dist}mm")
                            return

                        # Check side HC-SR04 for tree
                        side_dist = self.sensors.last_hcsr04_distance
                        if side_dist and HCSR04_TREE_DETECT_MIN_CM < side_dist < HCSR04_TREE_DETECT_MAX_CM:
                            self.motors.stop()
                            print(f"[Step 3] TREE DETECTED at {side_dist}cm!")
                            print("[Step 3] Turning servo to look at tree...")
                            time.sleep(0.3)
                            self.servos.look_left()  # Turn servo 90° to left where tree is
                            self.servos.look_straight()  # Tilt straight
                            print("[Step 3] Ready for YOLO verification (step4)")
                            return

                        # Keep moving forward slowly
                        self.motors.forward(NAV_BASE_SPEED // 2)
                        time.sleep(0.05)

                    self.motors.stop()
                    print("[Step 3] Timeout - no tree found")

                threading.Thread(target=run_step3, daemon=True).start()
                return "OK: Step 3 - Moving forward, watching for tree..."

            # Step 4: YOLO verify tree
            elif command == 'step4_verify_yolo':
                frame = self.camera.capture_csi_frame()
                if frame is None:
                    return "ERROR: Camera capture failed"

                if not self.yolo or not self.yolo.is_available():
                    return "ERROR: YOLO not available"

                result = self.yolo.verify_tree(frame)
                if result['verified']:
                    return f"OK: TREE VERIFIED! Class: {result['detection']['class']}, Confidence: {result['confidence']:.2f}"
                else:
                    return f"NOT A TREE: {result.get('reason', 'unknown')} - May be a post or other object"

            # Step 5: Follow trunk down + thermal water test
            elif command == 'step5_thermal_test':
                def run_step5():
                    print("[Step 5] Following trunk down to base...")

                    # Follow trunk down
                    base_result = self.trunk_follower.follow_down_to_base()
                    if not base_result['success']:
                        print(f"[Step 5] Could not find trunk base: {base_result.get('reason', 'unknown')}")
                        print("[Step 5] Proceeding with thermal anyway at current position...")

                    # Position for thermal scan
                    print("[Step 5] Positioning for thermal scan...")
                    self.servos.look_ground()
                    time.sleep(0.5)

                    # Thermal water test
                    print("[Step 5] Running thermal water detection...")
                    water_result = self.water_detector.analyze_ground()

                    if water_result.get('has_water'):
                        print(f"[Step 5] WATER DETECTED!")
                        print(f"         Water %: {water_result['water_percentage']:.1f}%")
                        print(f"         Confidence: {water_result['confidence']}")
                    else:
                        print(f"[Step 5] DRY SOIL")
                        print(f"         Water %: {water_result.get('water_percentage', 0):.1f}%")

                    print(f"         Temp range: {water_result.get('min_temp_c', 0):.1f}°C - {water_result.get('max_temp_c', 0):.1f}°C")

                threading.Thread(target=run_step5, daemon=True).start()
                return "OK: Step 5 - Following trunk down and running thermal test..."

            # Step 6: Follow trunk up + canopy photo
            elif command == 'step6_canopy_photo':
                def run_step6():
                    print("[Step 6] Following trunk up to canopy...")

                    # Follow trunk up
                    canopy_result = self.trunk_follower.follow_up_to_canopy()
                    print(f"[Step 6] Canopy result: green={canopy_result.get('green_percentage', 0):.1f}%")

                    # Take photo
                    print("[Step 6] Capturing canopy photo...")
                    timestamp = int(time.time())
                    image_path = f"{CANOPY_IMAGES_DIR}/manual_canopy_{timestamp}.jpg"

                    if self.camera.save_image(image_path, use_thermal=False):
                        print(f"[Step 6] Photo saved: {image_path}")
                    else:
                        print("[Step 6] Failed to save photo")

                    # Return to center
                    self.servos.center()
                    print("[Step 6] Complete!")

                threading.Thread(target=run_step6, daemon=True).start()
                return "OK: Step 6 - Following trunk up and taking canopy photo..."

            # Full auto sequence (all steps)
            elif command == 'step_auto':
                row_id = int(args[0]) if args else 1
                row = next((r for r in self.rows if r['id'] == row_id), None)
                if not row:
                    return f"ERROR: Row {row_id} not found"

                self.row_navigator.set_row(row['start'], row['end'])

                def tree_callback():
                    result = self.tree_scanner.scan_tree()
                    self.inspection_log.append(result)

                def run_auto():
                    print("[AUTO] Starting full autonomous sequence...")

                    # Step 1: Go to start
                    print("[AUTO] Step 1: Navigating to start...")
                    result = self.row_navigator.navigate_to_start()
                    if not result['success']:
                        print(f"[AUTO] Failed at step 1: {result}")
                        return

                    # Step 2: Orient
                    print("[AUTO] Step 2: Orienting to end...")
                    result = self.row_navigator.orient_to_end()
                    if not result['success']:
                        print(f"[AUTO] Failed at step 2: {result}")
                        return

                    # Steps 3-6: Navigate row (handles tree detection and scanning)
                    print("[AUTO] Steps 3-6: Navigating row...")
                    result = self.row_navigator.navigate_row(tree_callback=tree_callback)
                    print(f"[AUTO] Complete! Trees found: {result['trees_found']}")

                threading.Thread(target=run_auto, daemon=True).start()
                return f"OK: Starting FULL AUTO sequence on row {row_id}..."

            # =================================================================
            # TEST MODE 1: Compass Only (no visual, no strafe)
            # =================================================================
            elif command == 'test_mode_compass':
                row_id = int(args[0]) if args else 1
                row = next((r for r in self.rows if r['id'] == row_id), None)
                if not row:
                    return f"ERROR: Row {row_id} not found"

                self.row_navigator.set_row(row['start'], row['end'])

                def run_compass_only():
                    print("=" * 50)
                    print("[TEST MODE 1] COMPASS ONLY")
                    print("=" * 50)
                    print(f"  Row {row_id}: {row['start']} -> {row['end']}")
                    print("  Features: Compass heading ONLY")
                    print("  NO visual row detection")
                    print("  NO HC-SR04 strafe")
                    print("=" * 50)

                    # Go to start
                    print("[Mode 1] Going to START...")
                    result = self.row_navigator.navigate_to_start()
                    if not result['success']:
                        print(f"[Mode 1] Failed: {result.get('reason')}")
                        return

                    # Orient
                    print("[Mode 1] Orienting to END...")
                    result = self.row_navigator.orient_to_end()
                    if not result['success']:
                        print(f"[Mode 1] Failed: {result.get('reason')}")
                        return

                    print(f"[Mode 1] Heading: {result.get('heading', 0):.1f}°")
                    print("[Mode 1] Driving to END (compass only)...")

                    self.row_navigator.navigating = True
                    while self.row_navigator.navigating:
                        self.sensors.update_all()

                        # Check if reached end
                        distance = self.row_navigator._distance_to_gps(row['end'])
                        if distance < NAV_WAYPOINT_RADIUS_M:
                            self.motors.stop()
                            print(f"[Mode 1] REACHED END! ({distance:.1f}m)")
                            break

                        # Obstacle stop
                        if self.sensors.last_distance and self.sensors.last_distance < OBSTACLE_STOP_DISTANCE_MM:
                            self.motors.stop()
                            print(f"[Mode 1] OBSTACLE at {self.sensors.last_distance}mm")
                            time.sleep(1)
                            continue

                        # Compass steering only
                        heading = self.sensors.last_heading
                        target = self.row_navigator.target_heading
                        if heading is not None and target is not None:
                            error = target - heading
                            while error > 180: error -= 360
                            while error < -180: error += 360

                            if abs(error) < NAV_HEADING_TOLERANCE_DEG:
                                self.motors.forward(NAV_BASE_SPEED)
                            elif error > 0:
                                self.motors.curve_right(NAV_BASE_SPEED, 0.7)
                            else:
                                self.motors.curve_left(NAV_BASE_SPEED, 0.7)
                        else:
                            self.motors.forward(NAV_BASE_SPEED // 2)

                        time.sleep(0.1)

                    self.motors.stop()
                    self.row_navigator.navigating = False
                    print("[Mode 1] COMPLETE!")

                threading.Thread(target=run_compass_only, daemon=True).start()
                return f"OK: TEST MODE 1 - Compass Only (Row {row_id})"

            # =================================================================
            # TEST MODE 2: Compass + Visual Row (no strafe)
            # =================================================================
            elif command == 'test_mode_visual':
                row_id = int(args[0]) if args else 1
                row = next((r for r in self.rows if r['id'] == row_id), None)
                if not row:
                    return f"ERROR: Row {row_id} not found"

                self.row_navigator.set_row(row['start'], row['end'])

                def run_compass_visual():
                    print("=" * 50)
                    print("[TEST MODE 2] COMPASS + VISUAL ROW")
                    print("=" * 50)
                    print(f"  Row {row_id}: {row['start']} -> {row['end']}")
                    print("  Features: Compass + Visual row wall detection")
                    print("  NO HC-SR04 strafe")
                    print("=" * 50)

                    # Go to start
                    print("[Mode 2] Going to START...")
                    result = self.row_navigator.navigate_to_start()
                    if not result['success']:
                        print(f"[Mode 2] Failed: {result.get('reason')}")
                        return

                    # Orient
                    print("[Mode 2] Orienting to END...")
                    result = self.row_navigator.orient_to_end()
                    if not result['success']:
                        print(f"[Mode 2] Failed: {result.get('reason')}")
                        return

                    print(f"[Mode 2] Heading: {result.get('heading', 0):.1f}°")
                    print("[Mode 2] Driving to END (compass + visual)...")

                    self.row_navigator.navigating = True
                    while self.row_navigator.navigating:
                        self.sensors.update_all()

                        # Check if reached end
                        distance = self.row_navigator._distance_to_gps(row['end'])
                        if distance < NAV_WAYPOINT_RADIUS_M:
                            self.motors.stop()
                            print(f"[Mode 2] REACHED END! ({distance:.1f}m)")
                            break

                        # Obstacle stop
                        if self.sensors.last_distance and self.sensors.last_distance < OBSTACLE_STOP_DISTANCE_MM:
                            self.motors.stop()
                            print(f"[Mode 2] OBSTACLE at {self.sensors.last_distance}mm")
                            time.sleep(1)
                            continue

                        # Try visual guidance first
                        visual_correction = 0.0
                        if self.camera and self.row_detector:
                            frame = self.camera.capture_csi_frame()
                            if frame is not None:
                                result = self.row_detector.detect_rows(frame)
                                if result.get('found'):
                                    visual_correction = self.row_detector.get_steering_correction(frame)
                                    # print(f"[Mode 2] Visual correction: {visual_correction:.2f}")

                        # Compass correction
                        compass_correction = 0.0
                        heading = self.sensors.last_heading
                        target = self.row_navigator.target_heading
                        if heading is not None and target is not None:
                            error = target - heading
                            while error > 180: error -= 360
                            while error < -180: error += 360
                            compass_correction = max(-1, min(1, error / 45.0))

                        # Blend: 70% visual, 30% compass (if visual available)
                        if abs(visual_correction) > 0.01:
                            correction = 0.7 * visual_correction + 0.3 * compass_correction
                        else:
                            correction = compass_correction

                        # Apply steering
                        if abs(correction) < 0.1:
                            self.motors.forward(NAV_BASE_SPEED)
                        elif correction > 0:
                            self.motors.curve_right(NAV_BASE_SPEED, 0.7)
                        else:
                            self.motors.curve_left(NAV_BASE_SPEED, 0.7)

                        time.sleep(0.1)

                    self.motors.stop()
                    self.row_navigator.navigating = False
                    print("[Mode 2] COMPLETE!")

                threading.Thread(target=run_compass_visual, daemon=True).start()
                return f"OK: TEST MODE 2 - Compass + Visual (Row {row_id})"

            # =================================================================
            # TEST MODE 3: Compass + Visual + Strafe (full, no tree scan)
            # =================================================================
            elif command == 'test_mode_strafe':
                row_id = int(args[0]) if args else 1
                row = next((r for r in self.rows if r['id'] == row_id), None)
                if not row:
                    return f"ERROR: Row {row_id} not found"

                self.row_navigator.set_row(row['start'], row['end'])

                def run_full_strafe():
                    print("=" * 50)
                    print("[TEST MODE 3] COMPASS + VISUAL + STRAFE")
                    print("=" * 50)
                    print(f"  Row {row_id}: {row['start']} -> {row['end']}")
                    print("  Features: Compass + Visual + HC-SR04 strafe")
                    print(f"  Target distance: {ROW_TARGET_DISTANCE_CM}cm")
                    print("  NO tree scanning")
                    print("=" * 50)

                    # Go to start
                    print("[Mode 3] Going to START...")
                    result = self.row_navigator.navigate_to_start()
                    if not result['success']:
                        print(f"[Mode 3] Failed: {result.get('reason')}")
                        return

                    # Orient
                    print("[Mode 3] Orienting to END...")
                    result = self.row_navigator.orient_to_end()
                    if not result['success']:
                        print(f"[Mode 3] Failed: {result.get('reason')}")
                        return

                    print(f"[Mode 3] Heading: {result.get('heading', 0):.1f}°")
                    print("[Mode 3] Driving to END (compass + visual + strafe)...")

                    # Use navigate_row WITHOUT tree callback
                    result = self.row_navigator.navigate_row(
                        tree_callback=None,  # NO tree scanning
                        progress_callback=lambda d: None
                    )
                    print(f"[Mode 3] COMPLETE! Result: {result}")

                threading.Thread(target=run_full_strafe, daemon=True).start()
                return f"OK: TEST MODE 3 - Compass + Visual + Strafe (Row {row_id})"

            # === COMPASS CALIBRATION ===
            elif command == 'save_compass_cal':
                if self.sensors.compass.save_calibration():
                    return "OK: Compass calibration saved!"
                else:
                    cal = self.sensors.compass.get_calibration_status()
                    return f"ERROR: Not fully calibrated (need sys=3, mag=3). Current: sys={cal['sys']}, mag={cal['mag']}"

            # === STATUS ===
            elif command == 'status':
                status = {
                    'tree_type': TREE_TYPE,
                    'gps': self.sensors.last_position,
                    'heading': self.sensors.last_heading,
                    'compass_calibration': self.sensors.compass.get_calibration_status(),
                    'front_distance': self.sensors.last_distance,
                    'side_distance': self.sensors.last_hcsr04_distance,
                    'navigating': self.row_navigator.navigating,
                    'rows_saved': len(self.rows),
                    'trees_scanned': self.tree_scanner.scan_count,
                    'yolo_available': self.yolo.is_available()
                }
                return json.dumps(status, indent=2)

            elif command == 'help':
                return """Commands:
  Movement: forward, backward, left, right, stop
  Servo: servo <pan> <tilt>, center, look_left, look_ground, look_canopy
  Sensors: sensors, distance, gps, heading
  Rows: set_row_start, set_row_end, save_row, list_rows
  Navigation: navigate_row <id>, stop_nav, pause_nav, resume_nav
  Tests: test_row_detect, test_yolo, test_trunk_down, test_trunk_up, test_thermal, test_scan, test_hcsr04
  Status: status, help"""

            else:
                return f"ERROR: Unknown command: {command}"

        except Exception as e:
            return f"ERROR: {str(e)}"

    def cleanup(self):
        """Shutdown cleanup"""
        print("\n[Robot] Shutting down...")
        self.running = False
        self.motors.stop()
        self.servos.center()
        self.command_server.stop()
        self.stream_server.stop()
        self.camera.cleanup()
        self.sensors.cleanup()
        print("[Robot] Shutdown complete")


def signal_handler(sig, frame):
    """Handle Ctrl+C"""
    print("\n[Signal] Interrupt received")
    if 'robot' in globals():
        robot.cleanup()
    sys.exit(0)


def main():
    global robot

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    robot = VineyardRobot()
    robot.start()

    print("\nRobot running. Press Ctrl+C to stop.")
    print(f"Connect via: telnet <raspberry_pi_ip> {COMMAND_PORT}")
    print(f"Video stream: http://<raspberry_pi_ip>:{STREAM_PORT}")
    print("\n--- Sensor readings (every 3 seconds) ---\n")

    try:
        while robot.running:
            # Print sensor status every 3 seconds
            status = robot.sensors.get_status()

            # GPS
            gps = status.get('gps_position')
            gps_qual = status.get('gps_quality', {})
            if gps:
                print(f"GPS: {gps['latitude']:.6f}, {gps['longitude']:.6f} | Sats: {gps_qual.get('satellites', 0)}, HDOP: {gps_qual.get('hdop', 99):.1f}")
            else:
                print(f"GPS: No fix | Sats: {gps_qual.get('satellites', 0)}")

            # Compass
            heading = status.get('compass_heading')
            cal = status.get('compass_calibration', {})
            if heading is not None:
                print(f"Compass: {heading:.1f}° | Cal: sys={cal.get('sys',0)} gyro={cal.get('gyro',0)} accel={cal.get('accel',0)} mag={cal.get('mag',0)}")
            else:
                print("Compass: No data")

            # Ultrasonic sensors
            front_mm = status.get('ultrasonic_distance_mm')
            hcsr04_cm = status.get('hcsr04_distance_cm')
            print(f"Ultrasonic Front: {front_mm}mm | HC-SR04: {hcsr04_cm}cm")

            print("-" * 50)
            time.sleep(3)
    except KeyboardInterrupt:
        pass

    robot.cleanup()


if __name__ == '__main__':
    main()
