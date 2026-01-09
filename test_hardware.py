#!/usr/bin/env python3
"""
Hardware Test Script
Run individual tests on robot components
"""

import sys
import time
sys.path.append('/home/pi/Working Code/Updated FULL Code Standalone')


def test_motors():
    """Test motor control"""
    print("\n=== MOTOR TEST ===")
    from core.motors import MotorController

    motors = MotorController()

    print("Forward 2s...")
    motors.forward(80)
    time.sleep(2)

    print("Backward 2s...")
    motors.backward(80)
    time.sleep(2)

    print("Turn left 1s...")
    motors.turn_left(60)
    time.sleep(1)

    print("Turn right 1s...")
    motors.turn_right(60)
    time.sleep(1)

    motors.stop()
    print("Motors OK")


def test_servos():
    """Test servo control"""
    print("\n=== SERVO TEST ===")
    from core.motors import Raspbot
    from core.servos import ServoController

    raspbot = Raspbot()
    servos = ServoController(raspbot)

    print("Center...")
    servos.center()
    time.sleep(1)

    print("Look left...")
    servos.look_left()
    time.sleep(1)

    print("Look right...")
    servos.set_pan(135)
    time.sleep(1)

    print("Look ground...")
    servos.center()
    servos.look_ground()
    time.sleep(1)

    print("Look canopy...")
    servos.look_canopy()
    time.sleep(1)

    servos.center()
    print("Servos OK")


def test_sensors():
    """Test all sensors"""
    print("\n=== SENSOR TEST ===")
    from core.motors import Raspbot
    from core.sensors import SensorManager

    raspbot = Raspbot()
    sensors = SensorManager(raspbot)

    for _ in range(5):
        sensors.update_all()

        print(f"\nHeading: {sensors.last_heading}")
        print(f"GPS: {sensors.last_position}")
        print(f"Front ultrasonic: {sensors.last_distance}mm")
        print(f"Side HC-SR04: {sensors.last_hcsr04_distance} cm" if sensors.last_hcsr04_distance else "Side HC-SR04: N/A (disabled)")
        print(f"Tree detected: {sensors.is_tree_on_side()}")

        time.sleep(1)

    sensors.cleanup()
    print("\nSensors OK")


def test_camera():
    """Test camera capture"""
    print("\n=== CAMERA TEST ===")
    from core.cameras import CameraManager
    import cv2

    camera = CameraManager()

    print("Capturing CSI frame...")
    frame = camera.capture_csi_frame()
    if frame is not None:
        cv2.imwrite('/tmp/test_csi.jpg', frame)
        print(f"  Saved /tmp/test_csi.jpg ({frame.shape})")
    else:
        print("  CSI capture failed!")

    print("Capturing thermal...")
    if camera.is_thermal_available():
        temp = camera.capture_thermal_array()
        if temp is not None:
            import numpy as np
            print(f"  Thermal: min={np.min(temp):.1f}C, max={np.max(temp):.1f}C, mean={np.mean(temp):.1f}C")
        else:
            print("  Thermal capture failed!")
    else:
        print("  Thermal not available")

    camera.cleanup()
    print("Camera OK")


def test_hcsr04():
    """Test HC-SR04 specifically"""
    print("\n=== HC-SR04 TEST ===")
    from core.sensors import HCSR04Sensor

    sensor = HCSR04Sensor()

    print("Reading HC-SR04 distance (10 readings)...")
    for i in range(10):
        dist = sensor.read_distance()
        tree = sensor.is_tree_detected()
        print(f"  {i+1}: {dist}cm - Tree: {tree}")
        time.sleep(0.5)

    sensor.cleanup()
    print("HC-SR04 OK")


def test_row_detection():
    """Test row detection vision"""
    print("\n=== ROW DETECTION TEST ===")
    from core.cameras import CameraManager
    from vision.row_detector import RowDetector
    import cv2

    camera = CameraManager()
    detector = RowDetector()

    print("Capturing frame...")
    frame = camera.capture_csi_frame()
    if frame is None:
        print("Camera failed!")
        return

    print("Detecting rows...")
    result = detector.detect_rows(frame)
    print(f"  Found: {result['found']}")
    if result['found']:
        print(f"  Vanishing point: {result['vanishing_point']}")
        print(f"  Row angle: {result['row_angle']:.1f}°")
        print(f"  Lines found: {result['line_count']}")

    print("Saving visualization...")
    viz = detector.visualize(frame)
    cv2.imwrite('/tmp/test_row_detect.jpg', viz)
    print("  Saved /tmp/test_row_detect.jpg")

    camera.cleanup()
    print("Row detection OK")


def test_water_detection():
    """Test thermal water detection"""
    print("\n=== WATER DETECTION TEST ===")
    from core.motors import Raspbot
    from core.cameras import CameraManager
    from core.servos import ServoController
    from analysis.water_detector import WaterDetector

    raspbot = Raspbot()
    camera = CameraManager()
    servos = ServoController(raspbot)
    detector = WaterDetector(camera, servos)

    print("Analyzing ground for water...")
    result = detector.analyze_ground()

    print(f"\nResult:")
    print(f"  Water detected: {result.get('has_water', False)}")
    print(f"  Water percentage: {result.get('water_percentage', 0):.1f}%")
    print(f"  Temperature delta: {result.get('temp_delta_c', 0):.1f}°C")
    print(f"  Confidence: {result.get('confidence', 'N/A')}")

    if 'error' in result:
        print(f"  Error: {result['error']}")

    camera.cleanup()
    print("\nWater detection OK")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Test robot hardware')
    parser.add_argument('test', nargs='?', default='all',
                       choices=['all', 'motors', 'servos', 'sensors', 'camera', 'hcsr04', 'row', 'water'],
                       help='Which test to run')

    args = parser.parse_args()

    tests = {
        'motors': test_motors,
        'servos': test_servos,
        'sensors': test_sensors,
        'camera': test_camera,
        'hcsr04': test_hcsr04,
        'row': test_row_detection,
        'water': test_water_detection
    }

    if args.test == 'all':
        for name, func in tests.items():
            try:
                func()
            except Exception as e:
                print(f"\n{name} test FAILED: {e}")
    else:
        tests[args.test]()

    print("\n=== TESTS COMPLETE ===")


if __name__ == '__main__':
    main()
