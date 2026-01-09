#!/usr/bin/env python3
"""
Vineyard Robot Configuration
All tunable parameters in one place
COPIED FROM WORKING CODE - DO NOT CHANGE HARDWARE ADDRESSES
"""

# ============================================================================
# TREE TYPE SELECTION - Choose what you're scanning
# ============================================================================
TREE_TYPE = 'vineyard'  # Options: 'vineyard', 'orange', 'lemon', 'olive', 'apple'

# ============================================================================
# HARDWARE ADDRESSES - FROM WORKING CODE
# ============================================================================
I2C_BUS = 1
MOTOR_DRIVER_ADDR = 0x2b
BNO055_ADDR = 0x29
THERMAL_I2C_ADDR = 0x40

# GPS Configuration
GPS_PORT = '/dev/ttyAMA0'
GPS_BAUD = 9600

# Thermal Camera SPI
THERMAL_SPI_BUS = 0
THERMAL_SPI_DEVICE = 1  # CE1 pin

# HC-SR04 Ultrasonic (left side - tree detection)
# Using 5V power - needs voltage divider on ECHO (two 1kΩ resistors)
HCSR04_TRIGGER_PIN = 20  # GPIO 20 = Physical Pin 38 (TRIG)
HCSR04_ECHO_PIN = 21     # GPIO 21 = Physical Pin 40 (ECHO)

# ============================================================================
# BNO055 COMPASS REGISTERS - FROM WORKING CODE
# ============================================================================
BNO055_PAGE_ID = 0x07
BNO055_CHIP_ID = 0x00
BNO055_PWR_MODE = 0x3E
BNO055_OPR_MODE = 0x3D
BNO055_SYS_TRIGGER = 0x3F
BNO055_CALIB_STAT = 0x35
BNO055_EULER_H_LSB = 0x1A
BNO055_EULER_H_MSB = 0x1B

# Operation Modes
OPERATION_MODE_CONFIG = 0x00
OPERATION_MODE_NDOF = 0x0C

# Compass Calibration - ADJUST THIS FOR YOUR MOUNTING
# This offset is ADDED to the raw heading to get the robot's true front direction
# How to calibrate:
#   1. Point robot's FRONT toward a known direction (e.g., a landmark)
#   2. Read raw compass heading from sensors display
#   3. Calculate: COMPASS_OFFSET = (desired heading) - (raw heading)
#   Example: Robot front points North (0°), compass reads 90° → OFFSET = -90
COMPASS_OFFSET_DEG = 0  # Set to 0, recalibrate by comparing phone compass to robot heading

# ============================================================================
# NETWORK PORTS
# ============================================================================
COMMAND_PORT = 8890
STREAM_PORT = 8891
DATA_STREAM_PORT = 8892

# ============================================================================
# NAVIGATION - GPS + Compass
# ============================================================================
NAV_BASE_SPEED = 80
NAV_TURN_SPEED = 40  # Slower turning for orient_to_end
NAV_OBSTACLE_THRESHOLD_MM = 300
NAV_WAYPOINT_RADIUS_M = 1.5          # GPS arrival tolerance (meters)
NAV_HEADING_TOLERANCE_DEG = 10
NAV_ROW_OFFSET_M = 1.0               # Distance to maintain from tree row

# ============================================================================
# HC-SR04 TREE DETECTION (Left Side Scanner)
# ============================================================================
HCSR04_ENABLED = True                # HC-SR04 connected: TRIG->GPIO20, ECHO->GPIO21
HCSR04_TREE_DETECT_MIN_CM = 30       # Closer than this = too close
HCSR04_TREE_DETECT_MAX_CM = 150      # Further than this = no tree
HCSR04_SCAN_INTERVAL_MS = 100

# Row Distance Maintenance (strafe to stay at target distance from trees)
ROW_TARGET_DISTANCE_CM = 80          # Ideal distance from tree row
ROW_DISTANCE_TOLERANCE_CM = 20       # +/- tolerance (60-100cm is "good zone")
ROW_STRAFE_SPEED = 50                # Speed for strafe corrections
ROW_SCAN_DISTANCE_CM = 50            # Move this close before scanning tree

# ============================================================================
# FRONT ULTRASONIC - Obstacle Avoidance
# ============================================================================
OBSTACLE_STOP_DISTANCE_MM = 200
OBSTACLE_SLOW_DISTANCE_MM = 500
OBSTACLE_AVOID_DISTANCE_MM = 400
OBSTACLE_AVOID_TURN_ANGLE = 30
OBSTACLE_AVOID_FORWARD_TIME = 0.5
OBSTACLE_CHECK_INTERVAL_MS = 50

# ============================================================================
# ROW DETECTION - Camera-based visual wall
# ============================================================================
ROW_DETECT_CANNY_LOW = 50
ROW_DETECT_CANNY_HIGH = 150
ROW_DETECT_HOUGH_THRESHOLD = 50
ROW_DETECT_MIN_LINE_LENGTH = 100
ROW_DETECT_MAX_LINE_GAP = 20
ROW_DETECT_ANGLE_MIN = 60
ROW_DETECT_ANGLE_MAX = 120
ROW_DETECT_VANISHING_TOLERANCE = 50

# ============================================================================
# VISION PARAMETERS - FROM WORKING CODE
# ============================================================================
USE_ENHANCED_DETECTOR = True

# Tree Detection - Color Method (HSV range for brown bark)
TREE_DETECT_LOWER_HSV = [10, 50, 20]
TREE_DETECT_UPPER_HSV = [30, 200, 200]

# Tree Detection - Shape Filters
TREE_DETECT_MIN_AREA = 200
TREE_DETECT_MIN_ASPECT_RATIO = 1.5

# Visual Servoing
VISUAL_SERVO_KP = 0.5
VISUAL_SERVO_CENTER_THRESHOLD = 30
VISUAL_SERVO_TARGET_DISTANCE_CM = 40
VISUAL_SERVO_BASE_SPEED = 60
VISUAL_SERVO_TARGET_BBOX_HEIGHT = 350

# ============================================================================
# LEAF DETECTION (CANOPY) - FROM WORKING CODE
# ============================================================================
LEAF_DETECT_LOWER_HSV = [35, 40, 40]
LEAF_DETECT_UPPER_HSV = [85, 255, 255]
LEAF_DETECT_MIN_AREA = 1000
LEAF_DETECT_MIN_PERCENTAGE = 2.0

# Canopy detection (same values)
CANOPY_HSV_LOWER = [35, 40, 40]
CANOPY_HSV_UPPER = [85, 255, 255]
CANOPY_MIN_PERCENTAGE = 15
CANOPY_MIN_AREA = 1000

# ============================================================================
# YOLO TREE CLASSIFIER
# ============================================================================
USE_YOLO_VERIFICATION = True
YOLO_MODEL_PATH = '/home/pi/Working Code/Updated FULL Code Standalone/models/tree_detector.pt'
YOLO_CONFIDENCE_THRESHOLD = 0.45
YOLO_IOU_THRESHOLD = 0.5
YOLO_INPUT_SIZE = 640

YOLO_CLASSES = {
    'vineyard': ['vineyard_trunk', 'vineyard_post', 'grape_canopy'],
    'orange': ['orange_trunk', 'orange_canopy'],
    'lemon': ['lemon_trunk', 'lemon_canopy'],
    'olive': ['olive_trunk', 'olive_canopy'],
    'apple': ['apple_trunk', 'apple_canopy']
}

# ============================================================================
# SERVO POSITIONS - FROM WORKING CODE
# ============================================================================
SERVO_CENTER = 90
SERVO_TILT_GROUND = 45      # Tilt DOWN to look at ground
SERVO_TILT_CANOPY = 100     # Tilt UP to look at canopy (max)
SERVO_TILT_STRAIGHT = 70    # Looking straight ahead (middle position)
SERVO_PAN_MIN = 0
SERVO_PAN_MAX = 180
SERVO_TILT_MIN = 0
SERVO_TILT_MAX = 100

SERVO_LOOK_LEFT = 0       # Full left (tree side)
SERVO_LOOK_RIGHT = 180    # Full right
TRUNK_FOLLOW_STEP = 5
TRUNK_FOLLOW_DELAY = 0.3
TRUNK_LOST_THRESHOLD = 3

# ============================================================================
# THERMAL WATER DETECTION - FROM WORKING CODE
# ============================================================================
THERMAL_STD_MULTIPLIER = 1.5
THERMAL_WATER_PERCENTAGE_THRESHOLD = 10.0
THERMAL_MIN_TEMP_DELTA = 3.0
THERMAL_CAPTURE_DELAY_SEC = 0.5

# Renamed for clarity
WATER_STD_MULTIPLIER = 1.5
WATER_MIN_PERCENTAGE = 10.0
WATER_MIN_TEMP_DELTA_C = 3.0
WATER_HIGH_CONFIDENCE_PCT = 20
WATER_HIGH_CONFIDENCE_DELTA = 5
WATER_MEDIUM_CONFIDENCE_PCT = 10
WATER_MEDIUM_CONFIDENCE_DELTA = 3

# ============================================================================
# CAMERA SETTINGS - FROM WORKING CODE
# ============================================================================
CAMERA_RESOLUTION = (640, 480)
CAMERA_FORMAT = "YUYV"
CAMERA_FPS = 30

# ============================================================================
# DATA STORAGE PATHS
# ============================================================================
DATA_DIR = '/home/pi/Working Code/Updated FULL Code Standalone/data'
ROWS_FILE = f'{DATA_DIR}/rows.json'
INSPECTION_LOG = f'{DATA_DIR}/inspection_log.json'
CANOPY_IMAGES_DIR = f'{DATA_DIR}/canopy_images'
SESSION_LOG_DIR = f'{DATA_DIR}/logs'

# Training data paths
TRAINING_DIR = '/home/pi/Working Code/Updated FULL Code Standalone/training'
VINEYARD_PHOTOS_DIR = f'{TRAINING_DIR}/VineyardPhotos'

# ============================================================================
# PERFORMANCE TUNING - FROM WORKING CODE
# ============================================================================
SENSOR_UPDATE_RATE_HZ = 10
VISION_PROCESS_RATE_HZ = 10
MAIN_LOOP_RATE_HZ = 20

# ============================================================================
# DEBUG
# ============================================================================
DEBUG_VISION = False
DEBUG_NAVIGATION = False
DEBUG_SENSORS = False
LOG_TO_FILE = True
LOG_FILE = f'{DATA_DIR}/robot.log'
