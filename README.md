# Autonomous Tree-Crop Inspection Robot

An autonomous robot system for vineyard and orchard health/irrigation monitoring, designed to reduce manual inspection labor.

**Author:** Janno Louwrens
**Created:** 2025
**Status:** Proprietary - Technical demonstration

## Overview

This robot autonomously navigates vineyard/orchard rows, scanning individual trees for health indicators using computer vision and thermal imaging. Designed to operate on Raspberry Pi with real-time sensor fusion and remote monitoring capabilities.

## Features

### Navigation
- **GPS Waypoint Navigation** - NEO-8M GPS for autonomous row traversal
- **BNO055 Compass** - 9-DOF IMU for heading and orientation
- **Row Detection** - Visual row centering using Hough transform vanishing points
- **Obstacle Avoidance** - Ultrasonic sensors prevent collisions

### Computer Vision
- **YOLOv8 Object Detection** - Custom-trained for tree trunk detection
- **Thermal Analysis** - MI48 thermal camera for irrigation monitoring
- **HSV Color Analysis** - Vineyard health assessment
- **Video Streaming** - Real-time MJPEG stream to operator station

### Sensor Fusion
- GPS + compass + ultrasonic + thermal + camera data combined
- Statistical analysis of temperature patterns (water detection)
- PID-like steering corrections for row following

### Network Architecture
- **TCP Command Server** - Remote control interface
- **MJPEG Streaming** - Live video feed
- **Telemetry Broadcasting** - Real-time sensor data

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Raspberry Pi 4                                    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐                │
│  │   GPS        │   │   BNO055     │   │   Camera     │                │
│  │  (UART)      │   │   (I2C)      │   │   (CSI)      │                │
│  └──────┬───────┘   └──────┬───────┘   └──────┬───────┘                │
│         │                  │                  │                          │
│         ▼                  ▼                  ▼                          │
│  ┌───────────────────────────────────────────────────────┐              │
│  │                  Sensor Fusion Layer                   │              │
│  │     (navigation/gps.py, navigation/compass.py)        │              │
│  └───────────────────────┬───────────────────────────────┘              │
│                          │                                               │
│                          ▼                                               │
│  ┌───────────────────────────────────────────────────────┐              │
│  │                  Navigation Engine                     │              │
│  │     Waypoint → Row Detection → Steering Control       │              │
│  └───────────────────────┬───────────────────────────────┘              │
│                          │                                               │
│         ┌────────────────┼────────────────┐                             │
│         ▼                ▼                ▼                             │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐                    │
│  │   YOLOv8     │ │   Thermal    │ │   Motor      │                    │
│  │   Vision     │ │   Analysis   │ │   Control    │                    │
│  └──────────────┘ └──────────────┘ └──────────────┘                    │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
                              │
                              ▼ (WiFi)
┌─────────────────────────────────────────────────────────────────────────┐
│                      Laptop Controller                                   │
│     - Remote control GUI                                                │
│     - Live video display                                                │
│     - Sensor telemetry dashboard                                        │
│     - Mission planning                                                  │
└─────────────────────────────────────────────────────────────────────────┘
```

## Project Structure

```
autonomous-robot/
├── main.py                  # Robot main loop (runs on Pi)
├── laptop_controller.py     # Remote control GUI
├── config.py                # All hardware configuration
├── core/                    # Core robot functionality
├── navigation/              # GPS, compass, waypoint logic
│   ├── gps.py
│   ├── compass.py
│   └── waypoints.py
├── vision/                  # Computer vision modules
│   ├── yolo_detector.py
│   ├── row_detection.py
│   └── thermal_analysis.py
├── network/                 # TCP server, streaming
├── analysis/                # Data processing
├── training/                # YOLO training scripts
├── models/                  # Trained models
└── data/                    # Collected data
```

## Hardware Requirements

| Component | Model | Purpose |
|-----------|-------|---------|
| Computer | Raspberry Pi 4 (4GB+) | Main controller |
| GPS | NEO-8M | Waypoint navigation |
| IMU | BNO055 | Compass + orientation |
| Camera | Pi Camera v2 | Object detection |
| Thermal | MI48 | Temperature analysis |
| Ultrasonic | HC-SR04 | Distance sensing |
| Motors | Mecanum wheels | Omnidirectional movement |
| Driver | Custom I2C driver | Motor control |

## Configuration

Key parameters in `config.py`:

```python
# Tree type selection
TREE_TYPE = 'vineyard'  # 'vineyard', 'orange', 'lemon', 'olive', 'apple'

# Navigation
NAV_BASE_SPEED = 80
NAV_WAYPOINT_RADIUS_M = 1.5
NAV_HEADING_TOLERANCE_DEG = 10

# Row following
ROW_TARGET_DISTANCE_CM = 80
ROW_DISTANCE_TOLERANCE_CM = 20

# Network
COMMAND_PORT = 8890
STREAM_PORT = 8891
```

## Installation

### Robot (Raspberry Pi)

```bash
# Clone repository
git clone https://github.com/JannoLouwrens/autonomous-vineyard-robot.git
cd autonomous-vineyard-robot

# Install dependencies
pip3 install -r requirements.txt

# Uncomment Pi-specific packages in requirements.txt:
# RPi.GPIO, smbus2, spidev, pynmea2, picamera2
```

### Laptop Controller (Windows/macOS/Linux)

**Windows:**
```powershell
git clone https://github.com/JannoLouwrens/autonomous-vineyard-robot.git
cd autonomous-vineyard-robot

python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

**macOS / Linux:**
```bash
git clone https://github.com/JannoLouwrens/autonomous-vineyard-robot.git
cd autonomous-vineyard-robot

python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Usage

### On Robot (Raspberry Pi)

```bash
# Install as systemd service
sudo cp vineyard_robot.service /etc/systemd/system/
sudo systemctl enable vineyard_robot
sudo systemctl start vineyard_robot

# Or run directly
python3 main.py
```

### Remote Control (Laptop)

**Windows:**
```powershell
python laptop_controller.py
```

**macOS / Linux:**
```bash
python3 laptop_controller.py
```

## Technical Highlights

### Row Detection Algorithm
Uses Hough transform to detect vineyard row lines, then calculates vanishing point for centering:

1. Edge detection (Canny)
2. Line detection (HoughLinesP)
3. Filter by angle (near-vertical lines)
4. Calculate intersection point
5. PID-style correction to center

### Thermal Water Detection
Analyzes ground temperature patterns to detect irrigation issues:
- Ground temperature vs ambient
- Statistical variance analysis
- Wet vs dry soil classification

### YOLO Training Pipeline
Custom YOLOv8 nano model trained on vineyard/orchard images:
- Active learning reduced labeling 92.5%
- 640px input, <20MB model size
- Real-time inference on Pi 4

## License

Proprietary - Technical demonstration only.

## Contact

For technical details or demonstration requests, contact the author.
