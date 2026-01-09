#!/usr/bin/env python3
"""
Camera Management - CSI/USB Camera (via OpenCV) + Thermal Camera
COPIED FROM WORKING CODE - Handles dual camera switching and frame capture.
Uses cv2.VideoCapture for the main camera to ensure broad compatibility.
"""

import io
import time
import threading
import cv2
import numpy as np
import sys

# Add senxor library path (must come AFTER Updated FULL Code path in main.py)
# Only add if not already present to avoid path order issues
if '/home/pi/Working Code' not in sys.path:
    sys.path.append('/home/pi/Working Code')  # Append (not insert) for senxor library

from config import *

# Imports required for the thermal camera API
import smbus
import spidev

try:
    from gpiozero import DigitalOutputDevice
    from senxor.interfaces import I2C_Interface, SPI_Interface
    from senxor.mi48 import MI48
    THERMAL_AVAILABLE = True
except ImportError:
    THERMAL_AVAILABLE = False
    print("[Camera] Thermal camera libraries not available")


class MI48_reset:
    """Helper class to manage hardware reset of the MI48 sensor - FROM WORKING CODE"""
    def __init__(self, pin, assert_seconds=0.001, deassert_seconds=0.1):
        self.pin = pin
        self.assert_time = assert_seconds
        self.deassert_time = deassert_seconds

    def __call__(self):
        print("[Thermal Camera] Performing hardware reset...")
        self.pin.on()
        time.sleep(self.assert_time)
        self.pin.off()
        time.sleep(self.deassert_time)
        print("[Thermal Camera] ✓ Hardware reset complete.")


class StreamingOutput(io.BufferedIOBase):
    """Buffer to hold the latest frame for MJPEG streaming."""
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class CameraManager:
    """
    Manages dual camera system - FROM WORKING CODE
    """

    def __init__(self):
        print("\n=== Initializing Cameras ===")
        self.active_camera = "csi"
        self.streaming_output = StreamingOutput()

        self.csi_camera = None
        self.thermal_sensor = None
        self.csi_thread = None
        self.running = True
        self.last_good_thermal_frame = None

        self.init_csi_camera()
        self.init_thermal_camera()

        print("=== Cameras Ready ===\n")

    def init_csi_camera(self):
        """Initialize main camera using OpenCV, scanning for the first available device."""
        try:
            print("[CSI Camera] Initializing with OpenCV (scanning for camera)...")

            # Scan for the first available camera index (0-9)
            camera_index = -1
            for i in range(10):
                cap_test = cv2.VideoCapture(i, cv2.CAP_V4L2)
                if cap_test.isOpened():
                    camera_index = i
                    print(f"[CSI Camera] ✓ Found available camera at index {camera_index}")
                    cap_test.release()
                    break

            if camera_index == -1:
                raise RuntimeError("Could not find any available camera.")

            self.csi_camera = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
            if not self.csi_camera.isOpened():
                raise RuntimeError(f"Could not open camera at index {camera_index}.")

            # Start a background thread to continuously read frames
            self.csi_thread = threading.Thread(target=self._csi_stream_loop)
            self.csi_thread.daemon = True
            self.csi_thread.start()

            print("[CSI Camera] ✓ OpenCV capture active")

        except Exception as e:
            print(f"[CSI Camera] ✗ Initialization failed: {e}")
            if self.csi_camera:
                self.csi_camera.release()
            self.csi_camera = None

    def _csi_stream_loop(self):
        """Background thread to read frames and encode for streaming."""
        print("[CSI Camera] Streamer thread started")
        while self.running and self.csi_camera and self.csi_camera.isOpened():
            ret, frame = self.csi_camera.read()
            if ret:
                # Encode the frame as JPEG for the web stream
                _, buffer = cv2.imencode('.jpg', frame)
                self.streaming_output.write(buffer.tobytes())
            else:
                print("[CSI Camera] Warning: Failed to grab frame in stream loop.")
                time.sleep(0.1)
        print("[CSI Camera] Streamer thread stopped")

    def _init_thermal_sensor_object(self):
        """Helper function to create and configure the MI48 sensor object - FROM WORKING CODE"""
        if not THERMAL_AVAILABLE:
            return False

        # If a sensor object already exists, properly close it before creating a new one
        if self.thermal_sensor:
            print("[Thermal Camera] Closing existing thermal sensor object...")
            try:
                self.thermal_sensor.stop()
                time.sleep(0.5)
            except Exception as e:
                print(f"[Thermal Camera] Error while stopping existing sensor: {e}")

        try:
            print("[Thermal Camera] (Re)Initializing MI48 sensor object...")

            i2c_bus = smbus.SMBus(I2C_BUS)
            i2c_if = I2C_Interface(i2c_bus, THERMAL_I2C_ADDR)

            spi_dev = spidev.SpiDev()
            spi_dev.open(THERMAL_SPI_BUS, THERMAL_SPI_DEVICE)
            spi_dev.max_speed_hz = 8000000  # Lowered from 16Mhz to improve stability
            spi_if = SPI_Interface(spi_dev, xfer_size=4096)

            # Define the hardware reset pin and handler, as per Waveshare documentation
            try:
                reset_pin = DigitalOutputDevice(23, active_high=False, initial_value=True)
                reset_handler = MI48_reset(pin=reset_pin)
            except Exception as e:
                print(f"[Thermal Camera] ⚠ Could not initialize reset pin: {e}")
                reset_handler = None

            self.thermal_sensor = MI48(interfaces=[i2c_if, spi_if], reset_handler=reset_handler)
            return True
        except Exception as e:
            print(f"[Thermal Camera] ✗ Failed to create MI48 object: {e}")
            self.thermal_sensor = None
            return False

    def init_thermal_camera(self):
        """Initialize thermal camera HAT B, with robust warm-up and recovery - FROM WORKING CODE"""
        if not THERMAL_AVAILABLE:
            print("[Thermal Camera] Libraries not available, skipping initialization")
            return

        # First attempt to initialize the sensor object
        if not self._init_thermal_sensor_object():
            print("[Thermal Camera] ✗ Initial setup failed. Thermal features disabled.")
            return

        try:
            # Configure sensor for optimal performance
            print("[Thermal Camera] Configuring sensor...")
            try:
                self.thermal_sensor.stop_capture()
                time.sleep(0.2)
                self.thermal_sensor.start()
                time.sleep(0.3)
                print("[Thermal Camera] ✓ Configured for continuous capture")
            except Exception as config_error:
                print(f"[Thermal Camera] ⚠ Configuration warning: {config_error}")
                print("[Thermal Camera] Sensor will use default settings")

            # Set a stable frame rate to prevent overwhelming the SPI bus
            try:
                print("[Thermal Camera] Setting frame rate to 10 FPS...")
                self.thermal_sensor.set_fps(10)
                print("[Thermal Camera] ✓ Frame rate set")
            except Exception as fps_error:
                print(f"[Thermal Camera] ⚠ Could not set frame rate: {fps_error}")

            # CRITICAL: Warm-up period - sensor needs time to stabilize (FROM WORKING CODE)
            print("[Thermal Camera] Warming up sensor (10 seconds)...")
            print("[Thermal Camera] Please wait - sensor calibrating...")
            for i in range(10):
                try:
                    self.thermal_sensor.read()
                    time.sleep(1.0)
                    print(f"[Thermal Camera] Warm-up: {i+1}/10 frames read")
                except Exception as e:
                    print(f"[Thermal Camera] Error during warm-up read: {e}")
                    pass

            # Verify sensor is working with valid data (FROM WORKING CODE - 3 attempts)
            print("[Thermal Camera] Verifying sensor data...")

            temp_min = -999
            for attempt in range(3):
                result = self.thermal_sensor.read()
                if result:
                    frame = result[0] if isinstance(result, tuple) else result
                    if frame is not None:
                        temp_array = np.array(frame, dtype=np.float32).reshape(62, 80)
                        temp_min = np.nanmin(temp_array)

                        if temp_min > -100:
                            test_max = np.nanmax(temp_array)
                            print(f"[Thermal Camera] ✓ Sensor data valid on attempt {attempt+1}: min={temp_min:.1f}°C, max={test_max:.1f}°C")
                            print("[Thermal Camera] ✓ Initialization complete and verified")
                            return  # Success!

                print(f"[Thermal Camera] ✗ Verification attempt {attempt+1} failed. Min temp: {temp_min:.1f}°C")
                if attempt < 2:
                    print("[Thermal Camera] Attempting full sensor re-initialization...")
                    time.sleep(5)
                    self._init_thermal_sensor_object()
                    # After re-initializing, we must re-configure
                    self.thermal_sensor.stop_capture()
                    time.sleep(0.2)
                    self.thermal_sensor.start()
                    time.sleep(0.3)

            # If all attempts fail
            print(f"[Thermal Camera] ✗ All recovery attempts failed. Sensor still invalid.")
            print(f"[Thermal Camera] Final readings: min={temp_min:.1f}°C")
            print("[Thermal Camera] ⚠ THERMAL FEATURES DISABLED")
            print("[Thermal Camera] Try: sudo reboot to reset sensor")
            self.thermal_sensor = None

        except Exception as e:
            print(f"[Thermal Camera] ✗ Major initialization failure: {e}")
            import traceback
            traceback.print_exc()
            self.thermal_sensor = None

    def switch_camera(self, camera_type):
        """Switch active camera"""
        if camera_type in ["csi", "thermal"]:
            self.active_camera = camera_type
            return True
        return False

    def get_stream_frame(self):
        """Get current CSI camera frame for MJPEG streaming."""
        with self.streaming_output.condition:
            if self.streaming_output.condition.wait(timeout=1.0):
                return self.streaming_output.frame
        return None

    def capture_csi_frame(self):
        """Capture CSI camera frame as a numpy array."""
        if not self.csi_camera or not self.csi_camera.isOpened():
            return None
        try:
            # Read multiple frames to clear buffer and get the latest one
            for _ in range(3):
                self.csi_camera.grab()
            ret, frame = self.csi_camera.read()

            if ret:
                return frame
            else:
                return None
        except Exception as e:
            print(f"[CSI Camera] Capture error: {e}")
            return None

    def save_image(self, file_path, use_thermal=False):
        """Captures and saves a still image."""
        if use_thermal:
            img = self.get_thermal_image()
        else:
            img = self.capture_csi_frame()

        if img is not None:
            try:
                cv2.imwrite(file_path, img)
                print(f"[Camera] ✓ Image saved to {file_path}")
                return True
            except Exception as e:
                print(f"[Camera] ✗ Failed to save image: {e}")
                return False
        return False

    def get_thermal_image(self):
        """Get thermal as colorized image for display/saving."""
        temp_array = self.capture_thermal_array()
        if temp_array is None:
            return None

        try:
            temp_min = np.nanmin(temp_array)
            temp_max = np.nanmax(temp_array)
            temp_mean = np.nanmean(temp_array)

            # Check for invalid data
            if np.isnan(temp_min) or np.isnan(temp_max):
                print("[Thermal Error] Array contains NaN values!")
                return None

            # Check if temperature range is too small (< 0.5°C)
            if (temp_max - temp_min) < 0.5:
                # Use a fixed range centered on mean temperature
                fixed_range = 5.0
                center = temp_mean
                temp_normalized = np.clip(
                    ((temp_array - (center - fixed_range/2)) / fixed_range * 255),
                    0, 255
                ).astype(np.uint8)
            else:
                # Normal normalization when we have good temperature variation
                temp_normalized = ((temp_array - temp_min) / (temp_max - temp_min) * 255).astype(np.uint8)

            # Apply JET colormap (blue=cold, red=hot)
            colored = cv2.applyColorMap(temp_normalized, cv2.COLORMAP_JET)

            # Resize to camera resolution
            resized = cv2.resize(colored, CAMERA_RESOLUTION, interpolation=cv2.INTER_NEAREST)

            # Rotate the image 90 degrees counter-clockwise to correct orientation
            resized = cv2.rotate(resized, cv2.ROTATE_90_COUNTERCLOCKWISE)

            # Add temperature overlay text
            cv2.putText(resized, f"Min: {temp_min:.1f}C Max: {temp_max:.1f}C",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Cache the successfully created frame
            self.last_good_thermal_frame = resized

            return resized

        except Exception as e:
            print(f"[Thermal Camera] Frame encoding error: {e}")
            return None

    def capture_thermal_array(self):
        """
        Capture thermal camera frame, waiting for DATA_READY signal - FROM WORKING CODE
        This is the most robust method for preventing data corruption.
        """
        if not self.thermal_sensor:
            return None

        try:
            # 1. Wait for the DATA_READY flag from the sensor's status register.
            # This ensures we don't try to read while the sensor is writing.
            wait_start_time = time.time()
            data_ready = False
            while not data_ready:
                status = self.thermal_sensor.get_status()
                if status is not None and (status & 0x10):  # 0x10 is the DATA_READY bit
                    data_ready = True
                elif time.time() - wait_start_time > 0.5:  # 500ms timeout
                    print("[Thermal ERROR] Timed out waiting for DATA_READY flag.")
                    return None
                else:
                    time.sleep(0.01)  # Small delay before polling again

            # 2. Once data is ready, perform the read.
            result = self.thermal_sensor.read()

            if result is None:
                print("[Thermal Debug] read() returned None even after DATA_READY.")
                return None

            # 3. Process the frame
            frame = result[0] if isinstance(result, tuple) else result
            temp_array = np.array(frame, dtype=np.float32)

            if temp_array.size == 4960:
                temp_array = temp_array.reshape(62, 80)
            else:
                print(f"[Thermal Warning] Unexpected data size after DATA_READY: {temp_array.size}")
                return None

            # 4. Final validation
            temp_min_check = np.nanmin(temp_array)
            if temp_min_check < -100:
                print(f"[Thermal ERROR] Invalid data read despite DATA_READY. Min temp: {temp_min_check:.2f}°C")
                return None

            return temp_array

        except Exception as e:
            print(f"[Thermal Camera] Capture error: {e}")
            return None

    def get_active_frame(self):
        """Get frame from currently active camera"""
        if self.active_camera == "csi":
            return self.get_stream_frame()
        else:
            img = self.get_thermal_image()
            if img is not None:
                _, jpeg = cv2.imencode('.jpg', img)
                return jpeg.tobytes()
            return None

    def is_thermal_available(self):
        """Check if thermal camera is available"""
        return self.thermal_sensor is not None

    def cleanup(self):
        """Cleanup cameras on shutdown"""
        self.running = False
        if self.csi_thread and self.csi_thread.is_alive():
            self.csi_thread.join(timeout=1.0)
        if self.csi_camera:
            self.csi_camera.release()
        if self.thermal_sensor:
            try:
                self.thermal_sensor.stop()
            except:
                pass
