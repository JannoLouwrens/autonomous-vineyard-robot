#!/usr/bin/env python3
"""
Stream Server - MJPEG video stream + JSON data stream + Log streaming
"""

import socket
import threading
import json
import time
import cv2
import numpy as np
import sys
import io
from collections import deque
sys.path.append('/home/pi/Working Code/Updated FULL Code Standalone')
from config import *


class LogCapture:
    """Captures print output to a buffer for streaming to clients"""

    def __init__(self, max_lines=100):
        self.buffer = deque(maxlen=max_lines)
        self.original_stdout = sys.stdout
        self.lock = threading.Lock()

    def write(self, text):
        # Write to original stdout
        self.original_stdout.write(text)

        # Also capture to buffer (skip empty lines)
        if text.strip():
            with self.lock:
                self.buffer.append({
                    'time': time.time(),
                    'msg': text.strip()
                })

    def flush(self):
        self.original_stdout.flush()

    def get_recent_logs(self, since_time=0):
        """Get logs since a timestamp"""
        with self.lock:
            return [log for log in self.buffer if log['time'] > since_time]

    def get_all_logs(self):
        """Get all buffered logs"""
        with self.lock:
            return list(self.buffer)


class StreamServer:
    """
    Streams video (MJPEG) and sensor data (JSON) to laptop controller
    """

    def __init__(self, camera, sensors, yolo=None):
        self.camera = camera
        self.sensors = sensors
        self.yolo = yolo  # YOLO classifier for overlay
        self.running = False
        self.active_camera = "csi"  # "csi" or "thermal"
        self.yolo_overlay = False  # Live YOLO detection overlay

        self.video_socket = None
        self.data_socket = None

        self.video_clients = []
        self.data_clients = []
        self.clients_lock = threading.Lock()

        # YOLO overlay runs slower than video (Pi can only do ~1-2 FPS with YOLO)
        self.yolo_frame_skip = 5  # Process every Nth frame for YOLO
        self.yolo_frame_count = 0
        self.last_yolo_frame = None  # Cache last YOLO-processed frame

        print(f"[Stream Server] Video port: {STREAM_PORT}, Data port: {DATA_STREAM_PORT}")

    def set_camera(self, camera_type):
        """Switch between CSI and thermal camera for streaming"""
        if camera_type in ["csi", "thermal"]:
            self.active_camera = camera_type
            print(f"[Stream Server] Switched to {camera_type} camera")

    def set_yolo_overlay(self, enabled):
        """Enable/disable live YOLO detection overlay on video stream"""
        self.yolo_overlay = enabled
        status = "ENABLED" if enabled else "DISABLED"
        print(f"[Stream Server] YOLO overlay {status}")

    def _draw_yolo_overlay(self, frame):
        """
        Run YOLO detection and draw bounding boxes on frame
        Returns: annotated frame (BGR)
        """
        if frame is None:
            return frame

        # Check if YOLO is available
        if self.yolo is None:
            cv2.putText(frame, "YOLO: Not initialized", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            return frame

        if not self.yolo.is_available():
            cv2.putText(frame, "YOLO: Model not loaded", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            return frame

        try:
            # Run YOLO detection
            results = self.yolo.model(frame, conf=YOLO_CONFIDENCE_THRESHOLD, verbose=False)

            detection_count = 0
            for result in results:
                boxes = result.boxes
                if boxes is None:
                    continue

                for box in boxes:
                    detection_count += 1
                    # Get box coordinates
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])

                    # Get class name safely
                    if hasattr(self.yolo.model, 'names') and cls_id < len(self.yolo.model.names):
                        cls_name = self.yolo.model.names[cls_id]
                    else:
                        cls_name = f"class_{cls_id}"

                    # Color based on class (green for trunk, red for others)
                    if cls_name.lower() in ['trunk', 'tree', 'vine']:
                        color = (0, 255, 0)  # Green
                    else:
                        color = (0, 0, 255)  # Red

                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                    # Draw label with confidence
                    label = f"{cls_name}: {conf:.2f}"
                    label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)

                    # Label background
                    cv2.rectangle(frame, (x1, y1 - label_size[1] - 10),
                                  (x1 + label_size[0], y1), color, -1)

                    # Label text
                    cv2.putText(frame, label, (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Add YOLO status indicator
            cv2.putText(frame, f"YOLO: ON ({detection_count})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        except Exception as e:
            # Draw error indicator
            print(f"[YOLO Overlay] Error: {e}")
            cv2.putText(frame, f"YOLO Error: {str(e)[:30]}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        return frame

    def start(self):
        """Start streaming servers"""
        self.running = True

        # Video stream server
        self.video_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.video_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.video_socket.bind(('0.0.0.0', STREAM_PORT))
        self.video_socket.listen(5)
        self.video_socket.settimeout(1.0)

        # Data stream server
        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.data_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.data_socket.bind(('0.0.0.0', DATA_STREAM_PORT))
        self.data_socket.listen(5)
        self.data_socket.settimeout(1.0)

        # Start threads
        threading.Thread(target=self._video_accept_loop, daemon=True).start()
        threading.Thread(target=self._data_accept_loop, daemon=True).start()
        threading.Thread(target=self._video_stream_loop, daemon=True).start()
        threading.Thread(target=self._data_stream_loop, daemon=True).start()

        print("[Stream Server] Started")

    def _video_accept_loop(self):
        """Accept video stream clients"""
        while self.running:
            try:
                client_socket, addr = self.video_socket.accept()
                print(f"[Stream Server] Video client: {addr}")

                # Send HTTP header for MJPEG
                header = (
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
                    "\r\n"
                )
                client_socket.send(header.encode('utf-8'))

                with self.clients_lock:
                    self.video_clients.append(client_socket)

            except socket.timeout:
                continue
            except:
                pass

    def _data_accept_loop(self):
        """Accept data stream clients"""
        while self.running:
            try:
                client_socket, addr = self.data_socket.accept()
                print(f"[Stream Server] Data client: {addr}")

                with self.clients_lock:
                    self.data_clients.append(client_socket)

            except socket.timeout:
                continue
            except:
                pass

    def _video_stream_loop(self):
        """Stream video frames to all connected clients"""
        while self.running:
            try:
                # Get frame based on YOLO overlay mode
                # IMPORTANT: yolo_overlay must be explicitly True AND camera must be CSI
                if self.yolo_overlay == True and self.active_camera == "csi":
                    # Get raw frame
                    frame = self.camera.capture_csi_frame()
                    if frame is not None:
                        self.yolo_frame_count += 1

                        # Only run YOLO every N frames (slow on Pi)
                        if self.yolo_frame_count >= self.yolo_frame_skip:
                            self.yolo_frame_count = 0
                            # Run YOLO and cache result
                            frame = self._draw_yolo_overlay(frame)
                            _, frame_data = cv2.imencode('.jpg', frame,
                                [cv2.IMWRITE_JPEG_QUALITY, 80])
                            self.last_yolo_frame = frame_data.tobytes()

                        # Use cached YOLO frame if available, else raw frame
                        if self.last_yolo_frame is not None:
                            frame_data = self.last_yolo_frame
                        else:
                            _, frame_data = cv2.imencode('.jpg', frame,
                                [cv2.IMWRITE_JPEG_QUALITY, 80])
                            frame_data = frame_data.tobytes()
                    else:
                        frame_data = None
                else:
                    # Normal mode - use pre-encoded frame
                    frame_data = self.camera.get_active_frame()
                    # Clear YOLO cache when overlay disabled
                    self.last_yolo_frame = None
                    self.yolo_frame_count = 0

                if frame_data is None:
                    time.sleep(0.1)
                    continue

                # MJPEG frame
                header = (
                    f"--frame\r\n"
                    f"Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(frame_data)}\r\n"
                    f"\r\n"
                )
                packet = header.encode('utf-8') + frame_data + b"\r\n"

                # Send to all clients
                with self.clients_lock:
                    disconnected = []
                    for client in self.video_clients:
                        try:
                            client.sendall(packet)
                        except:
                            disconnected.append(client)

                    for client in disconnected:
                        self.video_clients.remove(client)
                        try:
                            client.close()
                        except:
                            pass

                time.sleep(1.0 / CAMERA_FPS)

            except Exception as e:
                print(f"[Stream Server] Video error: {e}")
                time.sleep(0.1)

    def _data_stream_loop(self):
        """Stream sensor data to all connected clients"""
        while self.running:
            try:
                # Gather sensor data
                data = {
                    'timestamp': time.time(),
                    'sensors': self.sensors.get_status(),
                    'front_distance_mm': self.sensors.last_distance,
                    'side_distance_cm': self.sensors.last_hcsr04_distance,
                    'heading': self.sensors.last_heading,
                    'gps': self.sensors.last_position
                }

                json_data = json.dumps(data) + '\n'

                # Send to all clients
                with self.clients_lock:
                    disconnected = []
                    for client in self.data_clients:
                        try:
                            client.sendall(json_data.encode('utf-8'))
                        except:
                            disconnected.append(client)

                    for client in disconnected:
                        self.data_clients.remove(client)
                        try:
                            client.close()
                        except:
                            pass

                time.sleep(1.0 / SENSOR_UPDATE_RATE_HZ)

            except Exception as e:
                print(f"[Stream Server] Data error: {e}")
                time.sleep(0.1)

    def stop(self):
        """Stop streaming"""
        self.running = False

        # Close all clients
        with self.clients_lock:
            for client in self.video_clients + self.data_clients:
                try:
                    client.close()
                except:
                    pass
            self.video_clients.clear()
            self.data_clients.clear()

        # Close server sockets
        for sock in [self.video_socket, self.data_socket]:
            if sock:
                try:
                    sock.close()
                except:
                    pass

        print("[Stream Server] Stopped")
