#!/usr/bin/env python3
"""
Vineyard Robot Laptop Controller - Enhanced GUI
Combines features from Working Code with navigation capabilities
Run on laptop: python3 laptop_controller.py <robot_ip>
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import socket
import threading
import json
import time
import io
import sys
import cv2
import numpy as np
from PIL import Image, ImageTk
import urllib.request

# Default connection settings
DEFAULT_ROBOT_IP = "192.168.1.100"
COMMAND_PORT = 8890
STREAM_PORT = 8891
DATA_PORT = 8892


class VineyardRobotController:
    """Main GUI application - Enhanced version"""

    def __init__(self, robot_ip=None):
        self.root = tk.Tk()
        self.root.title("Vineyard Robot Controller")
        self.root.geometry("1600x1000")

        # Connection
        self.robot_ip = robot_ip or DEFAULT_ROBOT_IP
        self.command_socket = None
        self.connected = False

        # Streaming
        self.streaming = False
        self.stream_thread = None
        self.data_thread = None
        self.active_camera = "csi"

        # Motor state (for WASD control)
        self.motors_active = {"w": False, "s": False, "a": False, "d": False, "q": False, "e": False}
        self.motor_speed = 100

        # Data
        self.sensor_data = {}
        self.last_response = ""

        self._build_gui()

        # Keyboard bindings
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)

        # Start motor update loop
        self.update_motors()

    def _build_gui(self):
        """Build the GUI layout"""
        # Main container
        main_frame = ttk.Frame(self.root, padding=5)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Left panel - Video + Controls (scrollable)
        left_container = ttk.Frame(main_frame)
        left_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        left_canvas = tk.Canvas(left_container)
        left_scrollbar = ttk.Scrollbar(left_container, orient="vertical", command=left_canvas.yview)
        left_panel = ttk.Frame(left_canvas)

        left_panel.bind(
            "<Configure>",
            lambda e: left_canvas.configure(scrollregion=left_canvas.bbox("all"))
        )

        left_canvas.create_window((0, 0), window=left_panel, anchor="nw")
        left_canvas.configure(yscrollcommand=left_scrollbar.set)

        left_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Enable mousewheel scrolling on left panel
        def on_left_mousewheel(event):
            left_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        left_canvas.bind_all("<MouseWheel>", on_left_mousewheel)

        # Right panel - Data + Tests (scrollable)
        right_panel = ttk.Frame(main_frame, width=420)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        right_panel.pack_propagate(False)

        # Create scrollable frame for right panel
        canvas = tk.Canvas(right_panel, width=400)
        scrollbar = ttk.Scrollbar(right_panel, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # === CONNECTION FRAME ===
        conn_frame = ttk.LabelFrame(left_panel, text="Connection", padding=5)
        conn_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(conn_frame, text="Robot IP:").pack(side=tk.LEFT)
        self.ip_entry = ttk.Entry(conn_frame, width=15)
        self.ip_entry.insert(0, self.robot_ip)
        self.ip_entry.pack(side=tk.LEFT, padx=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.pack(side=tk.LEFT, padx=10)

        self.stream_btn = ttk.Button(conn_frame, text="Start Stream", command=self.toggle_stream, state=tk.DISABLED)
        self.stream_btn.pack(side=tk.RIGHT, padx=5)

        # === VIDEO FRAME ===
        video_frame = ttk.LabelFrame(left_panel, text="Video Feed", padding=5)
        video_frame.pack(fill=tk.X, pady=(0, 10))

        # Large video display (800x600 pixels)
        self.video_label = tk.Label(video_frame, text="No video stream", bg='black', fg='white',
                                     width=800, height=600)
        self.video_label.pack()
        # Create placeholder image to set exact pixel size
        placeholder = Image.new('RGB', (800, 600), color='black')
        self.placeholder_photo = ImageTk.PhotoImage(placeholder)
        self.video_label.config(image=self.placeholder_photo)

        # Stream status bar
        stream_status_frame = ttk.Frame(video_frame)
        stream_status_frame.pack(fill=tk.X)

        self.stream_status = tk.Label(stream_status_frame, text="Stream: Not connected", fg="gray")
        self.stream_status.pack(side=tk.LEFT, padx=5)

        self.camera_status = tk.Label(stream_status_frame, text="Camera: CSI", fg="blue")
        self.camera_status.pack(side=tk.RIGHT, padx=5)

        # === CAMERA SELECTION ===
        camera_frame = ttk.LabelFrame(left_panel, text="Camera", padding=5)
        camera_frame.pack(fill=tk.X, pady=(0, 10))

        cam_btn_frame = ttk.Frame(camera_frame)
        cam_btn_frame.pack()

        self.csi_btn = ttk.Button(cam_btn_frame, text="CSI Camera", width=15,
                                   command=lambda: self.switch_camera("csi"))
        self.csi_btn.pack(side=tk.LEFT, padx=5)

        self.thermal_btn = ttk.Button(cam_btn_frame, text="Thermal Camera", width=15,
                                       command=lambda: self.switch_camera("thermal"))
        self.thermal_btn.pack(side=tk.LEFT, padx=5)

        # YOLO Overlay toggle
        yolo_frame = ttk.Frame(camera_frame)
        yolo_frame.pack(fill=tk.X, pady=5)

        self.yolo_overlay_enabled = False
        self.yolo_btn = ttk.Button(yolo_frame, text="YOLO Detection: OFF",
                                   command=self.toggle_yolo_overlay)
        self.yolo_btn.pack(fill=tk.X)

        self.yolo_status = tk.Label(yolo_frame, text="Live detection overlay disabled",
                                    fg="gray", font=('TkDefaultFont', 8))
        self.yolo_status.pack()

        # === MOTOR CONTROLS ===
        motor_frame = ttk.LabelFrame(left_panel, text="Motor Control (WASD + Q/E)", padding=5)
        motor_frame.pack(fill=tk.X, pady=(0, 10))

        # Speed slider
        speed_frame = ttk.Frame(motor_frame)
        speed_frame.pack(fill=tk.X, pady=5)
        ttk.Label(speed_frame, text="Speed:").pack(side=tk.LEFT)
        self.speed_scale = tk.Scale(speed_frame, from_=0, to=255, orient=tk.HORIZONTAL, length=200)
        self.speed_scale.set(100)
        self.speed_scale.pack(side=tk.LEFT, padx=10)

        # Direction buttons
        btn_frame = ttk.Frame(motor_frame)
        btn_frame.pack()

        ttk.Button(btn_frame, text="Turn L (Q)", width=10).grid(row=0, column=0, pady=2)
        ttk.Button(btn_frame, text="Forward (W)", width=10,
                  command=lambda: self.send_command("forward")).grid(row=0, column=1, pady=2)
        ttk.Button(btn_frame, text="Turn R (E)", width=10).grid(row=0, column=2, pady=2)
        ttk.Button(btn_frame, text="Left (A)", width=10,
                  command=lambda: self.send_command("strafe_left")).grid(row=1, column=0, padx=2)
        ttk.Button(btn_frame, text="STOP", width=10,
                  command=self.stop_motors).grid(row=1, column=1)
        ttk.Button(btn_frame, text="Right (D)", width=10,
                  command=lambda: self.send_command("strafe_right")).grid(row=1, column=2, padx=2)
        ttk.Button(btn_frame, text="Backward (S)", width=10,
                  command=lambda: self.send_command("backward")).grid(row=2, column=1, pady=2)

        # === RIGHT PANEL (scrollable_frame) ===

        # === SERVO CONTROLS (moved to right panel - always visible) ===
        servo_frame = ttk.LabelFrame(scrollable_frame, text="Servo Control", padding=5)
        servo_frame.pack(fill=tk.X, pady=(0, 10))

        # Pan slider
        ttk.Label(servo_frame, text="Pan (Left/Right):").pack()
        self.pan_scale = tk.Scale(servo_frame, from_=0, to=180, orient=tk.HORIZONTAL,
                                   command=self.on_pan_change, length=350)
        self.pan_scale.set(90)
        self.pan_scale.pack()

        # Tilt slider
        ttk.Label(servo_frame, text="Tilt (Up/Down):").pack()
        self.tilt_scale = tk.Scale(servo_frame, from_=0, to=100, orient=tk.HORIZONTAL,
                                    command=self.on_tilt_change, length=350)
        self.tilt_scale.set(90)
        self.tilt_scale.pack()

        servo_btn_frame = ttk.Frame(servo_frame)
        servo_btn_frame.pack(pady=5)

        ttk.Button(servo_btn_frame, text="Center", width=8,
                  command=self.center_servos).pack(side=tk.LEFT, padx=2)
        ttk.Button(servo_btn_frame, text="Left", width=8,
                  command=lambda: self.send_command("look_left")).pack(side=tk.LEFT, padx=2)
        ttk.Button(servo_btn_frame, text="Ground", width=8,
                  command=lambda: self.send_command("look_ground")).pack(side=tk.LEFT, padx=2)
        ttk.Button(servo_btn_frame, text="Canopy", width=8,
                  command=lambda: self.send_command("look_canopy")).pack(side=tk.LEFT, padx=2)

        # === WAYPOINT FRAME ===
        waypoint_frame = ttk.LabelFrame(scrollable_frame, text="Row Waypoints", padding=5)
        waypoint_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Button(waypoint_frame, text="Set Row START (current GPS)",
                  command=lambda: self.send_command("set_row_start")).pack(fill=tk.X, pady=2)
        ttk.Button(waypoint_frame, text="Set Row END (current GPS)",
                  command=lambda: self.send_command("set_row_end")).pack(fill=tk.X, pady=2)
        ttk.Button(waypoint_frame, text="Save Row",
                  command=lambda: self.send_command("save_row")).pack(fill=tk.X, pady=2)
        ttk.Button(waypoint_frame, text="List Rows",
                  command=lambda: self.send_command("list_rows")).pack(fill=tk.X, pady=2)

        # === NAVIGATION TEST MODES ===
        nav_frame = ttk.LabelFrame(scrollable_frame, text="Navigation Test Modes", padding=5)
        nav_frame.pack(fill=tk.X, pady=(0, 10))

        nav_row_frame = ttk.Frame(nav_frame)
        nav_row_frame.pack(fill=tk.X, pady=2)

        ttk.Label(nav_row_frame, text="Row ID:").pack(side=tk.LEFT)
        self.row_id_entry = ttk.Entry(nav_row_frame, width=5)
        self.row_id_entry.insert(0, "1")
        self.row_id_entry.pack(side=tk.LEFT, padx=5)

        ttk.Button(nav_row_frame, text="List Rows",
                  command=lambda: self.send_command("list_rows")).pack(side=tk.LEFT, padx=5)

        # Progressive test modes
        ttk.Label(nav_frame, text="Test each feature progressively:",
                  font=('TkDefaultFont', 9, 'italic')).pack(anchor='w', pady=(5,2))

        # Mode 1: Compass only
        mode1_frame = ttk.Frame(nav_frame)
        mode1_frame.pack(fill=tk.X, pady=1)
        ttk.Button(mode1_frame, text="① Compass Only",
                  command=lambda: self.send_step_command("test_mode_compass")).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Label(mode1_frame, text="(heading only)", font=('TkDefaultFont', 8)).pack(side=tk.LEFT, padx=5)

        # Mode 2: Compass + Visual
        mode2_frame = ttk.Frame(nav_frame)
        mode2_frame.pack(fill=tk.X, pady=1)
        ttk.Button(mode2_frame, text="② Compass + Visual Row",
                  command=lambda: self.send_step_command("test_mode_visual")).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Label(mode2_frame, text="(+ row wall)", font=('TkDefaultFont', 8)).pack(side=tk.LEFT, padx=5)

        # Mode 3: Compass + Visual + Strafe
        mode3_frame = ttk.Frame(nav_frame)
        mode3_frame.pack(fill=tk.X, pady=1)
        ttk.Button(mode3_frame, text="③ Compass + Visual + Strafe",
                  command=lambda: self.send_step_command("test_mode_strafe")).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Label(mode3_frame, text="(+ HC-SR04)", font=('TkDefaultFont', 8)).pack(side=tk.LEFT, padx=5)

        # Mode 4: Full Auto with tree scanning
        ttk.Separator(nav_frame, orient='horizontal').pack(fill=tk.X, pady=5)
        ttk.Button(nav_frame, text="④ FULL AUTO (+ Tree Scanning)",
                  command=self.navigate_row).pack(fill=tk.X, pady=1)

        # Stop/Pause/Resume
        ttk.Button(nav_frame, text="STOP Navigation",
                  command=lambda: self.send_command("stop_nav"),
                  style='Danger.TButton').pack(fill=tk.X, pady=(5,2))

        nav_ctrl_frame = ttk.Frame(nav_frame)
        nav_ctrl_frame.pack(fill=tk.X, pady=2)
        ttk.Button(nav_ctrl_frame, text="Pause",
                  command=lambda: self.send_command("pause_nav")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(nav_ctrl_frame, text="Resume",
                  command=lambda: self.send_command("resume_nav")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # === MANUAL STEP TESTING ===
        step_frame = ttk.LabelFrame(scrollable_frame, text="Manual Step Testing", padding=5)
        step_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(step_frame, text="Test each step (uses Row ID above):",
                  font=('TkDefaultFont', 9, 'italic')).pack(anchor='w')

        # Steps that need row ID
        steps_with_row = [
            ("0. Orient to Start (face start)", "step0_orient_to_start"),
            ("1. Go to Start Waypoint", "step1_goto_start"),
        ]

        # Steps that don't need row ID (use current state)
        steps_no_row = [
            ("2. Orient to End (face row)", "step2_orient"),
            ("3. Find Tree (HC-SR04 scan)", "step3_find_tree"),
            ("4. Verify Tree (YOLO)", "step4_verify_yolo"),
            ("5. Thermal Test (water)", "step5_thermal_test"),
            ("6. Canopy Photo (top)", "step6_canopy_photo"),
            ("7. Row Wall Detection", "step7_row_wall"),
        ]

        # Steps with row ID
        for text, cmd in steps_with_row:
            ttk.Button(step_frame, text=text,
                      command=lambda c=cmd: self.send_step_command(c)).pack(fill=tk.X, pady=1)

        # Other steps don't need row ID
        for text, cmd in steps_no_row:
            ttk.Button(step_frame, text=text,
                      command=lambda c=cmd: self.send_command(c)).pack(fill=tk.X, pady=1)

        ttk.Separator(step_frame, orient='horizontal').pack(fill=tk.X, pady=5)

        # Full auto button with row ID
        auto_frame = ttk.Frame(step_frame)
        auto_frame.pack(fill=tk.X, pady=2)
        ttk.Label(auto_frame, text="Row:").pack(side=tk.LEFT)
        self.auto_row_entry = ttk.Entry(auto_frame, width=5)
        self.auto_row_entry.insert(0, "1")
        self.auto_row_entry.pack(side=tk.LEFT, padx=5)
        ttk.Button(auto_frame, text="FULL AUTO (All Steps)",
                  command=self.run_full_auto).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        # Emergency stop
        ttk.Button(step_frame, text="EMERGENCY STOP",
                  command=self.emergency_stop,
                  style='Danger.TButton').pack(fill=tk.X, pady=(5, 0))

        # === HC-SR04 WATCH MODE (prominent tree detection tester) ===
        watch_frame = ttk.LabelFrame(scrollable_frame, text="HC-SR04 Tree Watch Mode", padding=10)
        watch_frame.pack(fill=tk.X, pady=(0, 10))

        # Big status display
        self.watch_status_label = tk.Label(
            watch_frame,
            text="INACTIVE",
            font=('Helvetica', 24, 'bold'),
            bg='gray',
            fg='white',
            width=20,
            height=2
        )
        self.watch_status_label.pack(fill=tk.X, pady=5)

        # Distance display
        self.watch_distance_label = tk.Label(
            watch_frame,
            text="Distance: --",
            font=('Courier', 14),
            fg='black'
        )
        self.watch_distance_label.pack()

        # Tree count
        self.watch_count_label = tk.Label(
            watch_frame,
            text="Trees Detected: 0",
            font=('Courier', 12),
            fg='darkgreen'
        )
        self.watch_count_label.pack()

        # Start/Stop buttons
        watch_btn_frame = ttk.Frame(watch_frame)
        watch_btn_frame.pack(fill=tk.X, pady=5)

        self.watch_start_btn = ttk.Button(
            watch_btn_frame,
            text="START WATCHING",
            command=self.start_hcsr04_watch
        )
        self.watch_start_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.watch_stop_btn = ttk.Button(
            watch_btn_frame,
            text="STOP",
            command=self.stop_hcsr04_watch,
            state=tk.DISABLED
        )
        self.watch_stop_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # Watch mode state
        self.watch_active = False
        self.watch_poll_thread = None

        # === TEST FUNCTIONS ===
        test_frame = ttk.LabelFrame(scrollable_frame, text="Test Functions", padding=5)
        test_frame.pack(fill=tk.X, pady=(0, 10))

        tests = [
            ("Test Row Detection", "test_row_detect"),
            ("Test YOLO Verify", "test_yolo"),
            ("Test Trunk Down", "test_trunk_down"),
            ("Test Trunk Up", "test_trunk_up"),
            ("Test Thermal/Water", "test_thermal"),
            ("Test HC-SR04", "test_hcsr04"),
            ("Full Tree Scan", "test_scan"),
        ]

        for text, cmd in tests:
            ttk.Button(test_frame, text=text,
                      command=lambda c=cmd: self.send_command(c)).pack(fill=tk.X, pady=2)

        # === SENSOR DATA ===
        sensor_frame = ttk.LabelFrame(scrollable_frame, text="Sensor Data", padding=5)
        sensor_frame.pack(fill=tk.X, pady=(0, 10))

        self.sensor_text = scrolledtext.ScrolledText(sensor_frame, height=10, width=45, font=('Courier', 9))
        self.sensor_text.pack(fill=tk.X)

        ttk.Button(sensor_frame, text="Refresh Sensors",
                  command=lambda: self.send_command("sensors")).pack(fill=tk.X, pady=2)

        # === COMMAND RESPONSE ===
        response_frame = ttk.LabelFrame(scrollable_frame, text="Command Response", padding=5)
        response_frame.pack(fill=tk.BOTH, expand=True)

        self.response_text = scrolledtext.ScrolledText(response_frame, height=10, width=45, font=('Courier', 9))
        self.response_text.pack(fill=tk.BOTH, expand=True)

        # === MANUAL COMMAND ===
        cmd_frame = ttk.Frame(scrollable_frame)
        cmd_frame.pack(fill=tk.X, pady=5)

        self.cmd_entry = ttk.Entry(cmd_frame)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        self.cmd_entry.bind('<Return>', lambda e: self.send_manual_command())

        ttk.Button(cmd_frame, text="Send", command=self.send_manual_command).pack(side=tk.RIGHT)

    def on_key_press(self, event):
        """Handle key press for WASD motor control"""
        key = event.keysym.lower()
        if key in "wasdqe":
            self.motors_active[key] = True
        elif key == 'space':
            self.stop_motors()

    def on_key_release(self, event):
        """Handle key release"""
        key = event.keysym.lower()
        if key in "wasdqe":
            self.motors_active[key] = False

    def update_motors(self):
        """Send motor commands based on key state"""
        if self.connected and any(self.motors_active.values()):
            speed = self.speed_scale.get()
            lf = l = rf = r = 0

            if self.motors_active['w']:
                lf, l, rf, r = speed, speed, speed, speed
            elif self.motors_active['s']:
                lf, l, rf, r = -speed, -speed, -speed, -speed
            elif self.motors_active['a']:  # Strafe left
                lf, l, rf, r = -speed, speed, speed, -speed
            elif self.motors_active['d']:  # Strafe right
                lf, l, rf, r = speed, -speed, -speed, speed
            elif self.motors_active['q']:  # Turn left
                lf, l, rf, r = -speed, -speed, speed, speed
            elif self.motors_active['e']:  # Turn right
                lf, l, rf, r = speed, speed, -speed, -speed

            self.send_command_silent(f"move {lf} {l} {rf} {r}")

        self.root.after(50, self.update_motors)

    def send_command_silent(self, cmd):
        """Send command without updating response display"""
        if not self.connected:
            return None
        try:
            self.command_socket.sendall((cmd + '\n').encode('utf-8'))
            response = self.command_socket.recv(4096).decode('utf-8')
            return response
        except (BrokenPipeError, ConnectionResetError):
            self.handle_disconnect()
            return None
        except:
            return None

    def stop_motors(self):
        """Stop all motors"""
        for key in self.motors_active:
            self.motors_active[key] = False
        self.send_command("stop")

    def on_pan_change(self, value):
        """Handle pan slider change"""
        if self.connected:
            self.send_command_silent(f"servo {int(value)} {self.tilt_scale.get()}")

    def on_tilt_change(self, value):
        """Handle tilt slider change"""
        if self.connected:
            self.send_command_silent(f"servo {self.pan_scale.get()} {int(value)}")

    def center_servos(self):
        """Center both servos"""
        self.pan_scale.set(90)
        self.tilt_scale.set(90)
        self.send_command("servo 90 90")

    def switch_camera(self, camera_type):
        """Switch between CSI and Thermal camera"""
        self.active_camera = camera_type
        self.send_command(f"camera {camera_type}")
        self.camera_status.config(text=f"Camera: {camera_type.upper()}")

        # Update button appearance
        if camera_type == "csi":
            self.csi_btn.config(style="Accent.TButton")
            self.camera_status.config(fg="blue")
        else:
            self.thermal_btn.config(style="Accent.TButton")
            self.camera_status.config(fg="red")

        # Disable YOLO overlay when switching to thermal
        if camera_type == "thermal" and self.yolo_overlay_enabled:
            self.toggle_yolo_overlay()  # Turn off

    def toggle_yolo_overlay(self):
        """Toggle live YOLO detection overlay on video stream"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Connect to robot first")
            return

        self.yolo_overlay_enabled = not self.yolo_overlay_enabled

        if self.yolo_overlay_enabled:
            response = self.send_command("yolo_overlay on")
            self.yolo_btn.config(text="YOLO Detection: ON")
            self.yolo_status.config(text="Live bounding boxes on video", fg="green")
        else:
            response = self.send_command("yolo_overlay off")
            self.yolo_btn.config(text="YOLO Detection: OFF")
            self.yolo_status.config(text="Live detection overlay disabled", fg="gray")

    def toggle_connection(self):
        """Connect or disconnect from robot"""
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        """Connect to robot"""
        self.robot_ip = self.ip_entry.get()
        try:
            self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.command_socket.settimeout(15)  # Longer timeout for YOLO commands
            self.command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            self.command_socket.connect((self.robot_ip, COMMAND_PORT))
            self.connected = True
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text="Connected", foreground="green")
            self.stream_btn.config(state=tk.NORMAL)

            # Start data stream
            self.start_data_stream()

            # Get initial status
            self.send_command("status")

            # Auto-start video stream
            self.start_stream()

        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def disconnect(self):
        """Disconnect from robot"""
        self.stop_stream()
        self.stop_data_stream()

        if self.command_socket:
            try:
                self.command_socket.close()
            except:
                pass

        self.connected = False
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Disconnected", foreground="red")
        self.stream_btn.config(state=tk.DISABLED)

    def send_command(self, cmd):
        """Send command to robot"""
        if not self.connected:
            return

        try:
            self.command_socket.sendall((cmd + '\n').encode('utf-8'))
            # Use larger buffer for YOLO responses which can be big
            response = self.command_socket.recv(8192).decode('utf-8')
            self.last_response = response
            self.update_response(cmd, response)
        except (BrokenPipeError, ConnectionResetError) as e:
            self.update_response(cmd, f"Connection lost: {e}")
            self.handle_disconnect()
        except socket.timeout:
            self.update_response(cmd, "Command timeout - robot may be busy")
        except Exception as e:
            self.update_response(cmd, f"Error: {e}")

    def handle_disconnect(self):
        """Handle unexpected disconnection"""
        self.connected = False
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Disconnected", foreground="red")
        self.stream_btn.config(state=tk.DISABLED)
        try:
            self.command_socket.close()
        except:
            pass

    def send_manual_command(self):
        """Send command from entry field"""
        cmd = self.cmd_entry.get().strip()
        if cmd:
            self.send_command(cmd)
            self.cmd_entry.delete(0, tk.END)

    def update_response(self, cmd, response):
        """Update response text area"""
        self.response_text.insert(tk.END, f"> {cmd}\n{response}\n\n")
        self.response_text.see(tk.END)

    def navigate_row(self):
        """Start row navigation"""
        row_id = self.row_id_entry.get()
        self.send_command(f"navigate_row {row_id}")

    def run_full_auto(self):
        """Run full autonomous sequence for a row"""
        row_id = self.auto_row_entry.get()
        self.send_command(f"step_auto {row_id}")

    def send_step_command(self, cmd):
        """Send step command with row ID from the entry field"""
        row_id = self.row_id_entry.get()
        self.send_command(f"{cmd} {row_id}")

    def run_simple_nav(self):
        """Run simple GPS navigation (compass only, stops on obstacle)"""
        row_id = self.row_id_entry.get()
        self.send_command(f"nav_simple {row_id}")

    def run_full_nav_no_scan(self):
        """Run full navigation with visual + obstacles but no tree scanning"""
        row_id = self.row_id_entry.get()
        self.send_command(f"nav_full_no_scan {row_id}")

    def emergency_stop(self):
        """Emergency stop - halt all operations immediately"""
        # Stop motors first
        for key in self.motors_active:
            self.motors_active[key] = False
        self.send_command("stop")
        # Stop any ongoing navigation
        self.send_command("stop_nav")
        # Stop watch mode if active
        if self.watch_active:
            self.stop_hcsr04_watch()
        # Update response area
        self.update_response("EMERGENCY STOP", "All operations halted")

    def start_hcsr04_watch(self):
        """Start HC-SR04 watch mode"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Connect to robot first")
            return

        # Send start command to robot
        response = self.send_command_silent("hcsr04_watch_start")
        if response and "OK" in response:
            self.watch_active = True
            self.watch_start_btn.config(state=tk.DISABLED)
            self.watch_stop_btn.config(state=tk.NORMAL)

            # Update status display
            self.watch_status_label.config(text="WATCHING...", bg='blue', fg='white')

            # Start polling for status updates
            self.watch_poll_thread = threading.Thread(target=self._watch_poll_loop, daemon=True)
            self.watch_poll_thread.start()

            self.update_response("HC-SR04 Watch", "Started - move a tree past the sensor!")

    def stop_hcsr04_watch(self):
        """Stop HC-SR04 watch mode"""
        self.watch_active = False

        # Send stop command
        response = self.send_command_silent("hcsr04_watch_stop")

        # Update buttons
        self.watch_start_btn.config(state=tk.NORMAL)
        self.watch_stop_btn.config(state=tk.DISABLED)

        # Reset display
        self.watch_status_label.config(text="INACTIVE", bg='gray', fg='white')
        self.watch_distance_label.config(text="Distance: --")

        if response:
            self.update_response("HC-SR04 Watch", response)

    def _watch_poll_loop(self):
        """Poll robot for watch mode status updates"""
        while self.watch_active and self.connected:
            try:
                response = self.send_command_silent("hcsr04_watch_status")
                if response:
                    self._update_watch_display(response)
            except:
                pass
            time.sleep(0.05)  # Poll at 20Hz for responsive updates

    def _update_watch_display(self, status_string):
        """Update watch mode display based on status"""
        # Parse status: "STATUS:distance|COUNT:n" or just "STATUS"
        try:
            parts = status_string.split("|")
            status_part = parts[0]
            count = 0

            if len(parts) > 1 and "COUNT:" in parts[1]:
                count = int(parts[1].replace("COUNT:", ""))

            # Parse status and distance
            if ":" in status_part:
                status, dist_str = status_part.split(":", 1)
                distance = int(dist_str)
            else:
                status = status_part
                distance = None

            # Update GUI on main thread
            def update_gui():
                # Update tree count
                self.watch_count_label.config(text=f"Trees Detected: {count}")

                # Update distance
                if distance is not None:
                    self.watch_distance_label.config(text=f"Distance: {distance} cm")
                else:
                    self.watch_distance_label.config(text="Distance: --")

                # Update status with color coding
                if status == "TREE_DETECTED":
                    self.watch_status_label.config(
                        text=f"TREE DETECTED!\n{distance} cm",
                        bg='green',
                        fg='white'
                    )
                elif status == "NO_TREE":
                    self.watch_status_label.config(
                        text="NO TREE",
                        bg='darkgray',
                        fg='white'
                    )
                elif status == "TOO_CLOSE":
                    self.watch_status_label.config(
                        text=f"TOO CLOSE!\n{distance} cm",
                        bg='orange',
                        fg='black'
                    )
                elif status == "NO_SIGNAL":
                    self.watch_status_label.config(
                        text="NO SIGNAL",
                        bg='red',
                        fg='white'
                    )
                elif status == "WATCHING":
                    self.watch_status_label.config(
                        text="WATCHING...",
                        bg='blue',
                        fg='white'
                    )

            self.root.after(0, update_gui)

        except Exception as e:
            print(f"Watch display error: {e}")

    def toggle_stream(self):
        """Toggle video stream"""
        if self.streaming:
            self.stop_stream()
        else:
            self.start_stream()

    def start_stream(self):
        """Start video stream"""
        self.streaming = True
        self.stream_btn.config(text="Stop Stream")
        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.stream_thread.start()

    def stop_stream(self):
        """Stop video stream"""
        self.streaming = False
        self.stream_btn.config(text="Start Stream")

    def _stream_loop(self):
        """Video stream receiving loop - handles both CSI and Thermal"""
        while self.streaming:
            try:
                stream_url = f"http://{self.robot_ip}:{STREAM_PORT}"
                stream = urllib.request.urlopen(stream_url, timeout=10)
                self.stream_status.config(text="Stream: Connected", fg="green")

                buffer = b''
                while self.streaming:
                    buffer += stream.read(1024)

                    # Find JPEG frame
                    start = buffer.find(b'\xff\xd8')
                    end = buffer.find(b'\xff\xd9')

                    if start != -1 and end != -1 and end > start:
                        jpg = buffer[start:end+2]
                        buffer = buffer[end+2:]

                        # Display frame
                        try:
                            # Decode with OpenCV for better handling
                            img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                            if img is not None:
                                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                                image = Image.fromarray(img_rgb)
                                image = image.resize((800, 600), Image.Resampling.LANCZOS)
                                photo = ImageTk.PhotoImage(image)

                                self.video_label.config(image=photo)
                                self.video_label.image = photo
                        except Exception as e:
                            print(f"Frame decode error: {e}")

            except Exception as e:
                self.stream_status.config(text=f"Stream: Error", fg="red")
                print(f"Stream error: {e}")
                time.sleep(2)

    def start_data_stream(self):
        """Start sensor data stream"""
        self.data_thread = threading.Thread(target=self._data_loop, daemon=True)
        self.data_thread.start()

    def stop_data_stream(self):
        """Stop sensor data stream"""
        pass  # Thread will stop when socket closes

    def _data_loop(self):
        """Sensor data receiving loop"""
        while self.connected:
            try:
                data_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                data_socket.settimeout(5)
                data_socket.connect((self.robot_ip, DATA_PORT))

                buffer = ""
                while self.connected:
                    data = data_socket.recv(1024).decode('utf-8')
                    if not data:
                        break

                    buffer += data
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        try:
                            self.sensor_data = json.loads(line)
                            self.root.after(0, self.update_sensor_display)
                        except:
                            pass

            except Exception as e:
                print(f"Data stream error: {e}")
                time.sleep(2)

    def update_sensor_display(self):
        """Update sensor data display"""
        text = ""
        if 'heading' in self.sensor_data:
            heading = self.sensor_data['heading']
            text += f"Heading: {heading:.1f}\n" if heading else "Heading: N/A\n"

        if 'gps' in self.sensor_data and self.sensor_data['gps']:
            gps = self.sensor_data['gps']
            text += f"GPS: {gps.get('latitude', 'N/A'):.6f}, {gps.get('longitude', 'N/A'):.6f}\n"
        else:
            text += "GPS: No fix\n"

        if 'front_distance_mm' in self.sensor_data:
            text += f"Front: {self.sensor_data['front_distance_mm']}mm\n"

        if 'side_distance_cm' in self.sensor_data:
            dist = self.sensor_data['side_distance_cm']
            if dist is not None:
                text += f"Side (HC-SR04): {dist}cm\n"
            else:
                text += "Side (HC-SR04): N/A\n"

        if 'tree_detected' in self.sensor_data:
            text += f"Tree Detected: {self.sensor_data['tree_detected']}\n"

        self.sensor_text.delete('1.0', tk.END)
        self.sensor_text.insert('1.0', text)

    def run(self):
        """Start the application"""
        self.root.mainloop()


def main():
    # Get robot IP from command line or use default
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else None

    if robot_ip:
        print(f"Connecting to robot at {robot_ip}")
    else:
        print("No IP provided, using default. Usage: python3 laptop_controller.py <robot_ip>")

    app = VineyardRobotController(robot_ip)
    app.run()


if __name__ == '__main__':
    main()
