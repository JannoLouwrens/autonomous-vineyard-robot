   94 -          """Background thread for CSI streaming"""
        95 -          while self.running and self.csi_camera:
        93 +      def _csi_stream_loop(self):
        94 +          """Background thread to read frames and encode for streaming."""
        95 +          print("[CSI Camera] Streamer thread started")
        96 +          while self.running and self.csi_camera and self.csi_camera.isOpened():
        97                ret, frame = self.csi_camera.read()
        98                if ret:
        99 +                  # Encode the frame as JPEG for the web stream
       100                    _, buffer = cv2.imencode('.jpg', frame)
       101                    self.streaming_output.write(buffer.tobytes())
       102                else:
       103 +                  print("[CSI Camera] Warning: Failed to grab frame in stream loop.")
       104                    time.sleep(0.1)
       105 +          print("[CSI Camera] Streamer thread stopped")
       106    
       107 -      def _init_thermal(self):
       108 -          """Initialize thermal camera"""
       107 +      def _init_thermal_sensor_object(self):
       108 +          """Helper function to create and configure the MI48 sensor object - FROM WORKING CODE"""
       109            if not THERMAL_AVAILABLE:
       110 -              return
       110 +              return False
       111    
       112 +          # If a sensor object already exists, properly close it before creating a new one
       113 +          if self.thermal_sensor:
       114 +              print("[Thermal Camera] Closing existing thermal sensor object...")
       115 +              try:
       116 +                  self.thermal_sensor.stop()
       117 +                  time.sleep(0.5)
       118 +              except Exception as e:
       119 +                  print(f"[Thermal Camera] Error while stopping existing sensor: {e}")
       120 +  
       121            try:
       122 -              print("[Thermal Camera] Initializing MI48...")
       122 +              print("[Thermal Camera] (Re)Initializing MI48 sensor object...")
       123    
       124                i2c_bus = smbus.SMBus(I2C_BUS)
       125                i2c_if = I2C_Interface(i2c_bus, THERMAL_I2C_ADDR)
       126    
       127                spi_dev = spidev.SpiDev()
       128                spi_dev.open(THERMAL_SPI_BUS, THERMAL_SPI_DEVICE)
       129 -              spi_dev.max_speed_hz = 8000000
       129 +              spi_dev.max_speed_hz = 8000000  # Lowered from 16Mhz to improve stability
       130                spi_if = SPI_Interface(spi_dev, xfer_size=4096)
       131    
       132 -              # Reset pin
       132 +              # Define the hardware reset pin and handler, as per Waveshare documentation
       133                try:
       134                    reset_pin = DigitalOutputDevice(23, active_high=False, initial_value=True)
       135 -                  reset_pin.on()
       136 -                  time.sleep(0.001)
       137 -                  reset_pin.off()
       138 -                  time.sleep(0.1)
       139 -              except:
       140 -                  pass
       135 +                  reset_handler = MI48_reset(pin=reset_pin)
       136 +              except Exception as e:
       137 +                  print(f"[Thermal Camera] ⚠ Could not initialize reset pin: {e}")
       138 +                  reset_handler = None
       139    
       140 -              self.thermal_sensor = MI48(interfaces=[i2c_if, spi_if])
       141 -              self.thermal_sensor.stop_capture()
       142 -              time.sleep(0.2)
       143 -              self.thermal_sensor.start()
       144 -              time.sleep(0.3)
       145 -              self.thermal_sensor.set_fps(10)
       140 +              self.thermal_sensor = MI48(interfaces=[i2c_if, spi_if], reset_handler=reset_handler)
       141 +              return True
       142 +          except Exception as e:
       143 +              print(f"[Thermal Camera] ✗ Failed to create MI48 object: {e}")
       144 +              self.thermal_sensor = None
       145 +              return False
       146    
       147 -              # Warm up
       148 -              print("[Thermal Camera] Warming up...")
       149 -              for _ in range(5):
       150 -                  self.thermal_sensor.read()
       151 -                  time.sleep(0.5)
       147 +      def init_thermal_camera(self):
       148 +          """Initialize thermal camera HAT B, with robust warm-up and recovery - FROM WORKING CODE"""
       149 +          if not THERMAL_AVAILABLE:
       150 +              print("[Thermal Camera] Libraries not available, skipping initialization")
       151 +              return
       152    
       153 -              print("[Thermal Camera] Ready")
       153 +          # First attempt to initialize the sensor object
       154 +          if not self._init_thermal_sensor_object():
       155 +              print("[Thermal Camera] ✗ Initial setup failed. Thermal features disabled.")
       156 +              return
       157 +  
       158 +          try:
       159 +              # Configure sensor for optimal performance
       160 +              print("[Thermal Camera] Configuring sensor...")
       161 +              try:
       162 +                  self.thermal_sensor.stop_capture()
       163 +                  time.sleep(0.2)
       164 +                  self.thermal_sensor.start()
       165 +                  time.sleep(0.3)
       166 +                  print("[Thermal Camera] ✓ Configured for continuous capture")
       167 +              except Exception as config_error:
       168 +                  print(f"[Thermal Camera] ⚠ Configuration warning: {config_error}")
       169 +                  print("[Thermal Camera] Sensor will use default settings")
       170 +  
       171 +              # Set a stable frame rate to prevent overwhelming the SPI bus
       172 +              try:
       173 +                  print("[Thermal Camera] Setting frame rate to 10 FPS...")
       174 +                  self.thermal_sensor.set_fps(10)
       175 +                  print("[Thermal Camera] ✓ Frame rate set")
       176 +              except Exception as fps_error:
       177 +                  print(f"[Thermal Camera] ⚠ Could not set frame rate: {fps_error}")
       178 +  
       179 +              # CRITICAL: Warm-up period - sensor needs time to stabilize (FROM WORKING CODE)
       180 +              print("[Thermal Camera] Warming up sensor (10 seconds)...")
       181 +              print("[Thermal Camera] Please wait - sensor calibrating...")
       182 +              for i in range(10):
       183 +                  try:
       184 +                      self.thermal_sensor.read()
       185 +                      time.sleep(1.0)
       186 +                      print(f"[Thermal Camera] Warm-up: {i+1}/10 frames read")
       187 +                  except Exception as e:
       188 +                      print(f"[Thermal Camera] Error during warm-up read: {e}")
       189 +                      pass
       190 +  
       191 +              # Verify sensor is working with valid data (FROM WORKING CODE - 3 attempts)
       192 +              print("[Thermal Camera] Verifying sensor data...")
       193 +  
       194 +              temp_min = -999
       195 +              for attempt in range(3):
       196 +                  result = self.thermal_sensor.read()
       197 +                  if result:
       198 +                      frame = result[0] if isinstance(result, tuple) else result
       199 +                      if frame is not None:
       200 +                          temp_array = np.array(frame, dtype=np.float32).reshape(62, 80)
       201 +                          temp_min = np.nanmin(temp_array)
       202 +  
       203 +                          if temp_min > -100:
       204 +                              test_max = np.nanmax(temp_array)
       205 +                              print(f"[Thermal Camera] ✓ Sensor data valid on attempt {attempt+1}: min={temp_min:.1f}°C, max={test_max:.1f}°C")
       206 +                              print("[Thermal Camera] ✓ Initialization complete and verified")
       207 +                              return  # Success!
       208 +  
       209 +                  print(f"[Thermal Camera] ✗ Verification attempt {attempt+1} failed. Min temp: {temp_min:.1f}°C")
       210 +                  if attempt < 2:
       211 +                      print("[Thermal Camera] Attempting full sensor re-initialization...")
       212 +                      time.sleep(5)
       213 +                      self._init_thermal_sensor_object()
       214 +                      # After re-initializing, we must re-configure
       215 +                      self.thermal_sensor.stop_capture()
       216 +                      time.sleep(0.2)
       217 +                      self.thermal_sensor.start()
       218 +                      time.sleep(0.3)
       219 +  
       220 +              # If all attempts fail
       221 +              print(f"[Thermal Camera] ✗ All recovery attempts failed. Sensor still invalid.")
       222 +              print(f"[Thermal Camera] Final readings: min={temp_min:.1f}°C")
       223 +              print("[Thermal Camera] ⚠ THERMAL FEATURES DISABLED")
       224 +              print("[Thermal Camera] Try: sudo reboot to reset sensor")
       225 +              self.thermal_sensor = None
       226 +  
       227            except Exception as e:
       228 -              print(f"[Thermal Camera] Init failed: {e}")
       228 +              print(f"[Thermal Camera] ✗ Major initialization failure: {e}")
       229 +              import traceback
       230 +              traceback.print_exc()
       231                self.thermal_sensor = None
       232    
       233 +      def switch_camera(self, camera_type):
       234 +          """Switch active camera"""
       235 +          if camera_type in ["csi", "thermal"]:
       236 +              self.active_camera = camera_type
       237 +              return True
       238 +          return False
       239 +  
       240 +      def get_stream_frame(self):
       241 +          """Get current CSI camera frame for MJPEG streaming."""
       242 +          with self.streaming_output.condition:
       243 +              if self.streaming_output.condition.wait(timeout=1.0):
       244 +                  return self.streaming_output.frame
       245 +          return None
       246 +  
       247        def capture_csi_frame(self):
       248 -          """Capture frame as numpy array"""
       249 -          if not self.csi_camera:
       248 +          """Capture CSI camera frame as a numpy array."""
       249 +          if not self.csi_camera or not self.csi_camera.isOpened():
       250                return None
       251            try:
       252 -              for _ in range(3):  # Clear buffer
       252 +              # Read multiple frames to clear buffer and get the latest one
       253 +              for _ in range(3):
       254                    self.csi_camera.grab()
       255                ret, frame = self.csi_camera.read()
       256 -              return frame if ret else None
       257 -          except:
       256 +  
       257 +              if ret:
       258 +                  return frame
       259 +              else:
       260 +                  return None
       261 +          except Exception as e:
       262 +              print(f"[CSI Camera] Capture error: {e}")
       263                return None
       264    
       265 +      def save_image(self, file_path, use_thermal=False):
       266 +          """Captures and saves a still image."""
       267 +          if use_thermal:
       268 +              img = self.get_thermal_image()
       269 +          else:
       270 +              img = self.capture_csi_frame()
       271 +  
       272 +          if img is not None:
       273 +              try:
       274 +                  cv2.imwrite(file_path, img)
       275 +                  print(f"[Camera] ✓ Image saved to {file_path}")
       276 +                  return True
       277 +              except Exception as e:
       278 +                  print(f"[Camera] ✗ Failed to save image: {e}")
       279 +                  return False
       280 +          return False
       281 +  
       282 +      def get_thermal_image(self):
       283 +          """Get thermal as colorized image for display/saving."""
       284 +          temp_array = self.capture_thermal_array()
       285 +          if temp_array is None:
       286 +              return None
       287 +  
       288 +          try:
       289 +              temp_min = np.nanmin(temp_array)
       290 +              temp_max = np.nanmax(temp_array)
       291 +              temp_mean = np.nanmean(temp_array)
       292 +  
       293 +              # Check for invalid data
       294 +              if np.isnan(temp_min) or np.isnan(temp_max):
       295 +                  print("[Thermal Error] Array contains NaN values!")
       296 +                  return None
       297 +  
       298 +              # Check if temperature range is too small (< 0.5°C)
       299 +              if (temp_max - temp_min) < 0.5:
       300 +                  # Use a fixed range centered on mean temperature
       301 +                  fixed_range = 5.0
       302 +                  center = temp_mean
       303 +                  temp_normalized = np.clip(
       304 +                      ((temp_array - (center - fixed_range/2)) / fixed_range * 255),
       305 +                      0, 255
       306 +                  ).astype(np.uint8)
       307 +              else:
       308 +                  # Normal normalization when we have good temperature variation
       309 +                  temp_normalized = ((temp_array - temp_min) / (temp_max - temp_min) * 255).astype(np.uint8)
       310 +  
       311 +              # Apply JET colormap (blue=cold, red=hot)
       312 +              colored = cv2.applyColorMap(temp_normalized, cv2.COLORMAP_JET)
       313 +  
       314 +              # Resize to camera resolution
       315 +              resized = cv2.resize(colored, CAMERA_RESOLUTION, interpolation=cv2.INTER_NEAREST)
       316 +  
       317 +              # Rotate the image 90 degrees counter-clockwise to correct orientation
       318 +              resized = cv2.rotate(resized, cv2.ROTATE_90_COUNTERCLOCKWISE)
       319 +  
       320 +              # Add temperature overlay text
       321 +              cv2.putText(resized, f"Min: {temp_min:.1f}C Max: {temp_max:.1f}C",
       322 +                         (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
       323 +  
       324 +              # Cache the successfully created frame
       325 +              self.last_good_thermal_frame = resized
       326 +  
       327 +              return resized
       328 +  
       329 +          except Exception as e:
       330 +              print(f"[Thermal Camera] Frame encoding error: {e}")
       331 +              return None
       332 +  
       333        def capture_thermal_array(self):
       334 -          """Capture thermal as temperature array (62x80)"""
       334 +          """
       335 +          Capture thermal camera frame, waiting for DATA_READY signal - FROM WORKING CODE
       336 +          This is the most robust method for preventing data corruption.
       337 +          """
       338            if not self.thermal_sensor:
       339                return None
       340 +  
       341            try:
       342 -              # Wait for data ready
       343 -              for _ in range(50):
       342 +              # 1. Wait for the DATA_READY flag from the sensor's status register.
       343 +              # This ensures we don't try to read while the sensor is writing.
       344 +              wait_start_time = time.time()
       345 +              data_ready = False
       346 +              while not data_ready:
       347                    status = self.thermal_sensor.get_status()
       348 -                  if status and (status & 0x10):
       349 -                      break
       350 -                  time.sleep(0.01)
       348 +                  if status is not None and (status & 0x10):  # 0x10 is the DATA_READY bit
       349 +                      data_ready = True
       350 +                  elif time.time() - wait_start_time > 0.5:  # 500ms timeout
       351 +                      print("[Thermal ERROR] Timed out waiting for DATA_READY flag.")
       352 +                      return None
       353 +                  else:
       354 +                      time.sleep(0.01)  # Small delay before polling again
       355    
       356 +              # 2. Once data is ready, perform the read.
       357                result = self.thermal_sensor.read()
       358 +  
       359                if result is None:
       360 +                  print("[Thermal Debug] read() returned None even after DATA_READY.")
       361                    return None
       362    
       363 +              # 3. Process the frame
       364                frame = result[0] if isinstance(result, tuple) else result
       365                temp_array = np.array(frame, dtype=np.float32)
       366    
       367                if temp_array.size == 4960:
       368                    temp_array = temp_array.reshape(62, 80)
       369                else:
       370 +                  print(f"[Thermal Warning] Unexpected data size after DATA_READY: {temp_array.size}")
       371                    return None
       372    
       373 -              # Validate
       374 -              if np.nanmin(temp_array) < -100:
       373 +              # 4. Final validation
       374 +              temp_min_check = np.nanmin(temp_array)
       375 +              if temp_min_check < -100:
       376 +                  print(f"[Thermal ERROR] Invalid data read despite DATA_READY. Min temp: {temp_min_check:.2f}°C")
       377                    return None
       378    
       379                return temp_array
       380 +  
       381            except Exception as e:
       382 -              print(f"[Thermal] Capture error: {e}")
       382 +              print(f"[Thermal Camera] Capture error: {e}")
       383                return None
       384    
       385 -      def get_thermal_image(self):
       386 -          """Get thermal as colorized image"""
       387 -          temp_array = self.capture_thermal_array()
       388 -          if temp_array is None:
       385 +      def get_active_frame(self):
       386 +          """Get frame from currently active camera"""
       387 +          if self.active_camera == "csi":
       388 +              return self.get_stream_frame()
       389 +          else:
       390 +              img = self.get_thermal_image()
       391 +              if img is not None:
       392 +                  _, jpeg = cv2.imencode('.jpg', img)
       393 +                  return jpeg.tobytes()
       394                return None
       395    
       396 -          # Normalize and colorize
       397 -          temp_min = np.nanmin(temp_array)
       398 -          temp_max = np.nanmax(temp_array)
       399 -  
       400 -          if temp_max - temp_min < 0.5:
       401 -              # Fixed range if too uniform
       402 -              center = np.nanmean(temp_array)
       403 -              temp_min, temp_max = center - 2.5, center + 2.5
       404 -  
       405 -          normalized = ((temp_array - temp_min) / (temp_max - temp_min) * 255).astype(np.uint8)
       406 -          colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
       407 -          resized = cv2.resize(colored, CAMERA_RESOLUTION, interpolation=cv2.INTER_NEAREST)
       408 -          resized = cv2.rotate(resized, cv2.ROTATE_90_COUNTERCLOCKWISE)
       409 -  
       410 -          # Add text overlay
       411 -          cv2.putText(resized, f"Min: {temp_min:.1f}C Max: {temp_max:.1f}C",
       412 -                      (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
       413 -  
       414 -          return resized
       415 -  
       416 -      def get_stream_frame(self):
       417 -          """Get JPEG frame for MJPEG stream"""
       418 -          with self.streaming_output.condition:
       419 -              if self.streaming_output.condition.wait(timeout=1.0):
       420 -                  return self.streaming_output.frame
       421 -          return None
       422 -  
       423 -      def switch_camera(self, camera_type):
       424 -          """Switch active camera"""
       425 -          if camera_type in ["csi", "thermal"]:
       426 -              self.active_camera = camera_type
       427 -              return True
       428 -          return False
       429 -  
       396        def is_thermal_available(self):
       397 +          """Check if thermal camera is available"""
       398            return self.thermal_sensor is not None
       399    
       400 -      def save_image(self, filepath, use_thermal=False):
       401 -          """Save image to file"""
       402 -          try:
       403 -              if use_thermal:
       404 -                  img = self.get_thermal_image()
       405 -              else:
       406 -                  img = self.capture_csi_frame()
       407 -  
       408 -              if img is not None:
       409 -                  cv2.imwrite(filepath, img)
       410 -                  return True
       411 -          except Exception as e:
       412 -              print(f"[Camera] Save error: {e}")
       413 -          return False
       414 -  
       400        def cleanup(self):
       401 +          """Cleanup cameras on shutdown"""
       402            self.running = False
       403 -          if self.csi_thread:
       403 +          if self.csi_thread and self.csi_thread.is_alive():
       404                self.csi_thread.join(timeout=1.0)
       405            if self.csi_camera:
       406                self.csi_camera.release()

● Now let me fix the test_hardware.py to match the sensor variable names:

● Update(Updated FULL Code Standalone/test_hardware.py)
  ⎿  Updated Updated FULL Code Standalone/test_hardware.py with 2 additions and 2 removals
       84    
       85            print(f"\nHeading: {sensors.last_heading}")
       86            print(f"GPS: {sensors.last_position}")
       87 -          print(f"Front ultrasonic: {sensors.last_front_distance_mm}mm")
       88 -          print(f"Side HC-SR04: {sensors.last_side_distance_cm}cm")
       87 +          print(f"Front ultrasonic: {sensors.last_distance}mm")
       88 +          print(f"Side HC-SR04: {sensors.last_hcsr04_distance}cm")
       89            print(f"Tree detected: {sensors.is_tree_on_side()}")
       90    
       91            time.sleep(1)

● Update(Updated FULL Code Standalone/test_hardware.py)
  ⎿  Updated Updated FULL Code Standalone/test_hardware.py with 1 addition and 1 removal
       134    
       135        print("Reading HC-SR04 distance (10 readings)...")
       136        for i in range(10):
       137 -          dist = sensor.read_distance_cm()
       137 +          dist = sensor.read_distance()
       138            tree = sensor.is_tree_detected()
       139            print(f"  {i+1}: {dist}cm - Tree: {tree}")
       140            time.sleep(0.5)

● Now all sensors match the working code. Here's the complete answer to your questions:

  ---
  Sensor Hardware Implementation Status

  YES, all sensors now use the exact same implementation as your working code:

  | Sensor                | Implementation                                                                              |
  |-----------------------|---------------------------------------------------------------------------------------------|
  | BNO055 Compass        | Same register addresses (0x1A/0x1B for heading), NDOF mode, config mode sequence            |
  | GPS (NEO-8M)          | Same $GNGLL parsing, same NMEA coordinate conversion, same fix quality checks               |
  | Front Ultrasonic      | Same I2C register reads (0x1a, 0x1b) from Raspbot motor controller                          |
  | Thermal Camera (MI48) | Same MI48_reset class, same 10-second warm-up, same DATA_READY polling, same recovery logic |
  | HC-SR04 (NEW)         | Uses gpiozero DistanceSensor on GPIO 5/6                                                    |

  ---
  How The Complete System Works

  1. Starting the Robot

  On the Raspberry Pi, run:
  cd /home/pi/Working\ Code/Updated\ FULL\ Code\ Standalone
  python3 main.py

  This starts:
  - Command server on port 8890 (receives commands)
  - Video stream on port 8891 (MJPEG)
  - Data stream on port 8892 (sensor JSON)

  2. Laptop Controller

  On your laptop, run:
  python3 laptop_controller.py

  Enter the robot's IP address and click "Connect".

  3. Setting GPS Row Waypoints

  Step 1: Drive to row START
  - Use motor controls (W/A/S/D keys or buttons) to manually drive the robot to the beginning of a tree row.

  Step 2: Mark START position
  - Click "Set Row START (current GPS)" button
  - The robot saves current GPS coordinates as the start waypoint

  Step 3: Drive to row END
  - Manually drive to the end of the same row

  Step 4: Mark END position
  - Click "Set Row END (current GPS)" button
  - The robot saves current GPS coordinates as the end waypoint

  Step 5: Save the row
  - Click "Save Row" button
  - This creates a named row entry (Row 1, Row 2, etc.) with start/end GPS coordinates
  - Click "List Rows" to see all saved rows

  4. Running Autonomous Survey

  Start navigation:
  1. Enter the row ID (e.g., "1") in the Row ID field
  2. Click "Navigate Row"

  What happens automatically:

  ┌─────────────────────────────────────────────────────────────────┐
  │  1. NAVIGATE TO START                                           │
  │     - Robot calculates bearing from current GPS to START GPS    │
  │     - Uses compass to turn toward START                         │
  │     - Drives forward until within 1.5m of START waypoint        │
  ├─────────────────────────────────────────────────────────────────┤
  │  2. ORIENT TOWARD END                                           │
  │     - Calculates bearing from START to END                      │
  │     - Turns to face END direction                               │
  │     - Camera detects row "walls" for visual guidance            │
  ├─────────────────────────────────────────────────────────────────┤
  │  3. DRIVE ALONG ROW (Main Loop)                                 │
  │                                                                 │
  │     While not at END waypoint:                                  │
  │       a) Check front ultrasonic for obstacles                   │
  │          - < 200mm: STOP                                        │
  │          - < 500mm: SLOW DOWN                                   │
  │          - < 400mm: AVOID (turn, go forward, return)            │
  │                                                                 │
  │       b) Check HC-SR04 (left side) for trees                    │
  │          - 30cm < distance < 150cm = tree detected              │
  │          - Requires 3 consecutive readings to confirm           │
  │                                                                 │
  │       c) Row detection keeps robot centered in row              │
  │          - Camera finds vanishing point                         │
  │          - Adjusts steering to stay on course                   │
  ├─────────────────────────────────────────────────────────────────┤
  │  4. TREE INSPECTION (When HC-SR04 detects tree)                 │
  │                                                                 │
  │     a) YOLO verifies it's a tree trunk (optional)               │
  │                                                                 │
  │     b) TRUNK DOWN SCAN:                                         │
  │        - Servo tilts camera down toward ground                  │
  │        - Visual servoing centers on trunk                       │
  │        - Thermal camera captures ground temperature             │
  │        - Analyzes for water (cold spots = wet soil)             │
  │                                                                 │
  │     c) TRUNK UP SCAN:                                           │
  │        - Servo tilts camera up toward canopy                    │
  │        - Visual servoing follows trunk upward                   │
  │        - Captures canopy photo                                  │
  │        - Detects green leaf percentage                          │
  │                                                                 │
  │     d) LOG RESULTS:                                             │
  │        - GPS location                                           │
  │        - Water detection result                                 │
  │        - Canopy image path                                      │
  │        - Saves to data/inspection_log.json                      │
  ├─────────────────────────────────────────────────────────────────┤
  │  5. COMPLETE                                                    │
  │     - Robot reaches END waypoint                                │
  │     - Stops and reports completion                              │
  └─────────────────────────────────────────────────────────────────┘

  5. Test Individual Components

  Use the test buttons in the laptop GUI:

  | Button             | What it tests                   |
  |--------------------|---------------------------------|
  | Test Row Detection | Camera row/wall detection       |
  | Test YOLO Verify   | Tree classification             |
  | Test Trunk Down    | Servo down + visual tracking    |
  | Test Trunk Up      | Servo up + visual tracking      |
  | Test Thermal/Water | Thermal camera + water analysis |
  | Test HC-SR04       | Side ultrasonic sensor          |
  | Full Tree Scan     | Complete inspection sequence    |

  6. Data Output

  After surveying, data is saved to:
  - /home/pi/Working Code/Updated FULL Code Standalone/data/inspection_log.json - All inspection results
  - /home/pi/Working Code/Updated FULL Code Standalone/data/rows.json - Saved row waypoints
  - /home/pi/Working Code/Updated FULL Code Standalone/data/canopy_images/ - Canopy photos

  Export to CSV:
  python3 -c "from data.data_logger import DataLogger; DataLogger().export_csv()"

  ---
  Quick Start Commands

  # On Pi - start robot
  cd /home/pi/Working\ Code/Updated\ FULL\ Code\ Standalone
  python3 main.py

  # On Pi - test hardware
  python3 test_hardware.py sensors
  python3 test_hardware.py hcsr04
  python3 test_hardware.py camera

  # On laptop - start controller
  python3 laptop_controller.py

> YOLO verifying isnt optional its required. and ultrasonic isnt one detection enough? and servo cant go full left like but half left so maybe robot must turn but it can turn on spot right... until tree is in vision... understand? why
 do you say YOLO is optional? it can be many things like posts ultrasonic sees and why must it be 3 consecutive when robot moves woont it be 1? 

● You're right on all points. Let me clarify my understanding:

  Your Corrections:

  1. YOLO is REQUIRED, not optional

  - HC-SR04 detects anything - posts, poles, fence, debris
  - YOLO must verify it's actually a tree trunk before inspection
  - Without YOLO verification, robot would waste time scanning posts

  2. Single HC-SR04 detection is enough

  - Robot is moving, so consecutive readings don't make sense
  - One detection triggers: STOP → turn to face → YOLO verify
  - The 3-consecutive check is for stationary scenarios, not this

  3. Servo can't pan far enough - robot must turn

  - Servo has limited pan range (not full 90° left)
  - When HC-SR04 detects something on left side:
    a. Robot STOPS
    b. Robot TURNS LEFT on the spot (it can spin in place, right?)
    c. Camera now faces the detected object
    d. YOLO checks if it's a tree trunk
    e. If YES → proceed with trunk down/up scan
    f. If NO (post, etc.) → turn back, continue forward

  Updated Flow:

  HC-SR04 detects object (30-150cm on left)
           ↓
      Robot STOPS
           ↓
      Robot TURNS LEFT (on spot)
           ↓
      Camera now faces object
           ↓
      YOLO: Is this a tree trunk?
          ↓              ↓
         YES             NO
          ↓              ↓
     Trunk scan     Turn back right
     (down→up)      Continue forward
          ↓
     Log results
          ↓
     Turn back right
     Continue forward

  Should I update the code to:
  1. Remove the 3-consecutive trigger requirement (single detection)
  2. Make YOLO verification mandatory (not skippable)
  3. Add robot turn-to-face logic before YOLO check?

> implement 1 and 2 

● Read(Updated FULL Code Standalone/navigation/tree_scanner.py)
  ⎿  Read 185 lines

● Update(Updated FULL Code Standalone/navigation/tree_scanner.py)
  ⎿  Updated Updated FULL Code Standalone/navigation/tree_scanner.py with 16 additions and 13 removals                                    
       72                result['reason'] = 'camera_failed'
       73                return result
       74    
       75 -          if self.yolo and self.yolo.is_available():
       76 -              verify_result = self.yolo.verify_tree(frame)
       77 -              if not verify_result['verified']:
       78 -                  print(f"[Tree Scanner] Not a valid tree - skipping")
       79 -                  result['reason'] = 'yolo_rejected'
       80 -                  self.servos.center()
       81 -                  return result
       82 -              result['verified'] = True
       83 -              print(f"[Tree Scanner] Tree verified: {verify_result['detection']['class']} ({verify_result['confidence']:.2f})")
       84 -          else:
       85 -              # No YOLO, assume it's a tree
       86 -              print("[Tree Scanner] YOLO not available, assuming valid tree")
       87 -              result['verified'] = True
       75 +          # YOLO verification is MANDATORY - HC-SR04 can detect posts, poles, etc.
       76 +          if not self.yolo or not self.yolo.is_available():
       77 +              print("[Tree Scanner] ERROR: YOLO not available - cannot verify tree")
       78 +              result['reason'] = 'yolo_unavailable'
       79 +              self.servos.center()
       80 +              return result
       81    
       82 +          verify_result = self.yolo.verify_tree(frame)
       83 +          if not verify_result['verified']:
       84 +              print(f"[Tree Scanner] Not a valid tree (probably post/pole) - skipping")
       85 +              result['reason'] = 'yolo_rejected'
       86 +              self.servos.center()
       87 +              return result
       88 +  
       89 +          result['verified'] = True
       90 +          print(f"[Tree Scanner] Tree verified: {verify_result['detection']['class']} ({verify_result['confidence']:.2f})")
       91 +  
       92            # Step 3: Follow trunk down to base
       93            print("[Step 3] Following trunk down to base...")
       94            base_result = self.trunk_follower.follow_down_to_base()

● Update(Updated FULL Code Standalone/navigation/tree_scanner.py)
  ⎿  Updated Updated FULL Code Standalone/navigation/tree_scanner.py with 6 additions and 6 removals
       151                self.servos.center()
       152                return False
       153    
       154 -          if self.yolo and self.yolo.is_available():
       155 -              result = self.yolo.verify_tree(frame)
       154 +          # YOLO verification is MANDATORY
       155 +          if not self.yolo or not self.yolo.is_available():
       156 +              print("[Tree Scanner] ERROR: YOLO not available for verification")
       157                self.servos.center()
       158 -              return result['verified']
       158 +              return False
       159    
       160 -          # Fallback: check for trunk using color
       161 -          result = self.trunk_follower.detect_trunk(frame)
       160 +          result = self.yolo.verify_tree(frame)
       161            self.servos.center()
       162 -          return result['found']
       162 +          return result['verified']
       163    
       164        def test_thermal(self):
       165            """Test thermal water detection at current position"""

● Now let me update the sensors.py to remove the 3-consecutive requirement:

● Update(Updated FULL Code Standalone/core/sensors.py)
  ⎿  Updated Updated FULL Code Standalone/core/sensors.py with 1 removal
       216            self.echo_pin = echo_pin
       217            self.sensor = None
       218            self.enabled = False
       219 -          self.trigger_count = 0
       219            self.init_sensor()
       220    
       221        def init_sensor(self):

● Update(Updated FULL Code Standalone/core/sensors.py)
  ⎿  Updated Updated FULL Code Standalone/core/sensors.py with 10 additions and 12 removals                
       254            return None
       255    
       256        def is_tree_detected(self):
       257 -          """Check if tree is within detection range"""
       257 +          """
       258 +          Check if object is within detection range (single reading)
       259 +          Robot is moving, so single detection is enough to trigger stop + verify
       260 +          """
       261            dist = self.read_distance()
       262            if dist is None:
       263                return False
       264            return HCSR04_TREE_DETECT_MIN_CM < dist < HCSR04_TREE_DETECT_MAX_CM
       265    
       266 -      def confirm_tree(self):
       267 -          """Confirm tree with multiple consecutive readings"""
       268 -          if self.is_tree_detected():
       269 -              self.trigger_count += 1
       270 -          else:
       271 -              self.trigger_count = 0
       272 -          return self.trigger_count >= HCSR04_TRIGGER_COUNT
       266 +      def get_detection_distance(self):
       267 +          """Get distance if object detected, else None"""
       268 +          dist = self.read_distance()
       269 +          if dist is not None and HCSR04_TREE_DETECT_MIN_CM < dist < HCSR04_TREE_DETECT_MAX_CM:
       270 +              return dist
       271 +          return None
       272    
       273 -      def reset_trigger(self):
       274 -          """Reset the trigger counter"""
       275 -          self.trigger_count = 0
       276 -  
       273        def cleanup(self):
       274            """Cleanup GPIO resources"""
       275            if self.sensor:

● Now update the SensorManager to remove the old methods:

● Update(Updated FULL Code Standalone/core/sensors.py)
  ⎿  Updated Updated FULL Code Standalone/core/sensors.py with 4 additions and 8 removals          
       326            return self.last_distance < OBSTACLE_STOP_DISTANCE_MM
       327    
       328        def is_tree_on_side(self):
       329 -          """Check for tree on left side"""
       329 +          """Check for object on left side (single detection triggers stop + verify)"""
       330            return self.hcsr04.is_tree_detected()
       331    
       332 -      def confirm_tree_on_side(self):
       333 -          """Confirm tree with multiple readings"""
       334 -          return self.hcsr04.confirm_tree()
       332 +      def get_side_detection_distance(self):
       333 +          """Get distance to detected object on side, or None"""
       334 +          return self.hcsr04.get_detection_distance()
       335    
       336 -      def reset_tree_trigger(self):
       337 -          """Reset tree detection trigger"""
       338 -          self.hcsr04.reset_trigger()
       339 -  
       336        def is_ready_for_navigation(self):
       337            """Check if sensors are ready for autonomous navigation"""
       338            gps_ok = self.last_position is not None

● Now update the config.py to remove the unused HCSR04_TRIGGER_COUNT:

● Update(Updated FULL Code Standalone/config.py)
  ⎿  Updated Updated FULL Code Standalone/config.py with 1 addition and 1 removal                   
       69    HCSR04_TREE_DETECT_MIN_CM = 30       # Closer than this = too close
       70    HCSR04_TREE_DETECT_MAX_CM = 150      # Further than this = no tree
       71    HCSR04_SCAN_INTERVAL_MS = 100
       72 -  HCSR04_TRIGGER_COUNT = 3             # Consecutive readings to confirm
       72 +  # Single detection triggers stop + YOLO verify (robot is moving, no consecutive needed)
       73    
       74    # ============================================================================
       75    # FRONT ULTRASONIC - Obstacle Avoidance

● Done. Here's what I changed:

  Changes Made:

  1. HC-SR04 Single Detection (sensors.py)

  - Removed trigger_count variable
  - Removed confirm_tree() method (3-consecutive check)
  - Removed reset_trigger() method
  - is_tree_detected() now returns True on single detection
  - Added get_detection_distance() to get the actual distance

  2. YOLO Verification Mandatory (tree_scanner.py)

  - scan_tree() now fails immediately if YOLO is not available
  - Returns reason: 'yolo_unavailable' instead of assuming valid tree
  - quick_verify() also requires YOLO - no fallback to color detection

  Updated Flow:

  HC-SR04 detects object (single reading, 30-150cm)
           ↓
      Robot STOPS
           ↓
      Robot TURNS LEFT
           ↓
      YOLO: Is this a tree trunk?
           ↓
      NO YOLO? → ERROR, skip
      NOT TREE? → Skip (post/pole)
      IS TREE? → Proceed with scan

> Connect via: telnet <raspberry_pi_ip> 8890
Video stream: http://<raspberry_pi_ip>:8891
[Stream Server] Data error: 'SensorManager' object has no attribute 'last_front_distance_mm'
[Stream Server] Data error: 'SensorManager' object has no attribute 'last_front_distance_mm'
[Stream Server] Data error: 'SensorManager' object has no attribute 'last_front_distance_mm'
[Stream Server] Data error: 'SensorManager' object has no attribute 'last_front_distance_mm'
[Stream Server] Data error: 'SensorManager' object has no attribute 'last_front_distance_mm'
[Stream Server] Data error: 'SensorManager' object has no attribute 

● The stream server is using old attribute names. Let me fix that:

● Read(Updated FULL Code Standalone/network/stream_server.py)
  ⎿  Read 199 lines

● Update(Updated FULL Code Standalone/network/stream_server.py)
  ⎿  Updated Updated FULL Code Standalone/network/stream_server.py with 2 additions and 2 removals
       143                    data = {
       144                        'timestamp': time.time(),
       145                        'sensors': self.sensors.get_status(),
       146 -                      'front_distance_mm': self.sensors.last_front_distance_mm,
       147 -                      'side_distance_cm': self.sensors.last_side_distance_cm,
       146 +                      'front_distance_mm': self.sensors.last_distance,
       147 +                      'side_distance_cm': self.sensors.last_hcsr04_distance,
       148                        'heading': self.sensors.last_heading,
       149                        'gps': self.sensors.last_position
       150                    }

● Fixed. The stream server was using last_front_distance_mm and last_side_distance_cm but the SensorManager uses last_distance and last_hcsr04_distance. Restart the robot and it should work now.

> ======================================================
VINEYARD ROBOT STARTING
Tree Type: vineyard
============================================================

[Motors] Initializing...
[Motors] Ready
[Servos] Initializing...
[Servos] Ready - centered

=== Initializing Sensors ===
[Compass] Initializing BNO055...
[Compass] ✓ BNO055 initialized - Move in figure-8 for calibration
[GPS] Initializing on /dev/ttyAMA0...
[GPS] ✓ GPS initialized
[Ultrasonic] Initializing...
[Ultrasonic] ✓ Sensor enabled
[HC-SR04] Initializing on GPIO 5 (TRIG), GPIO 6 (ECHO)...
/usr/lib/python3/dist-packages/gpiozero/input_devices.py:852: PWMSoftwareFallback: For more accurate readings, use the pigpio pin factory.See https://gpiozero.readthedocs.io/en/stable/api_input.html#distancesensor-hc-sr04 for more 
info
  warnings.warn(PWMSoftwareFallback(
[HC-SR04] ✓ Sensor enabled
=== Sensors Ready ===


=== Initializing Cameras ===
[CSI Camera] Initializing with OpenCV (scanning for camera)...
/usr/lib/python3/dist-packages/gpiozero/input_devices.py:975: DistanceSensorNoEcho: no echo received
  warnings.warn(DistanceSensorNoEcho('no echo received'))
[CSI Camera] ✓ Found available camera at index 0
[CSI Camera] Streamer thread started
[CSI Camera] ✓ OpenCV capture active
[Thermal Camera] (Re)Initializing MI48 sensor object...
[Thermal Camera] Performing hardware reset...
[Thermal Camera] ✓ Hardware reset complete.
WARNING:root:MI48         Capture in progress: 0x03
WARNING:root:MI48         Single capture in progress, Continuous streaming
WARNING:root:MI48         Capture in progress: 0x03
WARNING:root:MI48         Single capture in progress, Continuous streaming
WARNING:root:MI48         Capture in progress: 0x03
WARNING:root:MI48         Single capture in progress, Continuous streaming
WARNING:root:MI48         FRAME_MODE: 0x3 (expected 0x20)
WARNING:root:MI48         STATUS: 0x2 (expected 0x0)
WARNING:root:MI48         SENS_FACTOR: 0x6b (expected 0x64)
WARNING:root:MI48         FILTER_CTRL: 0x1 (expected 0x0)
WARNING:root:MI48         FILTER_1_LSB: 0xa (expected 0x32)
WARNING:root:MI48         FILTER_2: 0xff (expected 0x4)
WARNING:root:MI48         Capture in progress: 0x03
WARNING:root:MI48         Single capture in progress, Continuous streaming
WARNING:root:MI48         Attempting MI48.stop_capture()
WARNING:root:MI48         Capture in progress: 0x03
WARNING:root:MI48         Single capture in progress, Continuous streaming
WARNING:root:MI48         Capture in progress: 0x03
WARNING:root:MI48         Single capture in progress, Continuous streaming
WARNING:root:MI48         Capture in progress: 0x03
WARNING:root:MI48         Single capture in progress, Continuous streaming
[Thermal Camera] Configuring sensor...
WARNING:root:MI48         Capture in progress: 0x03
WARNING:root:MI48         Single capture in progress, Continuous streaming
WARNING:root:MI48         Capture in progress: 0x03
WARNING:root:MI48         Single capture in progress, Continuous streaming
WARNING:root:MI48         Capture in progress: 0x03
WARNING:root:MI48         Single capture in progress, Continuous streaming
[Thermal Camera] ✓ Configured for continuous capture
[Thermal Camera] Setting frame rate to 10 FPS...
[Thermal Camera] ✓ Frame rate set
[Thermal Camera] Warming up sensor (10 seconds)...
[Thermal Camera] Please wait - sensor calibrating...
[Thermal Camera] Error during warm-up read: could not broadcast input array from shape (2048,) into shape (944,)
[Thermal Camera] Error during warm-up read: could not broadcast input array from shape (2048,) into shape (944,)
[Thermal Camera] Error during warm-up read: could not broadcast input array from shape (2048,) into shape (944,)
[Thermal Camera] Error during warm-up read: could not broadcast input array from shape (2048,) into shape (944,)
[Thermal Camera] Error during warm-up read: could not broadcast input array from shape (2048,) into shape (944,)
[Thermal Camera] Error during warm-up read: could not broadcast input array from shape (2048,) into shape (944,)
[Thermal Camera] Error during warm-up read: could not broadcast input array from shape (2048,) into shape (944,)
[Thermal Camera] Error during warm-up read: could not broadcast input array from shape (2048,) into shape (944,)
[Thermal Camera] Error during warm-up read: could not broadcast input array from shape (2048,) into shape (944,)
[Thermal Camera] Error during warm-up read: could not broadcast input array from shape (2048,) into shape (944,)
[Thermal Camera] Verifying sensor data...
[Thermal Camera] ✗ Major initialization failure: could not broadcast input array from shape (2048,) into shape (944,)
Traceback (most recent call last):
  File "/home/pi/Working Code/Updated FULL Code Standalone/core/cameras.py", line 218, in init_thermal_camera
    result = self.thermal_sensor.read()
             ^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/pi/pysenxor-master/senxor/mi48.py", line 355, in read
    response = self.interfaces[1].read(size_in_words)
               ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/pi/pysenxor-master/senxor/interfaces.py", line 104, in read
    data[i0: i1] = _data
    ~~~~^^^^^^^^
ValueError: could not broadcast input array from shape (2048,) into shape (944,)
=== Cameras Ready ===

[Row Detector] Initialized
[YOLO] Model not found at /home/pi/Working Code/Updated FULL Code Standalone/training/models/tree_detector.pt
[YOLO] Run training script first to create model
[Canopy Detector] Initialized
[Trunk Follower] Initialized
[Water Detector] Initialized
  Threshold: mean - 1.5*std
  Min water %: 10.0
[Obstacle Avoider] Initialized
[Row Navigator] Initialized
[Tree Scanner] Initialized
[Command Server] Initialized on port 8890
[Stream Server] Video port: 8891, Data port: 8892

============================================================
ROBOT READY
============================================================

[Command Server] Listening on port 8890
[Stream Server] Started

[Robot] Servers started - ready for commands

Robot running. Press Ctrl+C to stop.
Connect via: telnet <raspberry_pi_ip> 8890
Video stream: http://<raspberry_pi_ip>:8891 
  ⎿  Credit balance too low · Add funds: https://console.anthropic.com/settings/billing

● How did that compaction go? (optional)
  1: Bad    2: Fine   3: Good   0: Dismiss

──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
>  
──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
  ⏵⏵ accept edits on (shift+tab to cycle)                                                                                                                                                    Update available! Run: brew upgrade claude-code




