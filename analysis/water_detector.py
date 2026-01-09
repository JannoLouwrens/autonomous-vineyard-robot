#!/usr/bin/env python3
"""
Water Detector - Thermal-based soil moisture detection
Wet soil is cooler due to evaporative cooling
"""

import numpy as np
import time
import sys
sys.path.append('/home/pi/Working Code/Updated FULL Code Standalone')
from config import *


class WaterDetector:
    """
    Detects water/moisture in soil using thermal camera
    Water causes cooling due to evaporation, showing as cooler spots
    """

    def __init__(self, camera, servos):
        self.camera = camera
        self.servos = servos
        print("[Water Detector] Initialized")
        print(f"  Threshold: mean - {WATER_STD_MULTIPLIER}*std")
        print(f"  Min water %: {WATER_MIN_PERCENTAGE}")

    def analyze_ground(self, pan=None, tilt=None):
        """
        Analyze ground for water at specified servo position
        Args:
            pan: servo pan angle (None = use current)
            tilt: servo tilt angle (None = use SERVO_TILT_GROUND)
        Returns: {
            'has_water': bool,
            'water_percentage': float,
            'mean_temp_c': float,
            'min_temp_c': float,
            'max_temp_c': float,
            'temp_delta_c': float,
            'confidence': 'high'|'medium'|'low',
            'error': str (if failed)
        }
        """
        if not self.camera.is_thermal_available():
            return {'has_water': False, 'error': 'thermal_unavailable'}

        # Position camera
        if pan is not None:
            self.servos.set_pan(pan)
        if tilt is not None:
            self.servos.set_tilt(tilt)
        else:
            self.servos.set_tilt(SERVO_TILT_GROUND)

        time.sleep(THERMAL_CAPTURE_DELAY_SEC)

        # Capture thermal
        temp_array = self.camera.capture_thermal_array()
        if temp_array is None:
            return {'has_water': False, 'error': 'capture_failed'}

        # Calculate statistics
        mean_temp = float(np.nanmean(temp_array))
        min_temp = float(np.nanmin(temp_array))
        max_temp = float(np.nanmax(temp_array))
        std_temp = float(np.nanstd(temp_array))
        temp_delta = mean_temp - min_temp

        # Water threshold: cooler than mean - (std * multiplier)
        threshold = mean_temp - (WATER_STD_MULTIPLIER * std_temp)

        # Count water pixels (cooler than threshold)
        water_mask = temp_array < threshold
        water_pixels = np.sum(water_mask)
        total_pixels = temp_array.size
        water_percentage = (water_pixels / total_pixels) * 100

        # Determine if water present
        has_water = (
            water_percentage > WATER_MIN_PERCENTAGE or
            temp_delta > WATER_MIN_TEMP_DELTA_C
        )

        # Confidence level
        if water_percentage > WATER_HIGH_CONFIDENCE_PCT or temp_delta > WATER_HIGH_CONFIDENCE_DELTA:
            confidence = 'high'
        elif water_percentage > WATER_MEDIUM_CONFIDENCE_PCT or temp_delta > WATER_MEDIUM_CONFIDENCE_DELTA:
            confidence = 'medium'
        else:
            confidence = 'low'

        result = {
            'has_water': has_water,
            'water_percentage': water_percentage,
            'mean_temp_c': mean_temp,
            'min_temp_c': min_temp,
            'max_temp_c': max_temp,
            'std_temp_c': std_temp,
            'temp_delta_c': temp_delta,
            'threshold_temp_c': threshold,
            'confidence': confidence
        }

        # Log
        status = "WATER" if has_water else "DRY"
        print(f"[Water Detector] {status} | {water_percentage:.1f}% cool | delta:{temp_delta:.1f}C | {confidence}")

        return result

    def get_thermal_stats(self):
        """Get thermal camera statistics (for debugging)"""
        temp_array = self.camera.capture_thermal_array()
        if temp_array is None:
            return None

        return {
            'min': float(np.nanmin(temp_array)),
            'max': float(np.nanmax(temp_array)),
            'mean': float(np.nanmean(temp_array)),
            'std': float(np.nanstd(temp_array)),
            'shape': temp_array.shape
        }

    def analyze_canopy_stress(self, pan=None, tilt=None):
        """
        Analyze canopy for water stress (hot spots = stressed leaves)
        """
        if not self.camera.is_thermal_available():
            return {'water_stress': False, 'error': 'thermal_unavailable'}

        if pan is not None:
            self.servos.set_pan(pan)
        if tilt is not None:
            self.servos.set_tilt(tilt)
        else:
            self.servos.set_tilt(SERVO_TILT_CANOPY)

        time.sleep(THERMAL_CAPTURE_DELAY_SEC)

        temp_array = self.camera.capture_thermal_array()
        if temp_array is None:
            return {'water_stress': False, 'error': 'capture_failed'}

        mean_temp = float(np.nanmean(temp_array))
        max_temp = float(np.nanmax(temp_array))
        std_temp = float(np.nanstd(temp_array))

        # Hot spots indicate stress
        stress_threshold = mean_temp + (2 * std_temp)
        stress_mask = temp_array > stress_threshold
        stress_percentage = (np.sum(stress_mask) / temp_array.size) * 100

        water_stress = stress_percentage > 15

        return {
            'water_stress': water_stress,
            'stress_percentage': stress_percentage,
            'mean_canopy_temp_c': mean_temp,
            'max_canopy_temp_c': max_temp,
            'stress_threshold_c': stress_threshold
        }
