#!/usr/bin/env python3
"""
YOLO Tree Classifier - Verify tree type using trained model
Supports both DETECTION mode (bounding boxes) and CLASSIFICATION mode (whole image)
"""

import cv2
import numpy as np
import sys
sys.path.append('/home/pi/Working Code/Updated FULL Code Standalone')
from config import *

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("[YOLO] ultralytics not installed - run: pip install ultralytics")


class YOLOClassifier:
    """
    YOLO-based tree type classifier
    Supports two modes:
    - DETECTION: finds bounding boxes of trunks/canopy (requires labeled training data)
    - CLASSIFICATION: answers "is this a tree?" yes/no (just needs folders of images)
    """

    def __init__(self, model_path=None, mode=None):
        self.model = None
        self.model_path = model_path or YOLO_MODEL_PATH
        self.mode = mode or getattr(sys.modules.get('config'), 'YOLO_MODE', 'detect')
        self.classes = YOLO_CLASSES.get(TREE_TYPE, ['trunk', 'canopy'])
        self.loaded = False

        if YOLO_AVAILABLE:
            self._load_model()
        else:
            print("[YOLO] Not available - tree verification disabled")

    def _load_model(self):
        """Load YOLO model"""
        try:
            import os
            if os.path.exists(self.model_path):
                print(f"[YOLO] Loading model from {self.model_path}...")
                self.model = YOLO(self.model_path)

                # Test that model loaded correctly
                if self.model is None:
                    print("[YOLO] ✗ Model is None after loading")
                    return

                self.loaded = True

                # Auto-detect mode from model type
                if hasattr(self.model, 'task'):
                    if self.model.task == 'classify':
                        self.mode = 'classify'
                    else:
                        self.mode = 'detect'

                print(f"[YOLO] ✓ Model loaded - mode: {self.mode}")
                if self.mode == 'detect':
                    print(f"[YOLO] Detection classes: {self.classes}")
                else:
                    print(f"[YOLO] Classification: tree vs not_tree")
            else:
                print(f"[YOLO] Model not found at {self.model_path}")
                print("[YOLO] Run training script first to create model")
        except Exception as e:
            print(f"[YOLO] Load error: {e}")

    def detect(self, frame):
        """
        Run YOLO detection on frame
        Args:
            frame: BGR image
        Returns:
            list of detections: [{'class': str, 'confidence': float, 'bbox': (x,y,w,h)}]
        """
        if not self.loaded or frame is None:
            return []

        try:
            results = self.model(frame, conf=YOLO_CONFIDENCE_THRESHOLD, max_det=1, verbose=False)

            detections = []
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names[cls_id]

                    detections.append({
                        'class': cls_name,
                        'confidence': conf,
                        'bbox': (int(x1), int(y1), int(x2 - x1), int(y2 - y1)),
                        'center': (int((x1 + x2) / 2), int((y1 + y2) / 2))
                    })

            return detections
        except Exception as e:
            print(f"[YOLO] Detection error: {e}")
            return []

    def verify_tree(self, frame, expected_class=None):
        """
        Verify if frame contains expected tree type
        Works in both DETECTION and CLASSIFICATION modes
        Args:
            frame: BGR image
            expected_class: specific class to look for (detection mode only)
        Returns:
            dict: {'verified': bool, 'confidence': float, 'detection': dict or None}
        """
        if not self.loaded or frame is None:
            return {'verified': False, 'confidence': 0, 'detection': None, 'reason': 'not_available'}

        # CLASSIFICATION MODE: Simple tree/not_tree answer
        if self.mode == 'classify':
            return self._verify_tree_classify(frame)

        # DETECTION MODE: Find trunk bounding boxes
        return self._verify_tree_detect(frame, expected_class)

    def _verify_tree_classify(self, frame):
        """
        Classification mode: Is this image a tree? (yes/no)
        """
        try:
            results = self.model(frame, verbose=False)

            for r in results:
                probs = r.probs
                top_class = r.names[probs.top1]
                confidence = float(probs.top1conf)

                # Check if classified as "tree"
                is_tree = 'tree' in top_class.lower() and 'not' not in top_class.lower()

                return {
                    'verified': is_tree,
                    'confidence': confidence,
                    'detection': {
                        'class': top_class,
                        'confidence': confidence,
                        'mode': 'classify'
                    },
                    'reason': None if is_tree else 'classified_as_not_tree'
                }

            return {'verified': False, 'confidence': 0, 'detection': None, 'reason': 'no_result'}

        except Exception as e:
            return {'verified': False, 'confidence': 0, 'detection': None, 'reason': str(e)}

    def _verify_tree_detect(self, frame, expected_class=None):
        """
        Detection mode: Find trunk bounding box
        """
        detections = self.detect(frame)

        if not detections:
            return {'verified': False, 'confidence': 0, 'detection': None, 'reason': 'no_detection'}

        # Look for trunk class
        trunk_classes = [c for c in self.classes if 'trunk' in c.lower()]
        if expected_class:
            trunk_classes = [expected_class]

        best_detection = None
        best_confidence = 0

        for det in detections:
            # Check if it's a trunk (not a post)
            class_lower = det['class'].lower()
            is_trunk = any(tc in class_lower for tc in ['trunk'])
            is_post = 'post' in class_lower

            # Accept trunk, reject post
            if is_trunk and not is_post:
                if det['confidence'] > best_confidence:
                    best_confidence = det['confidence']
                    best_detection = det
                    best_detection['mode'] = 'detect'

        if best_detection:
            return {
                'verified': True,
                'confidence': best_confidence,
                'detection': best_detection
            }

        # Check if we found a post (to give better feedback)
        for det in detections:
            if 'post' in det['class'].lower():
                return {'verified': False, 'confidence': det['confidence'],
                        'detection': det, 'reason': 'detected_post_not_tree'}

        return {'verified': False, 'confidence': 0, 'detection': None, 'reason': 'no_trunk_found'}

    def find_canopy(self, frame):
        """
        Find canopy/leaf detection in frame
        Returns: detection dict or None
        """
        detections = self.detect(frame)

        for det in detections:
            if any(c in det['class'].lower() for c in ['canopy', 'leaf', 'grape']):
                return det
        return None

    def visualize(self, frame):
        """Draw detections on frame"""
        if frame is None:
            return None

        annotated = frame.copy()
        detections = self.detect(frame)

        for det in detections:
            x, y, w, h = det['bbox']
            conf = det['confidence']
            cls = det['class']

            # Color by class type
            if 'trunk' in cls.lower() or 'post' in cls.lower():
                color = (0, 255, 0)  # Green for trunk
            elif 'canopy' in cls.lower() or 'leaf' in cls.lower():
                color = (0, 255, 255)  # Yellow for canopy
            else:
                color = (255, 0, 0)  # Blue for other

            cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
            label = f"{cls}: {conf:.2f}"
            cv2.putText(annotated, label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        if not detections:
            cv2.putText(annotated, "No detections",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        return annotated

    def is_available(self):
        """Check if YOLO model is loaded and ready"""
        return self.loaded and self.model is not None
