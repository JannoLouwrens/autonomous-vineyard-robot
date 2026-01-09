#!/usr/bin/env python3
"""
Test YOLO Model - Upload an image and test detection
Usage: python3 test_yolo_model.py <image_path> [model_path]
"""

import sys
import cv2
from ultralytics import YOLO

def test_model(image_path, model_path=None):
    # Default model path
    if model_path is None:
        model_path = '/home/pi/Working Code/Updated FULL Code Standalone/models/tree_detector_old.pt'

    print(f"Loading model: {model_path}")
    model = YOLO(model_path)

    # Print model info
    print(f"\n=== Model Info ===")
    print(f"Task: {model.task}")
    print(f"Classes: {model.names}")
    print(f"Epochs trained: {model.ckpt.get('epoch', 'unknown')}")
    print(f"Best fitness: {model.ckpt.get('best_fitness', 'unknown')}")

    # Load image
    print(f"\nLoading image: {image_path}")
    img = cv2.imread(image_path)
    if img is None:
        print(f"ERROR: Could not load image: {image_path}")
        return

    print(f"Image size: {img.shape}")

    # Run detection at various confidence thresholds
    print(f"\n=== Running Detection ===")
    for conf in [0.5, 0.25, 0.1, 0.05, 0.01]:
        results = model(img, conf=conf, verbose=False)
        num_detections = len(results[0].boxes)
        print(f"Confidence {conf:.0%}: {num_detections} detections")

        if num_detections > 0:
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                cls_name = model.names[cls_id]
                box_conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                print(f"  - {cls_name}: {box_conf:.1%} at ({x1},{y1})-({x2},{y2})")

    # Save annotated image
    print(f"\n=== Saving Result ===")
    results = model(img, conf=0.1, verbose=False)
    annotated = results[0].plot()

    output_path = image_path.rsplit('.', 1)[0] + '_detected.jpg'
    cv2.imwrite(output_path, annotated)
    print(f"Saved: {output_path}")

    if len(results[0].boxes) == 0:
        print("\n⚠ No detections found!")
        print("Possible reasons:")
        print("  1. Model not trained properly (epoch=-1 means training never finished)")
        print("  2. Image doesn't contain objects the model was trained on")
        print("  3. Confidence threshold too high")
        print("\nTo retrain, use Google Colab with labeled trunk images.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 test_yolo_model.py <image_path> [model_path]")
        print("\nExamples:")
        print("  python3 test_yolo_model.py /path/to/trunk_photo.jpg")
        print("  python3 test_yolo_model.py photo.jpg /path/to/custom_model.pt")
        sys.exit(1)

    image_path = sys.argv[1]
    model_path = sys.argv[2] if len(sys.argv) > 2 else None

    test_model(image_path, model_path)
