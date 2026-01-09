#!/usr/bin/env python3
"""
YOLO Training Script
Train YOLOv8 model on custom tree images

Usage:
    python train_yolo.py --tree-type vineyard --epochs 50
    python train_yolo.py --tree-type orange --epochs 100

Before training:
1. Put your photos in training/VineyardPhotos/ (or OrangePhotos/, etc.)
2. This script will help you label them using a simple GUI
3. Then it trains the YOLO model
"""

import os
import sys
import argparse
import shutil
import random
import cv2
import json

sys.path.append('/home/pi/Working Code/Updated FULL Code Standalone')
from config import *

# Training paths
TRAINING_BASE = '/home/pi/Working Code/Updated FULL Code Standalone/training'
DATASET_DIR = f'{TRAINING_BASE}/dataset'
MODELS_DIR = f'{TRAINING_BASE}/models'

# Photo directories for each tree type
PHOTO_DIRS = {
    'vineyard': f'{TRAINING_BASE}/VineyardPhotos',
    'orange': f'{TRAINING_BASE}/OrangePhotos',
    'lemon': f'{TRAINING_BASE}/LemonPhotos',
    'olive': f'{TRAINING_BASE}/OlivePhotos',
    'apple': f'{TRAINING_BASE}/ApplePhotos'
}


def create_directories():
    """Create necessary directories"""
    os.makedirs(DATASET_DIR, exist_ok=True)
    os.makedirs(f'{DATASET_DIR}/train/images', exist_ok=True)
    os.makedirs(f'{DATASET_DIR}/train/labels', exist_ok=True)
    os.makedirs(f'{DATASET_DIR}/val/images', exist_ok=True)
    os.makedirs(f'{DATASET_DIR}/val/labels', exist_ok=True)
    os.makedirs(MODELS_DIR, exist_ok=True)

    # Create photo directories
    for photo_dir in PHOTO_DIRS.values():
        os.makedirs(photo_dir, exist_ok=True)

    print(f"[Setup] Directories created")


def simple_label_photos(tree_type):
    """
    Simple labeling tool - assumes entire image is the trunk
    For quick training when photos are cropped trunk images
    """
    photo_dir = PHOTO_DIRS.get(tree_type)
    if not photo_dir or not os.path.exists(photo_dir):
        print(f"[Error] Photo directory not found: {photo_dir}")
        return []

    photos = [f for f in os.listdir(photo_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]

    if not photos:
        print(f"[Error] No photos found in {photo_dir}")
        return []

    print(f"[Labeling] Found {len(photos)} photos for {tree_type}")
    print("[Labeling] Creating labels (assuming each photo is a trunk)...")

    labeled_data = []

    for photo_file in photos:
        photo_path = os.path.join(photo_dir, photo_file)

        # Read image to get dimensions
        img = cv2.imread(photo_path)
        if img is None:
            continue

        height, width = img.shape[:2]

        # YOLO format: class x_center y_center width height (normalized 0-1)
        # Assume trunk is centered and takes 60% of image
        label = {
            'file': photo_file,
            'path': photo_path,
            'width': width,
            'height': height,
            'annotations': [
                {
                    'class': 0,  # trunk class
                    'x_center': 0.5,
                    'y_center': 0.5,
                    'width': 0.6,
                    'height': 0.9
                }
            ]
        }
        labeled_data.append(label)

    print(f"[Labeling] Created {len(labeled_data)} labels")
    return labeled_data


def interactive_labeler(tree_type):
    """
    Interactive labeling tool with GUI
    Draw bounding boxes around trunks
    """
    photo_dir = PHOTO_DIRS.get(tree_type)
    if not photo_dir or not os.path.exists(photo_dir):
        print(f"[Error] Photo directory not found: {photo_dir}")
        return []

    photos = [f for f in os.listdir(photo_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]

    if not photos:
        print(f"[Error] No photos found in {photo_dir}")
        return []

    print(f"\n{'='*50}")
    print(f"INTERACTIVE LABELING TOOL")
    print(f"{'='*50}")
    print(f"Photos: {len(photos)}")
    print(f"\nInstructions:")
    print("  - Click and drag to draw bounding box around trunk")
    print("  - Press 'n' for next image")
    print("  - Press 's' to skip image")
    print("  - Press 'r' to reset current box")
    print("  - Press 'q' to quit and save")
    print(f"{'='*50}\n")

    labeled_data = []
    drawing = False
    start_x, start_y = 0, 0
    end_x, end_y = 0, 0
    current_img = None
    current_file = None

    def mouse_callback(event, x, y, flags, param):
        nonlocal drawing, start_x, start_y, end_x, end_y, current_img

        if event == cv2.EVENT_LBUTTONDOWN:
            drawing = True
            start_x, start_y = x, y

        elif event == cv2.EVENT_MOUSEMOVE:
            if drawing:
                end_x, end_y = x, y

        elif event == cv2.EVENT_LBUTTONUP:
            drawing = False
            end_x, end_y = x, y

    cv2.namedWindow('Labeler')
    cv2.setMouseCallback('Labeler', mouse_callback)

    i = 0
    while i < len(photos):
        photo_file = photos[i]
        photo_path = os.path.join(photo_dir, photo_file)
        current_file = photo_file

        img = cv2.imread(photo_path)
        if img is None:
            i += 1
            continue

        current_img = img.copy()
        height, width = img.shape[:2]

        start_x, start_y, end_x, end_y = 0, 0, 0, 0

        while True:
            display = current_img.copy()

            # Draw current box
            if start_x != end_x and start_y != end_y:
                cv2.rectangle(display, (start_x, start_y), (end_x, end_y), (0, 255, 0), 2)

            # Show info
            cv2.putText(display, f"{i+1}/{len(photos)}: {photo_file}",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display, "n=next s=skip r=reset q=quit",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            cv2.imshow('Labeler', display)
            key = cv2.waitKey(30) & 0xFF

            if key == ord('n'):
                # Save and next
                if start_x != end_x and start_y != end_y:
                    x1, x2 = min(start_x, end_x), max(start_x, end_x)
                    y1, y2 = min(start_y, end_y), max(start_y, end_y)

                    x_center = ((x1 + x2) / 2) / width
                    y_center = ((y1 + y2) / 2) / height
                    box_width = (x2 - x1) / width
                    box_height = (y2 - y1) / height

                    labeled_data.append({
                        'file': photo_file,
                        'path': photo_path,
                        'width': width,
                        'height': height,
                        'annotations': [{
                            'class': 0,
                            'x_center': x_center,
                            'y_center': y_center,
                            'width': box_width,
                            'height': box_height
                        }]
                    })
                    print(f"  Saved: {photo_file}")
                i += 1
                break

            elif key == ord('s'):
                # Skip
                i += 1
                break

            elif key == ord('r'):
                # Reset
                start_x, start_y, end_x, end_y = 0, 0, 0, 0

            elif key == ord('q'):
                # Quit
                cv2.destroyAllWindows()
                return labeled_data

    cv2.destroyAllWindows()
    return labeled_data


def prepare_dataset(labeled_data, tree_type, val_split=0.2):
    """
    Prepare YOLO dataset from labeled data
    """
    print(f"\n[Dataset] Preparing dataset...")

    # Shuffle and split
    random.shuffle(labeled_data)
    split_idx = int(len(labeled_data) * (1 - val_split))
    train_data = labeled_data[:split_idx]
    val_data = labeled_data[split_idx:]

    print(f"  Train: {len(train_data)} images")
    print(f"  Val: {len(val_data)} images")

    # Copy images and create labels
    for split_name, data in [('train', train_data), ('val', val_data)]:
        for item in data:
            # Copy image
            dest_img = f"{DATASET_DIR}/{split_name}/images/{item['file']}"
            shutil.copy(item['path'], dest_img)

            # Create label file
            label_file = os.path.splitext(item['file'])[0] + '.txt'
            dest_label = f"{DATASET_DIR}/{split_name}/labels/{label_file}"

            with open(dest_label, 'w') as f:
                for ann in item['annotations']:
                    f.write(f"{ann['class']} {ann['x_center']:.6f} {ann['y_center']:.6f} {ann['width']:.6f} {ann['height']:.6f}\n")

    # Create dataset YAML
    classes = YOLO_CLASSES.get(tree_type, [f'{tree_type}_trunk'])
    yaml_content = f"""# YOLOv8 Dataset Configuration
# Tree type: {tree_type}

path: {DATASET_DIR}
train: train/images
val: val/images

# Classes
names:
"""
    for i, cls in enumerate(classes):
        yaml_content += f"  {i}: {cls}\n"

    yaml_path = f'{DATASET_DIR}/dataset.yaml'
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)

    print(f"[Dataset] Created dataset.yaml at {yaml_path}")
    return yaml_path


def train_model(yaml_path, tree_type, epochs=50, imgsz=640):
    """
    Train YOLOv8 model
    """
    try:
        from ultralytics import YOLO
    except ImportError:
        print("[Error] ultralytics not installed. Run: pip install ultralytics")
        return None

    print(f"\n{'='*50}")
    print(f"TRAINING YOLOv8 MODEL")
    print(f"{'='*50}")
    print(f"  Tree type: {tree_type}")
    print(f"  Epochs: {epochs}")
    print(f"  Image size: {imgsz}")
    print(f"{'='*50}\n")

    # Load pretrained YOLOv8n (nano - fast for Pi)
    model = YOLO('yolov8n.pt')

    # Train
    results = model.train(
        data=yaml_path,
        epochs=epochs,
        imgsz=imgsz,
        batch=8,
        name=f'{tree_type}_detector',
        project=MODELS_DIR,
        patience=10,
        save=True,
        plots=True
    )

    # Copy best model
    best_model = f'{MODELS_DIR}/{tree_type}_detector/weights/best.pt'
    final_model = f'{MODELS_DIR}/tree_detector.pt'

    if os.path.exists(best_model):
        shutil.copy(best_model, final_model)
        print(f"\n[Training] Model saved to: {final_model}")
        return final_model
    else:
        print("[Training] Training may have failed - check output above")
        return None


def main():
    parser = argparse.ArgumentParser(description='Train YOLO model for tree detection')
    parser.add_argument('--tree-type', type=str, default='vineyard',
                       choices=['vineyard', 'orange', 'lemon', 'olive', 'apple'],
                       help='Type of tree to train on')
    parser.add_argument('--epochs', type=int, default=50,
                       help='Number of training epochs')
    parser.add_argument('--interactive', action='store_true',
                       help='Use interactive labeling (draw boxes)')
    parser.add_argument('--skip-labeling', action='store_true',
                       help='Skip labeling, use existing dataset')

    args = parser.parse_args()

    print(f"\n{'='*60}")
    print(f"YOLO TREE DETECTOR TRAINING")
    print(f"{'='*60}")

    # Create directories
    create_directories()

    if not args.skip_labeling:
        # Label photos
        if args.interactive:
            labeled_data = interactive_labeler(args.tree_type)
        else:
            labeled_data = simple_label_photos(args.tree_type)

        if not labeled_data:
            print("\n[Error] No labeled data. Add photos to:")
            print(f"  {PHOTO_DIRS[args.tree_type]}/")
            return

        # Prepare dataset
        yaml_path = prepare_dataset(labeled_data, args.tree_type)
    else:
        yaml_path = f'{DATASET_DIR}/dataset.yaml'
        if not os.path.exists(yaml_path):
            print(f"[Error] Dataset not found: {yaml_path}")
            return

    # Train model
    model_path = train_model(yaml_path, args.tree_type, args.epochs)

    if model_path:
        print(f"\n{'='*60}")
        print("TRAINING COMPLETE!")
        print(f"{'='*60}")
        print(f"Model saved to: {model_path}")
        print(f"\nTo use this model, update config.py:")
        print(f"  YOLO_MODEL_PATH = '{model_path}'")
        print(f"  TREE_TYPE = '{args.tree_type}'")


if __name__ == '__main__':
    main()
