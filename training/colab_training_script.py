"""
YOLO Training Script for Google Colab
=====================================
Copy-paste these sections into Colab cells

Your folder structure should be:
  Google Drive/
    TreePhotos/
      vineyard/
        images/
          img001.jpg
          img002.jpg
          ...
        labels/
          img001.txt  (YOLO format: class_id x_center y_center width height)
          img002.txt
          ...
      tangerine/
        images/
        labels/

Label format (YOLO): each .txt file contains lines like:
  0 0.5 0.5 0.3 0.4
  (class_id, x_center, y_center, width, height - all normalized 0-1)
"""

#==============================================================================
# CELL 1: Install and import
#==============================================================================
"""
!pip install ultralytics
!pip install roboflow  # optional

from google.colab import drive
drive.mount('/content/drive')
"""

#==============================================================================
# CELL 2: Configuration - EDIT THESE
#==============================================================================
TREE_TYPE = "vineyard"  # or "tangerine"

# Classes for detection
VINEYARD_CLASSES = ['vineyard_trunk', 'vineyard_canopy', 'grape_cluster', 'post']
TANGERINE_CLASSES = ['tangerine_trunk', 'tangerine_canopy', 'tangerine_fruit', 'post']

CLASSES = VINEYARD_CLASSES if TREE_TYPE == "vineyard" else TANGERINE_CLASSES

# Paths in Google Drive
SOURCE_IMAGES = f"/content/drive/MyDrive/TreePhotos/{TREE_TYPE}/images"
SOURCE_LABELS = f"/content/drive/MyDrive/TreePhotos/{TREE_TYPE}/labels"

#==============================================================================
# CELL 3: Prepare dataset
#==============================================================================
"""
import os
import shutil
import random
import yaml

DATASET_PATH = "/content/dataset"
os.makedirs(f"{DATASET_PATH}/images/train", exist_ok=True)
os.makedirs(f"{DATASET_PATH}/images/val", exist_ok=True)
os.makedirs(f"{DATASET_PATH}/labels/train", exist_ok=True)
os.makedirs(f"{DATASET_PATH}/labels/val", exist_ok=True)

# Get all images
images = [f for f in os.listdir(SOURCE_IMAGES) if f.endswith(('.jpg', '.jpeg', '.png'))]
print(f"Found {len(images)} images")

# Split 80/20
random.shuffle(images)
split = int(len(images) * 0.8)
train_images = images[:split]
val_images = images[split:]

# Copy train
for img in train_images:
    shutil.copy(f"{SOURCE_IMAGES}/{img}", f"{DATASET_PATH}/images/train/{img}")
    label = img.rsplit('.', 1)[0] + '.txt'
    if os.path.exists(f"{SOURCE_LABELS}/{label}"):
        shutil.copy(f"{SOURCE_LABELS}/{label}", f"{DATASET_PATH}/labels/train/{label}")

# Copy val
for img in val_images:
    shutil.copy(f"{SOURCE_IMAGES}/{img}", f"{DATASET_PATH}/images/val/{img}")
    label = img.rsplit('.', 1)[0] + '.txt'
    if os.path.exists(f"{SOURCE_LABELS}/{label}"):
        shutil.copy(f"{SOURCE_LABELS}/{label}", f"{DATASET_PATH}/labels/val/{label}")

print(f"Train: {len(train_images)}, Val: {len(val_images)}")

# Create dataset.yaml
config = {
    'path': DATASET_PATH,
    'train': 'images/train',
    'val': 'images/val',
    'names': {i: name for i, name in enumerate(CLASSES)}
}

with open(f"{DATASET_PATH}/dataset.yaml", 'w') as f:
    yaml.dump(config, f)

print("Dataset prepared!")
"""

#==============================================================================
# CELL 4: Train model
#==============================================================================
"""
from ultralytics import YOLO

# Load pretrained YOLOv8 nano (best for Raspberry Pi)
model = YOLO('yolov8n.pt')

# Train
results = model.train(
    data=f"{DATASET_PATH}/dataset.yaml",
    epochs=100,
    imgsz=640,
    batch=16,
    patience=20,
    project='/content/runs',
    name=f'{TREE_TYPE}_detector',
    exist_ok=True,
)

print("Training complete!")
"""

#==============================================================================
# CELL 5: View results
#==============================================================================
"""
from IPython.display import Image, display

run_path = f'/content/runs/{TREE_TYPE}_detector'

# Show training curves
display(Image(filename=f'{run_path}/results.png', width=800))

# Show confusion matrix
display(Image(filename=f'{run_path}/confusion_matrix.png', width=600))
"""

#==============================================================================
# CELL 6: Test model
#==============================================================================
"""
import glob

best_model = YOLO(f'{run_path}/weights/best.pt')

# Test on validation image
test_imgs = glob.glob(f'{DATASET_PATH}/images/val/*.jpg')
if test_imgs:
    results = best_model(test_imgs[0])
    for r in results:
        annotated = r.plot()
        from PIL import Image as PILImage
        PILImage.fromarray(annotated[..., ::-1]).save('/content/test.jpg')
    display(Image(filename='/content/test.jpg', width=640))
"""

#==============================================================================
# CELL 7: Save to Drive & Download
#==============================================================================
"""
import shutil
import os

# Save to Drive
output_dir = "/content/drive/MyDrive/TreeModels"
os.makedirs(output_dir, exist_ok=True)
shutil.copy(f'{run_path}/weights/best.pt', f'{output_dir}/{TREE_TYPE}_best.pt')

print(f"Saved to: {output_dir}/{TREE_TYPE}_best.pt")

# Or download directly
from google.colab import files
files.download(f'{run_path}/weights/best.pt')
"""

#==============================================================================
# LABELING GUIDE
#==============================================================================
"""
QUICK LABELING with LabelImg:
==============================
1. Install: pip install labelImg
2. Run: labelImg
3. Open your images folder
4. Change to YOLO format (View > YOLO)
5. Press 'w' to draw box
6. Select class, save (Ctrl+S)

CLASS IDs (for YOLO .txt files):
- 0: vineyard_trunk / tangerine_trunk
- 1: vineyard_canopy / tangerine_canopy
- 2: grape_cluster / tangerine_fruit
- 3: post (fence posts, poles - NOT trees!)

Each label .txt file format:
class_id x_center y_center width height

Example (trunk at center of image, 30% width, 80% height):
0 0.5 0.5 0.3 0.8

All values normalized 0-1 (relative to image dimensions)
"""

#==============================================================================
# RASPBERRY PI DEPLOYMENT
#==============================================================================
"""
After training:
1. Download best.pt from Colab
2. Copy to Pi: scp best.pt pi@<IP>:/home/pi/Working\ Code/Updated\ FULL\ Code\ Standalone/models/
3. Rename: mv best.pt vineyard_yolov8n.pt (or tangerine_yolov8n.pt)
4. Update config.py:
   YOLO_MODEL_PATH = 'models/vineyard_yolov8n.pt'
5. Restart robot service:
   sudo systemctl restart vineyard_robot
"""
