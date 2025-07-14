# Programm: yolov8_obb_train.py
# Autor: Lukas Sambale
# Datum: 26.06.2025

from ultralytics import YOLO
from pathlib import Path
import os
from PIL import Image, ImageFile
ImageFile.LOAD_TRUNCATED_IMAGES = True

home = Path.home()
# Load a model
prev_trained_model_path = home / "runs" / "obb" / "train6" / "weights"/ "last.pt"
model = YOLO(str(prev_trained_model_path))  # load a pretrained model statndart = "yolov8n-obb.pt"

model_path = home / "Downloads" / "boiling_detection.v1i.yolov8-obb" / "data.yaml"
print(f"\n\npath = {model_path}\n\n")

# Train the model
results = model.train(data=str(model_path), epochs=200, imgsz=640)
