from ultralytics import YOLO
import os

# Charger un modèle YOLO pré-entraîné
model = YOLO("yolov8n.yaml")  

project_root = os.path.dirname(os.path.abspath(__file__))
data_path = os.path.join(project_root, "datasets", "data.yaml")
# Entraîner le modèle sur votre dataset
results = model.train(data = data_path, epochs=50, batch=16, imgsz=640)

