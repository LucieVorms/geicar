from ultralytics import YOLO
import os

# Charger un modèle YOLO pré-entraîné pour la segmentation
model = YOLO("yolov8n-seg.yaml")  # Assurez-vous d'utiliser le fichier YAML pour la segmentation

# Définir le chemin du fichier de configuration des données
project_root = os.path.dirname(os.path.abspath(__file__))
data_path = os.path.join(project_root, "datasets", "data.yaml")

# Entraîner le modèle sur votre dataset
results = model.train(data=data_path, epochs=50, batch=16, imgsz=640)

# Sauvegarder le modèle entraîné
model_path = os.path.join(project_root, "runs", "segment", "train", "weights", "best.pt")
print(f"Modèle sauvegardé dans : {model_path}")
