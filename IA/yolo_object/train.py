from ultralytics import YOLO
import os

# Charger un modèle YOLO pré-entraîné
model = YOLO("yolov8n.pt")  # Modèle pré-entraîné sur COCO

# Définir les chemins
project_root = os.path.dirname(os.path.abspath(__file__))
data_path = os.path.join(project_root, "datasets", "data.yaml")

# Réentraîner le modèle sur votre dataset
results = model.train(
    data=data_path,     # Chemin vers le fichier data.yaml
    epochs=50,          # Nombre d'époques
    batch=16,           # Taille de batch
    imgsz=640,          # Taille des images
    save=True           # Sauvegarder les résultats
)

# Afficher les résultats
print("Entraînement terminé. Résultats sauvegardés dans :", results)


