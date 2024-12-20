from ultralytics import YOLO
import cv2
import os

# Chemin du répertoire courant
script_dir = os.path.dirname(os.path.abspath(__file__))

# Chemin vers le modèle entraîné
model_path = os.path.join(script_dir, "runs", "detect", "train2", "weights", "best.pt")
model = YOLO(model_path)

# Chemin vers le répertoire contenant les images de test
test_images_dir = os.path.join(script_dir, "images_test")

# Vérifier si le répertoire existe
if not os.path.isdir(test_images_dir):
    print(f"Erreur : Le répertoire {test_images_dir} n'existe pas.")
else:
    # Parcourir toutes les images du répertoire
    for image_name in os.listdir(test_images_dir):
        image_path = os.path.join(test_images_dir, image_name)
        
        # Vérifier si c'est un fichier image
        if os.path.isfile(image_path) and image_name.lower().endswith(('.png', '.jpg', '.jpeg')):
            print(f"Traitement de l'image : {image_name}")
            
            # Charger l'image
            image = cv2.imread(image_path)

            if image is None:
                print(f"Erreur : Impossible de charger l'image {image_name}.")
                continue

            # Effectuer une prédiction
            results = model.predict(source=image_path, save=True)
            
    print("Traitement terminé pour toutes les images.")
