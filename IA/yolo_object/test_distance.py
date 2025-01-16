import math
from ultralytics import YOLO
import cv2
import os

#Selectionner 720p 4:3 qualité pour video

# Chemin du répertoire courant
script_dir = os.path.dirname(os.path.abspath(__file__))

# Chemin vers le modèle entraîné
model_path = os.path.join(script_dir, "runs", "detect", "train4", "weights", "best.pt")
model = YOLO(model_path)

# Chemin vers le répertoire contenant les images de test
test_images_dir = os.path.join(script_dir, "images_test")

# Chemin vers le répertoire de sortie
output_dir = os.path.join(script_dir, "output_images")
os.makedirs(output_dir, exist_ok=True)

# Paramètres de la caméra (Logitech C270)
FOCAL_LENGTH = 490  # Focale en pixels pour une résolution de 640x680
DEPTH_CONSTANT = 5.0  # Distance constante en mètres (hypothèse pour une simplification)

IMAGE_WIDTH = 640  # Largeur de l'image en pixels

# Fonction pour calculer la distance 2D entre deux centres (en mètres)
def calculate_2d_distance(center1, center2, focal_length, depth):
    # Coordonnées des centres en pixels
    x1_pixels, y1_pixels = center1
    x2_pixels, y2_pixels = center2

    # Conversion en coordonnées réelles : Pinhole camera model
    x1_real = (x1_pixels * depth) / focal_length
    y1_real = (y1_pixels * depth) / focal_length
    x2_real = (x2_pixels * depth) / focal_length
    y2_real = (y2_pixels * depth) / focal_length

    # Calcul de la distance 2D : norme
    distance_2d = math.sqrt((x2_real - x1_real)**2 + (y2_real - y1_real)**2)
    return distance_2d


######################## Test sur des images ###########################""
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
            results = model.predict(source=image, verbose=False)

            # Stocker les centres des boîtes
            left_boxes = []
            right_boxes = []

            # Parcourir les détections
            for result in results[0].boxes.data:
                x1, y1, x2, y2, confidence, class_id = result
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                
                # Calcul du centre
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                # Séparer en gauche/droite
                if center_x < IMAGE_WIDTH // 2:
                    left_boxes.append((center_x, center_y))
                else:
                    right_boxes.append((center_x, center_y))

                # Dessiner la bounding box et le centre
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(image, (center_x, center_y), 5, (255, 0, 0), -1)

            # Calculer la distance entre les deux objets
            if left_boxes and right_boxes:
                left_center = left_boxes[0]  # Première boîte à gauche
                right_center = right_boxes[0]  # Première boîte à droite

                # Calcul de la distance 2D
                distance_2d = calculate_2d_distance(
                    left_center, right_center, FOCAL_LENGTH, DEPTH_CONSTANT
                )
                distance_text = f"Distance: {distance_2d:.2f} m"

                # Annoter la distance
                cv2.putText(image, distance_text, (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # Dessiner la ligne entre les centres
                cv2.line(image, left_center, right_center, (0, 0, 255), 2)

            # Enregistrer l'image avec les annotations
            output_path = os.path.join(output_dir, image_name)
            cv2.imwrite(output_path, image)
            print(f"Image annotée sauvegardée : {output_path}")

    print("Traitement terminé pour toutes les images.")
