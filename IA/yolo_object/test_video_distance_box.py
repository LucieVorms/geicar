import math
from ultralytics import YOLO
import cv2
import os

# Chemin du répertoire courant
script_dir = os.path.dirname(os.path.abspath(__file__))

# Chemin vers le modèle entraîné
model_path = os.path.join(script_dir, "runs", "detect", "train4", "weights", "best.pt")
model = YOLO(model_path)

# Chemin vers la vidéo à traiter
video_path = os.path.join(script_dir, "WIN_20250109_10_10_05_Pro.mp4")
output_path = os.path.join(script_dir, "output_video_distance5.mp4")

# Paramètres de la caméra (Logitech C270)
FOCAL_LENGTH = 490  # Focale en pixels pour une résolution de 640x680
DEPTH_CONSTANT = 5 # Distance constante en mètres (hypothèse pour une simplification)
IMAGE_WIDTH = 640  # Largeur de l'image en pixels

# Fonction pour calculer la distance 2D entre deux centres (en mètres)
def calculate_2d_distance(center1, center2, focal_length, depth):
    # Coordonnées des centres en pixels
    x1_pixels, y1_pixels = center1
    x2_pixels, y2_pixels = center2

    # Conversion en coordonnées réelles
    x1_real = (x1_pixels * depth) / focal_length
    y1_real = (y1_pixels * depth) / focal_length
    x2_real = (x2_pixels * depth) / focal_length
    y2_real = (y2_pixels * depth) / focal_length

    # Calcul de la distance 2D
    distance_2d = math.sqrt((x2_real - x1_real)**2 + (y2_real - y1_real)**2)
    return distance_2d

# Vérifier si la vidéo existe
if not os.path.isfile(video_path):
    print(f"Erreur : La vidéo {video_path} n'existe pas.")
else:
    # Charger la vidéo
    video = cv2.VideoCapture(video_path)
    
    # Vérifier si la vidéo est valide
    if not video.isOpened():
        print(f"Erreur : Impossible de charger la vidéo {video_path}.")
    else:
        # Obtenir les paramètres de la vidéo
        fps = int(video.get(cv2.CAP_PROP_FPS))
        width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec pour la vidéo de sortie

        # Créer un writer pour la vidéo de sortie
        output_video = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

        print(f"Traitement de la vidéo : {video_path}")

        # Parcourir les frames de la vidéo
        while True:
            ret, frame = video.read()  # Lire une frame
            if not ret:  # Si la fin de la vidéo est atteinte
                break

            # Effectuer une prédiction
            results = model.predict(source=frame, save=False, verbose=False)

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
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

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
                cv2.putText(frame, distance_text, (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # Dessiner la ligne entre les centres
                cv2.line(frame, left_center, right_center, (0, 0, 255), 2)

            # Écrire la frame annotée dans la vidéo de sortie
            output_video.write(frame)

        # Libérer les ressources
        video.release()
        output_video.release()

        print(f"Vidéo traitée et sauvegardée dans : {output_path}")
