import math
from ultralytics import YOLO
import cv2
import os
import numpy as np

# Chemin du répertoire courant
script_dir = os.path.dirname(os.path.abspath(__file__))

# Chemin vers le modèle entraîné
model_path = os.path.join(script_dir, "runs", "segment", "train2", "weights", "best.pt")
model = YOLO(model_path)

# Chemin vers la vidéo à traiter
video_path = os.path.join(script_dir, "WIN_20250109_10_10_05_Pro.mp4")
output_path = os.path.join(script_dir, "output_video_segmentation_test1.mp4")

# Paramètres de la caméra (Logitech C270)
FOCAL_LENGTH = 490
DEPTH_CONSTANT = 5
IMAGE_WIDTH = 640

# Fonction pour calculer la distance 2D
def calculate_2d_distance(center1, center2, focal_length, depth):
    x1_pixels, y1_pixels = center1
    x2_pixels, y2_pixels = center2
    x1_real = (x1_pixels * depth) / focal_length
    y1_real = (y1_pixels * depth) / focal_length
    x2_real = (x2_pixels * depth) / focal_length
    y2_real = (y2_pixels * depth) / focal_length
    distance_2d = math.sqrt((x2_real - x1_real) ** 2 + (y2_real - y1_real) ** 2)
    return distance_2d

# Vérifiez la vidéo
if not os.path.isfile(video_path):
    print(f"Erreur : La vidéo {video_path} n'existe pas.")
else:
    video = cv2.VideoCapture(video_path)
    if not video.isOpened():
        print(f"Erreur : Impossible de charger la vidéo {video_path}.")
    else:
        fps = int(video.get(cv2.CAP_PROP_FPS))
        width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        output_video = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
        print(f"Traitement de la vidéo : {video_path}")

        while True:
            ret, frame = video.read()
            if not ret:
                break

            results = model.predict(source=frame, save=False, verbose=False)
            boxes = results[0].boxes.data.cpu().numpy()  # Bounding boxes [x1, y1, x2, y2, conf, class]
            class_names = results[0].names  # Nom des classes

            valid_centers = []

            # Parcourir toutes les boxes
            for box in boxes:
                x1, y1, x2, y2, conf, class_id = box

                # Calculer le centre de la box
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                valid_centers.append((center_x, center_y, class_id))

            # Trouver les centres les plus à gauche et à droite
            if valid_centers:
                leftmost = min(valid_centers, key=lambda c: c[0])  # Plus à gauche (min x)
                rightmost = max(valid_centers, key=lambda c: c[0])  # Plus à droite (max x)

                # Calculer la distance entre les deux
                distance_2d = calculate_2d_distance(
                    (leftmost[0], leftmost[1]), (rightmost[0], rightmost[1]), FOCAL_LENGTH, DEPTH_CONSTANT
                )

                # Annoter les centres
                cv2.circle(frame, (leftmost[0], leftmost[1]), 5, (0, 255, 0), -1)
                cv2.circle(frame, (rightmost[0], rightmost[1]), 5, (0, 0, 255), -1)

                # Annoter la distance
                distance_text = f"Distance: {distance_2d:.2f} m"
                cv2.putText(frame, distance_text, (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # Dessiner la ligne entre les centres
                cv2.line(frame, (leftmost[0], leftmost[1]), (rightmost[0], rightmost[1]), (255, 0, 0), 2)

            output_video.write(frame)

        video.release()
        output_video.release()
        print(f"Vidéo traitée et sauvegardée dans : {output_path}")
