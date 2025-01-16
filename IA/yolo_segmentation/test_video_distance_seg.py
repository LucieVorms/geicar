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
output_path = os.path.join(script_dir, "output_video_segmentation4.mp4")

# Paramètres de la caméra
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
            masks = results[0].masks.data.cpu().numpy()  # Masques segmentés
            boxes = results[0].boxes.data.cpu().numpy()  # Bounding boxes [x1, y1, x2, y2, conf, class]

            left_box = None
            right_box = None

            # Identifier les boxes les plus à gauche et à droite
            if boxes.size > 0:
                boxes = sorted(boxes, key=lambda b: (b[0] + b[2]) / 2)  # Trier par position centrale x
                left_box = boxes[0]
                right_box = boxes[-1]

            if left_box is not None and right_box is not None:
                # Calcul des centres
                left_center = (int((left_box[0] + left_box[2]) // 2), int((left_box[1] + left_box[3]) // 2))
                right_center = (int((right_box[0] + right_box[2]) // 2), int((right_box[1] + right_box[3]) // 2))

                # Dessiner les bounding boxes utilisées
                cv2.rectangle(frame, (int(left_box[0]), int(left_box[1])),
                            (int(left_box[2]), int(left_box[3])), (0, 255, 0), 2)
                cv2.rectangle(frame, (int(right_box[0]), int(right_box[1])),
                            (int(right_box[2]), int(right_box[3])), (0, 255, 0), 2)

                # Calculer la distance entre les deux
                distance_2d = calculate_2d_distance(left_center, right_center, FOCAL_LENGTH, DEPTH_CONSTANT)

                # Annoter les centres et la distance
                cv2.circle(frame, left_center, 5, (0, 255, 0), -1)
                cv2.circle(frame, right_center, 5, (0, 0, 255), -1)
                distance_text = f"Distance: {distance_2d:.2f} m"
                cv2.putText(frame, distance_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.line(frame, left_center, right_center, (255, 0, 0), 2)

            output_video.write(frame)

        video.release()
        output_video.release()
        print(f"Vidéo traitée et sauvegardée dans : {output_path}")
