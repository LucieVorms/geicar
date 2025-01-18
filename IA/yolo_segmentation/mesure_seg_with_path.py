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
output_path = os.path.join(script_dir, "output_video_path_distance12.mp4")

# Paramètres de la caméra
FOCAL_LENGTH = 490
DEPTH_CONSTANT = 4.5

# Fonction pour calculer la distance 2D
def calculate_2d_distance(point1, point2, focal_length, depth):
    x1_pixels, y1_pixels = point1
    x2_pixels, y2_pixels = point2
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
            masks = results[0].masks.data.cpu().numpy()
            class_names = results[0].names

            for i, mask in enumerate(masks):
                try:
                    # Extraire la classe associée au masque via les boîtes
                    class_id = int(results[0].boxes.cls[i].item())  # Associer l'indice du masque à l'identifiant de classe
                    if class_names[class_id] == "path":
                        # Obtenir les coordonnées de la bounding box de "path"
                        box = results[0].boxes.xyxy[i].cpu().numpy()  # [x1, y1, x2, y2]
                        x1, y1, x2, y2 = map(int, box)

                        # Trouver la position aux 2/3 de la bounding box en hauteur
                        adjusted_y = y2 - (2 * (y2 - y1)) // 3

                        # Trouver les bordures gauche et droite alignées avec ce niveau
                        active_pixels = np.where(mask > 0)
                        row_pixels = active_pixels[1][active_pixels[0] == adjusted_y]
                        if row_pixels.size > 0:
                            left_border = np.min(row_pixels)
                            right_border = np.max(row_pixels)

                            # Calculer la distance réelle
                            distance_2d = calculate_2d_distance(
                                (left_border, adjusted_y), (right_border, adjusted_y), FOCAL_LENGTH, DEPTH_CONSTANT
                            )

                            # Annoter uniquement les deux points
                            cv2.circle(frame, (left_border, adjusted_y), 5, (0, 255, 0), -1)  # Point gauche
                            cv2.circle(frame, (right_border, adjusted_y), 5, (0, 0, 255), -1)  # Point droit

                            # Tracer une ligne entre les deux points
                            cv2.line(frame, (left_border, adjusted_y), (right_border, adjusted_y), (0, 0, 0), 2)

                            # Annoter la distance en noir
                            cv2.putText(frame, f"Distance: {distance_2d:.2f} m", (50, 50),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
                except (IndexError, KeyError, AttributeError) as e:
                    print(f"Erreur lors du traitement du masque {i}: {e}")

            output_video.write(frame)

        video.release()
        output_video.release()
        print(f"Vidéo traitée et sauvegardée dans : {output_path}")
