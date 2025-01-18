import cv2
import os
import numpy as np
from ultralytics import YOLO

# Chemin du répertoire courant
script_dir = os.path.dirname(os.path.abspath(__file__))

# Chemin vers le modèle entraîné
model_path = os.path.join(script_dir, "runs", "segment", "train2", "weights", "best.pt")
model = YOLO(model_path)

# Chemin vers la vidéo à traiter
video_path = os.path.join(script_dir, "WIN_20250109_10_10_05_Pro.mp4")
output_path = os.path.join(script_dir, "output_video_path_distance.mp4")

# Vérifiez si la vidéo existe
if not os.path.isfile(video_path):
    print(f"Erreur : La vidéo {video_path} n'existe pas.")
else:
    # Charger la vidéo
    video = cv2.VideoCapture(video_path)

    if not video.isOpened():
        print(f"Erreur : Impossible de charger la vidéo {video_path}.")
    else:
        # Obtenir les paramètres de la vidéo
        fps = int(video.get(cv2.CAP_PROP_FPS))
        width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')

        # Créer un writer pour la vidéo de sortie
        output_video = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
        print(f"Traitement de la vidéo : {video_path}")

        while True:
            ret, frame = video.read()
            if not ret:
                break

            # Effectuer une prédiction de segmentation
            results = model.predict(source=frame, save=False, verbose=False)

            # Obtenir les masques et noms des classes
            masks = results[0].masks.data.cpu().numpy()  # Masques segmentés
            class_names = results[0].names  # Noms des classes

            # Initialiser les bordures gauche et droite
            left_border = None
            right_border = None

            # Parcourir les masques pour la classe "path"
            for i, mask in enumerate(masks):
                if i<3:
                    class_name = class_names[i]
                    if class_name == "path":
                        # Trouver les pixels où le masque est actif
                        active_pixels = np.where(mask > 0)

                        if active_pixels[1].size > 0:  # Vérifier que des pixels existent
                            left_border = np.min(active_pixels[1])  # Bordure gauche (min x)
                            right_border = np.max(active_pixels[1])  # Bordure droite (max x)

                        # Annoter les bordures
                        if left_border is not None and right_border is not None:
                            cv2.line(frame, (left_border, 0), (left_border, height), (0, 255, 0), 2)  # Bordure gauche
                            cv2.line(frame, (right_border, 0), (right_border, height), (0, 0, 255), 2)  # Bordure droite

                            # Calculer la distance entre les bordures
                            distance = right_border - left_border
                            distance_text = f"Distance: {distance:.2f} pixels"

                            # Annoter la distance
                            cv2.putText(frame, distance_text, (50, 50),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # Écrire la frame annotée dans la vidéo de sortie
            output_video.write(frame)

        # Libérer les ressources
        video.release()
        output_video.release()
        print(f"Vidéo annotée sauvegardée dans : {output_path}")
