import cv2
import os
from ultralytics import YOLO

# Chemin du répertoire courant
script_dir = os.path.dirname(os.path.abspath(__file__))

# Chemin vers le modèle entraîné
model_path = os.path.join(script_dir, "runs", "segment", "train2", "weights", "best.pt")
model = YOLO(model_path)

# Chemin vers la vidéo à traiter
video_path = os.path.join(script_dir, "WIN_20250109_10_10_05_Pro.mp4")
output_path = os.path.join(script_dir, "video_seg1.mp4")

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

            # Annoter la frame avec la segmentation
            annotated_frame = results[0].plot()  # Ajouter les masques et annotations sur la frame

            # Écrire la frame annotée dans la vidéo de sortie
            output_video.write(annotated_frame)

        # Libérer les ressources
        video.release()
        output_video.release()
        print(f"Vidéo annotée sauvegardée dans : {output_path}")
