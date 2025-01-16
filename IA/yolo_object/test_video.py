from ultralytics import YOLO
import cv2
import os

# Chemin du répertoire courant
script_dir = os.path.dirname(os.path.abspath(__file__))

# Chemin vers le modèle entraîné
model_path = os.path.join(script_dir, "runs", "detect", "train4", "weights", "best.pt")
model = YOLO(model_path)

# Chemin vers la vidéo à traiter
video_path = os.path.join(script_dir, "video_test_1.mp4")
output_path = os.path.join(script_dir, "output_video4.mp4")

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
            
            # Annoter la frame avec les résultats
            annotated_frame = results[0].plot()

            # Écrire la frame annotée dans la vidéo de sortie
            output_video.write(annotated_frame)

        # Libérer les ressources
        video.release()
        output_video.release()

        print(f"Vidéo traitée et sauvegardée dans : {output_path}")
