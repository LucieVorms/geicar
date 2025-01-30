from ultralytics import YOLO
import cv2
import os
import shutil

# Chemin du modèle entraîné
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "runs", "segment", "train2", "weights", "best.pt")
model = YOLO(model_path)

# Chemin des images de test
test_images_dir = os.path.join(script_dir, "images_test")
output_dir = os.path.join(script_dir, "segmentation_results")
os.makedirs(output_dir, exist_ok=True)

# Parcourir toutes les images
for image_name in os.listdir(test_images_dir):
    image_path = os.path.join(test_images_dir, image_name)

    if os.path.isfile(image_path) and image_name.lower().endswith(('.png', '.jpg', '.jpeg')):
        print(f"Traitement de l'image : {image_name}")

        try:
            # Effectuer la prédiction avec save=True
            results = model.predict(source=image_path, save=True, verbose=False)

            # Récupérer le dernier répertoire "runs/segment/predict"
            runs_dir = os.path.join(script_dir, "runs", "segment")
            latest_predict_dir = sorted(os.listdir(runs_dir), reverse=True)[0]
            predict_output_dir = os.path.join(runs_dir, latest_predict_dir)

            # Déplacer les résultats dans le répertoire "segmentation_results"
            for file in os.listdir(predict_output_dir):
                if file.endswith(('.png', '.jpg', '.jpeg')):
                    shutil.move(os.path.join(predict_output_dir, file),
                                os.path.join(output_dir, f"segmented_{image_name}"))

            print(f"Résultat sauvegardé dans : {output_dir}/segmented_{image_name}")

        except Exception as e:
            print(f"Erreur lors du traitement de l'image {image_name} : {e}")
            continue

print("Traitement terminé.")
