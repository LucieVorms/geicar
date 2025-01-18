import cv2
import time
import os

# Créer un dossier pour stocker les images capturées
output_dir = "captured_images"
os.makedirs(output_dir, exist_ok=True)

# Ouvrir la caméra (habituellement /dev/video0)
cap = cv2.VideoCapture(0)

# Vérifier si la caméra s'est ouverte correctement
if not cap.isOpened():
    print("Erreur : Impossible d'ouvrir la caméra.")
    exit()

print("Appuyez sur Ctrl+C pour arrêter la capture.")

try:
    # Boucle infinie pour capturer des images toutes les secondes
    while True:
        # Capturer une image
        ret, frame = cap.read()

        # Vérifier si l'image a été capturée avec succès
        if not ret:
            print("Erreur : Impossible de lire l'image.")
            break

        # Générer un nom de fichier unique basé sur le timestamp
        timestamp = int(time.time())
        image_filename = os.path.join(output_dir, f"image_{timestamp}.jpg")

        # Sauvegarder l'image capturée
        cv2.imwrite(image_filename, frame)
        print(f"Image sauvegardée sous {image_filename}")

        # Attendre une seconde avant la prochaine capture
        time.sleep(1)

except KeyboardInterrupt:
    print("\nCapture arrêtée par l'utilisateur.")

finally:
    # Libérer la caméra
    cap.release()
    print("Caméra libérée.")
