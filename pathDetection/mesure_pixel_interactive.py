import cv2
import math

# Variables globales pour stocker les points cliqués
points = []

def click_event(event, x, y, flags, param):
    """Capture les clics de souris pour sélectionner les points."""
    global points, temp_image
    if event == cv2.EVENT_LBUTTONDOWN:  # Clic gauche
        points.append((x, y))
        print(f"Point sélectionné : {x}, {y}")
        
        # Affiche le point cliqué sur l'image
        cv2.circle(temp_image, (x, y), 5, (0, 255, 0), -1)  # Dessine un cercle vert
        cv2.imshow("Image", temp_image)

        # Si deux points sont sélectionnés, affiche la distance et trace une ligne
        if len(points) == 2:
            # Calculer la distance entre les deux points
            p1, p2 = points
            distance_pixels = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
            print(f"Distance mesurée (pixels) : {distance_pixels:.2f}")

            # Entrer la distance réelle
            real_distance_cm = float(input("Entrez la distance réelle entre les deux points (en cm) : "))

            # Calculer l'échelle (cm/pixel)
            cm_per_pixel = real_distance_cm / distance_pixels
            print(f"Échelle : {cm_per_pixel:.4f} cm/pixel")
            print(f"1 pixel correspond à {1/cm_per_pixel:.4f} pixels/cm")

            # Dessiner une ligne entre les deux points
            cv2.line(temp_image, p1, p2, (255, 0, 0), 2)  # Trace une ligne bleue entre les points
            cv2.imshow("Image", temp_image)

            # Fermer la fenêtre après calcul
            cv2.waitKey(0)
            cv2.destroyAllWindows()

# Charger l'image
image_path = "images\WIN_20241129_11_06_13_Pro.jpg"  # Chemin vers l'image
image = cv2.imread(image_path)
temp_image = image.copy()

# Afficher l'image et attendre les clics
cv2.imshow("Image", temp_image)
cv2.setMouseCallback("Image", click_event)
cv2.waitKey(0)
cv2.destroyAllWindows()
