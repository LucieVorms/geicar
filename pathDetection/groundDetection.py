import cv2
import numpy as np

def is_majority_ground(image, lower_hsv, upper_hsv, threshold=0.3):
    # Convertir l'image en espace de couleur HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Créer un masque pour détecter les pixels dans la plage de teintes du sol
    mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)
    
    # Calculer la proportion de pixels correspondant au sol
    ground_pixels = np.sum(mask > 0)
    total_pixels = mask.size
    proportion = ground_pixels / total_pixels
    
    # Vérifier si la proportion de pixels du sol est supérieure au seuil
    if proportion > threshold:
        return True
    else:
        return False

# Charger l'image
image = cv2.imread("images/WIN_20241121_09_59_12_Pro.jpg")

# Définir les limites inférieure et supérieure de la teinte du sol en HSV
lower_hsv = np.array([0, 0, 0])   # Exemple de teinte pour le sol (à ajuster selon votre cas)
upper_hsv = np.array([45, 165, 60])


# Afficher l'image et le masque pour vérification
mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), lower_hsv, upper_hsv)
cv2.imshow('Image', image)
cv2.imshow('Masque Sol', mask)
cv2.waitKey(0)
cv2.destroyAllWindows()