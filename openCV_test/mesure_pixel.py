import cv2
import numpy as np
import math

# Charger l'image
image = cv2.imread("images/WIN_20241129_11_06_13_Pro.jpg")

# Coordonnées des extrémités de la référence dans l'image (en pixels)
point1 = (697, 358)  # Point de départ
point2 = (697, 332)  # Point de fin

# Distance réelle de la référence (en cm)
real_distance_cm = 1  # Exemple : 20 cm

# Calculer la distance entre les deux points dans l'image (en pixels)
distance_pixels = math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

# Échelle en cm/pixel
cm_per_pixel = real_distance_cm / distance_pixels

print(f"Distance mesurée (pixels) : {distance_pixels:.2f}")
print(f"Échelle : {cm_per_pixel:.4f} cm/pixel")
