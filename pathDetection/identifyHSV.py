import cv2
import numpy as np

def pick_color(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_image = cv2.cvtColor(param, cv2.COLOR_BGR2HSV)
        pixel_hsv = hsv_image[y, x]
        print(f"HSV value at ({x}, {y}): {pixel_hsv}")

# Charger l'image
image = cv2.imread('images/WIN_20241121_09_59_12_Pro.jpg')

# Créer une fenêtre et définir la fonction de rappel pour les clics de souris
cv2.imshow('Image', image)
cv2.setMouseCallback('Image', pick_color, param=image)

cv2.waitKey(0)
cv2.destroyAllWindows()

