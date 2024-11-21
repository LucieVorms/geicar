import cv2
import numpy as np

def detect_and_draw_lines_with_adjusted_bottom_limit(image_path):
    # Charger l'image
    image = cv2.imread(image_path)

    # Prétraitement
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(gray_image, (7, 7), 0)
    edges = cv2.Canny(blurred_image, threshold1=50, threshold2=200)

    # Trouver les contours
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Diviser les points des contours en deux parties : gauche et droite dans la moitié basse
    height, width = edges.shape
    mid_x = width // 2
    half_y = 2*height // 3

    # Points pour la moitié basse de l'image avec limite ajustée
    left_contours = [pt for contour in contours for pt in contour if pt[0][0] < mid_x and half_y <= pt[0][1] ]
    right_contours = [pt for contour in contours for pt in contour if pt[0][0] >= mid_x and half_y <= pt[0][1]]

    # Initialiser les points
    A1, A2, B1, B2 = None, None, None, None

    # Trouver les points pour la moitié droite (A1 et B1)
    if right_contours:
        A1 = min(right_contours, key=lambda pt: (pt[0][1], pt[0][0]))[0]  # Haut + droite
        B1 = max(right_contours, key=lambda pt: (pt[0][1], -pt[0][0]))[0]  # Bas + droite

    # Trouver les points pour la moitié gauche (A2 et B2)
    if left_contours:
        A2 = min(left_contours, key=lambda pt: (pt[0][1], -pt[0][0]))[0]  # Haut + gauche
        B2 = max(left_contours, key=lambda pt: pt[0][1])[0]  # Bas + gauche

    # Dessiner les lignes et marquer les points
    result_image = image.copy()
    if A1 is not None and B1 is not None:
        cv2.line(result_image, tuple(A1), tuple(B1), (255, 0, 0), 3)  # Ligne bleue (droite)
        cv2.circle(result_image, tuple(A1), 10, (255, 255, 0), -1)  # Marquer A1
        cv2.circle(result_image, tuple(B1), 10, (255, 255, 0), -1)  # Marquer B1
    if A2 is not None and B2 is not None:
        cv2.line(result_image, tuple(A2), tuple(B2), (0, 0, 255), 3)  # Ligne rouge (gauche)
        cv2.circle(result_image, tuple(A2), 10, (0, 255, 255), -1)  # Marquer A2
        cv2.circle(result_image, tuple(B2), 10, (0, 255, 255), -1)  # Marquer B2

    # Débogage visuel : Afficher les contours détectés
    debug_image = image.copy()
    cv2.drawContours(debug_image, contours, -1, (0, 255, 0), 1)

    # Afficher les résultats
    cv2.imshow("Contours detectes", debug_image)
    cv2.imshow("Resultat avec limite ajustee", result_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Appliquer le code sur l'image
detect_and_draw_lines_with_adjusted_bottom_limit("images/WIN_20241121_09_58_39_Pro.jpg")
