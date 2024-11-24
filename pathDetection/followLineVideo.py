import cv2
import numpy as np

def detect_edges(image):
    # Convertir l'image en niveaux de gris
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Appliquer un flou pour réduire le bruit
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Utiliser le détecteur de Canny pour détecter les bords
    edges = cv2.Canny(blurred, 50, 150)
    
    return edges

def find_contours(edges):
    # Trouver les contours à partir des bords détectés
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def draw_contours(frame, contours):
    # Dessiner les contours sur l'image originale
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)

def calculate_center(contours, frame):
    # Calculer le centre du passage basé sur les contours
    height, width, _ = frame.shape
    center_x = width // 2
    path_center = None

    if len(contours) > 0:
        # Trouver le plus grand contour (on suppose qu'il représente le chemin)
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            path_center = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
            # Dessiner un cercle au centre du contour
            cv2.circle(frame, path_center, 5, (0, 0, 255), -1)

    # Dessiner une ligne verticale au centre de l'image pour référence
    cv2.line(frame, (center_x, 0), (center_x, height), (255, 0, 0), 2)

    return path_center, center_x

def adjust_direction(path_center, center_x):
    # Ajuster la direction de la voiture en fonction de la position du centre du chemin
    if path_center is not None:
        if path_center[0] < center_x - 20:
            print("Tourner à gauche")
        elif path_center[0] > center_x + 20:
            print("Tourner à droite")
        else:
            print("Avancer tout droit")
    else:
        print("Chemin non détecté")

def main():
    # Charger la vidéo depuis la caméra
    cap = cv2.VideoCapture(0)  # 0 est l'ID pour la caméra par défaut
    
    if not cap.isOpened():
        print("Erreur: Impossible d'accéder à la caméra.")
        return
    
    while True:
        # Lire une frame de la caméra
        ret, frame = cap.read()
        
        if not ret:
            print("Erreur: Impossible de lire la vidéo.")
            break
        
        # Détecter les bords
        edges = detect_edges(frame)
        
        # Trouver les contours à partir des bords détectés
        contours = find_contours(edges)
        
        # Dessiner les contours sur l'image originale
        draw_contours(frame, contours)
        
        # Calculer le centre du passage et ajuster la direction
        path_center, center_x = calculate_center(contours, frame)
        adjust_direction(path_center, center_x)
        
        # Afficher l'image avec les contours et la ligne centrale
        cv2.imshow("Contours du chemin", frame)
        
        # Quitter la boucle si l'utilisateur appuie sur 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Libérer la caméra et fermer les fenêtres
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
