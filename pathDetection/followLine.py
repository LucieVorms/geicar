import cv2
import numpy as np

def detect_edges(image):
    # Convertir l'image en niveaux de gris
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Appliquer un flou pour réduire le bruit
    blurred = cv2.GaussianBlur(gray, (9, 9), 0)
    
    # Utiliser le détecteur de Canny pour détecter les bords
    edges = cv2.Canny(blurred, threshold1=50, threshold2=200)
    
    return edges

def find_contours(edges):
    # Trouver les contours à partir des bords détectés
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def draw_contours(frame, contours):
    # Dessiner les contours sur l'image originale
    cv2.drawContours(frame, contours,-1, (0, 255, 0), 1)


def calculate_path_center(contours, frame):
    # Calculer le centre du passage basé sur les deux contours les plus grands (supposant qu'ils encadrent le chemin)
    height, width, _ = frame.shape
    center_x = width // 2
    left_contour = None
    right_contour = None

    # Dessiner une ligne verticale au centre de l'image pour référence
    cv2.line(frame, (center_x, 0), (center_x, height), (255, 0, 0), 2)

    if len(contours) > 1:
        # Trier les contours par aire décroissante et sélectionner les deux plus grands
        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
        left_contour = sorted_contours[0]
        right_contour = sorted_contours[1]

        # Calculer les moments pour obtenir le centre de chaque contour
        M_left = cv2.moments(left_contour)
        M_right = cv2.moments(right_contour)

        if M_left["m00"] != 0 and M_right["m00"] != 0:
            left_center_x = int(M_left["m10"] / M_left["m00"])
            right_center_x = int(M_right["m10"] / M_right["m00"])
            path_center_x = (left_center_x + right_center_x) // 2
            path_center = (path_center_x, height // 2)

            # Dessiner un cercle au centre du chemin
            cv2.circle(frame, path_center, 5, (0, 0, 255), -1)
            return path_center, center_x

    return None, center_x

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
    # Charger les images fournies
    image_paths = [
        "images\WIN_20241121_09_58_39_Pro.jpg",
        "images\WIN_20241121_09_58_45_Pro.jpg",
        "images\WIN_20241121_09_59_00_Pro.jpg",
        "images\WIN_20241121_09_59_12_Pro.jpg",
        "images\WIN_20241121_09_59_24_Pro.jpg",
        "images\WIN_20241121_09_59_36_Pro.jpg",
        "images\WIN_20241121_10_00_02_Pro.jpg",
        "images\WIN_20241121_10_00_13_Pro.jpg",
        "images\WIN_20241121_10_00_23_Pro.jpg",
        "images\WIN_20241121_10_00_34_Pro.jpg",
        "images\WIN_20241121_10_00_47_Pro.jpg",
        "images\WIN_20241121_10_00_56_Pro.jpg",
        "images\WIN_20241121_10_01_08_Pro.jpg"
    ]
    
    for image_path in image_paths:
        # Charger l'image
        full_frame = cv2.imread(image_path)
        
        if full_frame is None:
            print(f"Erreur: Impossible de charger l'image {image_path}.")
            continue
        
        # Créer une copie de l'image complète pour l'affichage
        display_frame = full_frame.copy()
        
        # Considérer uniquement le premier quart inférieur de l'image pour l'analyse
        height, width, _ = full_frame.shape
        analysis_frame = full_frame[3 * height // 4:, :]
        
        # Détecter les bords
        edges = detect_edges(analysis_frame)
        
        # Trouver les contours à partir des bords détectés
        contours = find_contours(edges)
        
        # Dessiner les contours sur la partie affichée
        draw_contours(display_frame[3 * height // 4:, :], contours)
        
        # Calculer le centre du passage encadré par les deux plus grands contours et ajuster la direction
        path_center, center_x = calculate_path_center(contours, display_frame[3 * height // 4:, :])
        adjust_direction(path_center, center_x)
        
        # Afficher l'image complète avec les contours et la ligne centrale
        cv2.imshow("Contours du chemin", display_frame)
        
        # Attendre que l'utilisateur appuie sur une touche pour passer à l'image suivante
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break
    
    # Fermer les fenêtres
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


