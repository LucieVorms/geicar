import os
import shutil
import cv2
import numpy as np

# Dossiers d'entrée
GT_FINE_DIR = "..\\cityscape\\gtFine"
LEFT_IMG_DIR = "..\\cityscape\\leftImg8bit"
OUTPUT_DIR = "segmentation"

def process_cityscapes_for_segmentation():
    """Convert Cityscapes annotations into YOLO segmentation format."""
    splits = ['train', 'val']  # Vous pouvez inclure 'test' si nécessaire
    for split in splits:
        img_input_dir = os.path.join(LEFT_IMG_DIR, split)
        mask_input_dir = os.path.join(GT_FINE_DIR, split)
        img_output_dir = os.path.join(OUTPUT_DIR, split, "images")
        mask_output_dir = os.path.join(OUTPUT_DIR, split, "masks")

        # Créer les dossiers de sortie
        os.makedirs(img_output_dir, exist_ok=True)
        os.makedirs(mask_output_dir, exist_ok=True)

        # Parcourir chaque ville
        for city in os.listdir(img_input_dir):
            city_img_dir = os.path.join(img_input_dir, city)
            city_mask_dir = os.path.join(mask_input_dir, city)

            for img_file in os.listdir(city_img_dir):
                if img_file.endswith("_leftImg8bit.png"):
                    # Nom normalisé pour l'image et le masque
                    base_name = img_file.replace("_leftImg8bit.png", "")
                    
                    # Copier l'image avec un nom normalisé
                    shutil.copy(
                        os.path.join(city_img_dir, img_file),
                        os.path.join(img_output_dir, f"{base_name}.png")
                    )

                    # Utiliser le masque correspondant avec un nom normalisé
                    mask_file = img_file.replace("_leftImg8bit.png", "_gtFine_labelIds.png")
                    shutil.copy(
                        os.path.join(city_mask_dir, mask_file),
                        os.path.join(mask_output_dir, f"{base_name}.png")
                    )

                    print(f"Processed {base_name}.png")

def generate_yolo_labels():
    """Convert masks into YOLO format labels."""
    splits = ['train', 'val']
    for split in splits:
        mask_dir = os.path.join(OUTPUT_DIR, split, "masks")
        label_dir = os.path.join(OUTPUT_DIR, split, "labels")
        os.makedirs(label_dir, exist_ok=True)

        for mask_file in os.listdir(mask_dir):
            if mask_file.endswith(".png"):
                mask_path = os.path.join(mask_dir, mask_file)
                label_path = os.path.join(label_dir, mask_file.replace(".png", ".txt"))

                # Charger le masque
                mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
                unique_classes = np.unique(mask)  # Classes présentes dans le masque

                with open(label_path, "w") as label_file:
                    for cls in unique_classes:
                        if cls == 0:  # Ignorer le fond
                            continue

                        # Trouver les contours pour la classe
                        binary_mask = (mask == cls).astype(np.uint8)
                        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                        for contour in contours:
                            x, y, w, h = cv2.boundingRect(contour)
                            x_center = (x + w / 2) / mask.shape[1]
                            y_center = (y + h / 2) / mask.shape[0]
                            width = w / mask.shape[1]
                            height = h / mask.shape[0]

                            # Écrire au format YOLO : [class_id] [x_center] [y_center] [width] [height]
                            label_file.write(f"{cls} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")

                print(f"Generated label for {mask_file}")

# Appeler les fonctions pour générer les données
process_cityscapes_for_segmentation()
generate_yolo_labels()
