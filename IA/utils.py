import os

def get_cityscapes_pairs(image_folder, label_folder):
    image_paths = []
    label_paths = []

    for city_name in os.listdir(image_folder):
        img_dir = os.path.join(image_folder, city_name)
        lbl_dir = os.path.join(label_folder, city_name)

        for file_name in os.listdir(img_dir):
            if file_name.endswith("_leftImg8bit.png"):
                # Créer un chemin absolu pour l'image
                image_path = os.path.abspath(os.path.join(img_dir, file_name))
                
                # Créer le nom de fichier correspondant pour l'annotation
                label_file = file_name.replace("_leftImg8bit.png", "_gtFine_labelIds.png")
                
                # Créer un chemin absolu pour l'annotation
                label_path = os.path.abspath(os.path.join(lbl_dir, label_file))

                # Ajouter les chemins à leurs listes respectives
                image_paths.append(image_path)
                label_paths.append(label_path)

    return image_paths, label_paths
