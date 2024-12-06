import numpy as np
import cv2
import torch
from torch.utils.data import Dataset

# Dictionnaire de correspondance entre les IDs d'origine et les nouveaux IDs
id_mapping = {
    0: 0,     # Fond
    7: 1,     # Route
    8: 2,     # Trottoir
    11: 3,    # Bâtiment
    12: 4,    # Mur
    13: 5,    # Clôture
    17: 6,    # Poteau
    19: 7,    # Feu de signalisation
    20: 8,    # Panneau de signalisation
    21: 9,    # Végétation
    22: 10,   # Terre
    23: 11,   # Banc
    24: 12,   # Piéton
    25: 13,   # Cycliste
    26: 14,   # Voiture
    27: 15,   # Camion
    32: 16,   # Moto
    33: 17,   # Vélo
}

# Toute valeur qui n'est pas spécifiée dans le mapping sera ignorée (255)
default_value = 255

# Fonction de remapping des labels
def remap_labels(label):
    # Convertir le label en un tableau numpy
    label_array = np.array(label)
    
    # Créer un nouveau tableau pour stocker les labels remappés
    remapped_label = np.full_like(label_array, fill_value=default_value)

    # Appliquer la correspondance des labels
    for k, v in id_mapping.items():
        remapped_label[label_array == k] = v

    return remapped_label

class CityscapesDataset(Dataset):
    def __init__(self, image_paths, label_paths, image_transform=None, label_transform=None):
        self.image_paths = image_paths
        self.label_paths = label_paths
        self.image_transform = image_transform
        self.label_transform = label_transform

    def __len__(self):
        return len(self.image_paths)

    def __getitem__(self, idx):
        # Charger l'image
        image = cv2.imread(self.image_paths[idx])
        if image is None:
            raise FileNotFoundError(f"Image not found or cannot be read: {self.image_paths[idx]}")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Charger le masque d'annotation
        label = cv2.imread(self.label_paths[idx], cv2.IMREAD_GRAYSCALE)
        if label is None:
            raise FileNotFoundError(f"Label not found or cannot be read: {self.label_paths[idx]}")

        # Remapper les labels
        label = remap_labels(label)

        # Appliquer les transformations sur l'image
        if self.image_transform:
            image = self.image_transform(image)
        
        # Appliquer les transformations sur le label
        if self.label_transform:
            label = self.label_transform(label)
            # Convertir le label en NumPy array après transformation
            label = np.array(label)

        # Convertir le masque en tensor PyTorch
        label = torch.tensor(label, dtype=torch.long)

        return image, label
