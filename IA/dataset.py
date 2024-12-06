import cv2
import torch
from torch.utils.data import Dataset

class CityscapesDataset(Dataset):
    def __init__(self, image_paths, label_paths, transform=None):
        self.image_paths = image_paths
        self.label_paths = label_paths
        self.transform = transform

    def __len__(self):
        return len(self.image_paths)

    def __getitem__(self, idx):
        # Charger l'image
        image = cv2.imread(self.image_paths[idx])
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Charger le masque d'annotation
        label = cv2.imread(self.label_paths[idx], cv2.IMREAD_GRAYSCALE)

        # Appliquer les transformations
        if self.transform:
            image = self.transform(image)
            label = torch.tensor(label, dtype=torch.long)  # Garder le masque sous forme de tensor

        return image, label
