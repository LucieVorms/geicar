from utils import get_cityscapes_pairs
from dataset import CityscapesDataset
import torchvision.transforms as transforms
import torchvision.transforms.functional as F
from torch.utils.data import DataLoader
import matplotlib.pyplot as plt
import torch
import os

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

# Collecter les chemins des images et annotations
train_img_folder = 'cityscape/leftImg8bit/train'
train_lbl_folder = 'cityscape/gtFine/train'
train_images, train_labels = get_cityscapes_pairs(train_img_folder, train_lbl_folder)

transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize(256),  # Redimensionner pour que la hauteur soit 256 pixels, et ajuster la largeur pour garder le ratio d'aspect
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])

# Créer le dataset et le DataLoader
train_dataset = CityscapesDataset(train_images, train_labels, transform=transform)
train_loader = DataLoader(train_dataset, batch_size=8, shuffle=True, num_workers=4)

# Vérification et Validation des Données
# Afficher une image et son masque correspondant pour vérifier les données
def visualize_sample(dataset, idx=0):
    image, label = dataset[idx]

    # Convertir Tensor en image pour affichage
    image = denormalize(image)  # Dé-normaliser l'image
    image = image.permute(1, 2, 0)  # Convertir (C, H, W) à (H, W, C) pour matplotlib
    label = label.numpy()  # Convertir le masque en NumPy pour affichage

    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.imshow(image)
    plt.title("Input Image")

    plt.subplot(1, 2, 2)
    plt.imshow(label, cmap='gray')
    plt.title("Segmentation Mask")

    plt.show()

def denormalize(image):
    # Utiliser les mêmes mean et std que ceux utilisés dans la normalisation
    mean = torch.tensor([0.485, 0.456, 0.406])
    std = torch.tensor([0.229, 0.224, 0.225])
    
    image = F.normalize(image, mean=(-mean / std), std=(1 / std))
    return image

# Visualiser un échantillon du dataset
visualize_sample(train_dataset)
