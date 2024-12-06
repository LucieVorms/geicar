from utils import get_cityscapes_pairs
from dataset import CityscapesDataset
import torchvision.transforms as transforms
import torchvision.transforms.functional as F
from torch.utils.data import DataLoader
import matplotlib.pyplot as plt
import torch
import os
import numpy as np

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

# Collecter les chemins des images et annotations
train_img_folder = 'cityscape/leftImg8bit/train'
train_lbl_folder = 'cityscape/gtFine/train'
train_images, train_labels = get_cityscapes_pairs(train_img_folder, train_lbl_folder)

# Définir les transformations pour les images et les labels
image_transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((256, 512)),  # Fixer la hauteur et la largeur
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])

label_transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((256, 512), interpolation=F.InterpolationMode.NEAREST)  # Utiliser l'interpolation NEAREST pour les labels
])

# Créer le dataset et le DataLoader
train_dataset = CityscapesDataset(train_images, train_labels, image_transform=image_transform, label_transform=label_transform)
train_loader = DataLoader(train_dataset, batch_size=8, shuffle=True, num_workers=4)

# Vérification et Validation des Données
# Afficher une image et son masque correspondant pour vérifier les données
def visualize_sample(dataset, idx=0):
    image, label = dataset[idx]

    # Convertir Tensor en image pour affichage
    image = denormalize(image)  # Dé-normaliser l'image
    image = image.permute(1, 2, 0).numpy()  # Convertir (C, H, W) à (H, W, C) pour matplotlib
    label = label.numpy()  # Convertir le masque en NumPy pour affichage

    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.imshow(image)
    plt.title("Input Image")

    plt.subplot(1, 2, 2)
    plt.imshow(label, cmap='gray', vmin=0, vmax=18)  # Utiliser tab20 pour mieux distinguer les classes
    plt.title("Segmentation Mask")

    plt.show()

def denormalize(image):
    # Utiliser les mêmes mean et std que ceux utilisés dans la normalisation
    mean = torch.tensor([0.485, 0.456, 0.406])
    std = torch.tensor([0.229, 0.224, 0.225])
    
    image = F.normalize(image, mean=(-mean / std), std=(1 / std))
    return image

def visualize_remapped_label(dataset, idx=0):
    # Obtenir l'image et le label remappé à partir du dataset
    _, label = dataset[idx]

    # Convertir le label en NumPy pour l'affichage
    label = label.numpy()

    # Afficher le masque d'annotation
    plt.figure(figsize=(8, 8))
    plt.imshow(label, cmap='tab20', vmin=0, vmax=18)  # Utiliser un colormap distinct pour bien voir les différentes classes
    plt.title(f"Remapped Label for Sample {idx}")
    plt.colorbar()  # Ajouter une barre de couleurs pour voir la distribution des classes
    plt.show()

# Visualiser un échantillon du dataset
visualize_sample(train_dataset)
visualize_remapped_label(train_dataset, idx=0)


