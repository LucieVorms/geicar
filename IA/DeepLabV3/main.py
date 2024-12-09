from utils import get_cityscapes_pairs
from dataset import CityscapesDataset
import torchvision.transforms as transforms
from torch.utils.data import DataLoader
import matplotlib.pyplot as plt
import torchvision.transforms.functional as F
import torchvision.models.segmentation as models
from torchvision.models.segmentation import DeepLabV3_ResNet101_Weights
import torch.optim as optim
import time
import numpy as np
import torch
import os

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

if __name__ == "__main__":
    # Collecter les chemins des images et annotations
    train_img_folder = 'cityscape/leftImg8bit/train'
    train_lbl_folder = 'cityscape/gtFine/train'
    val_img_folder = 'cityscape/leftImg8bit/val'
    val_lbl_folder = 'cityscape/gtFine/val'

    train_images, train_labels = get_cityscapes_pairs(train_img_folder, train_lbl_folder)
    val_images, val_labels = get_cityscapes_pairs(val_img_folder, val_lbl_folder)


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
    # Réduire le dataset pour l'entraînement rapide
    # Utiliser l'ensemble complet pour l'entraînement et la validation
    train_dataset = CityscapesDataset(train_images, train_labels, image_transform=image_transform, label_transform=label_transform)
    val_dataset = CityscapesDataset(val_images, val_labels, image_transform=image_transform, label_transform=label_transform)

    # Créer les DataLoaders avec l'ensemble complet
    train_loader = DataLoader(train_dataset, batch_size=8, shuffle=True, num_workers=4)
    validation_loader = DataLoader(val_dataset, batch_size=4, shuffle=False, num_workers=4)
    

    # Utiliser des batch sizes petits pour valider l'entraînement
    train_loader_subset = DataLoader(train_dataset, batch_size=4, shuffle=True, num_workers=0)
    validation_loader_subset = DataLoader(val_dataset, batch_size=4, shuffle=False, num_workers=0)

    # #################"Entrainement du model##########################""
    # Définir le modèle DeepLabV3
    num_classes = 18  # 17 classes + fond
    weights = DeepLabV3_ResNet101_Weights.COCO_WITH_VOC_LABELS_V1
    model = models.deeplabv3_resnet101(weights=weights)  # Charger le modèle pré-entraîné avec les poids appropriés

    # Adapter la dernière couche de classification pour correspondre au nombre de classes
    model.classifier[4] = torch.nn.Conv2d(256, num_classes, kernel_size=(1, 1))

    # Déplacer le modèle sur le GPU si disponible
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = model.to(device)
        
    # Définir la fonction de perte
    criterion = torch.nn.CrossEntropyLoss(ignore_index=255)

    # Définir l'optimiseur
    optimizer = optim.Adam(model.parameters(), lr=0.001, weight_decay=1e-4)

    # Boucle d'entraînement
    # Définir le nombre d'époques
    num_epochs = 4
    for epoch in range(num_epochs):
        model.train()  # Mode entraînement
        running_loss = 0.0
        start_time = time.time()

        # Utiliser le sous-ensemble pour l'entraînement
        for batch_idx, (images, labels) in enumerate(train_loader_subset):
            images = images.to(device)
            labels = labels.to(device)

            # Assurez-vous que les labels sont de la forme correcte
            if labels.ndim == 4:
                labels = labels.squeeze(1)

            # Mettre à zéro les gradients
            optimizer.zero_grad()

            # Faire une prédiction
            outputs = model(images)['out']

            # Ajuster la taille des sorties si nécessaire
            if outputs.shape[-2:] != labels.shape[-2:]:
                outputs = torch.nn.functional.interpolate(outputs, size=labels.shape[-2:], mode="bilinear", align_corners=False)

            # Calculer la perte
            loss = criterion(outputs, labels)
            running_loss += loss.item()

            # Rétropropagation et optimisation
            loss.backward()
            optimizer.step()

            # Imprimer l'état de progression après chaque batch
            if (batch_idx + 1) % 2 == 0:  # Réduire la fréquence d'impression pour le test
                print(f"Epoch [{epoch+1}/{num_epochs}], Step [{batch_idx+1}/{len(train_loader_subset)}], Loss: {loss.item():.4f}")

        # Calculer et afficher la perte moyenne pour l'époque entière
        epoch_loss = running_loss / len(train_loader_subset)
        elapsed_time = time.time() - start_time
        print(f"Epoch {epoch+1}/{num_epochs}, Average Loss: {epoch_loss:.4f}, Time: {elapsed_time:.2f}s")

        def validate_model(model, validation_loader):
            model.eval()  # Mode évaluation (désactive le dropout et la batchnorm)
            running_loss = 0.0
            correct_pixels = 0
            total_pixels = 0

            with torch.no_grad():
                for images, labels in validation_loader:
                    images = images.to(device)
                    labels = labels.to(device)

                    # Faire une prédiction
                    outputs = model(images)['out']

                    # Calculer la perte
                    loss = criterion(outputs, labels)
                    running_loss += loss.item()

                    # Calculer la précision pixel par pixel
                    _, predicted = torch.max(outputs, 1)
                    correct_pixels += (predicted == labels).sum().item()
                    total_pixels += labels.numel()

            validation_loss = running_loss / len(validation_loader)
            accuracy = 100 * correct_pixels / total_pixels
            print(f'Validation Loss: {validation_loss:.4f}, Accuracy: {accuracy:.2f}%')

        # Valider le modèle sur l'ensemble de validation après chaque époque (facultatif pour un test rapide)
        validate_model(model, validation_loader_subset)

        def visualize_predictions(model, dataset, idx=0):
            """
            Visualise l'image d'entrée, la vérité terrain (ground truth) et la prédiction du modèle pour une image du dataset.

            Args:
                model (torch.nn.Module): Le modèle DeepLabV3 entraîné.
                dataset (torch.utils.data.Dataset): Le dataset de validation (ou d'entraînement) contenant les images et les labels.
                idx (int): L'index de l'image à visualiser.
            """
            model.eval()  # Mettre le modèle en mode évaluation pour désactiver dropout et batchnorm
            image, label = dataset[idx]
            image = image.unsqueeze(0).to(device)  # Ajouter une dimension batch et déplacer sur GPU si disponible

            # Faire la prédiction
            with torch.no_grad():
                output = model(image)['out']

            # Obtenir la prédiction
            predicted = torch.argmax(output, dim=1).squeeze(0).cpu().numpy()

            # Préparer l'image d'entrée pour l'affichage
            image = image.squeeze(0).cpu()  # Enlever la dimension batch et déplacer sur CPU
            image = image.permute(1, 2, 0)  # Convertir (C, H, W) en (H, W, C) pour l'affichage

            # Dé-normaliser l'image (revenir à l'échelle [0, 255])
            mean = torch.tensor([0.485, 0.456, 0.406])
            std = torch.tensor([0.229, 0.224, 0.225])
            image = image * std + mean
            image = torch.clamp(image, 0, 1)  # Limiter les valeurs entre 0 et 1 pour éviter des dépassements

            # Afficher l'image, la vérité terrain et la prédiction
            plt.figure(figsize=(15, 5))
            plt.subplot(1, 3, 1)
            plt.imshow(image)
            plt.title("Input Image")

            plt.subplot(1, 3, 2)
            plt.imshow(label.numpy(), cmap='gray', vmin=0, vmax=18)
            plt.title("Ground Truth")

            plt.subplot(1, 3, 3)
            plt.imshow(predicted, cmap='tab20', vmin=0, vmax=num_classes - 1)
            plt.title("Predicted Mask")

            plt.show()

        visualize_predictions(model, val_dataset, idx=0)  # Visualiser la prédiction sur une image de validation



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
        plt.imshow(label, cmap='gray', vmin=0, vmax=18)  
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



