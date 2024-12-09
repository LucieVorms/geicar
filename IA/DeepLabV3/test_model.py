import torch
import torchvision.models.segmentation as models
import cv2
import matplotlib.pyplot as plt
from main import num_classes, device  # Importer les variables nécessaires
import torchvision.transforms as transforms

def load_trained_model(model_path):
    model = models.deeplabv3_resnet101(weights=None)
    model.classifier[4] = torch.nn.Conv2d(256, num_classes, kernel_size=(1, 1))
    model.load_state_dict(torch.load(model_path))
    model = model.to(device)
    model.eval()
    return model

def test_model_on_image(model, image_path):
    # Charger et transformer l'image
    image = cv2.imread(image_path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize((256, 512)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])(image)
    image = image.unsqueeze(0).to(device)  # Ajouter une dimension batch

    # Faire une prédiction
    with torch.no_grad():
        output = model(image)['out']
        predicted = torch.argmax(output, dim=1).squeeze(0).cpu().numpy()

    # Afficher l'image et la prédiction
    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.imshow(image.squeeze(0).permute(1, 2, 0).cpu())
    plt.title("Image Input")

    plt.subplot(1, 2, 2)
    plt.imshow(predicted, cmap='tab20', vmin=0, vmax=num_classes - 1)
    plt.title("Segmentation Output")
    plt.show()


if __name__ == "__main__":
    model_path = "deeplabv3_cityscapes.pth"
    model = load_trained_model(model_path)
    test_model_on_image(model, "path_to_your_image.jpg")
