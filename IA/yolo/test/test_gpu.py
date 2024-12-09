import torch

# Vérifier si un GPU est disponible
if torch.cuda.is_available():
    print(f"GPU Available: {torch.cuda.get_device_name(0)}")
else:
    print("No GPU available, using CPU")
