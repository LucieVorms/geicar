import os
import json
import cv2

def convert_cityscapes_to_yolo(cityscapes_annotations_path, output_path, image_folder, class_mapping):
    os.makedirs(output_path, exist_ok=True)
    for annotation_file in os.listdir(cityscapes_annotations_path):
        with open(os.path.join(cityscapes_annotations_path, annotation_file), 'r') as f:
            data = json.load(f)
            label_file = os.path.join(output_path, annotation_file.replace('.json', '.txt'))
            with open(label_file, 'w') as label_out:
                for obj in data['objects']:
                    class_id = class_mapping.get(obj['label'], -1)
                    if class_id == -1:
                        continue  # Ignorer les classes non mappées
                    bbox = obj['bbox']
                    img = cv2.imread(os.path.join(image_folder, data['image']['file_name']))
                    h, w, _ = img.shape
                    x_center = (bbox[0] + bbox[2]) / 2 / w
                    y_center = (bbox[1] + bbox[3]) / 2 / h
                    width = (bbox[2] - bbox[0]) / w
                    height = (bbox[3] - bbox[1]) / h
                    label_out.write(f"{class_id} {x_center} {y_center} {width} {height}\n")
