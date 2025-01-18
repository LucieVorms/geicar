#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32  # Pour publier la distance
from cv_bridge import CvBridge
from ultralytics import YOLO  # Pour utiliser YOLOv8

class PathDetection(Node):
    def __init__(self):
        super().__init__('path_detection')

        # CvBridge pour la conversion ROS <-> OpenCV
        self.bridge = CvBridge()

        # Charger le modèle YOLOv8
        self.model = YOLO("/home/geicar/path_detection/runs/train/weights/best.pt")  # Remplacez par le chemin de votre modèle
        self.get_logger().info("Modèle YOLOv8 chargé avec succès !")

        # Souscription au topic de la caméra
        self.sub = self.create_subscription(
            Image,
            'image_raw',  # Topic de la caméra
            self.image_callback,
            10
        )

        # Publication de la distance détectée
        self.pub_distance = self.create_publisher(
            Float32,
            '/path_detection/results',  # Topic pour publier la distance
            10
        )

        # Publication des images annotées
        self.pub_annotated_image = self.create_publisher(
            Image,
            '/path_detection/annotated_image',  # Topic pour publier les images annotées
            10
        )

    def image_callback(self, msg):
        try:
            # Convertir l'image ROS en image OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la conversion de l'image : {e}")
            return

        # Effectuer la segmentation avec YOLO
        results = self.model.predict(source=frame, save=False, verbose=False)

        # Calculer et publier la distance
        distance, annotated_image = self.calculate_distance_and_annotate(frame, results)

        if distance is not None:
            distance_msg = Float32()
            distance_msg.data = distance
            self.pub_distance.publish(distance_msg)
            self.get_logger().info(f"Distance publiée : {distance:.2f} pixels")

        # Publier l'image annotée
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
            self.pub_annotated_image.publish(annotated_msg)
            self.get_logger().info("Image annotée publiée sur /path_detection/annotated_image")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la publication de l'image annotée : {e}")

    def calculate_distance_and_annotate(self, frame, results):
        """
        Calcule la distance entre les bordures gauche et droite pour la classe "path" et
        annote l'image avec les bordures et la distance.
        """
        if len(results) > 0 and results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy()  # Masques segmentés
            class_names = results[0].names  # Noms des classes

            left_border = None
            right_border = None

            # Parcourir les masques pour trouver la classe "path"
            for i, mask in enumerate(masks):
                class_name = class_names[i]
                if class_name == "path":
                    # Trouver les pixels actifs dans le masque
                    active_pixels = np.where(mask > 0)

                    if active_pixels[1].size > 0:  # Vérifier si des pixels existent
                        left_border = np.min(active_pixels[1])  # Bordure gauche (min x)
                        right_border = np.max(active_pixels[1])  # Bordure droite (max x)

                    # Si les deux bordures sont détectées, calculer la distance
                    if left_border is not None and right_border is not None:
                        distance = right_border - left_border

                        # Annoter les bordures sur l'image
                        cv2.line(frame, (left_border, 0), (left_border, frame.shape[0]), (0, 255, 0), 2)  # Bordure gauche
                        cv2.line(frame, (right_border, 0), (right_border, frame.shape[0]), (0, 0, 255), 2)  # Bordure droite

                        # Annoter la distance
                        cv2.putText(frame, f"Distance: {distance:.2f} pixels", (50, 50),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                        return distance, frame

        # Si aucune bordure n'est détectée, retourner l'image originale et None
        return None, frame


def main(args=None):
    rclpy.init(args=args)

    path_detection_node = PathDetection()

    try:
        rclpy.spin(path_detection_node)
    except KeyboardInterrupt:
        pass

    path_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
