#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32  # Pour publier la distance
from cv_bridge import CvBridge
from ultralytics import YOLO  # Pour utiliser YOLOv8
import math

class PathDetection(Node):
    def __init__(self):
        super().__init__('path_detection')

        # CvBridge pour la conversion ROS <-> OpenCV
        self.bridge = CvBridge()

        # Charger le modèle YOLOv8
        self.model = YOLO("/home/geicar/path_detection/runs/train2/best.pt")  # Remplacez par le chemin de votre modèle
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

        self.pub_compressed = self.create_publisher(
            CompressedImage,
            '/path_detection/compressed',
            10
        )

        # Paramètres de la caméra
        self.FOCAL_LENGTH = 490
        self.DEPTH_CONSTANT = 4

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
            self.get_logger().info(f"Distance publiée : {distance:.2f} m")

        # Publier l'image annotée
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
            annotated_image_compressed = self.bridge.cv2_to_compressed_imgmsg(annotated_image, dst_format='jpg')
            self.pub_annotated_image.publish(annotated_msg)
            self.get_logger().info("Image annotée publiée sur /path_detection/annotated_image")
            self.pub_compressed.publish(annotated_image_compressed)
            self.get_logger().info("Image annotée publiée sur /path_detection/compressed")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la publication de l'image annotée : {e}")

    def calculate_distance_and_annotate(self, frame, results):
        """
        Calcule la distance entre les bordures gauche et droite pour la classe "path" et
        annote l'image avec les bordures et la distance moyenne calculée.
        """
        if len(results) > 0 and results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy()  # Masques segmentés
            class_names = results[0].names  # Noms des classes

            distance_buffer = []  # Stocker les distances pour calculer la moyenne
            fps = 30  # Exemple d'estimation d'une valeur par seconde

            for i, mask in enumerate(masks):
                try:
                    class_id = int(results[0].boxes.cls[i].item())  # ID de la classe
                    if class_names[class_id] == "path":
                        # Obtenir les coordonnées de la bounding box
                        box = results[0].boxes.xyxy[i].cpu().numpy()
                        x1, y1, x2, y2 = map(int, box)

                        # Trouver la position aux 2/3 de la bounding box en hauteur
                        adjusted_y = y2 - (2 * (y2 - y1)) // 3

                        # Trouver les bordures gauche et droite alignées avec ce niveau
                        active_pixels = np.where(mask > 0)
                        row_pixels = active_pixels[1][active_pixels[0] == adjusted_y]
                        if row_pixels.size > 0:
                            left_border = np.min(row_pixels)
                            right_border = np.max(row_pixels)

                            # Calculer la distance réelle
                            distance_2d = self.calculate_2d_distance(
                                (left_border, adjusted_y), (right_border, adjusted_y), self.FOCAL_LENGTH, self.DEPTH_CONSTANT
                            )

                            # Ajouter la distance dans le buffer
                            distance_buffer.append(distance_2d)

                            # Maintenir uniquement les distances sur 2 secondes (fps frames)
                            if len(distance_buffer) > 2 * fps:
                                distance_buffer.pop(0)

                            # Calculer la moyenne des distances
                            avg_distance = sum(distance_buffer) / len(distance_buffer)

                            # Annoter uniquement les deux points
                            cv2.circle(frame, (left_border, adjusted_y), 5, (0, 255, 0), -1)  # Point gauche
                            cv2.circle(frame, (right_border, adjusted_y), 5, (0, 0, 255), -1)  # Point droit

                            # Tracer une ligne entre les deux points
                            cv2.line(frame, (left_border, adjusted_y), (right_border, adjusted_y), (0, 0, 0), 2)

                            # Annoter la distance moyenne en noir
                            cv2.putText(frame, f"Distance: {avg_distance:.2f} m", (50, 50),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
                            return avg_distance, frame
                except (IndexError, KeyError, AttributeError) as e:
                    self.get_logger().error(f"Erreur lors du traitement du masque {i}: {e}")

        # Si aucune bordure n'est détectée, retourner l'image originale et None
        return None, frame

    def calculate_2d_distance(self, point1, point2, focal_length, depth):
        """
        Calcule la distance 2D réelle entre deux points donnés en pixels.
        """
        x1_pixels, y1_pixels = point1
        x2_pixels, y2_pixels = point2
        x1_real = (x1_pixels * depth) / focal_length
        y1_real = (y1_pixels * depth) / focal_length
        x2_real = (x2_pixels * depth) / focal_length
        y2_real = (y2_pixels * depth) / focal_length
        distance_2d = math.sqrt((x2_real - x1_real) ** 2 + (y2_real - y1_real) ** 2)
        return distance_2d


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
