#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32  
from cv_bridge import CvBridge
from ultralytics import YOLO 
import math

class PathDetection(Node):
    def __init__(self):
        super().__init__('path_detection')

        # Convert ROS <-> OpenCV
        self.bridge = CvBridge()

        # YOLOv8
        self.model = YOLO("/home/geicar/path_detection/runs/train2/best.pt")  
        self.get_logger().info("Modèle YOLOv8 chargé avec succès !")

        # Subscribe to the camera topic
        self.sub = self.create_subscription(
            Image,
            'image_raw', 
            self.image_callback,
            10
        )

        #Publish the distance
        self.pub_distance = self.create_publisher(
            Float32,
            '/path_detection/results',  
            10
        )

        # Publish annotated images
        self.pub_annotated_image = self.create_publisher(
            Image,
            '/path_detection/annotated_image', 
            10
        )

        self.pub_compressed = self.create_publisher(
            CompressedImage,
            '/path_detection/compressed',
            10
        )

        #Camera parameter
        self.FOCAL_LENGTH = 490 
        self.DEPTH_CONSTANT = 4 #Distance between the point taken to measure and the camera

    def image_callback(self, msg):
        try:
            # Convert ROS image in OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la conversion de l'image : {e}")
            return

        # Predict with YOLO
        results = self.model.predict(source=frame, save=False, verbose=False)

        # Compute and publish the distance
        distance, annotated_image = self.calculate_distance_and_annotate(frame, results)

        if distance is not None:
            distance_msg = Float32()
            distance_msg.data = distance
            self.pub_distance.publish(distance_msg)
            self.get_logger().info(f"Distance publiée : {distance:.2f} m")

        # Publish the annotated image and the one compressed (for the web)
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
        Take two point on each edge of the "path" detected, compute the distance and annotate the image
        """

        # Loop that runs through predictions
        if len(results) > 0 and results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy()  
            class_names = results[0].names 

            # Variable initialization for averaging
            distance_buffer = []
            fps = 2  # Number of estimation per seconds

            for i, mask in enumerate(masks):
                try:
                    class_id = int(results[0].boxes.cls[i].item())
                    # Focus on the "path" only and not on "grass" and "concrete block"
                    if class_names[class_id] == "path":
                        # Get the coordinates of the bounding box
                        box = results[0].boxes.xyxy[i].cpu().numpy()
                        x1, y1, x2, y2 = map(int, box)

                        # Choose to take the 2 points at 2/3 of the height (on the image) of the path 
                        adjusted_y = y2 - (2 * (y2 - y1)) // 3

                        # Find the edges
                        active_pixels = np.where(mask > 0)
                        row_pixels = active_pixels[1][active_pixels[0] == adjusted_y]
                        if row_pixels.size > 0:
                            left_border = np.min(row_pixels)
                            right_border = np.max(row_pixels)

                            # Convert into meters
                            distance_2d = self.calculate_2d_distance(
                                (left_border, adjusted_y), (right_border, adjusted_y), self.FOCAL_LENGTH, self.DEPTH_CONSTANT
                            )

                            # Averaging during 5 seconds : on 10 predictions
                            distance_buffer.append(distance_2d)
                            
                            if len(distance_buffer) > 2 * 10 :
                                distance_buffer.pop(0)
                                
                            avg_distance = sum(distance_buffer) / len(distance_buffer)

                            # Annotations
                            cv2.circle(frame, (left_border, adjusted_y), 5, (0, 255, 0), -1)  # Point gauche
                            cv2.circle(frame, (right_border, adjusted_y), 5, (0, 0, 255), -1)  # Point droit
                            cv2.line(frame, (left_border, adjusted_y), (right_border, adjusted_y), (0, 0, 0), 2)
                            cv2.putText(frame, f"Distance: {avg_distance:.2f} m", (50, 50),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
                            return avg_distance, frame
                except (IndexError, KeyError, AttributeError) as e:
                    self.get_logger().error(f"Erreur lors du traitement du masque {i}: {e}")

        # If no border is detected, return the original image and None
        return None, frame

    def calculate_2d_distance(self, point1, point2, focal_length, depth):
        """
        Convert pixels into meters
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
