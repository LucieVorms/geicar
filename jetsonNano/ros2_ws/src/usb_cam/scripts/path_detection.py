#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String  # For textual detection messages
from cv_bridge import CvBridge
from ultralytics import YOLO  # To use YOLOv8


class PathDetection(Node):
    def __init__(self):
        super().__init__('path_detection')

        # CvBridge for ROS <-> OpenCV conversion
        self.bridge = CvBridge()

        # Load YOLO segmentation model
        self.model = YOLO("/home/geicar/path_detection/runs/train/weights/best.pt")  # Replace with your model path
        self.get_logger().info("YOLOv8 segmentation model loaded successfully!")

        # Subscribe to the camera topic
        self.sub = self.create_subscription(
            Image,
            'image_raw',  # Camera topic
            self.image_callback,
            10
        )

        # Publishers for results
        self.pub_detection = self.create_publisher(
            String,
            '/path_detection/results',  # Topic for textual detection results
            10
        )
        self.pub_annotated_image = self.create_publisher(
            Image,
            '/path_detection/annotated_image',  # Topic for annotated images
            10
        )
        self.pub_compressed_annotated_image = self.create_publisher(
            CompressedImage,
            '/path_detection/annotated_image/compressed',
            10
        )

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Run YOLO segmentation
        results = self.model(frame)

        # Annotate the image and extract detection details
        annotated_image, detection_info = self.process_segmentation_results(frame, results)

        # Publish detection details
        self.publish_detection_info(detection_info)

        # Publish the annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
            annotated_msg_compressed = self.bridge.cv2_to_compressed_imgmsg(annotated_image, dst_format='jpg')
            self.pub_annotated_image.publish(annotated_msg)
            self.pub_compressed_annotated_image.publish(annotated_msg_compressed)
            self.get_logger().info("Published annotated image on /path_detection/annotated_image")
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")

    def process_segmentation_results(self, frame, results):
        """
        Processes YOLO segmentation results, annotates the image, and extracts detection details.
        """
        detection_info = []
        if len(results) > 0 and results[0].masks is not None:
            annotated_image = frame.copy()
            for i, mask_data in enumerate(results[0].masks.data):
                class_id = results[0].boxes[i].cls[0].item()  # Class ID
                confidence = results[0].boxes[i].conf[0].item()  # Confidence score
                class_name = self.model.names[int(class_id)]  # Class name

                if class_name in ['grass', 'path', 'concrete-block']:
                    # Extract the mask and overlay it on the annotated image
                    mask = mask_data.cpu().numpy().astype(np.uint8) * 255
                    color = (0, 255, 0) if class_name == 'grass' else (255, 255, 0) if class_name == 'path' else (0, 0, 255)
                    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(annotated_image, contours, -1, color, 2)

                    # Add detection info
                    detection_info.append(f"{class_name} (Confidence: {confidence:.2f})")

            return annotated_image, detection_info
        else:
            # Return the original image and no detections if there are no results
            return frame, ["No detections"]

    def publish_detection_info(self, detection_info):
        """
        Publishes the detection information as a single message.
        """
        detection_msg = String()
        detection_msg.data = ", ".join(detection_info)
        self.pub_detection.publish(detection_msg)
        self.get_logger().info(f"Published detection results: {detection_msg.data}")


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
