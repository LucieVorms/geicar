#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO  # To use YOLOv8


class PathDetection(Node):
    def __init__(self):
        super().__init__('path_detection')

        # CvBridge for the conversion between ROS and OpenCV
        self.bridge = CvBridge()

        # Load the YOLOv8 model (Segmentation model)
        self.model = YOLO("/root/geicar/path_detection/runs/train/weights/best.pt")  # Replace with your model path
        self.get_logger().info("YOLOv8 segmentation model loaded successfully!")

        # Subscribe to the camera topic
        self.sub = self.create_subscription(
            Image,
            'image_raw',  # Topic of the camera
            self.image_callback,
            10
        )

        # Create publishers for the segmentation mask
        self.pub_mask = self.create_publisher(
            Image,
            '/path_detection/mask',  # Topic for the segmentation mask
            10
        )
        self.pub_compressed_mask = self.create_publisher(
            CompressedImage,
            '/path_detection/mask/compressed',
            10
        )

    def image_callback(self, msg):
        try:
            # Convert the ROS message to an OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Apply YOLOv8 for segmentation on the image
        results = self.model(frame)
        
        # Extract the segmentation mask
        segmentation_mask = self.process_segmentation_results(results)

        # Publish the segmentation mask
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(segmentation_mask, encoding="mono8")
            mask_msg_compressed = self.bridge.cv2_to_compressed_imgmsg(segmentation_mask, dst_format='jpg')
            self.pub_mask.publish(mask_msg)
            self.pub_compressed_mask.publish(mask_msg_compressed)
            self.get_logger().info("Published segmentation mask on /path_detection/mask")
        except Exception as e:
            self.get_logger().error(f"Failed to publish segmentation mask: {e}")

    def process_segmentation_results(self, results):
        """
        Processes the YOLOv8 segmentation results and creates a binary mask.
        """
        if len(results) > 0 and results[0].masks is not None:
            # Combine all masks into one binary mask
            mask = np.zeros((results[0].masks.shape[1], results[0].masks.shape[2]), dtype=np.uint8)
            for mask_data in results[0].masks.data:
                mask = cv2.bitwise_or(mask, mask_data.cpu().numpy().astype(np.uint8) * 255)
            return mask
        else:
            # Return an empty mask if no results
            return np.zeros((640, 640), dtype=np.uint8)


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
