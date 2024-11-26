import rclpy  # To use ROS2 with Python
from rclpy.node import Node  # To make a node
from sensor_msgs.msg import Image  # To work with Image's type messages (use by the cam)
from cv_bridge import CvBridge  # To convert ROS images in OpenCV format 
import cv2  # To display and treat images

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber') # Calls the constructor of the parent class (Node) and gives our node a name "image_subscriber"

        #Subscribe to 'image_raw' topic
        self.subscription = self.create_subscription(
            Image, #Type
            'image_raw', #Topic name
            self.listener_callback, #Callback function when msg is received
            10) #Size of waiting list
        
        self.bridge = CvBridge()

    #Each time a message on image_raw is received, it converts it in OpenCV image and display it 
    def listener_callback(self, msg):
        try :
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Conversion error ROS to openCV : {str(e)}")
            return
        cv2.imshow('Image', frame)
        cv2.waitKey(1)  # wait a little bit before the next image

def main(args=None):
    rclpy.init(args=args) #ROS2 initialization

    #Node creation and start listening
    image_subscriber = ImageSubscriber()

     #The node run until we stop it
    try :
         rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass

    #Stop the node
    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()



