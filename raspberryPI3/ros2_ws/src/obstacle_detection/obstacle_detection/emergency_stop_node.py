import rclpy
from rclpy.node import Node
from interfaces.msg import Ultrasonic
from std_msgs.msg import Bool
from interfaces.msg import ObstacleInfo


OBSTACLE_THRESHOLD = 56


class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        
        
        self.publisher_ = self.create_publisher(Bool, 'obstacle_detected', 10)
        
        self.detailed_publisher_ = self.create_publisher(ObstacleInfo, 'obstacle_info', 10)

        self.subscription = self.create_subscription(Ultrasonic,'us_data',  self.listener_callback,10)



    def listener_callback(self, msg):
        
        obstacle_detected = False
        detected_sides = []

        # Vérifier les capteurs pour détecter des obstacles
        if msg.front_left < OBSTACLE_THRESHOLD:
            detected_sides.append("Front Left")
            obstacle_detected = True
        if msg.front_center < OBSTACLE_THRESHOLD:
            detected_sides.append("Front Center")
            obstacle_detected = True
        if msg.front_right < OBSTACLE_THRESHOLD:
            detected_sides.append("Front Right")
            obstacle_detected = True

        
        
        self.publisher_.publish(Bool(data=obstacle_detected))
        
        obstacle_info_msg = ObstacleInfo()
        obstacle_info_msg.obstacle_detected = obstacle_detected
        obstacle_info_msg.sides_detected = ", ".join(detected_sides) if detected_sides else "None"
        self.detailed_publisher_.publish(obstacle_info_msg)



def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
