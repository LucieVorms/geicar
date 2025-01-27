import rclpy
from rclpy.node import Node
from interfaces.msg import Ultrasonic
from std_msgs.msg import Bool
from interfaces.msg import ObstacleInfo


OBSTACLE_THRESHOLD = 56
CRITICAL_THRESHOLD = 38


class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        
        
        self.publisher_ = self.create_publisher(Bool, 'obstacle_detected', 10)
        
        self.detailed_publisher_ = self.create_publisher(ObstacleInfo, 'obstacle_info', 10)

        self.subscription = self.create_subscription(Ultrasonic,'us_data',  self.listener_callback,10)



    def listener_callback(self, msg):
        
        obstacle_detected = False
        critical_detected = False
        detected_sides = []
        critical_sides = []
        rear_sides = []
        rear_obstacles_detected  = False

        # Vérifier les capteurs pour détecter des obstacles
        if msg.front_left < (OBSTACLE_THRESHOLD-15):
            detected_sides.append("Front Left")
            obstacle_detected = True
            if msg.front_left < (CRITICAL_THRESHOLD-8):
                critical_sides.append("Front Left")
                critical_detected = True
        if msg.front_center < (OBSTACLE_THRESHOLD-6):
            detected_sides.append("Front Center")
            obstacle_detected = True
            if msg.front_center < CRITICAL_THRESHOLD:
                critical_sides.append("Front Center")
                critical_detected = True
        if msg.front_right < (OBSTACLE_THRESHOLD-15):
            detected_sides.append("Front Right")
            obstacle_detected = True
            if msg.front_right < (CRITICAL_THRESHOLD-8):
                critical_sides.append("Front Right")
                critical_detected = True
        if msg.rear_right < OBSTACLE_THRESHOLD:
            rear_sides.append("Rear Right")
            rear_obstacles_detected = True
        if msg.rear_center < OBSTACLE_THRESHOLD:
            rear_sides.append("Rear Center")
            rear_obstacles_detected = True
        if msg.rear_left < OBSTACLE_THRESHOLD:
            rear_sides.append("Rear Left")
            rear_obstacles_detected = True 

        
        
        self.publisher_.publish(Bool(data=obstacle_detected))
        
        obstacle_info_msg = ObstacleInfo()
        obstacle_info_msg.obstacle_detected = obstacle_detected
        obstacle_info_msg.sides_detected = ", ".join(detected_sides) if detected_sides else "None"
        obstacle_info_msg.critical_detected = critical_detected
        obstacle_info_msg.critical_sides = ", ".join(critical_sides) if critical_sides else "None"
        obstacle_info_msg.rear_sides = ", ".join(rear_sides) if rear_sides else "None"
        obstacle_info_msg.rear_obstacles_detected = rear_obstacles_detected
        self.detailed_publisher_.publish(obstacle_info_msg)



def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
