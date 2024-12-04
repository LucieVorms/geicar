import rclpy
from rclpy.node import Node
from interfaces.msg import Ultrasonic
from std_msgs.msg import Bool

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        
        
        self.publisher_ = self.create_publisher(Bool, 'obstacle_detected', 10)
        
       
        self.subscription = self.create_subscription(
            Ultrasonic,
            'us_data',  
            self.listener_callback,
            10
        )
        self.subscription 

    def listener_callback(self, msg):
        
        obstacle_detected = any([

            msg.front_left < 56,
            msg.front_center < 56,
            msg.front_right < 56,
                       ])

        
        
        self.publisher_.publish(Bool(data=obstacle_detected))
        



def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
