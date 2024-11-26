import rclpy
from rclpy.node import Node
from interfaces.msg import Ultrasonic
from std_msgs.msg import Bool

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        
        # Publisher pour envoyer si un obstacle est détecté ou non
        self.publisher_ = self.create_publisher(Bool, 'obstacle_detected', 10)
        
        # Subscription pour recevoir les données du capteur à ultrasons
        self.subscription = self.create_subscription(
            Ultrasonic,
            'us_data',  # Topic de l'ultrason
            self.listener_callback,
            10
        )
        self.subscription  # Empêche l'avertissement d'objet inutilisé

    def listener_callback(self, msg):
        # Logique de détection d'obstacle
        # Si une des distances est inférieure à 30 cm, un obstacle est détecté
        obstacle_detected = any([
            msg.front_left < 30,
            msg.front_center < 30,
            msg.front_right < 30,
            msg.rear_left < 30,
            msg.rear_center < 30,
            msg.rear_right < 30,
        ])
        
        # Publier le résultat (True pour obstacle, False sinon)
        self.publisher_.publish(Bool(data=obstacle_detected))
        
        # Log pour afficher l'état de détection
        self.get_logger().info(f'Obstacle detected: {obstacle_detected}')


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
