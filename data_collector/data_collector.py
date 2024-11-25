import rclpy
from rclpy.node import Node
import csv
from interfaces.msg import MotorsFeedback, MotorsOrder, JoystickOrder

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.data = []
        self.start_time = self.get_clock().now().nanoseconds

        self.subscription_motors_feedback = self.create_subscription(
            MotorsFeedback,
            '/motors_feedback',
            self.motors_feedback_callback,
            10
        )
        self.subscription_motors_order = self.create_subscription(
            MotorsOrder,
            '/motors_order',
            self.motors_order_callback,
            10
        )
        self.subscription_joystick_order = self.create_subscription(
            JoystickOrder,
            '/joystick_order',
            self.joystick_order_callback,
            10
        )

    def motors_feedback_callback(self, msg):
        self.record_data('motors_feedback', {
            'steering_angle': msg.steering_angle,
            'left_rear_speed': msg.left_rear_speed,
            'right_rear_speed': msg.right_rear_speed
        })

    def motors_order_callback(self, msg):
        self.record_data('motors_order', {
            'steering_pwm': msg.steering_pwm,
            'left_rear_pwm': msg.left_rear_pwm,
            'right_rear_pwm': msg.right_rear_pwm
        })

    def joystick_order_callback(self, msg):
        self.record_data('joystick_order', {
            'mode': msg.mode,
            'throttle': msg.throttle,
            'steer': msg.steer,
            'reverse': msg.reverse
        })

    def record_data(self, topic, data):
        timestamp = (self.get_clock().now().nanoseconds - self.start_time) / 1e9
        self.data.append({
            'timestamp': timestamp,
            'topic': topic,
            'data': data
        })

    def save_to_csv(self, filename='data_log.csv'):
        keys = ['timestamp', 'topic', 'data']
        with open(filename, 'w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self.data)

def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_to_csv()
    finally:
        rclpy.shutdown()
