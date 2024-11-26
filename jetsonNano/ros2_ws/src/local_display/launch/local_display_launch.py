from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    image_subscriber = Node(
        name='image_subscriber',
        package='local_display',
        executable='images_subscriber_node',
        emulate_tty=True
    )

    ld.add_action(image_subscriber)

    return ld