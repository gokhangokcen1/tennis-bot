import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("tennis_bot_perception")
    config = os.path.join(share, "config", "ball_detector.yaml")

    return LaunchDescription([
        Node(
            package="tennis_bot_perception",
            executable="ball_detector_node",
            name="ball_detector_node",
            parameters=[config],
            output="screen",
        )
    ])
