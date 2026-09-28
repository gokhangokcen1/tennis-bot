import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory("nav2_bringup")
    own_share = get_package_share_directory("tennis_bot_navigation")

    map_file = os.path.join(own_share, "maps", "tennis_court.yaml")
    params_file = os.path.join(own_share, "config", "nav2_params.yaml")
    planner_config = os.path.join(own_share, "config", "planner.yaml")

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "use_sim_time": "true",
            "params_file": params_file,
            "autostart": "true",
            "slam": "False",
            "use_composition": "False",
        }.items(),
    )

    relay = Node(
        package="tennis_bot_navigation",
        executable="cmd_vel_relay_node",
        name="cmd_vel_relay_node",
        output="screen",
    )

    planner = Node(
        package="tennis_bot_navigation",
        executable="collection_planner_node",
        name="collection_planner_node",
        parameters=[planner_config],
        output="screen",
    )

    return LaunchDescription([nav2, relay, planner])
