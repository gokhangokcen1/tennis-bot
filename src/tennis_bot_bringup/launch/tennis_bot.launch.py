import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")
    desc = get_package_share_directory("tennis_bot_description")
    perc = get_package_share_directory("tennis_bot_perception")
    nav = get_package_share_directory("tennis_bot_navigation")
    bringup = get_package_share_directory("tennis_bot_bringup")


    rviz_config = os.path.join(bringup, "rviz", "tennis_bot.rviz")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc, "launch", "simulation.launch.py")
        )
    )
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perc, "launch", "perception.launch.py")
        )
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav, "launch", "navigation.launch.py")
        )
    )

    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        simulation,
        perception,
        navigation,
        rviz_node,
    ])
