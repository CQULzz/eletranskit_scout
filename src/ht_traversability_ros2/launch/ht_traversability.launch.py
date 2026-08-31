import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("ht_traversability_ros2")
    default_config = os.path.join(package_share, "config", "ht_real_car.yaml")
    default_rviz = os.path.join(package_share, "rviz", "ht_traversability.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="HT ROS 2 parameter YAML",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz with the HT display preset",
            ),
            Node(
                package="ht_traversability_ros2",
                executable="ht_traversability_node",
                name="ht_traversability_node",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="ht_traversability_rviz",
                arguments=["-d", default_rviz],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
