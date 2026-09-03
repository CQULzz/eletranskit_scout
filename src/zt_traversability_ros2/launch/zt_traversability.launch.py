from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    package_share = get_package_share_directory("zt_traversability_ros2")
    default_config = os.path.join(package_share, "config", "zt_real_car.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="ZT ROS 2 parameter YAML",
            ),
            Node(
                package="zt_traversability_ros2",
                executable="zt_traversability_node",
                name="zt_traversability_node",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
