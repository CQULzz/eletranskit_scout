"""Start the bundled real-car localization, LiDAR, TF, mapping, and RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _static_transform_node(name, parent, child, values, condition):
    x, y, z, roll, pitch, yaw = values
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        output="screen",
        arguments=[
            "--x", x,
            "--y", y,
            "--z", z,
            "--roll", roll,
            "--pitch", pitch,
            "--yaw", yaw,
            "--frame-id", parent,
            "--child-frame-id", child,
        ],
        condition=condition,
    )


def generate_launch_description():
    start_fixposition = LaunchConfiguration("start_fixposition")
    start_rslidar = LaunchConfiguration("start_rslidar")
    publish_static_tf = LaunchConfiguration("publish_static_tf")
    start_mapping = LaunchConfiguration("start_mapping")
    start_rviz = LaunchConfiguration("start_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    fixposition_config = LaunchConfiguration("fixposition_config")
    rslidar_config = LaunchConfiguration("rslidar_config")
    mapping_core_config = LaunchConfiguration("mapping_core_config")
    mapping_config = LaunchConfiguration("mapping_config")
    rviz_config = LaunchConfiguration("rviz_config")
    mapping_start_delay = LaunchConfiguration("mapping_start_delay")

    poi_to_lidar = (
        LaunchConfiguration("poi_to_lidar_x"),
        LaunchConfiguration("poi_to_lidar_y"),
        LaunchConfiguration("poi_to_lidar_z"),
        LaunchConfiguration("poi_to_lidar_roll"),
        LaunchConfiguration("poi_to_lidar_pitch"),
        LaunchConfiguration("poi_to_lidar_yaw"),
    )
    identity = ("0", "0", "0", "0", "0", "0")

    fixposition_node = Node(
        package="fixposition_driver_ros2",
        executable="fixposition_driver_ros2_exec",
        name="fixposition_driver_ros2",
        output="screen",
        # Keep nav2_mode disabled so the driver does not also publish map/odom.
        parameters=[fixposition_config, {"nav2_mode": False}],
        respawn=True,
        respawn_delay=5.0,
        condition=IfCondition(start_fixposition),
    )

    rslidar_node = Node(
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        namespace="rslidar_sdk",
        output="screen",
        parameters=[{"config_path": rslidar_config}],
        respawn=True,
        respawn_delay=3.0,
        condition=IfCondition(start_rslidar),
    )

    elevation_mapping_node = Node(
        package="elevation_mapping_cupy",
        executable="elevation_mapping_node.py",
        name="elevation_mapping_node",
        output="screen",
        parameters=[
            mapping_core_config,
            mapping_config,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(start_mapping),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(start_rviz),
    )

    declarations = [
        DeclareLaunchArgument(
            "start_fixposition",
            default_value="true",
            description="Start the Fixposition ROS 2 driver.",
        ),
        DeclareLaunchArgument(
            "start_rslidar",
            default_value="true",
            description="Start the RoboSense LiDAR driver without its demo RViz.",
        ),
        DeclareLaunchArgument(
            "publish_static_tf",
            default_value="true",
            description="Publish the temporary FP/map/odom/LiDAR TF bridge.",
        ),
        DeclareLaunchArgument(
            "start_mapping",
            default_value="true",
            description="Start elevation_mapping_cupy.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz with the bundled real-car display config.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock instead of wall time.",
        ),
        DeclareLaunchArgument(
            "mapping_start_delay",
            default_value="5.0",
            description="Seconds to wait before starting elevation mapping.",
        ),
        DeclareLaunchArgument(
            "fixposition_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("ele_scout_kit"),
                "config",
                "fixposition.yaml",
            ]),
            description="Absolute path to the bundled Fixposition parameter YAML.",
        ),
        DeclareLaunchArgument(
            "rslidar_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("ele_scout_kit"),
                "config",
                "rslidar.yaml",
            ]),
            description="Absolute path to the RoboSense SDK YAML.",
        ),
        DeclareLaunchArgument(
            "mapping_core_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("elevation_mapping_cupy"),
                "config",
                "core",
                "core_param.yaml",
            ]),
            description="Absolute path to elevation_mapping_cupy core parameters.",
        ),
        DeclareLaunchArgument(
            "mapping_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("ele_scout_kit"),
                "config",
                "elevation_mapping_real_car.yaml",
            ]),
            description="Absolute path to the real-car mapping parameters.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("ele_scout_kit"),
                "config",
                "real_car.rviz",
            ]),
            description="RViz configuration for live elevation-map viewing.",
        ),
        DeclareLaunchArgument("poi_to_lidar_x", default_value="0.0"),
        DeclareLaunchArgument("poi_to_lidar_y", default_value="0.0"),
        DeclareLaunchArgument("poi_to_lidar_z", default_value="0.0"),
        DeclareLaunchArgument("poi_to_lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("poi_to_lidar_pitch", default_value="0.0"),
        DeclareLaunchArgument("poi_to_lidar_yaw", default_value="0.0"),
    ]

    tf_condition = IfCondition(publish_static_tf)
    tf_nodes = [
        _static_transform_node(
            "fp_poi_to_rslidar_static_tf",
            "FP_POI",
            "rslidar",
            poi_to_lidar,
            tf_condition,
        ),
        _static_transform_node(
            "fp_enu0_to_map_static_tf",
            "FP_ENU0",
            "map",
            identity,
            tf_condition,
        ),
        _static_transform_node(
            "map_to_odom_static_tf",
            "map",
            "odom",
            identity,
            tf_condition,
        ),
    ]

    return LaunchDescription(
        declarations
        + [
            LogInfo(
                msg=(
                    "Using temporary static TFs. Replace FP_POI -> rslidar "
                    "zeros with calibrated extrinsics before production use."
                ),
                condition=tf_condition,
            ),
            fixposition_node,
            rslidar_node,
        ]
        + tf_nodes
        + [
            rviz_node,
            TimerAction(
                period=mapping_start_delay,
                actions=[elevation_mapping_node],
                condition=IfCondition(start_mapping),
            )
        ]
    )
