"""Start the bundled real-car localization, LiDAR, TF, mapping, HT, and RViz."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    start_traversability = LaunchConfiguration("start_traversability")
    start_rviz = LaunchConfiguration("start_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    static_pose_fallback = LaunchConfiguration("static_pose_fallback")

    fixposition_config = LaunchConfiguration("fixposition_config")
    rslidar_config = LaunchConfiguration("rslidar_config")
    mapping_core_config = LaunchConfiguration("mapping_core_config")
    mapping_config = LaunchConfiguration("mapping_config")
    global_map_storage_path = LaunchConfiguration("global_map_storage_path")
    traversability_config = LaunchConfiguration("traversability_config")
    rviz_config = LaunchConfiguration("rviz_config")
    robot_description_file = LaunchConfiguration("robot_description_file")
    mapping_start_delay = LaunchConfiguration("mapping_start_delay")
    traversability_start_delay = LaunchConfiguration("traversability_start_delay")

    poi_to_base_footprint = (
        LaunchConfiguration("poi_to_base_footprint_x"),
        LaunchConfiguration("poi_to_base_footprint_y"),
        LaunchConfiguration("poi_to_base_footprint_z"),
        LaunchConfiguration("poi_to_base_footprint_roll"),
        LaunchConfiguration("poi_to_base_footprint_pitch"),
        LaunchConfiguration("poi_to_base_footprint_yaw"),
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
        # The static-pose fallback owns FP_POI's parent transform.  Never run
        # the live Fixposition broadcaster at the same time, otherwise FP_POI
        # can acquire two parents when localization becomes valid.
        condition=IfCondition(
            AndSubstitution(start_fixposition, NotSubstitution(static_pose_fallback))
        ),
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

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="scout_robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command([
                    FindExecutable(name="xacro"),
                    " ",
                    robot_description_file,
                ]),
                value_type=str,
            ),
            "use_sim_time": use_sim_time,
        }],
        condition=IfCondition(publish_static_tf),
    )

    elevation_mapping_node = Node(
        package="elevation_mapping_cupy",
        executable="elevation_mapping_node.py",
        name="elevation_mapping_node",
        output="screen",
        parameters=[
            mapping_core_config,
            mapping_config,
            {
                "use_sim_time": use_sim_time,
                "global_map_storage_path": global_map_storage_path,
            },
        ],
        condition=IfCondition(start_mapping),
    )

    traversability_node = Node(
        package="ht_traversability_ros2",
        executable="ht_traversability_node",
        name="ht_traversability_node",
        output="screen",
        parameters=[traversability_config, {"use_sim_time": use_sim_time}],
        condition=IfCondition(start_traversability),
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
            "start_traversability",
            default_value="true",
            description="Infer eight-direction traversability from the global-map local crop.",
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
            "static_pose_fallback",
            default_value="false",
            description=(
                "Disable Fixposition and publish an identity odom -> FP_POI TF. "
                "For stationary sensor tests only; vehicle motion is not represented."
            ),
        ),
        DeclareLaunchArgument(
            "mapping_start_delay",
            default_value="5.0",
            description="Seconds to wait before starting elevation mapping.",
        ),
        DeclareLaunchArgument(
            "traversability_start_delay",
            default_value="7.0",
            description="Seconds to wait before starting global-local HT inference.",
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
            "global_map_storage_path",
            default_value=os.path.join(
                os.path.expanduser("~"),
                ".ros",
                "elevation_mapping_cupy",
                "global_tiles.sqlite3",
            ),
            description="SQLite path for the persistent sparse global elevation map.",
        ),
        DeclareLaunchArgument(
            "traversability_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("ele_scout_kit"),
                "config",
                "ht_traversability_real_car.yaml",
            ]),
            description="HT parameters for the persistent-global local crop.",
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
        DeclareLaunchArgument(
            "robot_description_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ele_scout_kit"),
                "urdf",
                "scout_sensor_rack.urdf.xacro",
            ]),
            description="Combined Scout and calibrated sensor-rack xacro.",
        ),
        # The URDF gives base_footprint -> FP_POI as
        # (0.2005158179045505, 0, 0.65899), all rotations zero.  The launch
        # bridge needs its inverse so Fixposition remains the localization
        # parent and the URDF remains a single-parent TF tree.
        DeclareLaunchArgument(
            "poi_to_base_footprint_x", default_value="-0.2005158179045505"
        ),
        DeclareLaunchArgument("poi_to_base_footprint_y", default_value="0.0"),
        DeclareLaunchArgument("poi_to_base_footprint_z", default_value="-0.65899"),
        DeclareLaunchArgument("poi_to_base_footprint_roll", default_value="0.0"),
        DeclareLaunchArgument("poi_to_base_footprint_pitch", default_value="0.0"),
        DeclareLaunchArgument("poi_to_base_footprint_yaw", default_value="0.0"),
    ]

    tf_condition = IfCondition(publish_static_tf)
    static_pose_fallback_condition = IfCondition(
        AndSubstitution(publish_static_tf, static_pose_fallback)
    )
    tf_nodes = [
        _static_transform_node(
            "fp_poi_to_base_footprint_static_tf",
            "FP_POI",
            "base_footprint",
            poi_to_base_footprint,
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
        _static_transform_node(
            "odom_to_fp_poi_static_fallback_tf",
            "odom",
            "FP_POI",
            identity,
            static_pose_fallback_condition,
        ),
    ]

    return LaunchDescription(
        declarations
        + [
            LogInfo(
                msg=(
                    "Publishing the calibrated Scout URDF TF tree. Assumption: "
                    "FP_POI is co-located with the URDF fixposition_link; the "
                    "derived FP_POI -> rslidar translation is "
                    "(-0.0280158179, 0, 0.126) m with zero RPY."
                ),
                condition=tf_condition,
            ),
            LogInfo(
                msg=(
                    "STATIC POSE FALLBACK ACTIVE: Fixposition is disabled and "
                    "odom -> FP_POI is fixed at identity. Do not move the vehicle."
                ),
                condition=IfCondition(static_pose_fallback),
            ),
            fixposition_node,
            rslidar_node,
            robot_state_publisher_node,
        ]
        + tf_nodes
        + [
            rviz_node,
            TimerAction(
                period=mapping_start_delay,
                actions=[elevation_mapping_node],
                condition=IfCondition(start_mapping),
            ),
            TimerAction(
                period=traversability_start_delay,
                actions=[traversability_node],
                condition=IfCondition(start_traversability),
            ),
        ]
    )
