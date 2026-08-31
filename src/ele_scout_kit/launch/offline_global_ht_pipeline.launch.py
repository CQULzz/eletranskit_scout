"""Offline mapping -> persistent global -> local crop -> HT -> GIF pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    output_dir = LaunchConfiguration("output_dir")
    database_path = LaunchConfiguration("database_path")
    frame_stride = LaunchConfiguration("frame_stride")
    gif_frame_ms = LaunchConfiguration("gif_frame_ms")

    mapping_core = PathJoinSubstitution(
        [
            FindPackageShare("elevation_mapping_cupy"),
            "config",
            "core",
            "core_param.yaml",
        ]
    )
    mapping_vehicle = PathJoinSubstitution(
        [FindPackageShare("ele_scout_kit"), "config", "elevation_mapping_real_car.yaml"]
    )
    ht_config = PathJoinSubstitution(
        [
            FindPackageShare("ele_scout_kit"),
            "config",
            "ht_traversability_real_car.yaml",
        ]
    )

    mapping = Node(
        package="elevation_mapping_cupy",
        executable="elevation_mapping_node.py",
        name="elevation_mapping_node",
        output="screen",
        parameters=[
            mapping_core,
            mapping_vehicle,
            {
                "use_sim_time": False,
                "global_map_storage_path": database_path,
                "global_map_resume_from_disk": False,
                "global_map_publish_fps": 10.0,
                "global_map_local_publish_fps": 10.0,
                "global_map_flush_fps": 2.0,
            },
        ],
    )
    ht = Node(
        package="ht_traversability_ros2",
        executable="ht_traversability_node",
        name="ht_traversability_node",
        output="screen",
        parameters=[
            ht_config,
            {
                "use_sim_time": False,
                # Offline replay is acknowledgement-controlled, so do not
                # throttle distinct persistent-global revisions.
                "inference_fps": 0.0,
                # Keep all direction probabilities inside this isolated
                # comparison run. The recorder derives the conservative
                # binary result from the exact same inference frame.
                "aggregate_only": False,
            },
        ],
    )
    recorder = Node(
        package="ele_scout_kit",
        executable="global_ht_gif_recorder.py",
        name="global_ht_gif_recorder",
        output="screen",
        arguments=[
            output_dir,
            "--frame-stride",
            frame_stride,
            "--gif-frame-ms",
            gif_frame_ms,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value="/tmp/global_ht_pipeline",
                description="Directory for synchronized PNG frames, GIF, and summary.",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value="/tmp/global_ht_pipeline/global_tiles.sqlite3",
                description="SQLite persistent global elevation map.",
            ),
            DeclareLaunchArgument("frame_stride", default_value="1"),
            DeclareLaunchArgument("gif_frame_ms", default_value="100"),
            mapping,
            ht,
            recorder,
        ]
    )
