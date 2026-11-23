"""Unified real-robot bringup for Mid-360 FAST-LIO and KISS-SAM."""

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DIAGNOSTIC_TOPICS = [
    "/livox/lidar",
    "/livox/imu",
    "/odometry",
    "/cloud_registered",
    "/lidar/stream_status",
    "/localization/health",
    "/recovery/state",
    "/system_mode/state",
    "/frontend/map_updates_allowed",
    "/rosout",
]


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("1", "true", "yes", "on")


def _validate_and_include_driver(context):
    map_directory = LaunchConfiguration("map_directory").perform(context)
    if _as_bool(context, "start_relocalizer"):
        product = Path(map_directory)
        required = ("metadata.yaml", "timestamps.txt", "odometry_tum.txt", "keyframes")
        missing = [name for name in required if not (product / name).exists()]
        if not map_directory or missing:
            raise RuntimeError(
                "start_relocalizer=true requires a complete map product; missing: "
                + ", ".join(missing or ["map_directory"])
            )

    if not _as_bool(context, "start_driver"):
        return []
    driver_package = LaunchConfiguration("driver_package").perform(context)
    driver_launch_file = LaunchConfiguration("driver_launch_file").perform(context)
    driver_path = Path(get_package_share_directory(driver_package)) / "launch" / driver_launch_file
    if not driver_path.is_file():
        raise RuntimeError(f"Livox driver launch file does not exist: {driver_path}")
    return [IncludeLaunchDescription(AnyLaunchDescriptionSource(str(driver_path)))]


def generate_launch_description():
    kiss_share = get_package_share_directory("kiss_matcher_ros")
    fast_lio_share = get_package_share_directory("spark_fast_lio")
    diagnostic_default = "/tmp/slam_diagnostics_" + datetime.now().strftime("%Y%m%d_%H%M%S")

    fast_lio = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            str(Path(fast_lio_share) / "launch" / "mapping_mid360.launch.yaml")
        ),
        launch_arguments={
            "start_rviz": "false",
            "environment": LaunchConfiguration("environment"),
            "display_mode": LaunchConfiguration("display_mode"),
        }.items(),
    )

    mode_manager = Node(
        package="kiss_matcher_ros",
        executable="system_mode_manager",
        name="system_mode_manager",
        output="screen",
        parameters=[{"map_directory": LaunchConfiguration("map_directory")}],
        on_exit=Shutdown(reason="system mode manager exited"),
    )

    kiss_sam = Node(
        package="kiss_matcher_ros",
        executable="kiss_matcher_sam",
        namespace="km_sam",
        name="kiss_matcher_sam",
        output="screen",
        parameters=[
            str(Path(kiss_share) / "config" / "slam_config.yaml"),
            {
                "map_frame": LaunchConfiguration("map_frame"),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "mode_control_enabled": True,
                "incremental_map_directory": LaunchConfiguration("map_directory"),
                "result.output_directory": LaunchConfiguration("output_directory"),
                "result.seq_name": LaunchConfiguration("output_sequence"),
            },
        ],
        remappings=[("/odom", "/odometry"), ("/cloud", "/cloud_registered")],
        on_exit=Shutdown(reason="KISS-SAM exited"),
    )

    relocalizer = Node(
        condition=IfCondition(LaunchConfiguration("start_relocalizer")),
        package="kiss_matcher_ros",
        executable="kiss_matcher_relocalizer",
        name="kiss_matcher_relocalizer",
        output="screen",
        parameters=[
            str(Path(kiss_share) / "config" / "relocalization_config.yaml"),
            {"map_directory": LaunchConfiguration("map_directory")},
        ],
        remappings=[("odom", "/odometry"), ("cloud", "/cloud_registered")],
        on_exit=Shutdown(reason="relocalizer exited"),
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        package="rviz2",
        executable="rviz2",
        name="slam_rviz",
        output="screen",
        arguments=["-d", str(Path(kiss_share) / "rviz" / "kiss_matcher_sam.rviz")],
    )

    recorder = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("record_diagnostics")),
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            LaunchConfiguration("diagnostics_output"),
            *DIAGNOSTIC_TOPICS,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("environment", default_value="outdoor", choices=["indoor", "outdoor"]),
            DeclareLaunchArgument("display_mode", default_value="original", choices=["original", "height_limited"]),
            DeclareLaunchArgument("map_directory", default_value=""),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="livox_frame"),
            DeclareLaunchArgument("output_directory", default_value="maps"),
            DeclareLaunchArgument("output_sequence", default_value="robot_mapping"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("start_relocalizer", default_value="false"),
            DeclareLaunchArgument("record_diagnostics", default_value="true"),
            DeclareLaunchArgument("diagnostics_output", default_value=diagnostic_default),
            DeclareLaunchArgument("start_driver", default_value="false"),
            DeclareLaunchArgument("driver_package", default_value="livox_ros_driver2"),
            DeclareLaunchArgument("driver_launch_file", default_value="msg_MID360_launch.py"),
            OpaqueFunction(function=_validate_and_include_driver),
            fast_lio,
            mode_manager,
            kiss_sam,
            relocalizer,
            rviz,
            recorder,
        ]
    )
