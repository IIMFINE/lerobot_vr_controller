#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("lerobot_vr_controller"),
                "config",
                "vr_to_arm.yaml",
            ]
        ),
        description="Path to the VR to ARM configuration YAML file",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level for the node (debug, info, warn, error, fatal)",
    )

    # VR TF Receiver Node
    vr_tf_receiver_node = Node(
        package="lerobot_vr_controller",
        executable="lerobot_vr_controller_node",
        name="lerobot_vr_controller_node",
        output="screen",
        parameters=[
            {
                "config_file": LaunchConfiguration("config_file"),
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        # respawn= true,
        # respawn_delay=2.0,
    )

    return LaunchDescription(
        [
            config_file_arg,
            log_level_arg,
            vr_tf_receiver_node,
        ]
    )
