#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare launch arguments
    urdf_file_arg = DeclareLaunchArgument(
        "urdf_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("lerobot_vr_controller"),
                "model",
                "so101_new_calib.urdf",
            ]
        ),
        description="Path to the URDF file for kinematics",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level for the node (debug, info, warn, error, fatal)",
    )

    # Robot State Publisher Node - to publish robot description
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["cat ", LaunchConfiguration("urdf_file")]), value_type=str
                ),
                "use_sim_time": False,
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # IK Test Node
    ik_test_node = Node(
        package="lerobot_vr_controller",
        executable="ik_test_node",
        name="ik_test_node",
        output="screen",
        parameters=[
            {
                "urdf_file": LaunchConfiguration("urdf_file"),
                "use_sim_time": False,
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(
        [
            # Launch arguments
            urdf_file_arg,
            log_level_arg,
            # Nodes
            robot_state_publisher_node,
            ik_test_node,
        ]
    )
