#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory("lerobot_vr_controller")

    # Path to URDF file and read it
    urdf_file_path = os.path.join(pkg_dir, "model", "so101_new_calib.urdf")
    with open(urdf_file_path, "r") as infp:
        robot_desc = infp.read()

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

    urdf_file_arg = DeclareLaunchArgument(
        "urdf_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("lerobot_vr_controller"),
                "model",
                "so101_new_calib.urdf",
            ]
        ),
        description="Path to the URDF file",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level for the node (debug, info, warn, error, fatal)",
    )

    joint_motor_config_file_arg = DeclareLaunchArgument(
        "joint_motor_config_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("lerobot_vr_controller"),
                "config",
                "motor",
                "so101_follower",
                "joint_motor_config.yaml",
            ]
        ),
        description="Path to the joint motor configuration file",
    )

    motor_calibration_file_arg = DeclareLaunchArgument(
        "motor_calibration_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("lerobot_vr_controller"),
                "config",
                "motor",
                "so101_follower",
                "motor_calibration.json",
            ]
        ),
        description="Path to the motor calibration file",
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
                "urdf_file": LaunchConfiguration("urdf_file"),
                "joint_motor_config_file": LaunchConfiguration("joint_motor_config_file"),
                "motor_calibration_file": LaunchConfiguration("motor_calibration_file"),
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        # respawn= true,
        # respawn_delay=2.0,
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
    )

    return LaunchDescription(
        [
            config_file_arg,
            urdf_file_arg,
            log_level_arg,
            joint_motor_config_file_arg,
            motor_calibration_file_arg,
            vr_tf_receiver_node,
            robot_state_publisher_node,
        ]
    )
