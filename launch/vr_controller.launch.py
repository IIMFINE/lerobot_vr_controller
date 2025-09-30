#!/usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EqualsSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory("lerobot_vr_controller")

    # Path to URDF file and read it
    urdf_file_path = os.path.join(pkg_dir, "model", "so101_new_calib.urdf")
    with open(urdf_file_path, "r") as infp:
        robot_desc = infp.read()

    # Load remapping configuration based on arm_side
    def load_remapping_config(side):
        remapping_file = os.path.join(pkg_dir, "config", f"{side}_arm_remapping.yaml")

        if not os.path.exists(remapping_file):
            raise FileNotFoundError(f"Remapping file not found: {remapping_file}")

        with open(remapping_file, "r") as f:
            config = yaml.safe_load(f)

        topic_remapping = []
        if "topic" in config and isinstance(config["topic"], dict):
            for original, remapped in config["topic"].items():
                topic_remapping.append((original, remapped))

        node_name = config.get("node", f"{side}_lerobot_vr_controller")
        return topic_remapping, node_name

    # Declare launch arguments
    # Note: config_file is now automatically selected based on arm_side parameter
    # config_file_arg is kept for backward compatibility but not used directly

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

    arm_side_arg = DeclareLaunchArgument(
        "arm_side",
        default_value="right",
        description="Arm side (left, right, or both) for topic remapping",
    )

    # Function to launch nodes based on arm_side parameter
    def launch_nodes(context, *args, **kwargs):
        # Get arm_side value from launch context
        arm_side = LaunchConfiguration("arm_side").perform(context)

        # Helper function to create VR controller node
        def create_vr_controller_node(side):
            remapping_config, node_name = load_remapping_config(side)
            vr_to_arm_config_file = PathJoinSubstitution(
                [
                    FindPackageShare("lerobot_vr_controller"),
                    "config",
                    f"{side}_vr_to_arm.yaml",
                ]
            ).perform(context)

            return Node(
                package="lerobot_vr_controller",
                executable="lerobot_vr_controller_node",
                name=node_name,
                output="screen",
                parameters=[
                    {
                        "config_file": vr_to_arm_config_file,
                        "urdf_file": LaunchConfiguration("urdf_file"),
                        "joint_motor_config_file": LaunchConfiguration(
                            "joint_motor_config_file"
                        ),
                        "motor_calibration_file": LaunchConfiguration(
                            "motor_calibration_file"
                        ),
                    }
                ],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                remappings=remapping_config,
            )

        # Helper function to create robot state publisher node
        def create_robot_state_publisher_node(side):
            remapping_config, _ = load_remapping_config(side)
            node_name = f"{side}_robot_state_publisher"

            return Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name=node_name,
                output="screen",
                parameters=[{"robot_description": robot_desc}],
                remappings=remapping_config,
            )

        # Helper function to create VR TF replier node
        def create_vr_tf_replier_node(side):
            remapping_config, _ = load_remapping_config(side)
            config_file = PathJoinSubstitution(
                [
                    FindPackageShare("lerobot_vr_controller"),
                    "config",
                    f"{side}_reply_vr_tf.yaml",
                ]
            ).perform(context)

            node_name = f"{side}_vr_tf_replier"

            return Node(
                package="lerobot_vr_controller",
                executable="reply_vr_tf.py",
                name=node_name,
                output="screen",
                parameters=[{"config_file": config_file}],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                remappings=remapping_config,
            )

        # Create nodes list based on arm_side
        nodes_to_launch = []

        # Determine which nodes to create based on arm_side
        if arm_side == "left":
            nodes_to_launch.append(create_vr_controller_node("left"))
            nodes_to_launch.append(create_robot_state_publisher_node("left"))
            nodes_to_launch.append(create_vr_tf_replier_node("left"))
        elif arm_side == "right":
            nodes_to_launch.append(create_vr_controller_node("right"))
            nodes_to_launch.append(create_robot_state_publisher_node("right"))
            nodes_to_launch.append(create_vr_tf_replier_node("right"))
        elif arm_side == "both":
            nodes_to_launch.append(create_vr_controller_node("left"))
            nodes_to_launch.append(create_vr_controller_node("right"))
            nodes_to_launch.append(create_robot_state_publisher_node("left"))
            nodes_to_launch.append(create_robot_state_publisher_node("right"))
            nodes_to_launch.append(create_vr_tf_replier_node("left"))
            nodes_to_launch.append(create_vr_tf_replier_node("right"))

        return nodes_to_launch

    return LaunchDescription(
        [
            urdf_file_arg,
            log_level_arg,
            joint_motor_config_file_arg,
            motor_calibration_file_arg,
            arm_side_arg,
            OpaqueFunction(function=launch_nodes),
        ]
    )
