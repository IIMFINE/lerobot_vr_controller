#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory("lerobot_vr_controller")

    # Path to URDF file
    urdf_file = os.path.join(pkg_dir, "model", "so101_new_calib.urdf")

    # Read URDF file
    with open(urdf_file, "r") as infp:
        robot_desc = infp.read()

    # Declare launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    use_joint_gui = LaunchConfiguration("use_joint_gui", default="false")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "use_joint_gui",
                default_value="false",
                description="Launch joint state publisher GUI for manual control",
            ),
            # Robot State Publisher
            # Node(
            #     package="robot_state_publisher",
            #     executable="robot_state_publisher",
            #     name="robot_state_publisher",
            #     output="screen",
            #     parameters=[
            #         {"robot_description": robot_desc, "use_sim_time": use_sim_time}
            #     ],
            # ),
            # Joint State Publisher GUI (optional - only when not using VR control)
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                condition=IfCondition(use_joint_gui),
            ),
            # RViz2
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(pkg_dir, "rviz", "display_robot.rviz")],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
