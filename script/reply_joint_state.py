#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateForwarder(Node):
    def __init__(self):
        super().__init__("joint_state_forwarder")

        # Publisher for /right/joint_states
        self.joint_state_publisher = self.create_publisher(
            JointState, "/joint_states", 10
        )

        # Publisher for /left/joint_states
        self.left_joint_state_publisher = self.create_publisher(
            JointState, "/left/joint_states", 10
        )

        # Subscriber for /right/vr_controller/joint_cmd
        self.vr_joint_state_subscriber = self.create_subscription(
            JointState,
            "/right/vr_controller/joint_cmd",
            lambda msg: self.vr_joint_state_callback(msg, "right"),
            10,
        )

        # Subscriber for /left/vr_controller/joint_cmd
        self.left_vr_joint_state_subscriber = self.create_subscription(
            JointState,
            "/left/vr_controller/joint_cmd",
            lambda msg: self.vr_joint_state_callback(msg, "left"),
            10,
        )

        # Timer to continuously publish the last recorded joint state for both arms
        self.publish_timer = self.create_timer(
            0.001, self.publish_last_joint_state
        )  # 20Hz

        # Store the last received joint state message for right and left
        self.last_joint_state_msg = None
        self.last_left_joint_state_msg = None

        # Statistics for right
        self.forwarded_count = 0
        self.zero_published_count = 0

        # Statistics for left
        self.left_forwarded_count = 0
        self.left_zero_published_count = 0

        # Default joint names and zero positions (assume same for both arms)
        self.default_joint_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ]

        # Lock for thread safety
        self.lock = threading.Lock()

        self.get_logger().info("Joint state forwarder node started")
        self.get_logger().info(
            "Subscribing to /right/vr_controller/joint_cmd and /left/vr_controller/joint_cmd"
        )
        self.get_logger().info(
            "Publishing to /right/joint_states and /left/joint_states"
        )

    def vr_joint_state_callback(self, msg, arm_side):
        """Callback for both /right/vr_controller/joint_cmd and /left/vr_controller/joint_cmd"""
        with self.lock:
            if arm_side == "right":
                self.last_joint_state_msg = msg
                self.forwarded_count += 1
                count = self.forwarded_count
            else:  # left
                self.last_left_joint_state_msg = msg
                self.left_forwarded_count += 1
                count = self.left_forwarded_count

        joint_positions = [
            f"{name}: {pos:.3f}" for name, pos in zip(msg.name, msg.position)
        ]
        arm_label = arm_side.upper()
        self.get_logger().info(f"[{arm_label}] Recorded #{count}: {joint_positions}")
        self.get_logger().debug(
            f"[{arm_label}] Recorded joint state with {len(msg.name)} joints"
        )

    def publish_last_joint_state(self):
        """Continuously publish the last recorded joint state or zero states for both arms if none received"""
        with self.lock:
            # Right arm
            if self.last_joint_state_msg is not None:
                self.joint_state_publisher.publish(self.last_joint_state_msg)
            else:
                self.publish_zero_joint_states()

            # Left arm
            if self.last_left_joint_state_msg is not None:
                self.left_joint_state_publisher.publish(self.last_left_joint_state_msg)
            else:
                self.publish_zero_left_joint_states()

    def publish_zero_joint_states(self):
        """Publish right joint states with all joints set to zero"""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = "world"
        joint_state_msg.name = self.default_joint_names
        joint_state_msg.position = [0.0] * len(self.default_joint_names)
        joint_state_msg.velocity = [0.0] * len(self.default_joint_names)
        joint_state_msg.effort = [0.0] * len(self.default_joint_names)
        self.joint_state_publisher.publish(joint_state_msg)
        self.zero_published_count += 1
        self.get_logger().debug(
            f"[RIGHT] Published zero joint states #{self.zero_published_count}"
        )
        self.get_logger().debug(
            "[RIGHT] Published zero joint states due to no initial message"
        )

    def publish_zero_left_joint_states(self):
        """Publish left joint states with all joints set to zero"""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = "world"
        joint_state_msg.name = self.default_joint_names
        joint_state_msg.position = [0.0] * len(self.default_joint_names)
        joint_state_msg.velocity = [0.0] * len(self.default_joint_names)
        joint_state_msg.effort = [0.0] * len(self.default_joint_names)
        self.left_joint_state_publisher.publish(joint_state_msg)
        self.left_zero_published_count += 1
        self.get_logger().debug(
            f"[LEFT] Published zero joint states #{self.left_zero_published_count}"
        )
        self.get_logger().debug(
            "[LEFT] Published zero joint states due to no initial message"
        )


def main(args=None):
    rclpy.init(args=args)

    try:
        node = JointStateForwarder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
