#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import time


class JointStateForwarder(Node):
    def __init__(self):
        super().__init__("joint_state_forwarder")

        # Publisher for /joint_states
        self.joint_state_publisher = self.create_publisher(
            JointState, "/joint_states", 10
        )

        # Subscriber for /vr_controller/joint_cmd
        self.vr_joint_state_subscriber = self.create_subscription(
            JointState, "/vr_controller/joint_cmd", self.vr_joint_state_callback, 10
        )

        # Timer to continuously publish the last recorded joint state
        self.publish_timer = self.create_timer(
            0.001, self.publish_last_joint_state
        )  # 20Hz

        # Store the last received joint state message
        self.last_joint_state_msg = None

        # Statistics
        self.forwarded_count = 0
        self.zero_published_count = 0

        # Default joint names and zero positions
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
        self.get_logger().info("Subscribing to /vr_controller/joint_cmd")
        self.get_logger().info("Publishing to /joint_states")

    def vr_joint_state_callback(self, msg):
        """Callback for /vr_controller/joint_cmd"""
        with self.lock:
            # Store the received message
            self.last_joint_state_msg = msg

        # Update statistics
        self.forwarded_count += 1

        # Log the joint state information
        joint_positions = [
            f"{name}: {pos:.3f}" for name, pos in zip(msg.name, msg.position)
        ]
        self.get_logger().info(f"Recorded #{self.forwarded_count}: {joint_positions}")
        self.get_logger().debug(f"Recorded joint state with {len(msg.name)} joints")

    def publish_last_joint_state(self):
        """Continuously publish the last recorded joint state or zero states if none received"""
        with self.lock:
            if self.last_joint_state_msg is not None:
                # Publish the last recorded joint state
                self.joint_state_publisher.publish(self.last_joint_state_msg)
                self.get_logger().debug("Published last recorded joint state")
            else:
                # No message received yet, publish zero states
                self.publish_zero_joint_states()

    def publish_zero_joint_states(self):
        """Publish joint states with all joints set to zero"""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = "world"

        # Set joint names and zero positions
        joint_state_msg.name = self.default_joint_names
        joint_state_msg.position = [0.0] * len(self.default_joint_names)
        joint_state_msg.velocity = [0.0] * len(self.default_joint_names)
        joint_state_msg.effort = [0.0] * len(self.default_joint_names)

        # Publish the zero joint state
        self.joint_state_publisher.publish(joint_state_msg)

        # Update statistics
        self.zero_published_count += 1

        self.get_logger().debug(
            f"Published zero joint states #{self.zero_published_count}"
        )
        self.get_logger().debug("Published zero joint states due to no initial message")


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
