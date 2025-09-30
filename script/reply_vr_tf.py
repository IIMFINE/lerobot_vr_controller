#!/usr/bin/env python3

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class VRTFReplier(Node):
    def __init__(self):
        super().__init__("vr_tf_replier")

        # 声明参数
        self.declare_parameter("config_file", "")

        # 获取配置文件路径
        config_file = (
            self.get_parameter("config_file").get_parameter_value().string_value
        )

        if not config_file:
            self.get_logger().error(
                "No config file specified. Use --ros-args -p config_file:=path/to/config.yaml"
            )
            return

        self.get_logger().info(f"Starting VR TF Replier with config: {config_file}")

        # 读取配置文件
        self.config = self._load_config(config_file)
        if not self.config:
            self.get_logger().error(f"Failed to load config file: {config_file}")
            return

        # 创建QoS配置
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        # 创建TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 创建订阅者字典
        self.subscribers = {}

        # 根据配置文件创建订阅者
        self._create_subscribers()

        self.get_logger().info("VR TF Replier initialized successfully")

    def _load_config(self, config_file: str) -> dict:
        """加载YAML配置文件"""
        try:
            with open(config_file, "r", encoding="utf-8") as file:
                config = yaml.safe_load(file)
                self.get_logger().info(f"Loaded config: {config}")
                return config
        except FileNotFoundError:
            self.get_logger().error(f"Config file not found: {config_file}")
            return {}
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML file: {e}")
            return {}
        except Exception as e:
            self.get_logger().error(f"Unexpected error loading config: {e}")
            return {}

    def _create_subscribers(self):
        """根据配置文件创建tf订阅者"""
        if "topic" not in self.config:
            self.get_logger().error("No topic configuration found")
            return

        for topic_name, transform_configs in self.config["topic"].items():
            if not isinstance(transform_configs, list):
                self.get_logger().error(
                    f"Invalid config format for topic: {topic_name}"
                )
                continue

            # 为每个topic创建订阅者
            subscriber = self.create_subscription(
                TFMessage,
                topic_name,
                lambda msg, configs=transform_configs: self._tf_callback(msg, configs),
                self.qos_profile,
            )

            self.subscribers[topic_name] = {
                "subscriber": subscriber,
                "configs": transform_configs,
            }

            self.get_logger().info(f"Created subscriber for topic: {topic_name}")
            self.get_logger().info(f"Transform configs: {transform_configs}")

    def _tf_callback(self, msg: TFMessage, transform_configs: list):
        """处理接收到的tf消息"""
        if not msg.transforms:
            return

        # 遍历消息中的每个transform
        for received_transform in msg.transforms:
            # 检查是否匹配配置中的任何transform
            for config in transform_configs:
                if not self._validate_config(config):
                    continue

                parent_link = config["parent_link"]
                child_link = config["child_link"]

                # 检查transform是否匹配配置
                if (
                    received_transform.header.frame_id == parent_link
                    and received_transform.child_frame_id == child_link
                ):

                    # 转发transform到tf2_ros
                    self._forward_transform(received_transform)

                    self.get_logger().debug(
                        f"Forwarded transform: {parent_link} -> {child_link}"
                    )

    def _validate_config(self, config: dict) -> bool:
        """验证配置格式"""
        if not isinstance(config, dict):
            self.get_logger().error(f"Invalid config type: {type(config)}")
            return False

        if "parent_link" not in config:
            self.get_logger().error("Missing parent_link in config")
            return False

        if "child_link" not in config:
            self.get_logger().error("Missing child_link in config")
            return False

        return True

    def _forward_transform(self, transform: TransformStamped):
        """将transform转发到tf2_ros"""
        try:
            # 创建新的transform消息
            forwarded_transform = TransformStamped()

            # 复制header信息
            forwarded_transform.header.stamp = self.get_clock().now().to_msg()
            forwarded_transform.header.frame_id = transform.header.frame_id
            forwarded_transform.child_frame_id = transform.child_frame_id

            # 复制transform数据
            forwarded_transform.transform.translation = transform.transform.translation
            forwarded_transform.transform.rotation = transform.transform.rotation

            # 广播transform
            self.tf_broadcaster.sendTransform(forwarded_transform)

        except Exception as e:
            self.get_logger().error(f"Error forwarding transform: {e}")


def main():
    """主函数"""
    # 初始化ROS2
    rclpy.init()

    try:
        # 创建节点
        node = VRTFReplier()

        # 运行节点
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # 清理资源
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
