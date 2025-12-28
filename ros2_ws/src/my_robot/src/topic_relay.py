#!/usr/bin/env python3
"""
Topic Relay Node
Relays sensor topics to costmap-namespaced topics for Nav2 obstacle detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class TopicRelay(Node):
    def __init__(self):
        super().__init__('topic_relay')
        
        # QoS for sensors (best effort)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Scan relay: /scan -> /local_costmap/scan and /global_costmap/scan
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.scan_pub_local = self.create_publisher(
            LaserScan, '/local_costmap/scan', sensor_qos)
        self.scan_pub_global = self.create_publisher(
            LaserScan, '/global_costmap/scan', sensor_qos)
        
        # Pointcloud relay: /camera/pointcloud -> costmap topics
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/camera/pointcloud', self.cloud_callback, sensor_qos)
        self.cloud_pub_local = self.create_publisher(
            PointCloud2, '/local_costmap/camera/pointcloud', sensor_qos)
        self.cloud_pub_global = self.create_publisher(
            PointCloud2, '/global_costmap/camera/pointcloud', sensor_qos)
        
        self.get_logger().info('Topic relay node started')
        self.get_logger().info('  /scan -> /local_costmap/scan, /global_costmap/scan')
        self.get_logger().info('  /camera/pointcloud -> /local_costmap/camera/pointcloud, /global_costmap/camera/pointcloud')
    
    def scan_callback(self, msg):
        self.scan_pub_local.publish(msg)
        self.scan_pub_global.publish(msg)
    
    def cloud_callback(self, msg):
        self.cloud_pub_local.publish(msg)
        self.cloud_pub_global.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TopicRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
