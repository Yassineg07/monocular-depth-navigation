#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from PIL import Image
import numpy as np
from ament_index_python.packages import get_package_share_directory

class StaticMapPublisher(Node):
    def __init__(self):
        super().__init__('static_map_publisher')
        self.publisher = self.create_publisher(Marker, '/world_map_marker', 10)
        self.timer = self.create_timer(2.0, self.check_and_publish)
        self.marker = None
        self.last_sub_count = 0
        self.load_map()
        self.get_logger().info('Static map publisher started')
        
    def load_map(self):
        self.marker = Marker()
        self.marker.header.frame_id = "odom"
        self.marker.ns = "world_map"
        self.marker.id = 0
        self.marker.type = Marker.CUBE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.05
        self.marker.scale.y = 0.05
        self.marker.scale.z = 2.0
        self.marker.color.r = 0.1
        self.marker.color.g = 0.1
        self.marker.color.b = 0.1
        self.marker.color.a = 0.3 # Smoked glass appearance
        self.marker.lifetime.sec = 0
        
        # Use ament_index to find the installed package path
        package_share = get_package_share_directory('my_robot')
        map_path = os.path.join(package_share, 'maps', 'world_reference.png')
        img = Image.open(map_path)
        arr = np.array(img)
        
        resolution = 0.05
        origin_x = 0.0
        origin_y = -15.0  # Simple maze origin
        
        for y in range(0, arr.shape[0], 1):
            for x in range(0, arr.shape[1], 1):
                if arr[y, x] < 128:
                    p = Point()
                    p.x = float(origin_x + x * resolution)
                    p.y = float(origin_y + (arr.shape[0] - y) * resolution)
                    p.z = 1.0
                    self.marker.points.append(p)
        
        self.get_logger().info(f'Loaded map with {len(self.marker.points)} cubes')
        
    def check_and_publish(self):
        sub_count = self.publisher.get_subscription_count()
        if sub_count > 0 and sub_count != self.last_sub_count:
            self.marker.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.marker)
            self.get_logger().info('Published map (subscriber detected)')
        self.last_sub_count = sub_count

def main():
    rclpy.init()
    node = StaticMapPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
