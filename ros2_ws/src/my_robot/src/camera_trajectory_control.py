#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool
import math
import sys
import select
import termios
import tty

class CameraTrajectoryControl(Node):
    def __init__(self):
        super().__init__('camera_trajectory_control')
        
        # Parameters
        self.declare_parameter('amplitude', 0.349)       # 20 degrees
        self.declare_parameter('frequency', 0.4)         # Hz
        
        self.amplitude = self.get_parameter('amplitude').value
        self.frequency = self.get_parameter('frequency').value
        
        # Publishers
        self.traj_pub = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)
        self.motion_active_pub = self.create_publisher(Bool, '/vision/camera_motion_active', 10)
        
        # State
        self.sweep_start_time = self.get_clock().now()
        self.running = True
        
        # Control timer
        self.timer = self.create_timer(0.1, self.control_callback)
        
        # Terminal settings for key detection
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        self.get_logger().info('Camera trajectory control started - Press Q to quit')
        self.get_logger().info('Camera sweeping continuously...')
    
    def check_keyboard(self):
        """Check if Q key is pressed"""
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1).lower()
            if key == 'q':
                self.get_logger().info('Q pressed - shutting down...')
                self.publish_camera_angle(0.0)  # Reset camera to forward
                self.running = False
                return True
        return False
    
    def publish_camera_angle(self, angle):
        """Publish camera angle with smooth interpolation"""
        traj = JointTrajectory()
        traj.header.stamp.sec = 0
        traj.header.stamp.nanosec = 0
        traj.header.frame_id = 'world'
        traj.joint_names = ['camera_pan_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [angle]
        point.velocities = [0.0]
        point.time_from_start = Duration(sec=0, nanosec=200000000)  # 0.2s smooth interpolation
        
        traj.points = [point]
        self.traj_pub.publish(traj)
    
    def control_callback(self):
        """Continuous sweeping motion"""
        # Check for Q key press
        if self.check_keyboard():
            raise SystemExit
        
        # Continuous sinusoidal motion
        sweep_elapsed = (self.get_clock().now() - self.sweep_start_time).nanoseconds / 1e9
        target_angle = self.amplitude * math.sin(2 * math.pi * self.frequency * sweep_elapsed)
        self.publish_camera_angle(target_angle)
        
        # Publish motion_active flag
        motion_msg = Bool()
        motion_msg.data = True
        self.motion_active_pub.publish(motion_msg)
    
    def destroy_node(self):
        """Cleanup terminal settings"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraTrajectoryControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()