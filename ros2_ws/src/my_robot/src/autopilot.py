#!/usr/bin/env python3
"""
Autopilot node for autonomous robot movement.
Moves 5m forward, turns 180 degrees, repeats.
Uses smoothed velocity commands (Twist) and Odometry feedback.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import sys
import termios
import tty
import select
import threading

class AutopilotNode(Node):
    def __init__(self):
        super().__init__('autopilot')
        
        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Movement parameters
        self.TARGET_LINEAR_SPEED = 0.5      # m/s
        self.TARGET_ANGULAR_SPEED = 0.4     # rad/s (Slower for better accuracy)
        self.RAMP_FACTOR = 0.05             # Smoothing factor (0.0-1.0)

        # Goals and stop thresholds (user requested 2m and precise 180deg)
        self.GOAL_DISTANCE = 2.0            # meters (logical goal)
        self.GOAL_ANGLE = math.pi           # 180 degrees (logical goal)
        self.STOP_DISTANCE = 1.95           # meters - stop slightly early (2.0 - 0.05)
        self.STOP_ANGLE = math.radians(179) # stop approach at 179 degrees

        # Correction parameters (to counter sliding / overshoot)
        # Linear correction removed (not used) per user request
        self.ANGLE_TOLERANCE = math.radians(1.0)  # radians (~1 degree)
        self.CORRECT_ANGULAR_MAX = 0.12     # rad/s max correction
        self.CORRECT_KP_ANGULAR = 0.8       # P gain for angular correction
        
        # State machine
        self.state = 'IDLE'  # IDLE, MOVE, STOP_MOVE, TURN, STOP_TURN
        self.next_state = 'MOVE'
        
        # Current velocities (for ramping)
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Odometry tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.last_yaw = 0.0
        self.total_angle_turned = 0.0
        self.odom_received = False
        
        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Autopilot node started')
        self.get_logger().info('Pattern: 5m forward -> 180 deg turn -> repeat')
        # Print a persistent hint for the user about quitting
        print("[autopilot] Running — press 'q' to stop and quit")

        # Start the sequence
        self.state = 'INIT'

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        if not self.odom_received:
            self.odom_received = True

    def get_distance_traveled(self):
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        return math.sqrt(dx*dx + dy*dy)

    def normalize_angle(self, a: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def signed_angle_from_start(self) -> float:
        """Return signed angle turned since `start_yaw`, normalized to [-pi,pi]."""
        diff = self.current_yaw - self.start_yaw
        return self.normalize_angle(diff)

    def ramp_value(self, current, target):
        diff = target - current
        step = diff * self.RAMP_FACTOR
        if abs(diff) < 0.01:
            return target
        return current + step

    def control_loop(self):
        if not self.odom_received:
            return

        target_linear = 0.0
        target_angular = 0.0
        
        # State Machine
        if self.state == 'INIT':
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = self.current_yaw
            self.state = 'MOVE'
            self.get_logger().info('Starting MOVE phase...')

        elif self.state == 'MOVE':
            dist = self.get_distance_traveled()
            target_linear = self.TARGET_LINEAR_SPEED
            
            # Check if we reached stop threshold (stop slightly early)
            if dist >= self.STOP_DISTANCE:
                self.state = 'STOP_MOVE'
                self.get_logger().info(f'Distance reached ({dist:.2f}m). Stopping...')

        elif self.state == 'STOP_MOVE':
            target_linear = 0.0
            # Wait until fully stopped
            if abs(self.current_linear) < 0.01:
                # Prepare for turning: reset yaw references and proceed to TURN
                self.start_yaw = self.current_yaw
                self.last_yaw = self.current_yaw
                self.total_angle_turned = 0.0
                self.state = 'TURN'
                self.get_logger().info('Stopped. Starting TURN phase...')

        elif self.state == 'TURN':
            # Use signed angle from start to track progress
            signed = self.signed_angle_from_start()
            angle = abs(signed)

            # Proportional control to slow down as we approach the stop angle
            remaining = self.STOP_ANGLE - angle
            if remaining < 0.0:
                remaining = 0.0
            target_angular = max(0.08, min(self.TARGET_ANGULAR_SPEED, remaining * 0.5))
            # Keep the correct turn direction
            target_angular = math.copysign(target_angular, signed if signed != 0 else 1.0)

            # Check if we reached stop angle (near 179 degrees)
            if angle >= self.STOP_ANGLE:
                self.state = 'STOP_TURN'
                self.get_logger().info(f'Turn reached ({math.degrees(angle):.1f} deg). Stopping...')

        elif self.state == 'STOP_TURN':
            target_angular = 0.0
            # Wait until fully stopped
            if abs(self.current_angular) < 0.01:
                # After stopping rotation, check accuracy and correct if needed
                signed = self.signed_angle_from_start()
                angle_abs = abs(signed)
                angle_error = self.GOAL_ANGLE - angle_abs
                if abs(angle_error) <= self.ANGLE_TOLERANCE:
                    # Good enough
                    self.start_x = self.current_x # Reset position reference
                    self.start_y = self.current_y
                    self.state = 'MOVE'
                    self.get_logger().info('Turn accurate. Restarting MOVE phase...')
                else:
                    self.state = 'CORRECT_TURN'
                    self.get_logger().info(f'Angle error {math.degrees(angle_error):.2f} deg, performing fine correction...')

        elif self.state == 'CORRECT_TURN':
            # Fine correction until within ANGLE_TOLERANCE
            signed = self.signed_angle_from_start()
            angle_abs = abs(signed)
            angle_error = self.GOAL_ANGLE - angle_abs
            if abs(angle_error) <= self.ANGLE_TOLERANCE:
                target_angular = 0.0
                if abs(self.current_angular) < 0.01:
                    self.start_x = self.current_x
                    self.start_y = self.current_y
                    self.state = 'MOVE'
                    self.get_logger().info('Fine turn correction complete. Restarting MOVE phase...')
            else:
                # If under-turned, continue same direction; if over-turned, rotate opposite
                if angle_abs < self.GOAL_ANGLE:
                    direction = 1.0 if signed >= 0 else -1.0
                    corr = direction * min(self.CORRECT_ANGULAR_MAX, self.CORRECT_KP_ANGULAR * angle_error)
                else:
                    # Over-turned: rotate back a small amount
                    direction = -1.0 if signed >= 0 else 1.0
                    corr = direction * min(self.CORRECT_ANGULAR_MAX, self.CORRECT_KP_ANGULAR * abs(angle_error))
                target_angular = corr

        # Apply Ramping
        self.current_linear = self.ramp_value(self.current_linear, target_linear)
        self.current_angular = self.ramp_value(self.current_angular, target_angular)
        
        # Publish
        msg = Twist()
        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular
        self.cmd_pub.publish(msg)
        # Print status line with quit hint
        try:
            self.print_status()
        except Exception:
            pass

    def print_status(self):
        """Print a single-line status that updates in-place; includes quit hint."""
        status = (f"\r[autopilot] State:{self.state} | "
                  f"Lin:{self.current_linear:+.2f} m/s "
                  f"Ang:{self.current_angular:+.2f} rad/s   "
                  "| Press 'q' to quit ")
        sys.stdout.write(status)
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = AutopilotNode()
    # Set up a keyboard listener thread so user can press 'q' to stop autopilot and quit
    try:
        orig_settings = termios.tcgetattr(sys.stdin)
    except Exception:
        orig_settings = None

    stop_event = threading.Event()

    def keyboard_listener():
        try:
            if orig_settings:
                tty.setcbreak(sys.stdin.fileno())
            while not stop_event.is_set() and rclpy.ok():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    ch = sys.stdin.read(1)
                    if ch == 'q':
                        node.get_logger().info("'q' pressed: stopping autopilot and quitting.")
                        stop_msg = Twist()
                        node.cmd_pub.publish(stop_msg)
                        rclpy.shutdown()
                        stop_event.set()
                        break
                    if ch == '\x03':
                        node.get_logger().info('Ctrl-C detected in keyboard listener.')
                        rclpy.shutdown()
                        stop_event.set()
                        break
        except Exception as e:
            node.get_logger().warn(f'Keyboard listener error: {e}')
        finally:
            if orig_settings:
                try:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
                except Exception:
                    pass

    listener_thread = threading.Thread(target=keyboard_listener, daemon=True)
    listener_thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot on exit
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        stop_event.set()
        try:
            listener_thread.join(timeout=0.5)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
