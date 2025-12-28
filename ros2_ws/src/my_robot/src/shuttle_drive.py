#!/usr/bin/env python3
"""Shuttle-drive node for autonomous robot movement.

Moves forward a fixed distance, stops, then moves backward the same distance, repeats.
Uses smoothed velocity commands (Twist) and Odometry feedback.
Press 'q' to stop and quit.
"""

import sys
import termios
import tty
import select
import threading
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class ShuttleDriveNode(Node):
    def __init__(self):
        super().__init__('shuttle_drive')

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Movement parameters
        self.TARGET_LINEAR_SPEED = 0.5  # m/s (magnitude)
        self.RAMP_FACTOR = 0.05         # smoothing factor (0.0-1.0)

        # Goal and stop threshold
        self.GOAL_DISTANCE = 2.0   # meters
        self.STOP_DISTANCE = 1.95  # stop slightly early

        # State machine
        # INIT, FORWARD, STOP_FORWARD, BACKWARD, STOP_BACKWARD
        self.state = 'INIT'

        # Current velocities (for ramping)
        self.current_linear = 0.0

        # Odometry tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.odom_received = False

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Shuttle-drive node started')
        self.get_logger().info('Pattern: forward -> backward -> repeat (no rotation)')
        print("[shuttle_drive] Running — press 'q' to stop and quit")

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        if not self.odom_received:
            self.odom_received = True

    def get_distance_traveled(self) -> float:
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        return math.sqrt(dx * dx + dy * dy)

    def ramp_value(self, current: float, target: float) -> float:
        diff = target - current
        step = diff * self.RAMP_FACTOR
        if abs(diff) < 0.01:
            return target
        return current + step

    def control_loop(self):
        if not self.odom_received:
            return

        target_linear = 0.0

        if self.state == 'INIT':
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.state = 'FORWARD'
            self.get_logger().info('Starting FORWARD phase...')

        elif self.state == 'FORWARD':
            dist = self.get_distance_traveled()
            target_linear = +self.TARGET_LINEAR_SPEED
            if dist >= self.STOP_DISTANCE:
                self.state = 'STOP_FORWARD'
                self.get_logger().info(f'Distance reached ({dist:.2f}m). Stopping...')

        elif self.state == 'STOP_FORWARD':
            target_linear = 0.0
            if abs(self.current_linear) < 0.01:
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.state = 'BACKWARD'
                self.get_logger().info('Stopped. Starting BACKWARD phase...')

        elif self.state == 'BACKWARD':
            dist = self.get_distance_traveled()
            target_linear = -self.TARGET_LINEAR_SPEED
            if dist >= self.STOP_DISTANCE:
                self.state = 'STOP_BACKWARD'
                self.get_logger().info(f'Distance reached ({dist:.2f}m). Stopping...')

        elif self.state == 'STOP_BACKWARD':
            target_linear = 0.0
            if abs(self.current_linear) < 0.01:
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.state = 'FORWARD'
                self.get_logger().info('Stopped. Restarting FORWARD phase...')

        # Apply ramping
        self.current_linear = self.ramp_value(self.current_linear, target_linear)

        # Publish (no rotation)
        msg = Twist()
        msg.linear.x = self.current_linear
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

        try:
            self.print_status()
        except Exception:
            pass

    def print_status(self):
        status = (
            f"\r[shuttle_drive] State:{self.state} | "
            f"Lin:{self.current_linear:+.2f} m/s | Press 'q' to quit "
        )
        sys.stdout.write(status)
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = ShuttleDriveNode()

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
                        node.get_logger().info("'q' pressed: stopping and quitting.")
                        node.cmd_pub.publish(Twist())
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
        node.cmd_pub.publish(Twist())
        stop_event.set()
        try:
            listener_thread.join(timeout=0.5)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
