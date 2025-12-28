#!/usr/bin/env python3
"""
Smoothed keyboard teleop controller for ROS2.
Uses target velocities and ramping to provide smooth control.
"""

import sys
import termios
import tty
import select
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Movement parameters
LINEAR_SPEED = 1.0      # m/s - max forward/backward speed
TURN_RATE = 0.8         # rad/s - max rotation rate
PUBLISH_RATE = 20.0     # Hz - higher rate for smoother ramping
RAMP_FACTOR = 0.1       # 0.0 to 1.0 - how fast to reach target (higher = faster)

# Time-based hold thresholds (seconds)
KEY_TIMEOUT = 0.5       # Time before a key is considered released

# Control instructions
INSTRUCTIONS = """

    KEYBOARD TELEOP CONTROLLER

            (Forward) 
                ↑                 
(Turn Left) ←       → (Turn Right)
                ↓   
            (Backward)

  Combine ↑/↓ + ←/→ to turn!

  SPACE : Emergency Stop
  q     : Quit

  Max Speed : 1.0 m/s
  Max Turn  : 0.8 rad/s

  Press Ctrl+C to exit 

Press arrow keys to move. Robot stops when released.
"""


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.timer_callback)
        
        # Velocity state
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Key state tracking with individual timestamps
        self.key_times = {
            'forward': 0.0,
            'backward': 0.0,
            'left': 0.0,
            'right': 0.0
        }
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Teleop keyboard node started')
        print(INSTRUCTIONS.replace('\n', '\r\n')) # Ensure proper formatting
        
        tty.setraw(sys.stdin.fileno())
    
    def timer_callback(self):
        # Check each key's timeout individually
        current_time = time.time()
        key_forward = (current_time - self.key_times['forward']) < KEY_TIMEOUT
        key_backward = (current_time - self.key_times['backward']) < KEY_TIMEOUT
        key_left = (current_time - self.key_times['left']) < KEY_TIMEOUT
        key_right = (current_time - self.key_times['right']) < KEY_TIMEOUT
        
        # Determine target velocities based on keys
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        if key_forward:
            self.target_linear = LINEAR_SPEED
        elif key_backward:
            self.target_linear = -LINEAR_SPEED
            
        if key_left:
            self.target_angular = TURN_RATE
        elif key_right:
            self.target_angular = -TURN_RATE
            
        # Ramp current velocities towards target
        self.current_linear = self.ramp_value(self.current_linear, self.target_linear)
        self.current_angular = self.ramp_value(self.current_angular, self.target_angular)
        
        # Publish Twist
        msg = Twist()
        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular
        self.publisher.publish(msg)
        
        self.print_status()

    def ramp_value(self, current, target):
        """Move current towards target by RAMP_FACTOR."""
        diff = target - current
        step = diff * RAMP_FACTOR
        
        # If close enough, snap to target to avoid infinite approach
        if abs(diff) < 0.01:
            return target
        return current + step
    
    def get_key(self, timeout=0.01): # Very short timeout for non-blocking feel
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = ''
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x03':
                return key
            if key == '\x1b':
                c2 = sys.stdin.read(1)
                c3 = sys.stdin.read(1)
                key = '\x1b[' + c3
        return key
    
    def print_status(self):
        # Simple status bar
        status = f'\rSpeed: {self.current_linear:+.2f} m/s | Turn: {self.current_angular:+.2f} rad/s   '
        sys.stdout.write(status)
        sys.stdout.flush()
    
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key:
                    if key == '\x03':
                        raise KeyboardInterrupt
                    if key == 'q':
                        print('\n\nQuitting...')
                        break
                    if key == ' ':
                        self.target_linear = 0.0
                        self.target_angular = 0.0
                        self.current_linear = 0.0
                        self.current_angular = 0.0
                        self.key_times = {k: 0.0 for k in self.key_times}
                        print('\n[EMERGENCY STOP]')
                    else:
                        current_time = time.time()
                        if key == '\x1b[A':
                            self.key_times['forward'] = current_time
                        elif key == '\x1b[B':
                            self.key_times['backward'] = current_time
                        elif key == '\x1b[C':
                            self.key_times['right'] = current_time
                        elif key == '\x1b[D':
                            self.key_times['left'] = current_time
                
                rclpy.spin_once(self, timeout_sec=0)
        except KeyboardInterrupt:
            print('\n\nShutdown requested (Ctrl+C)')
        finally:
            self.current_linear = 0.0
            self.current_angular = 0.0
            stop_msg = Twist()
            self.publisher.publish(stop_msg)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print('\nRobot stopped. Goodbye!')


def main(args=None):
    rclpy.init(args=args)
    try:
        teleop = TeleopKeyboard()
        teleop.run()
    finally:
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
