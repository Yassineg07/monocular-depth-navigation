#!/usr/bin/env python3
"""
GUI keyboard teleop controller for ROS2 with proper multi-key support.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk

# Movement parameters
LINEAR_SPEED = 0.4
TURN_RATE = 0.3
PUBLISH_RATE = 20.0


class TeleopGUI(Node):
    def __init__(self, root):
        super().__init__('teleop_gui')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.root = root
        self.root.title("Robot Teleop")
        self.root.geometry("400x320")
        
        # Velocity state
        self.linear = 0.0
        self.angular = 0.0
        
        # Key state - tracks actual key press/release
        self.keys_pressed = {
            'Up': False,
            'Down': False,
            'Left': False,
            'Right': False
        }
        
        # Track if left/right are currently held
        self.turn_held = False
        
        # Setup UI
        self.setup_ui()
        
        # Bind keys
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)
        
        # Timer for publishing (10 Hz)
        self.timer_ms = 100
        self.update()
        
        self.get_logger().info('Teleop GUI started')
    
    def setup_ui(self):
        # Instructions
        title = tk.Label(self.root, text="ROBOT TELEOP CONTROLLER", 
                        font=("Arial", 14, "bold"))
        title.pack(pady=5)
        
        instructions = tk.Label(self.root, text=
            "Use Arrow Keys to move | Hold multiple keys to combine!\n"
            "SPACE = Emergency Stop",
            font=("Arial", 9))
        instructions.pack(pady=5)
        
        # Status display
        self.status_label = tk.Label(self.root, text="Speed: 0.00 m/s | Turn: 0.00 rad/s",
                                     font=("Courier", 11), bg="black", fg="lime",
                                     padx=10, pady=5)
        self.status_label.pack(pady=10)
        
        # Controls frame
        controls = tk.Frame(self.root)
        controls.pack(pady=10, padx=20, fill='x')
        
        # Max Speed slider
        tk.Label(controls, text="Max Speed (m/s):", font=("Arial", 9)).grid(row=0, column=0, sticky='w', pady=2)
        self.speed_slider = tk.Scale(controls, from_=0.1, to=1.0, resolution=0.05, orient='horizontal', length=200)
        self.speed_slider.set(LINEAR_SPEED)
        self.speed_slider.grid(row=0, column=1, pady=2)
        
        # Max Turn Rate slider
        tk.Label(controls, text="Max Turn (rad/s):", font=("Arial", 9)).grid(row=1, column=0, sticky='w', pady=2)
        self.turn_slider = tk.Scale(controls, from_=0.1, to=1.0, resolution=0.05, orient='horizontal', length=200)
        self.turn_slider.set(TURN_RATE)
        self.turn_slider.grid(row=1, column=1, pady=2)
        
        # Keep window focused
        self.root.focus_force()
    
    def on_key_press(self, event):
        if event.keysym in self.keys_pressed:
            self.keys_pressed[event.keysym] = True
            if event.keysym in ['Left', 'Right']:
                self.turn_held = True
        elif event.keysym == 'space':
            self.emergency_stop()
    
    def on_key_release(self, event):
        if event.keysym in self.keys_pressed:
            self.keys_pressed[event.keysym] = False
            if event.keysym in ['Left', 'Right']:
                # Check if both turn keys are released
                if not self.keys_pressed['Left'] and not self.keys_pressed['Right']:
                    self.turn_held = False
    
    def emergency_stop(self):
        self.linear = 0.0
        self.angular = 0.0
        for key in self.keys_pressed:
            self.keys_pressed[key] = False
        self.turn_held = False
        self.get_logger().info('EMERGENCY STOP')
    
    def update(self):
        # Get current settings from sliders
        max_speed = self.speed_slider.get()
        max_turn = self.turn_slider.get()
        
        # Calculate velocities from key states
        self.linear = 0.0
        self.angular = 0.0
        
        if self.keys_pressed['Up']:
            self.linear = max_speed
        elif self.keys_pressed['Down']:
            self.linear = -max_speed
        
        # Only turn if actively holding turn keys
        if self.turn_held:
            if self.keys_pressed['Left']:
                self.angular = max_turn
            elif self.keys_pressed['Right']:
                self.angular = -max_turn
        
        # Publish
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.publisher.publish(msg)
        
        # Update status
        self.status_label.config(
            text=f"Speed: {self.linear:+.2f} m/s | Turn: {self.angular:+.2f} rad/s"
        )
        
        # Spin ROS
        rclpy.spin_once(self, timeout_sec=0)
        
        # Schedule next update
        self.root.after(self.timer_ms, self.update)
    

    def on_closing(self):
        # Stop robot
        stop_msg = Twist()
        self.publisher.publish(stop_msg)
        self.get_logger().info('Shutting down')
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    
    root = tk.Tk()
    teleop = TeleopGUI(root)
    
    root.protocol("WM_DELETE_WINDOW", teleop.on_closing)
    
    try:
        root.mainloop()
    finally:
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
