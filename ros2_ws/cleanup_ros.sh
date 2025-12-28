#!/bin/bash
echo "=== ROS2 Cleanup Script ==="

echo "1. Killing all ROS2 nodes..."

pkill -f ros2
pkill -f nav2
pkill -f gazebo
pkill -f gzserver
pkill -f gzclient
pkill -f robot_state_publisher
pkill -f controller_server
pkill -f planner_server
pkill -f behavior_server
pkill -f bt_navigator
pkill -f waypoint_follower
pkill -f lifecycle_manager
pkill -f component_container
pkill -f sensor_relay
pkill -f rviz
pkill -f mono_depth_node

echo "2. Stopping ROS2 Daemon..."
ros2 daemon stop

echo "3. Cleaning up shared memory segments (if any)..."
# This might require sudo, but we'll try user-owned segments
if [ -d "/dev/shm" ]; then
    rm -f /dev/shm/fastrtps_*
fi

echo "=== Cleanup Complete ==="
echo "Please restart your terminals and launch files"
