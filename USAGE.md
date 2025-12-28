# How to Use Monocular Depth Navigation

This guide provides detailed step-by-step instructions for running the monocular depth navigation system.

---

## 📋 Prerequisites

Before running, ensure you have:
1. Built the workspace successfully

```bash
cd ~/monocular-depth-navigation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

2. All dependencies installed (see README.md)
3. Sourced the ROS2 environment

```bash
cd ~/monocular-depth-navigation/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 🚀 Running the System

The simplest way to run the complete system:

### Terminal 1: Gazebo Simulation

```bash
cd ~/monocular-depth-navigation/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot simple_maze.launch.py
```

**Wait for:** Gazebo window to open and robot to appear.

### Terminal 2: Perception + Navigation (Combined)

```bash
cd ~/monocular-depth-navigation/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot depth_nav.launch.py
```

This single command starts:
- Ground edge detection node
- Nav2 controller server
- Nav2 planner server
- Nav2 behavior server
- Lifecycle manager

**Wait for:** "Lifecycle nodes are active" or "Creating bond timer.." message.

### Terminal 3: Visualization
```bash
cd ~/monocular-depth-navigation/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot rviz.launch.py
```

---

## 🎮 Teleoperation

### GUI Teleop (Recommended)

```bash
ros2 run my_robot teleop_gui.py
```

**Features:**
- Graphical window with sliders
- Adjustable max speed and turn rate
- Visual feedback of current velocities
- Multi-key support (hold multiple arrows)

**Controls:**
| Key | Action |
|-----|--------|
| ↑ | Forward |
| ↓ | Backward |
| ← | Turn left |
| → | Turn right |
| Space | Emergency stop |

### Terminal Teleop (Alternative)

```bash
ros2 run my_robot teleop.py
```

Same controls, but in terminal without GUI.

---

## 📷 Camera Sweep (Optional)

The camera sweep node makes the camera pan left and right continuously, giving the robot a wider field of view:

```bash
ros2 run my_robot camera_trajectory_control.py
```

**Features:**
- Sinusoidal sweep motion (±20° at 0.4 Hz)
- Increases visible area for depth perception
- Press **Q** to stop and reset camera to forward

> **Note:** This doesn't significantly improve depth perception accuracy, but it allows the robot to detect obstacles outside its normal forward-facing view.

---

## 🌍 Choosing a World

### Simple Maze (Default, Recommended for Testing)
```bash
ros2 launch my_robot simple_maze.launch.py
```
- Size: 25×25 meters
- Features: Multiple paths, dead ends
- Good for: Initial testing, learning the system

### Large Maze
```bash
ros2 launch my_robot my_maze.launch.py
```
- Size: 30×30 meters
- Features: More complex layout
- Good for: Extended navigation tests

### TurtleBot3 House
```bash
ros2 launch my_robot house.launch.py
```
- Realistic indoor environment
- Furniture, rooms, corridors
- Good for: Real-world scenario testing

### Colorful Obstacles
```bash
ros2 launch my_robot colorful_obstacles.launch.py
```
- Open area with cylinder obstacles
- Various colors and sizes
- Good for: Perception testing

---

## 🎯 Sending Navigation Goals

### Using RViz (Recommended)

1. Launch RViz: `ros2 launch my_robot rviz.launch.py`
2. Click the **"2D Goal Pose"** button in the toolbar
3. Click and drag on the map to set goal position and orientation
4. Robot will plan a path and navigate autonomously


### `or:` Using Command Line

```bash
# Navigate to position (x=2.0, y=1.0)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

## 🗺️ Creating Maps

### Step 1: Generate a Maze World

```bash
cd ~/monocular-depth-navigation/ros2_ws/src/my_robot/scripts

# Option A: Generate simple 25x25m maze
python3 generate_simple_maze.py

# Option B: Generate larger 30x30m maze
python3 generate_maze.py

# Option C: Generate colorful obstacle world
python3 generate_colorful_world.py
```

### Step 2: Create Occupancy Grid Map

After generating the world file, create a map for Nav2:

```bash
cd ~/monocular-depth-navigation/ros2_ws/src/my_robot/scripts

# Generate the PGM map file
python3 create_map.py

# Convert to PNG (required by static_map_publisher.py)
convert ~/monocular-depth-navigation/ros2_ws/src/my_robot/maps/world_reference.pgm \
        ~/monocular-depth-navigation/ros2_ws/src/my_robot/maps/world_reference.png

# Remove the PGM file (not needed)
rm ~/monocular-depth-navigation/ros2_ws/src/my_robot/maps/world_reference.pgm
```

> **Important:** Always save as `world_reference.png` — the RViz display and `static_map_publisher.py` are configured to use this filename.

> **Note:** If the map doesn't appear in RViz, check that the Map display is enabled. The static map publisher sends updates every 2 seconds to keep the map visible.

### Step 3: Rebuild (if needed)

```bash
cd ~/monocular-depth-navigation/ros2_ws
colcon build --packages-select my_robot
source install/setup.bash
```

---

## 🤖 Real Robot Deployment

### Key Difference: RTAB-Map Required

In Gazebo simulation, odometry comes from the diff_drive plugin (ground truth).
On a real robot, you need RTAB-Map for visual odometry.

### Launch for Real Robot

```bash
# Terminal 1: RTAB-Map (provides visual odometry)
ros2 launch my_robot rtabmap.launch.py

# Terminal 2: Perception + Navigation
ros2 launch my_robot depth_nav.launch.py
```

### RTAB-Map Topics

| Topic | Purpose |
|-------|---------|
| `/rtabmap/odom` | Visual odometry |
| `/rtabmap/mapData` | 3D map data |
| `/rtabmap/grid_map` | 2D occupancy grid |

---

## 🔧 Tuning Parameters

### Ground Edge Detection

Edit `ros2_ws/src/mono_depth_perception/config/ground_edge_params.yaml`:

```yaml
ground_edge:
  roi_ratio: 0.42        # Increase for lower obstacles
  min_edge_strength: 50  # Decrease for low-contrast environments
  camera_height: 0.3     # Must match actual camera height
  depth_scale: 0.60      # Calibrate for accurate distances
```

**Rebuild after changes:**
```bash
colcon build --packages-select mono_depth_perception
source install/setup.bash
```

### Nav2 Controller

Edit `ros2_ws/src/my_robot/config/nav2_local_params.yaml`:

```yaml
controller_server:
  ros__parameters:
    max_vel_x: 0.30      # Maximum forward speed (m/s)
    max_vel_theta: 1.0   # Maximum rotation speed (rad/s)
```

---

## 📊 Debugging Commands

### Check Topic Rates
```bash
ros2 topic hz /camera_sensor/image_raw    # Camera: ~20 Hz
ros2 topic hz /ground_edge_scan           # Perception: ~20 Hz
ros2 topic hz /odom                       # Odometry: ~50 Hz
```

### View Raw Data
```bash
ros2 topic echo /ground_edge_scan         # See laser scan values
ros2 topic echo /cmd_vel                  # See velocity commands
```

### Check TF Tree
```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf showing transform tree
```

### List Active Nodes
```bash
ros2 node list
# Should show: ground_edge_node, controller_server, planner_server, etc.
```

### Check Node Status
```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

---

## ⚠️ Common Issues

### "Waiting for transform..."
**Cause:** TF tree incomplete  
**Fix:** 
- Ensure Gazebo is running
- Check robot_state_publisher is active
- For real robot: ensure RTAB-Map is running

### Costmap shows no obstacles
**Cause:** Ground edge not detecting edges  
**Fix:**
- Verify `/ground_edge_scan` is publishing: `ros2 topic echo /ground_edge_scan`
- Check that obstacles touch the ground
- Ensure sufficient lighting contrast

### Robot moves too slow / fast
**Fix:** Adjust in `nav2_local_params.yaml`:
```yaml
max_vel_x: 0.30        # Increase for faster motion
min_speed_xy: 0.05     # Minimum speed when moving
```

### Robot gets stuck
**Cause:** Nav2 recovery behaviors triggered  
**Fix:** Recovery behaviors (backup, spin, wait) are automatic. If still stuck:
- Check if goal is reachable
- Reduce inflation radius in costmap config

### GUI Teleop window not responding
**Cause:** Window needs focus  
**Fix:** Click on the teleop window to give it keyboard focus

### Gazebo hangs on exit
**Fix:**
```bash
pkill -9 gzserver gzclient
```

---

## 🛑 Stopping the System

1. Press `Ctrl+C` in each terminal
2. If Gazebo hangs:
   ```bash
   pkill -9 gzserver gzclient
   ```

---
## 🚫 Kill all ros2 services (if needed)
```bash
cd ~/monocular-depth-navigation/ros2_ws
./cleanup_ros.sh
```
---

## 📝 Quick Reference

### Essential Commands

| Action | Command |
|--------|---------|
| Start simulation | `ros2 launch my_robot simple_maze.launch.py` |
| Start perception + nav | `ros2 launch my_robot depth_nav.launch.py` |
| Start visualization | `ros2 launch my_robot rviz.launch.py` |
| GUI teleop | `ros2 run my_robot teleop_gui.py` |

### Alternative Worlds

| World | Command |
|-------|---------|
| Large maze | `ros2 launch my_robot my_maze.launch.py` |
| House | `ros2 launch my_robot house.launch.py` |
| Obstacles | `ros2 launch my_robot colorful_obstacles.launch.py` |

### For Real Robot

| Action | Command |
|--------|---------|
| Start RTAB-Map | `ros2 launch my_robot rtabmap.launch.py` |
| Start navigation | `ros2 launch my_robot depth_nav.launch.py` |

---

## 💡 Tips

1. **Start in order** — Gazebo first, then perception+nav
2. **Use depth_nav.launch.py** — Simpler than launching separately
3. **Use teleop_gui.py** — Better than terminal teleop
4. **Check RViz** — Best way to see what's happening
5. **Check the costmap** — If obstacles aren't showing, navigation won't avoid them
6. **Tune depth_scale** — If obstacles appear at wrong distance, adjust this
7. **For real robot** — Always run RTAB-Map for odometry

---

## 🧪 Development/Testing Tools

These scripts are available for testing and debugging but are not part of the normal workflow:

| Script | Purpose |
|--------|---------|
| `autopilot.py` | Moves 5m forward, turns 180°, repeats (press Q to quit) |
| `shuttle_drive.py` | Moves forward/backward repeatedly without turning (press Q to quit) |

Useful for:
- Testing depth perception during continuous motion
- Verifying odometry accuracy
- Automated data collection

```bash
# Example
ros2 run my_robot autopilot.py
```
