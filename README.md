# Monocular Depth Navigation

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange)](http://gazebosim.org/)

**Autonomous robot navigation using only a monocular RGB camera** — no LiDAR, no depth sensor, no AI and no map.

This project demonstrates that a robot can navigate complex environments using computer vision techniques applied to a single camera feed. The robot detects obstacles by analyzing ground-to-wall transitions in the camera image and uses Nav2 for path planning and obstacle avoidance.

---

## 🎯 Project Goal

Build a fully autonomous navigation system that relies solely on:
- **1 RGB camera** (640×480 @ 30Hz)
- **Visual odometry** (RTAB-Map with ORB features — required for real robot, optional in simulation)
- **Ground edge detection** (custom perception algorithm)
- **Nav2** (global + local planning)

No expensive sensors required!

---

## 🏗️ System Architecture

At a glance:
- Camera → ground-edge detection → `/ground_edge_scan` → Nav2 costmaps → planner/controller → `/cmd_vel`
- Nav2 plugins used: `RegulatedPurePursuitController` (local controller) + `SmacPlannerHybrid` (global planner)
- Nav2 config: [ros2_ws/src/my_robot/config/nav2_local_params.yaml](ros2_ws/src/my_robot/config/nav2_local_params.yaml)

```
┌─────────────────────────────────────────────────────────────────────┐
│                         GAZEBO SIMULATION                           │
│   Robot with Camera (640×480) + IMU + Differential Drive            │
│   (Provides ground-truth odometry via diff_drive plugin)            │
└─────────────────────────────────────────────────────────────────────┘
                                  │
              ┌───────────────────┼───────────────────┐
              ▼                   ▼                   ▼
   ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────┐
   │  RTAB-Map SLAM  │  │ Ground Edge Node│  │ Map Creation Tools  │
   │ ─────────────── │  │ ─────────────── │  │ ─────────────────── │
   │ • ORB features  │  │ • Edge detection│  │ • generate_maze.py  │
   │ • Visual odom   │  │ • Depth from    │  │ • create_map.py     │
   │ • Loop closure  │  │   geometry      │  │ → Occupancy grid    │
   │ → /odom, /tf    │  │ → /ground_edge_ │  └─────────────────────┘
   │                 │  │   scan          │
   │ (Real robot     │  └─────────────────┘
   │  only - not     │            │
   │  needed in sim) │            │
   └─────────────────┘            │
              │                   │
              └─────────┬─────────┘
                        ▼
          ┌─────────────────────────┐
          │         NAV2            │
          │  ─────────────────────  │
          │  • Global planner       │
          │  • Local costmap        │
          │  • Regulated Pure       │
          │     Pursuit controller  │
          │  → /cmd_vel             │
          └─────────────────────────┘
                        │
                        ▼
                 Robot Motion
```

### Simulation vs Real Robot

| Component | Gazebo Simulation | Real Robot |
|-----------|-------------------|------------|
| Odometry | Gazebo diff_drive plugin (ground truth) | **RTAB-Map required** |
| RTAB-Map | Optional (can use for mapping) | **Required for visual odometry** |
| Perception | Ground Edge Detection | Ground Edge Detection |
| Navigation | Nav2 | Nav2 |

---

## 🔬 How Ground Edge Detection Works

Traditional monocular depth estimation requires robot motion (optical flow triangulation) and suffers from drift during rotation. This project uses a simpler, more robust approach:

### The Algorithm

1. **Focus on ground region** — Analyze bottom 42% of camera image
2. **Scan each column** — From bottom to top, detect intensity changes
3. **Find obstacle edge** — Where ground meets wall/obstacle
4. **Calculate depth** — Using camera geometry:
   ```
   depth = camera_height / tan(pixel_angle)
   ```
5. **Publish LaserScan** — Convert to `/ground_edge_scan` for Nav2

### Advantages

| Feature | Ground Edge | Optical Flow |
|---------|------------|--------------|
| Requires motion | ❌ No | ✅ Yes |
| Drift during rotation | ❌ No | ✅ Yes |
| Computational cost | Low | High |
| Works stationary | ✅ Yes | ❌ No |

---

## ✅ Features

- **Complete Navigation Stack** — Global planning + local obstacle avoidance
- **Monocular Vision Only** — Single RGB camera, no depth sensor
- **Ground Edge Detection** — Stable, motion-independent obstacle detection
- **RTAB-Map Integration** — Visual SLAM with ORB features (for real robot)
- **Nav2 Integration** — Full navigation with costmaps and path planning
- **Multiple Environments** — Simple maze, large maze, house, custom worlds
- **Map Creation Tools** — Scripts to generate mazes and create occupancy grid maps
- **GUI Teleoperation** — Graphical keyboard control with adjustable speed
- **Combined Launch** — Single command to start perception + navigation
- **RViz Visualization** — Real-time costmap and sensor visualization

---

## 📦 Installation

### Prerequisites

**Ubuntu 22.04 + ROS2 Humble**

```bash
# Install ROS2 and dependencies
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-rtabmap-ros \
  ros-humble-rtabmap-slam \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-tf2-tools
```

### Clone & Build

```bash
# Clone repository
git clone https://github.com/yourusername/monocular-depth-navigation.git ~/monocular-depth-navigation

# Build workspace
cd ~/monocular-depth-navigation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## 🚀 Quick Start

See **[USAGE.md](USAGE.md)** for detailed step-by-step instructions.

```bash
# Terminal 1: Gazebo simulation
ros2 launch my_robot simple_maze.launch.py

# Terminal 2: Perception + Navigation (combined)
ros2 launch my_robot depth_nav.launch.py

# Terminal 3: RViz visualization
ros2 launch my_robot rviz.launch.py

# Terminal 4: GUI Teleoperation (optional)
ros2 run my_robot teleop_gui.py
```
---

## 🔎 Key Files (Where Things Live)

- Combined launch (perception + Nav2): [ros2_ws/src/my_robot/launch/depth_nav.launch.py](ros2_ws/src/my_robot/launch/depth_nav.launch.py)
- Nav2 bringup + servers: [ros2_ws/src/my_robot/launch/nav2_local.launch.py](ros2_ws/src/my_robot/launch/nav2_local.launch.py)
- Nav2 parameters (planner/controller/costmaps/BT): [ros2_ws/src/my_robot/config/nav2_local_params.yaml](ros2_ws/src/my_robot/config/nav2_local_params.yaml)
- Ground-edge perception node (C++): [ros2_ws/src/mono_depth_perception/src/ground_edge_node.cpp](ros2_ws/src/mono_depth_perception/src/ground_edge_node.cpp)
- Ground-edge launch + params: [ros2_ws/src/mono_depth_perception/launch/ground_edge.launch.py](ros2_ws/src/mono_depth_perception/launch/ground_edge.launch.py), [ros2_ws/src/mono_depth_perception/config/ground_edge_params.yaml](ros2_ws/src/mono_depth_perception/config/ground_edge_params.yaml)
- RViz launch: [ros2_ws/src/my_robot/launch/rviz.launch.py](ros2_ws/src/my_robot/launch/rviz.launch.py)
- GUI teleop: [ros2_ws/src/my_robot/src/teleop_gui.py](ros2_ws/src/my_robot/src/teleop_gui.py)
- Static map publisher + map tools: [ros2_ws/src/my_robot/scripts/static_map_publisher.py](ros2_ws/src/my_robot/scripts/static_map_publisher.py), [ros2_ws/src/my_robot/scripts/create_map.py](ros2_ws/src/my_robot/scripts/create_map.py), [ros2_ws/src/my_robot/scripts/create_simple_maze_map.py](ros2_ws/src/my_robot/scripts/create_simple_maze_map.py)
- TF tree (generated): [ros2_ws/frames_2026-01-01_15.33.37.pdf](ros2_ws/frames_2026-01-01_15.33.37.pdf)
- Node graph (nodes-only): [ros2_ws/nodes_graph.png](ros2_ws/nodes_graph.png)

---

## 🌍 Available Worlds

| World | Launch Command | Description |
|-------|---------------|-------------|
| Simple Maze | `ros2 launch my_robot simple_maze.launch.py` | 25×25m maze, good for testing |
| Large Maze | `ros2 launch my_robot my_maze.launch.py` | 30×30m maze, more complex |
| TurtleBot3 House | `ros2 launch my_robot house.launch.py` | Indoor house with furniture |
| Colorful Obstacles | `ros2 launch my_robot colorful_obstacles.launch.py` | Open area with cylinders |

---

## 🗺️ Map Creation

The project includes tools to generate maze worlds and create occupancy grid maps:

### Generate a New Maze World

```bash
cd ~/monocular-depth-navigation/ros2_ws/src/my_robot/scripts

# Generate simple maze (25x25m)
python3 generate_simple_maze.py

# Generate large maze (30x30m)  
python3 generate_maze.py

# Generate colorful obstacle world
python3 generate_colorful_world.py
```

### Create Occupancy Grid Map from World

After generating a world, create the corresponding map for Nav2:

```bash
# For simple maze
python3 create_simple_maze_map.py

# For large maze
python3 create_map.py
```

Maps are saved to `my_robot/maps/` as PNG/PGM files.

---

## 🤖 Robot Specifications

| Component | Specification |
|-----------|--------------|
| Chassis | Two-plank design (0.25×0.17m) |
| Wheels | 4 wheels, 70mm diameter |
| Drive | Differential (front wheels active) |
| Camera | Monocular, 640×480 @ 30Hz, FOV 1.05 rad |
| IMU | 100Hz, orientation + angular velocity |
| Camera Height | 0.3m above ground |

---

## 📁 Project Structure

```
monocular-depth-navigation/
├── README.md                      # This file
├── USAGE.md                       # Detailed usage instructions
├── LICENSE                        # MIT License
└── ros2_ws/
    └── src/
        ├── mono_depth_perception/ # Ground edge detection package
        │   ├── src/
        │   │   └── ground_edge_node.cpp
        │   ├── config/
        │   │   └── ground_edge_params.yaml
        │   └── launch/
        │       └── ground_edge.launch.py
        │
        └── my_robot/              # Robot description & launch
            ├── urdf/              # Robot URDF/Xacro
            ├── worlds/            # Gazebo worlds
            │   ├── simple_maze.world
            │   ├── my_maze.world
            │   ├── house.world
            │   └── colorful_obstacles.world
            ├── maps/              # Occupancy grid maps
            ├── scripts/           # Map generation tools
            │   ├── generate_maze.py
            │   ├── generate_simple_maze.py
            │   └── create_map.py
            ├── launch/
            │   ├── depth_nav.launch.py    # Combined perception + nav
            │   ├── simple_maze.launch.py
            │   ├── my_maze.launch.py
            │   ├── nav2_local.launch.py
            │   ├── rtabmap.launch.py      # For real robot
            │   └── rviz.launch.py
            ├── config/            # Nav2 params, RViz config
            └── src/
                ├── teleop_gui.py  # GUI teleoperation (recommended)
                └── teleop.py      # Terminal teleoperation
```

---

## 🚀 Launch Files

| Launch File | Purpose |
|-------------|---------|
| `depth_nav.launch.py` | **Recommended** — Starts ground edge + Nav2 together |
| `simple_maze.launch.py` | Gazebo with a simple maze |
| `nav2_local.launch.py` | Nav2 navigation stack only |
| `rtabmap.launch.py` | RTAB-Map SLAM (**required for real robot**) |
| `rviz.launch.py` | RViz visualization |
| `ground_edge.launch.py` | Ground edge detection only |

---

## 📡 ROS2 Topics

### Perception
| Topic | Type | Description |
|-------|------|-------------|
| `/camera_sensor/image_raw` | Image | RGB camera feed |
| `/camera_sensor/camera_info` | CameraInfo | Camera calibration |
| `/ground_edge_scan` | LaserScan | Obstacle distances from ground edge |
| `/ground_edge_depth` | Float32MultiArray | Per-column depth values |

### Navigation
| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | Odometry | Robot odometry (sim: Gazebo, real: RTAB-Map) |
| `/cmd_vel` | Twist | Velocity commands to robot |
| `/local_costmap/costmap` | OccupancyGrid | Local obstacle map |

---

## ⚙️ Configuration

### Ground Edge Parameters

`mono_depth_perception/config/ground_edge_params.yaml`:

```yaml
ground_edge_node:
  ros__parameters:
    camera_topic: "/camera_sensor/image_raw"
    camera_info_topic: "/camera_sensor/camera_info"
    ground_edge:
      roi_ratio: 0.42        # Bottom 42% of image
      min_edge_strength: 50  # Intensity change threshold
      camera_height: 0.3     # Camera height (meters)
      depth_scale: 0.60      # Depth calibration factor
```

### Nav2 Parameters

Key settings in `my_robot/config/nav2_local_params.yaml`:
- Local controller: `RegulatedPurePursuitController` (FollowPath)
- Global planner: `SmacPlannerHybrid` (Hybrid-A*)
- Local + global costmaps: obstacle + inflation layers (obstacle source: `/ground_edge_scan`)
- BT Navigator: behavior-tree-driven navigation + recovery behaviors

---

## 🔮 Future Enhancements

This project is **complete and functional**. Potential future additions:

- [ ] **Depth AI Models** — MiDaS, DPT, or Depth Anything for learned monocular depth
- [ ] **YOLO Object Detection** — Identify and classify specific obstacles
- [ ] **Dynamic Obstacles** — Detect and avoid moving objects
- [ ] **Real Hardware Deployment** — Raspberry Pi + camera on physical robot

---

## 🛠️ Troubleshooting

### Gazebo won't start
```bash
pkill -9 gzserver gzclient
ros2 launch my_robot simple_maze.launch.py
```

### No obstacles in costmap
- Check `/ground_edge_scan` is publishing: `ros2 topic echo /ground_edge_scan`
- Verify camera sees ground-wall transitions
- Adjust `min_edge_strength` for different lighting

### Do obstacles stay during a run?
- During a run, Nav2 maintains a **live obstacle grid** (costmap).
- In this project, the **global costmap keeps obstacles** while Nav2 is running (so they usually don’t disappear).
- The **local costmap updates faster** and can clear obstacles as the sensor updates.
- Nothing is saved to disk (doesn't create a map): restarting Nav2 resets the costmaps.

### Robot doesn't move to goal
- Ensure all nodes are running (simulation, perception, Nav2)
- Check TF tree: `ros2 run tf2_tools view_frames`
- Verify `/odom` is publishing

### Ground edge not detecting obstacles
- Obstacles must touch the ground (floating objects won't be detected)
- Need sufficient contrast between ground and obstacles
- Camera must see the ground-obstacle boundary

### For real robot: No odometry
- **RTAB-Map is required** for real robot — it provides visual odometry
- Launch with: `ros2 launch my_robot rtabmap.launch.py`

---

## 🙏 Credits

- [ROS2 Humble](https://docs.ros.org/en/humble/) — Robot Operating System
- [Nav2](https://navigation.ros.org/) — Navigation stack
- [RTAB-Map](https://github.com/introlab/rtabmap_ros) — Visual SLAM
- [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) — Gazebo models

---

## 📄 License

MIT License — See [LICENSE](LICENSE) file for details.

---

## ✉️ Contact

For questions or support, please open an issue or contact [Yassineg07](mailto:gharbiyasine040@gmail.com).
