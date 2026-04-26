# Monocular Depth Navigation

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Nav2](https://img.shields.io/badge/Nav2-Smac_Hybrid_Planner-green)](https://docs.nav2.org/configuration/packages/smac/configuring-smac-hybrid.html)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange)](http://gazebosim.org/)

**Autonomous robot navigation using only a monocular RGB camera**. No LiDAR, no depth sensor, no AI and no map.

This project demonstrates that a robot can navigate complex environments using computer vision techniques applied to a single camera feed. The robot detects obstacles by analyzing ground-to-wall transitions in the camera image and uses Nav2 for path planning and obstacle avoidance.

[![Demo Video](https://img.youtube.com/vi/ioWUWzyHvF8/maxresdefault.jpg)](https://youtu.be/ioWUWzyHvF8)

*(click the image to view the video)*

---

## 🎯 Project Goal

Build a fully autonomous navigation system that relies solely on:
- **1 RGB camera** (640×480 @ 30Hz)
- **Visual odometry** (RTAB-Map with ORB features — required for real robot, optional in simulation)
- **Ground edge detection** (custom perception algorithm)
- **Nav2** (global + local planning)

No expensive sensors required!

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

### Do obstacles “stay” while running?

- At startup, the robot is basically **blind**: obstacle grids start empty.
- While Nav2 is running, obstacles are added to a **live obstacle grid** (costmap) as the robot detects them.
- In this project, the **global costmap tends to keep obstacles** during the run, while the **local costmap can clear them** as the sensor updates.
- Nothing is saved to disk: restarting Nav2 resets the costmaps.

### About the reference “map” in RViz (occupancy grid)

This repo can generate an occupancy grid image for **RViz/visual reference and easier goal placement only**.

- Nav2 navigation here is driven by the live costmaps built from `/ground_edge_scan`.
- The occupancy grid image is **not** used by Nav2 for planning/navigation in this project.

---

## 🏗️ System Architecture

At a glance:
- Camera → ground-edge detection → `/ground_edge_scan` → Nav2 costmaps → planner/controller → `/cmd_vel`
- Nav2 plugins used: `RegulatedPurePursuitController` (local controller) + `SmacPlannerHybrid` (global planner)

```
┌─────────────────────────────────────────────────────────────────────┐
│                         GAZEBO SIMULATION                           │
│   Robot with Camera (640×480) + IMU + Differential Drive            │
│   (Provides ground-truth odometry via diff_drive plugin)            │
└─────────────────────────────────┬───────────────────────────────────┘
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
   │ (Real robot     │  └─────────┬───────┘
   │  only - not     │            │
   │  needed in sim) │            │
   └──────────┬──────┘            │
              │                   │
              └─────────┬─────────┘
                        ▼
           ┌─────────────────────────┐
           │           NAV2          │
           │  ─────────────────────  |
           │  • Costmaps (obstacles) |
           │  • Global planner:      |
           |     Smac Planner Hybrid │
           │  • Local planner:       │
           │     Regulated Pure      │
           │      Pursuit controller │
           │  → /cmd_vel             │
           └────────────┬────────────┘
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

## 📦 Setup

Detailed setup and run instructions are in [USAGE.md](USAGE.md).

### Clone & Build

```bash
# Clone repository
git clone https://github.com/Yassineg07/monocular-depth-navigation.git ~/monocular-depth-navigation

# Build workspace
cd ~/monocular-depth-navigation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## 📁 Project Structure

```
monocular-depth-navigation/
├── README.md
├── USAGE.md
├── LICENSE
├── .gitignore
└── ros2_ws/
    ├── cleanup_ros.sh
    ├── nodes_graph.png            # Node graph artifact
    ├── frames_2026-01-01_15.33.37.pdf  # TF tree artifact
    ├── frames_2026-01-01_15.33.37.gv
    └── src/
        ├── mono_depth_perception/      # Ground edge detection package
        │   ├── launch/ground_edge.launch.py
        │   ├── config/ground_edge_params.yaml
        │   ├── src/ground_edge_node.cpp
        │   └── include/
        └── my_robot/                   # Robot description + launch + Nav2 config
            ├── launch/                 # Gazebo + Nav2 + RViz entrypoints
            ├── config/                 # Nav2 params (costmaps/planner/controller)
            │   └── nav2_local_params.yaml
            ├── scripts/                # Map tools + static_map_publisher
            ├── src/                    # Teleop + utilities
            ├── urdf/
            ├── worlds/
            ├── maps/
            ├── meshes/
            ├── models/
            └── include/
```

---

## 🔮 Future Enhancements

This project is **complete and functional**. Potential future additions:

- [ ] **Depth AI Models** — MiDaS, DPT, or Depth Anything for learned monocular depth
- [ ] **YOLO Object Detection** — Identify and classify specific obstacles
- [ ] **Dynamic Obstacles** — Detect and avoid moving objects
- [ ] **Real Hardware Deployment** — Raspberry Pi + camera on physical robot

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
