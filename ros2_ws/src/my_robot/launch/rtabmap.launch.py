#!/usr/bin/env python3
"""
RTAB-Map Visual SLAM Launch File for my_robot
Uses monocular camera with ORB features for pure visual odometry and SLAM

Subscribes to:
  - /camera_sensor/image_raw (monocular camera)
  - /camera_sensor/camera_info (camera calibration)

Publishes:
  - /rtabmap/odom (visual odometry from ORB features)
  - /rtabmap/mapData (3D map)
  - /rtabmap/grid_map (2D occupancy grid)
  - /tf (map -> odom transform)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    # RTAB-Map SLAM Node - Uses wheel odometry + monocular visual SLAM
    # NOTE: We use the robot's differential drive odometry from Gazebo
    # instead of rgbd_odometry because we don't have a depth camera
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'subscribe_rgb': True,
            'subscribe_depth': False,
            'subscribe_odom_info': False,
            'approx_sync': True,
            'wait_for_transform': 0.2,
            'qos': 1,
            
            # Pure monocular SLAM - OPTIMIZED
            'RGBD/Enabled': 'false',
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            
            # Feature detection - faster settings
            'Kp/DetectorStrategy': '6',     # GFTT (fastest)
            'Vis/FeatureType': '6',         # GFTT
            'Vis/MaxFeatures': '300',       # Reduced to 300 for better FPS
            'Kp/MaxFeatures': '300',
            'Vis/MinInliers': '8',          # Lower threshold
            'Vis/EstimationType': '0',      # 3D->3D
            'Vis/CorNNType': '1',           # Brute force (faster)
            
            # Performance optimization
            'Rtabmap/DetectionRate': '2.0',      # Reduced to 2 Hz for better FPS
            'Rtabmap/TimeThr': '0',
            'RGBD/OptimizeFromGraphEnd': 'false',
            'RGBD/ProximityBySpace': 'false',
            'RGBD/AngularUpdate': '0.1',         # Less frequent updates
            'RGBD/LinearUpdate': '0.1',
            'RGBD/LocalRadius': '3',             # Smaller radius
            'Mem/STMSize': '10',
            'Mem/ImageKept': 'false',            # Don't keep images in memory
            'Optimizer/Strategy': '0',           # TORO (faster than g2o)
            
            # 2D Grid mapping for obstacles - CRITICAL FOR VISUALIZATION
            'Grid/FromDepth': 'false',           # Can't use depth (monocular)
            'Grid/RangeMax': '3.0',              # Assume obstacles within 3m
            'Grid/CellSize': '0.05',             # 5cm cells
            'Grid/3D': 'false',                  # 2D mapping only
            'Grid/GroundIsObstacle': 'false',    # Don't mark ground as obstacle
            'Grid/RayTracing': 'true',           # Enable ray tracing for free space
            'Grid/ClusterRadius': '0.1',
            'Grid/MaxObstacleHeight': '1.0',     # Obstacles up to 1m high
            'Grid/MinClusterSize': '5',
            'GridGlobal/MinSize': '20.0',        # 20m x 20m global map
            'GridGlobal/Eroded': 'true',         # Erode obstacles slightly
            'GridGlobal/FullUpdate': 'false',    # Incremental updates for speed
            'GridGlobal/OccupancyThr': '0.5',    # Occupancy threshold
            
            # Disable unnecessary publishers (octomap, prob grid)
            'Grid/PubProbMap': 'false',          # Don't publish grid_prob_map
            
            # Map projection - project 3D points to 2D grid
            'Proj/MaxGroundAngle': '45',         # Ground angle tolerance
            'Proj/MaxGroundHeight': '0.0',       # Ground at z=0
            'Proj/MinClusterSize': '3',
            'Proj/MaxHeight': '2.0',             # Max obstacle height
        }],
        remappings=[
            ('rgb/image', '/camera_sensor/image_raw'),
            ('rgb/camera_info', '/camera_sensor/camera_info'),
            ('odom', '/odom'),
        ],
        arguments=['--delete_db_on_start'],
        namespace=''
    )
    
    # Visualization
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'subscribe_rgb': True,
            'subscribe_depth': False,
            'subscribe_odom_info': False,
            'frame_id': 'base_link',
            'approx_sync': True,
            'qos': 1
        }],
        remappings=[
            ('rgb/image', '/camera_sensor/image_raw'),
            ('rgb/camera_info', '/camera_sensor/camera_info'),
            ('odom', '/odom'),
        ],
        namespace=''
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        rtabmap_slam,
        # rtabmap_viz
    ])
