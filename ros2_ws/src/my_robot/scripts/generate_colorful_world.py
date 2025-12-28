#!/usr/bin/env python3
import os
import random

# Get the script's directory to build relative paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)  # my_robot package root

def generate_colorful_world():
    sdf = []
    sdf.append('<?xml version="1.0"?>')
    sdf.append('<sdf version="1.6">')
    sdf.append('  <world name="colorful_obstacle_world">')
    
    sdf.append('    <include><uri>model://ground_plane</uri></include>')
    sdf.append('    <include><uri>model://sun</uri></include>')
    
    sdf.append('''
    <scene>
      <shadows>false</shadows>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.7 0.9 1.0 1</background>
    </scene>
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>
    ''')
    
    # Generate 100 unique colors and textures
    def generate_unique_colors(n):
        colors = []
        for i in range(n):
            hue = (i * 360 / n) % 360
            h = hue / 60
            x = 1 - abs(h % 2 - 1)
            if h < 1: r, g, b = 1, x, 0
            elif h < 2: r, g, b = x, 1, 0
            elif h < 3: r, g, b = 0, 1, x
            elif h < 4: r, g, b = 0, x, 1
            elif h < 5: r, g, b = x, 0, 1
            else: r, g, b = 1, 0, x
            colors.append((r, g, b, 1.0))
        return colors
    
    # Valid Gazebo textures
    textures = [
        'Gazebo/Red', 'Gazebo/Green', 'Gazebo/Blue', 'Gazebo/Yellow',
        'Gazebo/Purple', 'Gazebo/Orange', 'Gazebo/Turquoise',
        'Gazebo/Grey', 'Gazebo/DarkGrey',
        'Gazebo/Wood', 'Gazebo/WoodFloor', 'Gazebo/CeilingTiled', 'Gazebo/PaintedWall',
        'Gazebo/BuildingFrame', 'Gazebo/Bricks'
    ]
    
    unique_colors = generate_unique_colors(85)  # 85 colors
    # Total: 85 colors + 15 textures = 100 unique appearances
    
    wall_height = 2.0
    wall_thickness = 0.2
    
    # Black walls
    for name, pose in [('north', '15 0'), ('south', '-15 0'), ('east', '0 15'), ('west', '0 -15')]:
        size = f'{wall_thickness} 30' if 'north' in name or 'south' in name else f'30 {wall_thickness}'
        sdf.append(f'''
    <model name="wall_{name}">
      <static>true</static>
      <pose>{pose} {wall_height/2} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{size} {wall_height}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{size} {wall_height}</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Black</name></script></material>
        </visual>
      </link>
    </model>''')
    
    obstacle_count = 0
    shapes = ['box', 'cylinder', 'sphere']
    placed_obstacles = []
    
    attempts = 0
    while obstacle_count < 100 and attempts < 500:
        attempts += 1
        x = random.uniform(-12, 12)
        y = random.uniform(-12, 12)
        
        if abs(x) < 2 and abs(y) < 2:
            continue
        
        shape = random.choice(shapes)
        height = random.uniform(0.5, 2.5)
        
        if shape == 'box':
            w = random.uniform(0.3, 1.5)
            d = random.uniform(0.3, 1.5)
            radius = max(w, d) / 2
        elif shape == 'cylinder':
            radius = random.uniform(0.2, 0.8)
        else:
            radius = random.uniform(0.2, 0.8)
        
        collision = False
        for ox, oy, orad in placed_obstacles:
            dist = ((x - ox)**2 + (y - oy)**2)**0.5
            if dist < (radius + orad + 0.3):
                collision = True
                break
        
        if collision:
            continue
        
        placed_obstacles.append((x, y, radius))
        
        # Use texture for first 15, colors for rest
        if obstacle_count < 15:
            material_str = f'<material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{textures[obstacle_count]}</name></script></material>'
        else:
            color = unique_colors[obstacle_count - 15]
            material_str = f'<material><ambient>{color[0]} {color[1]} {color[2]} {color[3]}</ambient><diffuse>{color[0]} {color[1]} {color[2]} {color[3]}</diffuse></material>'
        
        if shape == 'box':
            sdf.append(f'''
    <model name="obstacle_{obstacle_count}">
      <static>true</static>
      <pose>{x} {y} {height/2} 0 0 {random.uniform(0, 3.14)}</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{w} {d} {height}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{w} {d} {height}</size></box></geometry>
          {material_str}
        </visual>
      </link>
    </model>''')
        elif shape == 'cylinder':
            sdf.append(f'''
    <model name="obstacle_{obstacle_count}">
      <static>true</static>
      <pose>{x} {y} {height/2} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry></collision>
        <visual name="v">
          <geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>
          {material_str}
        </visual>
      </link>
    </model>''')
        else:
            sdf.append(f'''
    <model name="obstacle_{obstacle_count}">
      <static>true</static>
      <pose>{x} {y} {radius} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><sphere><radius>{radius}</radius></sphere></geometry></collision>
        <visual name="v">
          <geometry><sphere><radius>{radius}</radius></sphere></geometry>
          {material_str}
        </visual>
      </link>
    </model>''')
        
        obstacle_count += 1
    
    sdf.append('  </world>')
    sdf.append('</sdf>')
    
    return '\n'.join(sdf)

if __name__ == "__main__":
    content = generate_colorful_world()
    with open(os.path.join(PACKAGE_DIR, 'worlds', 'colorful_obstacles.world'), 'w') as f:
        f.write(content)
    print(f"Colorful obstacle world generated!")
