import os
import random

# Get the script's directory to build relative paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)  # my_robot package root

def generate_simple_maze_sdf(width=25, height=25, cell_size=2.5, wall_height=2.0, wall_thickness=0.2):
    rows = int(height / cell_size)
    cols = int(width / cell_size)
    
    # Create maze with multiple paths and dead ends
    ver_walls = [[True for _ in range(cols + 1)] for _ in range(rows)]
    hor_walls = [[True for _ in range(cols)] for _ in range(rows + 1)]
    
    visited = [[False for _ in range(cols)] for _ in range(rows)]
    
    def get_neighbors(r, c):
        neighbors = []
        if r > 0: neighbors.append(('N', r - 1, c))
        if r < rows - 1: neighbors.append(('S', r + 1, c))
        if c > 0: neighbors.append(('W', r, c - 1))
        if c < cols - 1: neighbors.append(('E', r, c + 1))
        return neighbors

    # Recursive Backtracker for complex maze
    start_c = cols // 2
    stack = [(0, start_c)]
    visited[0][start_c] = True
    
    while stack:
        current_r, current_c = stack[-1]
        neighbors = get_neighbors(current_r, current_c)
        unvisited = [n for n in neighbors if not visited[n[1]][n[2]]]
        
        if unvisited:
            direction, next_r, next_c = random.choice(unvisited)
            
            # Remove wall
            if direction == 'N': hor_walls[current_r][current_c] = False
            elif direction == 'S': hor_walls[next_r][next_c] = False
            elif direction == 'W': ver_walls[current_r][current_c] = False
            elif direction == 'E': ver_walls[current_r][next_c] = False
            
            visited[next_r][next_c] = True
            stack.append((next_r, next_c))
        else:
            stack.pop()
    
    # Create SDF content
    sdf = []
    sdf.append('<?xml version="1.0"?>')
    sdf.append('<sdf version="1.6">')
    sdf.append('  <world name="simple_maze_world">')
    
    sdf.append('    <include><uri>model://ground_plane</uri></include>')
    sdf.append('    <include><uri>model://sun</uri></include>')
    
    sdf.append('''
    <scene>
      <shadows>false</shadows>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.8 0.8 0.8 1</background>
    </scene>
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>
    ''')

    x_offset = 3.5  # Move maze further right so robot doesn't spawn in wall
    y_offset = -width / 2.0
    
    wall_count = 0
    materials = ['Gazebo/Wood', 'Gazebo/WoodFloor']
    
    # Generate Horizontal Walls
    for r in range(rows + 1):
        for c in range(cols):
            if hor_walls[r][c]:
                # Skip entrance and exit
                if r == 0 and c == cols // 2: continue
                if r == rows and c == cols // 2: continue
                
                cx = x_offset + r * cell_size
                cy = y_offset + c * cell_size + cell_size/2
                
                material = materials[wall_count % 2]
                sdf.append(f'''
    <model name="h_wall_{wall_count}">
      <static>true</static>
      <pose>{cx} {cy} {wall_height/2} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{wall_thickness} {cell_size} {wall_height}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{wall_thickness} {cell_size} {wall_height}</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{material}</name></script></material>
        </visual>
      </link>
    </model>''')
                wall_count += 1

    # Generate Vertical Walls
    for r in range(rows):
        for c in range(cols + 1):
            if ver_walls[r][c]:
                cx = x_offset + r * cell_size + cell_size/2
                cy = y_offset + c * cell_size
                
                material = materials[wall_count % 2]
                sdf.append(f'''
    <model name="v_wall_{wall_count}">
      <static>true</static>
      <pose>{cx} {cy} {wall_height/2} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{cell_size} {wall_thickness} {wall_height}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{cell_size} {wall_thickness} {wall_height}</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{material}</name></script></material>
        </visual>
      </link>
    </model>''')
                wall_count += 1
    
    # Add entrance cage walls (outside maze at start)
    entrance_x = x_offset - cell_size
    entrance_y = y_offset + start_c * cell_size + cell_size/2
    
    # Left wall of entrance cage
    sdf.append(f'''
    <model name="entrance_left">
      <static>true</static>
      <pose>{entrance_x + cell_size/2} {entrance_y - cell_size/2} {wall_height/2} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{cell_size} {wall_thickness} {wall_height}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{cell_size} {wall_thickness} {wall_height}</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Wood</name></script></material>
        </visual>
      </link>
    </model>''')
    
    # Right wall of entrance cage
    sdf.append(f'''
    <model name="entrance_right">
      <static>true</static>
      <pose>{entrance_x + cell_size/2} {entrance_y + cell_size/2} {wall_height/2} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{cell_size} {wall_thickness} {wall_height}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{cell_size} {wall_thickness} {wall_height}</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Wood</name></script></material>
        </visual>
      </link>
    </model>''')
    
    # Back wall of entrance cage
    sdf.append(f'''
    <model name="entrance_back">
      <static>true</static>
      <pose>{entrance_x} {entrance_y} {wall_height/2} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{wall_thickness} {cell_size} {wall_height}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{wall_thickness} {cell_size} {wall_height}</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Wood</name></script></material>
        </visual>
      </link>
    </model>''')
    
    # Add exit cage walls (outside maze at end)
    exit_x = x_offset + rows * cell_size
    exit_y = y_offset + (cols // 2) * cell_size + cell_size/2
    
    # Left wall of exit cage
    sdf.append(f'''
    <model name="exit_left">
      <static>true</static>
      <pose>{exit_x + cell_size/2} {exit_y - cell_size/2} {wall_height/2} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{cell_size} {wall_thickness} {wall_height}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{cell_size} {wall_thickness} {wall_height}</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Wood</name></script></material>
        </visual>
      </link>
    </model>''')
    
    # Right wall of exit cage
    sdf.append(f'''
    <model name="exit_right">
      <static>true</static>
      <pose>{exit_x + cell_size/2} {exit_y + cell_size/2} {wall_height/2} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{cell_size} {wall_thickness} {wall_height}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{cell_size} {wall_thickness} {wall_height}</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Wood</name></script></material>
        </visual>
      </link>
    </model>''')
    
    # Back wall of exit cage
    sdf.append(f'''
    <model name="exit_back">
      <static>true</static>
      <pose>{exit_x + cell_size} {exit_y} {wall_height/2} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{wall_thickness} {cell_size} {wall_height}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{wall_thickness} {cell_size} {wall_height}</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Wood</name></script></material>
        </visual>
      </link>
    </model>''')

    sdf.append('  </world>')
    sdf.append('</sdf>')
    
    return '\n'.join(sdf)

if __name__ == "__main__":
    content = generate_simple_maze_sdf(width=25, height=25, cell_size=2.5)
    with open(os.path.join(PACKAGE_DIR, 'worlds', 'simple_maze.world'), 'w') as f:
        f.write(content)
    print("Simple maze generated: simple_maze.world")
