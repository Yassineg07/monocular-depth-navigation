import os
import random

# Get the script's directory to build relative paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)  # my_robot package root

def generate_maze_sdf(width=30, height=30, cell_size=2.0, wall_height=2.0, wall_thickness=0.2):
    rows = int(height / cell_size)
    cols = int(width / cell_size)
    
    # Initialize grid with all walls present
    # Vertical walls: (rows) x (cols + 1)
    # Horizontal walls: (rows + 1) x (cols)
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

    # Recursive Backtracker
    stack = [(0, 0)]
    visited[0][0] = True
    
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
    sdf.append('  <world name="labyrinth_world">')
    
    # Standard includes
    sdf.append('    <include><uri>model://ground_plane</uri></include>')
    sdf.append('    <include><uri>model://sun</uri></include>')
    
    # Scene & Physics
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

    # Offset to place maze in front of robot (x=2)
    x_offset = 2.0
    y_offset = -width / 2.0 # Center Y around 0
    
    wall_count = 0
    materials = ['Gazebo/Wood', 'Gazebo/WoodFloor']
    
    def add_wall(name, x, y, w, h, length, orientation):
        # orientation: 0 for horizontal (along X), 1 for vertical (along Y)
        # Actually in Gazebo:
        # Box size x, y, z.
        # If horizontal wall in grid (separating rows), it runs along Y axis? No.
        # Let's map:
        # Grid Rows increase along X axis (Front)
        # Grid Cols increase along Y axis (Left)
        
        # Map grid (r, c) to world (x, y)
        # r=0 is x=0 (relative to maze start), c=0 is y=0 (relative to maze start)
        
        # Horizontal walls (separating rows r and r+1) -> Run along Y axis
        # Vertical walls (separating cols c and c+1) -> Run along X axis
        
        # Wait, standard maze terminology:
        # Horizontal wall: ----------------
        # Vertical wall:   |
        
        # Let's stick to:
        # r maps to X, c maps to Y.
        # hor_walls[r][c] is the wall "above" cell (r,c) in grid terms (lower X?)
        # Let's say r=0 is X=0. r increases as X increases.
        # hor_walls[r][c] is the wall at X = r * cell_size. It spans from Y = c*cell_size to (c+1)*cell_size.
        # ver_walls[r][c] is the wall at Y = c * cell_size. It spans from X = r*cell_size to (r+1)*cell_size.
        
        pass

    # Generate Horizontal Walls (Constant X, vary Y)
    for r in range(rows + 1):
        for c in range(cols):
            if hor_walls[r][c]:
                # Skip entrance (Start) and exit (End)
                if r == 0 and c == int(cols/2): continue # Entrance
                if r == rows and c == int(cols/2): continue # Exit
                
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

    # Generate Vertical Walls (Constant Y, vary X)
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

    sdf.append('  </world>')
    sdf.append('</sdf>')
    
    return '\n'.join(sdf)

if __name__ == "__main__":
    content = generate_maze_sdf(width=30, height=30, cell_size=2.0)
    with open(os.path.join(PACKAGE_DIR, 'worlds', 'my_maze.world'), 'w') as f:
        f.write(content)
