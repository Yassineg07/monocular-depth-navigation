#!/usr/bin/env python3
import os
import xml.etree.ElementTree as ET
from PIL import Image, ImageDraw

# Get the script's directory to build relative paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)  # my_robot package root

# Parse the world file
tree = ET.parse(os.path.join(PACKAGE_DIR, 'worlds', 'my_maze.world'))
root = tree.getroot()

# Map parameters
resolution = 0.05
map_width_px = 700
map_height_px = 680

# Create white image
img = Image.new('L', (map_width_px, map_height_px), 254)
draw = ImageDraw.Draw(img)

# Origin offset
origin_x = 0.0
origin_y = -17.0

def world_to_pixel(x, y):
    px = int((x - origin_x) / resolution)
    py = int((y - origin_y) / resolution)
    return px, map_height_px - py

# Parse all wall models
for model in root.findall('.//model'):
    pose_elem = model.find('pose')
    if pose_elem is None:
        continue
    
    pose = list(map(float, pose_elem.text.split()))
    cx, cy, cz = pose[0], pose[1], pose[2]
    
    # Get box size
    box = model.find('.//box/size')
    if box is None:
        continue
    
    size = list(map(float, box.text.split()))
    sx, sy, sz = size[0], size[1], size[2]
    
    # Draw wall
    x1, y1 = world_to_pixel(cx - sx/2, cy - sy/2)
    x2, y2 = world_to_pixel(cx + sx/2, cy + sy/2)
    draw.rectangle([x1, y1, x2, y2], fill=0)

img.save(os.path.join(PACKAGE_DIR, 'maps', 'world_reference.pgm'))
print(f"Map created from world file: {map_width_px}x{map_height_px}")
