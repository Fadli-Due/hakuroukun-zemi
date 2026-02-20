#!/usr/bin/env python3
import yaml
import cv2
import numpy as np
import os
from simple_astar import SimpleOccupancyGrid, world_to_grid
from boustrophedon_planner import BoustrophedonPlanner

def verify_on_real_map():
    # 1. Load the Map Metadata (YAML)
    yaml_file = "saved_map.yaml"
    
    if not os.path.exists(yaml_file):
        print(f"Error: Could not find {yaml_file}")
        return

    with open(yaml_file, 'r') as f:
        map_info = yaml.safe_load(f)

    image_file = map_info['image']
    resolution = map_info['resolution']
    origin = map_info['origin'] # [x, y, z]
    origin_x = origin[0]
    origin_y = origin[1]

    print(f"Loaded Map: Res={resolution}, Origin=({origin_x}, {origin_y})")

    # 2. Load the Map Image (PGM) using OpenCV
    # standard ROS maps: White(255)=Free, Black(0)=Occupied, Gray(205)=Unknown
    img = cv2.imread(image_file, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Error: Could not find image file {image_file}")
        return

    height, width = img.shape
    
    # 3. Convert Image to SimpleOccupancyGrid Format
    # The planner expects a 1D list where 0=Free, 100=Occupied
    # Note: ROS map images are often flipped vertically relative to grid coordinates, 
    # but let's assume standard formatting for now.
    
    # Flatten the image logic: 
    # If pixel < 230 (not white), treat as occupied for safety
    flat_data = []
    # ROS maps usually interpret (0,0) as bottom-left, but images are top-left.
    # To align with our simple_astar logic which assumes (0,0) index is bottom-left:
    # We need to read the image from bottom row to top row.
    for r in range(height - 1, -1, -1):
        row_data = img[r, :]
        for pixel in row_data:
            if pixel > 230: 
                flat_data.append(0) # Free
            else:
                flat_data.append(100) # Occupied

    ogrid = SimpleOccupancyGrid(width, height, resolution, origin_x, origin_y, flat_data)

    # 4. Run the Planner
    # Let's set cleaning width to 1.0 meter (adjust as needed)
    clean_width = 1.0 
    planner = BoustrophedonPlanner(cleaning_width_meters=clean_width)
    
    print("Generating path... (this might take a second)")
    waypoints = planner.generate_waypoints(ogrid)
    print(f"Generated {len(waypoints)} waypoints.")

    # 5. Draw the Path on the Image for Visualization
    # We need to convert World Coords back to Pixel Coords to draw
    vis_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) # Convert to Color for drawing red lines
    
    for i in range(len(waypoints) - 1):
        p1 = waypoints[i]
        p2 = waypoints[i+1]
        
        # Convert World -> Grid
        g1 = world_to_grid(ogrid, p1[0], p1[1])
        g2 = world_to_grid(ogrid, p2[0], p2[1])
        
        if g1 and g2:
            # Convert Grid -> Image Pixels
            # Remember we flipped rows earlier. Image (0,0) is Top-Left. 
            # Grid (0,0) is Bottom-Left.
            # Pixel Y = height - 1 - Grid Y
            px1, py1 = g1[0], height - 1 - g1[1]
            px2, py2 = g2[0], height - 1 - g2[1]
            
            # Draw Line (Blue, thickness 2)
            cv2.line(vis_img, (px1, py1), (px2, py2), (0, 0, 255), 2)
            
            # Draw Dots at waypoints (Green)
            cv2.circle(vis_img, (px1, py1), 3, (0, 255, 0), -1)

    # 6. Save the Result
    output_filename = "coverage_preview.png"
    cv2.imwrite(output_filename, vis_img)
    print(f"Success! Check the file '{output_filename}' to see the path.")

if __name__ == "__main__":
    verify_on_real_map()