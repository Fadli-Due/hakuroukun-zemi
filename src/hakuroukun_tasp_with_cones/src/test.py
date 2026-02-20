#!/usr/bin/env python3
from simple_astar import SimpleOccupancyGrid
from boustrophedon_planner import BoustrophedonPlanner

# 1. Create a fake 10x10 map
# 0 = Free, 100 = Wall
# Let's put a wall in the middle at x=5, y=5
data = [0] * 100 
width = 10
height = 10
res = 1.0
idx = 5 * width + 5 
data[idx] = 100 # Obstacle in middle

ogrid = SimpleOccupancyGrid(width, height, res, 0, 0, data)

# 2. Run Planner
planner = BoustrophedonPlanner(cleaning_width_meters=1.0)
path = planner.generate_waypoints(ogrid)

# 3. Print result
print("Generated Path:")
for p in path:
    print(p)