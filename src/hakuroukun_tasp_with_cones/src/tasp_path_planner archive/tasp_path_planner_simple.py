#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import OccupancyGrid

# ============================================================
#      TASP SIMPLE: PERIMETER → INTERIOR SWEEP → RETURN
#   Fully corrected, safe indexing, polygon closure (F2)
# ============================================================

class TASPPathPlannerSimple:
    def __init__(self):
        # ------------------ PARAMETERS ------------------
        self.tasp_cell = rospy.get_param("tasp_cell_size", 0.5)
        self.sweep_half = rospy.get_param("tasp_sweep_half_width", 10.0)

        # Costmap interpretation (compatible with ROS costmap_2d)
        self.occ_free = 0
        self.occ_inflated_max = 30     # treat <30 as free-ish
        self.occ_obstacle = 100
        self.occ_unknown = 255         # 255 or -1 both mean unknown

        # ------------------ INTERNAL STATE ------------------
        self.mode = "PERIMETER"
        self.perimeter_pts = []
        self.interior_pts = []
        self.return_pt = None
        self.goal_index = 0
        self.start_pose = None

    # ============================================================
    #                 MAIN ENTRY POINT
    # ============================================================
    def tasp_path_planning(self, current_pose, costmap, start_pose):
        if self.start_pose is None:
            self.start_pose = start_pose

        if self.mode == "PERIMETER":
            return self.run_perimeter(current_pose, costmap)

        elif self.mode == "INTERIOR":
            return self.run_interior(current_pose, costmap)

        elif self.mode == "RETURN":
            return self.run_return(current_pose)

        return None

    # ============================================================
    #                    PERIMETER FUNCTIONS
    # ============================================================
    def run_perimeter(self, current_pose, costmap):
        # Build perimeter only once
        if not self.perimeter_pts:
            rospy.loginfo("[TASP] Extracting perimeter...")
            self.perimeter_pts = self.build_perimeter(costmap, self.start_pose)

            if not self.perimeter_pts:
                rospy.logerr("[TASP][PERIMETER] No contour extracted!")
                return None

            rospy.loginfo(f"[TASP][PERIMETER] Extracted {len(self.perimeter_pts)} pts")
            self.goal_index = 0

        # Done perimeter → switch
        if self.goal_index >= len(self.perimeter_pts):
            rospy.loginfo("[TASP] Perimeter complete → INTERIOR")
            self.mode = "INTERIOR"
            return self.tasp_path_planning(current_pose, costmap, self.start_pose)

        gx, gy = self.perimeter_pts[self.goal_index]

        if self.reached(current_pose, [gx, gy], 0.6):
            self.goal_index += 1

        return [gx, gy]

    # ============================================================
    #                    INTERIOR FUNCTIONS
    # ============================================================
    def run_interior(self, current_pose, costmap):
        if not self.interior_pts:
            rospy.loginfo("[TASP] Generating interior sweep...")
            self.interior_pts = self.build_interior(costmap)

            if not self.interior_pts:
                rospy.logwarn("[TASP][INTERIOR] No interior points found → RETURN")
                self.mode = "RETURN"
                return self.tasp_path_planning(current_pose, costmap, self.start_pose)

            rospy.loginfo(f"[TASP][INTERIOR] Loaded {len(self.interior_pts)} points")
            self.goal_index = 0

        if self.goal_index >= len(self.interior_pts):
            rospy.loginfo("[TASP] Interior sweep complete → RETURN")
            self.mode = "RETURN"
            return self.tasp_path_planning(current_pose, costmap, self.start_pose)

        gx, gy = self.interior_pts[self.goal_index]

        if self.reached(current_pose, [gx, gy], 0.5):
            self.goal_index += 1

        return [gx, gy]

    # ============================================================
    #                     RETURN FUNCTIONS
    # ============================================================
    def run_return(self, current_pose):
        if self.return_pt is None:
            self.return_pt = [self.start_pose[0], self.start_pose[1]]

        if self.reached(current_pose, self.return_pt, 0.8):
            rospy.loginfo("[TASP] Returned to start. Coverage complete.")
            return None

        return self.return_pt

    # ============================================================
    #               PERIMETER EXTRACTION (SAFE)
    # ============================================================
    def build_perimeter(self, costmap, start_pose):
        sx, sy = start_pose[0], start_pose[1]
        half = self.sweep_half

        grid, wx0, wy0, res = self.build_local_grid(costmap, sx, sy, half)
        if grid is None:
            return []

        contour = self.trace_contour(grid)
        if not contour:
            return []

        world_pts = []
        for (ix, iy) in contour:
            wx = wx0 + ix * res
            wy = wy0 + iy * res
            world_pts.append([wx, wy])

        # Close polygon
        if world_pts[0] != world_pts[-1]:
            world_pts.append(world_pts[0])

        return self.resample_path(world_pts, self.tasp_cell)

    # ============================================================
    #                INTERIOR SWEEP GENERATION
    # ============================================================
    def build_interior(self, costmap):
        if not self.perimeter_pts:
            return []

        xs = [p[0] for p in self.perimeter_pts]
        ys = [p[1] for p in self.perimeter_pts]
        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)

        pts = []
        xvals = self.frange(minx, maxx, self.tasp_cell)
        yvals = self.frange(miny, maxy, self.tasp_cell)

        poly = self.perimeter_pts

        for i, x in enumerate(xvals):
            col = yvals if (i % 2 == 0) else reversed(yvals)
            for y in col:
                if self.point_in_poly(x, y, poly):
                    occ = self.occ_value(costmap, x, y)
                    if occ <= self.occ_inflated_max:  # free or lightly inflated
                        pts.append([x, y])

        return pts

    # ============================================================
    #                SAFE LOCAL GRID BUILDER
    # ============================================================
    def build_local_grid(self, costmap, sx, sy, half):
        res = costmap.info.resolution
        ox = costmap.info.origin.position.x
        oy = costmap.info.origin.position.y
        width = costmap.info.width
        height = costmap.info.height

        wx0 = sx - half
        wy0 = sy - half

        ix0 = int((wx0 - ox) / res)
        iy0 = int((wy0 - oy) / res)

        size = int((2.0 * half) / res)

        grid = []
        for r in range(size):
            row = []
            gy = iy0 + r

            for c in range(size):
                gx = ix0 + c

                if gx < 0 or gy < 0 or gx >= width or gy >= height:
                    row.append(self.occ_obstacle)
                else:
                    idx = gy * width + gx
                    v = costmap.data[idx]

                    if v < 0:
                        v = self.occ_unknown

                    row.append(v)

            grid.append(row)

        return grid, wx0, wy0, res

    # ============================================================
    #                 MOORE-NEIGHBOR CONTOUR TRACE
    # ============================================================
    def trace_contour(self, grid):
        h = len(grid)
        w = len(grid[0])
        start = None

        # find boundary (free with obstacle neighbor)
        for y in range(h):
            for x in range(w):
                if grid[y][x] <= self.occ_inflated_max:
                    if self.has_obstacle_neighbor(grid, x, y):
                        start = (x, y)
                        break
            if start:
                break

        if not start:
            return []

        contour = [start]
        dirs = [
            (1,0),(1,1),(0,1),(-1,1),
            (-1,0),(-1,-1),(0,-1),(1,-1)
        ]
        cur = start
        dir_index = 0

        for _ in range(6000):
            found = False
            for k in range(8):
                i = (dir_index + k) % 8
                nx = cur[0] + dirs[i][0]
                ny = cur[1] + dirs[i][1]

                if 0 <= nx < w and 0 <= ny < h:
                    if grid[ny][nx] <= self.occ_inflated_max and \
                       self.has_obstacle_neighbor(grid, nx, ny):
                        contour.append((nx, ny))
                        cur = (nx, ny)
                        dir_index = (i + 5) % 8
                        found = True
                        break

            if not found:
                break

            if len(contour) > 20 and contour[-1] == start:
                break

        return contour

    def has_obstacle_neighbor(self, grid, x, y):
        h = len(grid)
        w = len(grid[0])
        for dy in [-1,0,1]:
            for dx in [-1,0,1]:
                if dx == 0 and dy == 0:
                    continue
                nx = x + dx
                ny = y + dy
                if 0 <= nx < w and 0 <= ny < h:
                    if grid[ny][nx] >= self.occ_obstacle:
                        return True
        return False

    # ============================================================
    #                   PATH RESAMPLING
    # ============================================================
    def resample_path(self, pts, spacing):
        if not pts:
            return []
        new_pts = [pts[0]]
        acc = 0.0

        for i in range(1, len(pts)):
            prev = pts[i-1]
            cur = pts[i]
            d = math.hypot(cur[0]-prev[0], cur[1]-prev[1])
            acc += d
            if acc >= spacing:
                new_pts.append(cur)
                acc = 0.0
        return new_pts

    # ============================================================
    #              POINT-IN-POLYGON (RAY CASTING)
    # ============================================================
    def point_in_poly(self, x, y, poly):
        inside = False
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i+1) % n]
            if ((y1 > y) != (y2 > y)) and \
               (x < (x2-x1) * (y - y1) / (y2 - y1 + 1e-9) + x1):
                inside = not inside
        return inside

    # ============================================================
    #                   COSTMAP VALUE GETTER
    # ============================================================
    def occ_value(self, occ, x, y):
        ox = occ.info.origin.position.x
        oy = occ.info.origin.position.y
        r = occ.info.resolution
        gx = int((x - ox)/r)
        gy = int((y - oy)/r)

        if gx < 0 or gy < 0 or gx >= occ.info.width or gy >= occ.info.height:
            return self.occ_obstacle

        v = occ.data[gy * occ.info.width + gx]
        if v < 0:
            return self.occ_unknown
        return v

    # ============================================================
    #                     REACHED GOAL?
    # ============================================================
    def reached(self, pose, goal, tol):
        dx = pose[0] - goal[0]
        dy = pose[1] - goal[1]
        return (dx * dx + dy * dy) < (tol * tol)

    # ============================================================
    #                      RANGE UTILITY
    # ============================================================
    def frange(self, a, b, step):
        x = a
        out = []
        if a <= b:
            while x <= b:
                out.append(x)
                x += step
        else:
            while x >= b:
                out.append(x)
                x -= step
        return out
