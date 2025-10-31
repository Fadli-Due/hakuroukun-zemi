#!/usr/bin/env python3
import math
import rospy

class TASPPathPlanner:
    def __init__(self):
        """
        Initialize the TASPPathPlanner with configurable parameters.

        Args:
            self.tasp_cell_size (float): Step size when exploring neighboring cells (meters).
            threshold_distance (float): Threshold for removing points near the chosen goal from BTP (meters).
            sampling_resolution (float): Resolution for checking area occupancy (meters).
        """
        # keep one source of truth for cell size (float)
        self.tasp_cell_size = rospy.get_param("tasp_cell_size", 1.0)
        self.sampling_resolution = rospy.get_param("sampling_resolution", 0.1)
        self.remove_btp_threshold_distance = rospy.get_param("remove_btp_threshold_distance", 0.5)
        self.max_allowed_distance = rospy.get_param("range_max", 6.0)
        self.inflated_tasp_cell = rospy.get_param("inflated_tasp_cell", 3.0)

        # BackTracking Points
        self.BTP = []
        # TASP trajectory (a list of [x, y] waypoints)
        self.TASPtrajectory = []
        self.goto_closest_BTP = False
        self.is_initial_position = True

        # ---- Human-like lane behavior params ----
        self.pattern_mode = rospy.get_param("pattern/mode", "lanes_then_edges")  # or "edges_then_lanes"
        self.heading_keep_window = rospy.get_param("pattern/heading_keep_window_deg", 20.0)

        # Scoring weights (higher = stronger effect)
        self.w_length      = rospy.get_param("weights/length", 1.2)        # reward longer straight run
        self.w_heading     = rospy.get_param("weights/heading", 0.9)       # penalize turning away from current heading
        self.w_lane_change = rospy.get_param("weights/lane_change", 1.1)   # small penalty to change direction class
        self.w_clearance   = rospy.get_param("weights/clearance", 0.5)     # prefer more free space
        self.w_edge        = rospy.get_param("weights/edge", 0.6)          # only used if edges-first
        self.edge_dist_m   = rospy.get_param("weights/edge_distance_target", 0.35)  # ideal offset from walls (m)


    def tasp_path_planning(self, current_pose, costmap, start_pose):
        """
        Main TASP function that determines a new TASP goal.
        Uses a lane/edge-aware scorer to prefer long straight strokes.
        """
        # Initialize trajectory with (current -> start) the first time
        if len(self.TASPtrajectory) < 2:
            cx, cy, cth = current_pose  # (x, y, theta)
            prev = [cx - math.cos(cth) * self.tasp_cell_size,
                    cy - math.sin(cth) * self.tasp_cell_size]
            cur  = [cx, cy]
            self.TASPtrajectory = [prev, cur]

        TASPcurrentPos = self.TASPtrajectory[-1]
        TASPprevPos    = self.TASPtrajectory[-2]

        free_cells = self.get_free_cells(TASPcurrentPos, costmap, self.TASPtrajectory)

        # Maintain BTP list for dead-ends
        self.update_BTP(free_cells)

        # ---------- Decision ----------
        if not free_cells:
            # No forward options -> go to closest BTP (backtracking)
            if not self.BTP:
                rospy.logwarn("[TASP] No BTP available. Stopping the robot.")
                return None

            next_goal = self.find_closest_BTP(TASPcurrentPos, self.BTP, start_pose)
            if next_goal is None:
                rospy.logwarn("[TASP] No valid path to BTP. Stopping the robot.")
                return None
            self.TASPtrajectory.append(next_goal)

        elif len(free_cells) == 1:
            # Only one choice -> take it
            self.TASPtrajectory.append(free_cells[0])

        else:
            # Multiple choices -> human-like lane/edge-aware selection
            best_cell, dbg = self.score_candidates(
                free_cells, TASPcurrentPos, TASPprevPos, costmap
            )
            rospy.loginfo_throttle(1.0, f"[TASP] chosen {best_cell} | dbg={dbg.get(tuple(best_cell), {})}")
            self.goto_closest_BTP = False
            self.TASPtrajectory.append(best_cell)

        # ---------- Housekeeping ----------
        newly_chosen = self.TASPtrajectory[-1]
        # Remove nearby BTP so we don't bounce back immediately
        for btp_point in list(self.BTP):
            if self.euclidean_distance(newly_chosen, btp_point) < self.remove_btp_threshold_distance:
                self.BTP.remove(btp_point)
                break

        return self.TASPtrajectory[-1]

    
    # --------------------------------------------------------------------------
    # HELPER FUNCTIONS
    # --------------------------------------------------------------------------
    def get_front_cell(self, TASPcurrentPos, TASPprevPos):
        """
        Return the cell directly in front of the robot (based on vector from prev_pos -> current_pos).
        """
        dx = TASPcurrentPos[0] - TASPprevPos[0]
        dy = TASPcurrentPos[1] - TASPprevPos[1]
        length = math.hypot(dx, dy)
        if self.goto_closest_BTP:
            return None
        elif length < 1e-1:
            # If no movement, pick an arbitrary 'front cell'
            return [TASPcurrentPos[0] + self.tasp_cell_size, TASPcurrentPos[1]]
        else:
            # Unit direction
            ux = dx / length
            uy = dy / length
            return [
                TASPcurrentPos[0] + ux * self.tasp_cell_size,
                TASPcurrentPos[1] + uy * self.tasp_cell_size
            ]

    def get_free_cells(self, position, costmap, TASPtrajectory):
        px, py = position
        neighbors = [
            [px - self.tasp_cell_size, py],
            [px + self.tasp_cell_size, py],
            [px, py - self.tasp_cell_size],
            [px, py + self.tasp_cell_size]
        ]

        free_cells = []
        for cell in neighbors:
            if self.is_area_free(cell, costmap):
                if not self.list_has_row(TASPtrajectory, cell):
                    free_cells.append(cell)

        return free_cells
    
    def find_front_cell_with_min_rotation(self, free_cells, front_cell, current_pose):
        """
        Find the front cell from free_cells that requires minimal rotation from the robot's current orientation.
        """
        current_x, current_y, current_theta = current_pose
        min_rotation = float('inf')
        best_cell = None

        for cell in free_cells:
            dx = cell[0] - current_x
            dy = cell[1] - current_y
            angle_to_cell = math.atan2(dy, dx)
            angular_diff = abs(self.normalize_angle(angle_to_cell - current_theta))
            if angular_diff < min_rotation:
                min_rotation = angular_diff
                best_cell = cell

        rospy.loginfo_throttle(1.0, f"[TASP] Front cell {best_cell}, rotation Δ={min_rotation:.2f}")
        return best_cell if best_cell is not None else front_cell

    def normalize_angle(self, angle):
        """
        Normalize an angle to the range [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def is_area_free(self, center_cell, costmap):
        half_size = self.inflated_tasp_cell * self.tasp_cell_size / 2.0
        x_min = center_cell[0] - half_size
        x_max = center_cell[0] + half_size
        y_min = center_cell[1] - half_size
        y_max = center_cell[1] + half_size

        xs = self.frange(x_min, x_max, self.sampling_resolution)
        ys = self.frange(y_min, y_max, self.sampling_resolution)

        for x_s in xs:
            for y_s in ys:
                occ_val = self.get_occupancy_value(costmap, x_s, y_s)
                if occ_val >= 50:
                    return False
        return True

    def update_BTP(self, free_cells):
        for cell in free_cells:
            if self.is_initial_position and len(self.TASPtrajectory) >= 2:
                cur = self.TASPtrajectory[-1]
                prev = self.TASPtrajectory[-2]
                fwd = (cur[0]-prev[0], cur[1]-prev[1])
                vec = (cell[0]-cur[0], cell[1]-cur[1])
                # if behind, skip once at startup
                if (fwd[0]*vec[0] + fwd[1]*vec[1]) < 0.0:
                    self.is_initial_position = False
                    continue
            if not self.list_has_row(self.BTP, cell):
                self.BTP.append(cell)

    def find_closest_BTP(self, current_pos, BTP, start_pose):
        """
        For each point in BTP, pick the closest (Euclidean). 
        If tie => pick one that is farthest from start_pose.
        """
        if not BTP:
            return None
        min_dist = float('inf')
        chosen = None
        for point in BTP:
            dist = self.euclidean_distance(current_pos, point)
            if dist < min_dist:
                min_dist = dist
                chosen = point
            elif abs(dist - min_dist) < 1e-1:
                # Tie => pick one farther from start_pose
                if (self.euclidean_distance(point, [start_pose[0], start_pose[1]]) >
                    self.euclidean_distance(chosen, [start_pose[0], start_pose[1]])):
                    chosen = point
                    self.goto_closest_BTP = True
        return chosen

    def is_away_from_start(self, cell, start_pose, current_pos):
        """
        Return True if 'cell' is farther from start than 'current_pos'.
        """
        dist_cell_to_start = self.euclidean_distance(cell, [start_pose[0], start_pose[1]])
        dist_current_to_start = self.euclidean_distance(current_pos, [start_pose[0], start_pose[1]])
        return dist_cell_to_start > dist_current_to_start
    
    def get_distance_to_obstacle(self, current_pos, cell, costmap):
        """
        From 'cell' in direction from 'current_pos'->'cell', step by self.tasp_cell_size
        until occupancy = 100 occupied. Return total distance.
        """
        dx = cell[0] - current_pos[0]
        dy = cell[1] - current_pos[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return 0.0
        ux = dx / length
        uy = dy / length

        distance = 0.0
        current_x, current_y = cell[0], cell[1]

        while True:
            next_x = current_x + ux * self.tasp_cell_size
            next_y = current_y + uy * self.tasp_cell_size
            occ_val = self.get_occupancy_value(costmap, next_x, next_y)
            if occ_val == 100:
                break
            distance += self.tasp_cell_size
            current_x, current_y = next_x, next_y

            if distance > self.max_allowed_distance:
                break

        return distance

    def current_heading(self, prev_pos, cur_pos):
        dx, dy = cur_pos[0] - prev_pos[0], cur_pos[1] - prev_pos[1]
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return None  # unknown yet
        return math.atan2(dy, dx)

    def heading_delta(self, h1, h2):
        if h1 is None or h2 is None:
            return 0.0
        d = self.normalize_angle(h2 - h1)
        return abs(d)

    def estimate_straight_run(self, from_pos, toward_cell, costmap, max_m=10.0):
        """
        Estimate how far we can keep going in the direction from 'from_pos' -> 'toward_cell'
        before hitting occupied/unknown. Step by tasp_cell_size.
        """
        dx = toward_cell[0] - from_pos[0]
        dy = toward_cell[1] - from_pos[1]
        L  = math.hypot(dx, dy)
        if L < 1e-6:
            return 0.0
        ux, uy = dx/L, dy/L
        run = 0.0
        x, y = toward_cell[0], toward_cell[1]
        step = self.tasp_cell_size
        while run < max_m:
            x += ux*step; y += uy*step
            occ = self.get_occupancy_value(costmap, x, y)
            if occ >= 50:  # unknown or occupied -> stop (edges “later” pass will clean)
                break
            run += step
        return run

    def approx_edge_distance(self, cell, costmap, samples=8):
        """
        Approximate distance (m) from 'cell' to nearest obstacle by radial probing.
        Used when pattern_mode == 'edges_then_lanes' to reward perimeter sweeping.
        """
        res  = costmap.info.resolution
        rmax = 1.5
        dmin = float('inf')
        for i in range(samples):
            th = 2.0 * math.pi * i / samples
            rr = res
            while rr <= rmax:
                xx = cell[0] + rr * math.cos(th)
                yy = cell[1] + rr * math.sin(th)
                if self.get_occupancy_value(costmap, xx, yy) >= 50:  # unknown/occupied
                    dmin = min(dmin, rr)
                    break
                rr += res
        return dmin if dmin != float('inf') else rmax


    def score_candidates(self, free_cells, TASPcurrentPos, TASPprevPos, costmap):
        """
        Returns (best_cell, debug_dict). Uses lane-style scoring:
        - reward long straight runs
        - penalize turning away from current heading
        - small penalty for 'direction class' switch (e.g., horizontal<->vertical)
        - prefer reasonable clearance (center-ish in aisles)
        - if pattern is 'edges_then_lanes', invert: reward being near the edge first
        """
        cur_heading = self.current_heading(TASPprevPos, TASPcurrentPos)
        best_cell, best_score = None, -1e18
        dbg = {}

        fwdx = TASPcurrentPos[0] - TASPprevPos[0]
        fwdy = TASPcurrentPos[1] - TASPprevPos[1]
        fwdL = math.hypot(fwdx, fwdy)
        if fwdL < 1e-6:
            # fallback: face +x (should not happen after the seeding above)
            fwdx, fwdy, fwdL = 1.0, 0.0, 1.0
        ux_fwd, uy_fwd = fwdx / fwdL, fwdy / fwdL

        for c in free_cells:
            # 1) Heading to candidate
            dx, dy = c[0] - TASPcurrentPos[0], c[1] - TASPcurrentPos[1]
            cand_heading = math.atan2(dy, dx)
            dtheta = self.heading_delta(cur_heading, cand_heading)
            # Is it roughly same direction as before?
            heading_keep_pen = (dtheta / math.radians(self.heading_keep_window))**2

            # 2) Estimate straight lane length from this candidate
            straight_len = self.estimate_straight_run(TASPcurrentPos, c, costmap, max_m=10.0)

            # 3) Simple “lane change” detector: horizontal vs vertical class
            # (since neighbors are axis-aligned, this is enough)
            if cur_heading is None:
                lane_change = 0
            else:
                was_horiz = abs(math.cos(cur_heading)) > abs(math.sin(cur_heading))
                now_horiz = abs(dx) > abs(dy)
                lane_change = 1 if (was_horiz != now_horiz) else 0

            # 4) Clearance proxy (bigger is nicer)
            clearance = self.get_distance_to_obstacle(TASPcurrentPos, c, costmap)

            # 5) Edge preference toggle
            edge_bonus = 0.0
            if self.pattern_mode != "edges_then_lanes":
                self.w_edge = 0.0

            vecx, vecy = c[0] - TASPcurrentPos[0], c[1] - TASPcurrentPos[1]
            dot = ux_fwd * vecx + uy_fwd * vecy

            if len(self.TASPtrajectory) <= 3 and dot < 0:
                score = -1e9  # effectively "not allowed" at startup
            else:
            # Final score: higher is better
                score = (
                    self.w_length * straight_len
                    - self.w_heading * heading_keep_pen
                    - self.w_lane_change * lane_change
                    + self.w_clearance * clearance
                    + self.w_edge * edge_bonus
                )

            dbg[tuple(c)] = {"len": straight_len, "dtheta": dtheta, "lane_change": lane_change,
                            "clear": clearance, "edge_bonus": edge_bonus, "score": score}
            if score > best_score:
                best_score, best_cell = score, c

        return best_cell, dbg

    # --------------------------------------------------------------------------
    # UTILITY / STUB FUNCTIONS
    # --------------------------------------------------------------------------
    def get_occupancy_value(self, costmap, x, y):
        """
        Return an integer in {0, 50, 100}:
          0   => free
          50  => unknown (-1 in OccupancyGrid)
          100 => occupied
        """
        gx, gy = self.world_to_grid(costmap, x, y)
        if (gx < 0 or gx >= costmap.info.width or
            gy < 0 or gy >= costmap.info.height):
            # Out of bounds => treat as occupied
            return 100
        
        idx = gy * costmap.info.width + gx
        val = costmap.data[idx]  # -1 (unknown), 0..100
        if val < 0:
            # unknown => treat as 50
            return 50
        return val  # 0 => free, 100 => occupied

    def world_to_grid(self, occupancy_grid, x, y):
        """
        Convert world coords to grid coords given occupancy_grid.info.
        """
        origin_x = occupancy_grid.info.origin.position.x
        origin_y = occupancy_grid.info.origin.position.y
        res = occupancy_grid.info.resolution
        gx = int((x - origin_x) / res)
        gy = int((y - origin_y) / res)
        return gx, gy

    def euclidean_distance(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def list_has_row(self, arr, element):
        """
        Check if arr (list of [x, y]) has an entry matching element (list [x, y]).
        """
        for row in arr:
            if (abs(row[0] - element[0]) < 1e-6 and
                abs(row[1] - element[1]) < 1e-6):
                return True
        return False

    def frange(self, start, stop, step):
        """
        Generate a list of float values from start to stop (inclusive) with a given step.
        """
        vals = []
        x = start
        # Use a small epsilon to avoid floating rounding issues
        epsilon = 1e-9
        while x <= stop + epsilon:
            vals.append(x)
            x += step
        return vals
    