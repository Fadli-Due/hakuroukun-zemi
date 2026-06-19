#!/usr/bin/env python3
import math
import rospy

class TASPPathPlanner:
    def __init__(self):
        """
        TASP path planner (rule-based, no weighted score).
        - Uses global coverage map
        - Uses BackTracking Points (BTP)
        - Local decisions via prioritized rules:
          1) unvisited > visited
          2) smaller heading change
          3) longer straight run
          4) larger clearance
        """

        # ---------- basic parameters ----------
        self.tasp_cell_size = rospy.get_param("tasp_cell_size", 1.0)
        self.sampling_resolution = rospy.get_param("sampling_resolution", 0.1)
        self.remove_btp_threshold_distance = rospy.get_param("remove_btp_threshold_distance", 0.5)
        self.max_allowed_distance = rospy.get_param("range_max", 6.0)
        self.inflated_tasp_cell = rospy.get_param("inflated_tasp_cell", 3.0)

        # Neighbor selection angle thresholds (degrees)
        self.forward_angle_deg = rospy.get_param("rules/forward_angle_deg", 35.0)
        self.heading_margin_deg = rospy.get_param("rules/heading_margin_deg", 10.0)

        # ---------- internal state ----------
        self.BTP = []                 # BackTracking Points
        self.TASPtrajectory = []      # list of [x, y]
        self.goto_closest_BTP = False
        self.is_initial_position = True

        # coverage grid (global)
        self.cover_res = self.tasp_cell_size
        self.cover_map = {}   # (ix, iy) -> 0/1

    # =================================================================
    #                         MAIN ENTRY POINT
    # =================================================================
    def tasp_path_planning(self, current_pose, costmap, start_pose):
        """
        Main TASP function: returns a new [x, y] goal or None.
        current_pose: (x, y, theta)
        start_pose:   (x0, y0, theta0)
        """

        # ---------- bootstrap trajectory (2 points) ----------
        if len(self.TASPtrajectory) < 2:
            cx, cy, cth = current_pose
            prev = [cx - math.cos(cth) * self.tasp_cell_size,
                    cy - math.sin(cth) * self.tasp_cell_size]
            cur  = [cx, cy]
            self.TASPtrajectory = [prev, cur]
            self.mark_covered(prev)
            self.mark_covered(cur)

        TASPcurrentPos = self.TASPtrajectory[-1]
        TASPprevPos    = self.TASPtrajectory[-2]

        # ---------- find neighbors & manage BTP ----------
        free_cells = self.get_free_cells(TASPcurrentPos, costmap, TASPprevPos)
        self.update_BTP(free_cells)

        # ---------- decision logic ----------
        if not free_cells:
            # no local options -> backtrack
            if not self.BTP:
                rospy.logwarn("[TASP] No free cells and no BTP. Stopping.")
                return None

            next_goal = self.find_closest_BTP(TASPcurrentPos, self.BTP, start_pose)
            if next_goal is None:
                rospy.logwarn("[TASP] Could not find a valid BTP goal. Stopping.")
                return None
            rospy.loginfo_throttle(1.0, f"[TASP] Backtracking to {next_goal}")

        elif len(free_cells) == 1:
            # exactly one choice
            next_goal = free_cells[0]
            rospy.loginfo_throttle(1.0, f"[TASP] Only one free cell -> {next_goal}")

        else:
            # multiple choices -> rule-based selection
            next_goal, dbg = self.choose_next_cell_rule_based(
                free_cells, TASPcurrentPos, TASPprevPos, costmap
            )
            rospy.loginfo_throttle(1.0, f"[TASP] chosen {next_goal} | dbg={dbg.get(tuple(next_goal), {})}")

        # ---------- housekeeping ----------
        self.TASPtrajectory.append(next_goal)
        self.mark_covered(next_goal)

        # remove nearby BTP to avoid bouncing
        for btp_point in list(self.BTP):
            if self.euclidean_distance(next_goal, btp_point) < self.remove_btp_threshold_distance:
                self.BTP.remove(btp_point)
                break

        return next_goal

    # =================================================================
    #                  RULE-BASED NEIGHBOR SELECTION
    # =================================================================
    def choose_next_cell_rule_based(self, free_cells, cur, prev, costmap):
        """
        Priority:
          1) Prefer unvisited cells (global coverage)
          2) Among them, prefer smallest heading change
          3) Among them, prefer longest straight-run
          4) Among them, prefer largest clearance
        No weighted sum; just hierarchical filtering.
        """
        dbg = {}
        cur_heading = self.current_heading(prev, cur)
        if cur_heading is None:
            cur_heading = 0.0

        # --- compute metrics for each candidate ---
        for c in free_cells:
            dx = c[0] - cur[0]
            dy = c[1] - cur[1]
            cand_heading = math.atan2(dy, dx)
            dtheta = abs(self.normalize_angle(cand_heading - cur_heading))

            straight_len = self.estimate_straight_run(cur, c, costmap)
            clearance = self.get_distance_to_obstacle(cur, c, costmap)
            visited = self.is_covered(c)

            dbg[tuple(c)] = {
                "dtheta": dtheta,
                "len": straight_len,
                "clear": clearance,
                "visited": visited,
            }

        # 1) unvisited > visited
        unvisited = [c for c in free_cells if not dbg[tuple(c)]["visited"]]
        candidates = unvisited if unvisited else list(free_cells)

        # 2) smallest heading change (+ margin)
        min_dtheta = min(dbg[tuple(c)]["dtheta"] for c in candidates)
        margin = math.radians(self.heading_margin_deg)
        candidates = [c for c in candidates if dbg[tuple(c)]["dtheta"] <= min_dtheta + margin]

        # 3) longest straight-run
        max_len = max(dbg[tuple(c)]["len"] for c in candidates)
        eps_len = 1e-3
        candidates = [c for c in candidates if abs(dbg[tuple(c)]["len"] - max_len) <= eps_len]

        # 4) largest clearance (tie-break final)
        best_cell = max(candidates, key=lambda c: dbg[tuple(c)]["clear"])
        dbg[tuple(best_cell)]["selected"] = True
        return best_cell, dbg

    # =================================================================
    #              NEIGHBOR GENERATION & COVERAGE HELPERS
    # =================================================================
    def get_free_cells(self, pos, costmap, prev_pos):
        """
        Generate 4-neighbors ±x, ±y at tasp_cell_size.
        - Checks against inflated occupancy
        - Prevents immediate step back into prev_pos
        """
        px, py = pos
        neighbors = [
            [px - self.tasp_cell_size, py],
            [px + self.tasp_cell_size, py],
            [px, py - self.tasp_cell_size],
            [px, py + self.tasp_cell_size],
        ]

        free = []
        for cell in neighbors:
            if self.is_area_free(cell, costmap):
                # avoid oscillation: don't directly return to previous cell
                if not (abs(cell[0] - prev_pos[0]) < 1e-6 and
                        abs(cell[1] - prev_pos[1]) < 1e-6):
                    free.append(cell)
        return free

    def mark_covered(self, pos):
        ix = int(round(pos[0] / self.cover_res))
        iy = int(round(pos[1] / self.cover_res))
        self.cover_map[(ix, iy)] = 1

    def is_covered(self, pos):
        ix = int(round(pos[0] / self.cover_res))
        iy = int(round(pos[1] / self.cover_res))
        return self.cover_map.get((ix, iy), 0) == 1

    # =================================================================
    #                        BTP MANAGEMENT
    # =================================================================
    def update_BTP(self, free_cells):
        """
        Maintain the BackTracking Points (BTP) list.
        - At startup, skip cells that are clearly "behind" the robot once.
        - Then just make sure we don't duplicate entries.
        """
        for cell in free_cells:
            if self.is_initial_position and len(self.TASPtrajectory) >= 2:
                cur = self.TASPtrajectory[-1]
                prev = self.TASPtrajectory[-2]
                fwd = (cur[0] - prev[0], cur[1] - prev[1])
                vec = (cell[0] - cur[0], cell[1] - cur[1])
                if (fwd[0]*vec[0] + fwd[1]*vec[1]) < 0.0:
                    self.is_initial_position = False
                    continue

            if not self.list_has_row(self.BTP, cell):
                self.BTP.append(cell)

    def find_closest_BTP(self, cur, BTP, start_pose):
        """
        Among BTP, choose the closest in Euclidean distance.
        (You could add tie-breaking using distance from start if you want.)
        """
        if not BTP:
            return None
        best = None
        best_d = float('inf')
        for p in BTP:
            d = self.euclidean_distance(cur, p)
            if d < best_d:
                best_d = d
                best = p
        return best

    # =================================================================
    #                 COSTMAP / OCCUPANCY UTILITIES
    # =================================================================
    def get_occupancy_value(self, costmap, x, y):
        """
        Returns:
          0   => free
          50  => unknown
          100 => occupied or out-of-bounds
        """
        gx, gy = self.world_to_grid(costmap, x, y)
        if gx < 0 or gy < 0 or gx >= costmap.info.width or gy >= costmap.info.height:
            return 100
        val = costmap.data[gy * costmap.info.width + gx]
        if val < 0:
            return 50
        return val

    def is_area_free(self, center_cell, costmap):
        """
        Check a square around center_cell with side inflated_tasp_cell * cell_size
        at given sampling_resolution.
        """
        half = self.inflated_tasp_cell * self.tasp_cell_size / 2.0
        xs = self.frange(center_cell[0] - half, center_cell[0] + half, self.sampling_resolution)
        ys = self.frange(center_cell[1] - half, center_cell[1] + half, self.sampling_resolution)
        for x in xs:
            for y in ys:
                if self.get_occupancy_value(costmap, x, y) >= 50:
                    return False
        return True

    def world_to_grid(self, occ, x, y):
        ox, oy = occ.info.origin.position.x, occ.info.origin.position.y
        r = occ.info.resolution
        return int((x - ox) / r), int((y - oy) / r)

    # =================================================================
    #                    GEOMETRY / MATH HELPERS
    # =================================================================
    def euclidean_distance(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def frange(self, a, b, step):
        x = a
        out = []
        while x <= b + 1e-9:
            out.append(x)
            x += step
        return out

    def current_heading(self, prev, cur):
        dx = cur[0] - prev[0]
        dy = cur[1] - prev[1]
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return None
        return math.atan2(dy, dx)

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2.0*math.pi
        while a < -math.pi:
            a += 2.0*math.pi
        return a

    def estimate_straight_run(self, from_pos, toward_cell, costmap, max_m=10.0):
        """
        Estimate how far we can continue in the direction from from_pos to toward_cell,
        stepping by tasp_cell_size until hit occ>=50 or max_m.
        """
        dx = toward_cell[0] - from_pos[0]
        dy = toward_cell[1] - from_pos[1]
        L = math.hypot(dx, dy)
        if L < 1e-6:
            return 0.0
        ux, uy = dx/L, dy/L

        run = 0.0
        x, y = toward_cell[0], toward_cell[1]
        step = self.tasp_cell_size
        while run < max_m:
            x += ux*step
            y += uy*step
            occ = self.get_occupancy_value(costmap, x, y)
            if occ >= 50:
                break
            run += step
        return run

    def get_distance_to_obstacle(self, current_pos, cell, costmap):
        """
        From 'cell' along direction current_pos -> cell, step by tasp_cell_size
        until occ==100 or max_allowed_distance.
        """
        dx = cell[0] - current_pos[0]
        dy = cell[1] - current_pos[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return 0.0
        ux = dx / length
        uy = dy / length

        distance = 0.0
        cx, cy = cell[0], cell[1]
        while distance < self.max_allowed_distance:
            nx = cx + ux * self.tasp_cell_size
            ny = cy + uy * self.tasp_cell_size
            occ = self.get_occupancy_value(costmap, nx, ny)
            if occ == 100:
                break
            distance += self.tasp_cell_size
            cx, cy = nx, ny
        return distance

    def list_has_row(self, arr, element):
        """
        Check if arr (list of [x, y]) already has element (within small epsilon).
        """
        ex, ey = element
        for row in arr:
            if abs(row[0] - ex) < 1e-6 and abs(row[1] - ey) < 1e-6:
                return True
        return False
