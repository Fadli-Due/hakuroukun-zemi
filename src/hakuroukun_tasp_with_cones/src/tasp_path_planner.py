#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Float32

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
        self.remove_btp_threshold_distance = rospy.get_param("remove_btp_threshold_distance", 0.1)
        self.max_allowed_distance = rospy.get_param("range_max", 6.0)
        self.inflated_tasp_cell = rospy.get_param("inflated_tasp_cell", 3.0)

        # --- Dynamic cell size params (all optional; default keeps legacy behavior) ---
        self.enable_dynamic_cell_size = rospy.get_param("enable_dynamic_cell_size", False)
        self.base_cell_size       = rospy.get_param("base_cell_size_m", self.tasp_cell_size)
        self.min_cell_size        = rospy.get_param("min_cell_size_m", max(0.2, self.tasp_cell_size))
        self.max_cell_size        = rospy.get_param("max_cell_size_m", max(self.min_cell_size, self.tasp_cell_size))
        self.narrow_clearance     = rospy.get_param("narrow_clearance_m", 1.2)
        self.wide_clearance       = rospy.get_param("wide_clearance_m", 2.0)
        self.cellsize_hyst        = rospy.get_param("cellsize_hysteresis_m", 0.2)
        self.clearance_radius     = rospy.get_param("clearance_scan_radius_m", 2.5)
        self.min_dwell_s          = rospy.get_param("dynamic_cell_min_dwell_s", 5.0)
        self.cooldown_s           = rospy.get_param("dynamic_cell_replan_cooldown_s", 3.0)
        self.publish_debug        = rospy.get_param("publish_debug_topics", True)
        self.sampling_res_factor  = rospy.get_param("sampling_resolution_factor", None)  # e.g., 0.2

        # State for dynamic sizing
        self._last_cell_size = float(self.tasp_cell_size)
        self._last_size_change_time = rospy.Time.now()
        if self.publish_debug:
            self._dbg_clearance_pub = rospy.Publisher("/tasp/debug/clearance", Float32, queue_size=1)
            self._dbg_cell_size_pub = rospy.Publisher("/tasp/debug/cell_size", Float32, queue_size=1)

        # BackTracking Points
        self.BTP = []

        # TASP trajectory (a list of [x, y] waypoints)
        self.TASPtrajectory = []
        self.goto_closest_BTP = False
        self.is_initial_position = True

    def tasp_path_planning(self, current_pose, costmap, start_pose):
        """
        Main TASP function that determines a new TASP goal.

        Args:
            current_pose (tuple): (x, y, theta) of the robot's current position
            costmap: OccupancyGrid or equivalent map structure.
            start_pose (tuple): (x, y, theta) the robot’s initial pose

        Returns:
            TASPgoal (list [x, y]) or None if no goal is available
        """
        # --- Dynamic cell size update (before neighbor expansion) ---
        if self.enable_dynamic_cell_size:
            self._maybe_update_cell_size(current_pose, costmap)

        if len(self.TASPtrajectory) < 2:
            self.TASPtrajectory = [
                [current_pose[0], current_pose[1]],
                [start_pose[0], start_pose[1]]
            ]

        TASPcurrentPos = self.TASPtrajectory[-1]
        TASPprevPos = self.TASPtrajectory[-2]

        front_cell = self.get_front_cell(TASPcurrentPos, TASPprevPos)
        free_cells = self.get_free_cells(TASPcurrentPos, costmap, self.TASPtrajectory)

        self.update_BTP(free_cells)

        if not free_cells:
            if not self.BTP:
                print("[TASP] No BTP available. Stopping the robot.")
                return None

            next_goal = self.find_closest_BTP(TASPcurrentPos, self.BTP, start_pose)
            if next_goal is None:
                print("[TASP] No valid path to BTP. Stopping the robot.")
                return None
            self.TASPtrajectory.append(next_goal)
        else:
            if len(free_cells) == 1:
                self.TASPtrajectory.append(free_cells[0])
            elif self.goto_closest_BTP:
                cell = self.find_front_cell_with_min_rotation(free_cells, front_cell, current_pose)
                self.goto_closest_BTP = False
                self.TASPtrajectory.append(cell)
            elif self.list_has_row(free_cells, front_cell):
                self.TASPtrajectory.append(front_cell)
            elif len(free_cells) == 2:
                cell1 = free_cells[0]
                cell2 = free_cells[1]
                c1_away = self.is_away_from_start(cell1, start_pose, TASPcurrentPos)
                c2_away = self.is_away_from_start(cell2, start_pose, TASPcurrentPos)

                if c1_away and not c2_away:
                    self.TASPtrajectory.append(cell1)
                elif c2_away and not c1_away:
                    self.TASPtrajectory.append(cell2)
                else:
                    d1 = self.get_distance_to_obstacle(TASPcurrentPos, cell1, costmap)
                    d2 = self.get_distance_to_obstacle(TASPcurrentPos, cell2, costmap)
                    if d1 > d2:
                        self.TASPtrajectory.append(cell1)
                    else:
                        self.TASPtrajectory.append(cell2)

        newly_chosen = self.TASPtrajectory[-1]

        for btp_point in self.BTP:
            if self.euclidean_distance(newly_chosen, btp_point) < self.remove_btp_threshold_distance:
                self.BTP.remove(btp_point)
                break

        return self.TASPtrajectory[-1]
    
    # --------------------------------------------------------------------------
    # DYNAMIC CELL SIZE (clearance, hysteresis, dwell/cooldown)
    # --------------------------------------------------------------------------
    def _maybe_update_cell_size(self, current_pose, costmap):
        """Decide and (if allowed) apply a new tasp_cell_size based on local clearance."""
        # 1) measure clearance
        clearance = self._get_local_clearance(current_pose, costmap, self.clearance_radius)
        if self.publish_debug:
            self._dbg_clearance_pub.publish(Float32(clearance))

        # 2) pick target bin with hysteresis
        target = self._pick_cell_size(clearance, self._last_cell_size)

        # 3) dwell + cooldown gates
        now = rospy.Time.now()
        elapsed = (now - self._last_size_change_time).to_sec()
        if target != self._last_cell_size and elapsed >= self.min_dwell_s and elapsed >= self.cooldown_s:
            # apply the change
            self._last_cell_size = float(target)
            self.tasp_cell_size = float(target)
            # optionally scale sampling resolution with size (clamped to map resolution)
            if self.sampling_res_factor is not None and hasattr(costmap.info, "resolution"):
                self.sampling_resolution = max(costmap.info.resolution, self.sampling_res_factor * self.tasp_cell_size)
            self._last_size_change_time = now
            if self.publish_debug:
                self._dbg_cell_size_pub.publish(Float32(self.tasp_cell_size))
        elif self.publish_debug:
            # Even if size doesn’t change, keep emitting current size for plotting
            self._dbg_cell_size_pub.publish(Float32(self.tasp_cell_size))

    def _pick_cell_size(self, clearance, last_size):
        """Hysteresis: narrow if <= (narrow−hyst); wide if >= (wide+hyst); else stick to last/base."""
        # Default to last -> avoids thrashing inside band
        target = last_size if last_size is not None else self.base_cell_size
        if clearance <= (self.narrow_clearance - self.cellsize_hyst):
            target = self.min_cell_size
        elif clearance >= (self.wide_clearance + self.cellsize_hyst):
            target = self.max_cell_size
        return float(target)

    def _get_local_clearance(self, current_pose, costmap, max_radius):
        """
        Radially expand from the robot, stopping at the first occupied/unknown hit.
        Returns max free radius (meters). Unknown (>=50) is treated as boundary.
        """
        cx, cy, _ = current_pose
        # Use existing sampling_resolution; ensure reasonable step sizes
        step_r = max(0.05, self.sampling_resolution)
        step_ang = math.radians(10.0)  # 36 rays per ring
        r = 0.0
        last_free_r = 0.0
        while r <= max_radius + 1e-6:
            blocked = False
            a = 0.0
            while a < 2.0 * math.pi:
                x = cx + r * math.cos(a)
                y = cy + r * math.sin(a)
                if self.get_occupancy_value(costmap, x, y) >= 50:
                    blocked = True
                    break
                a += step_ang
            if blocked:
                # first ring that collides → clearance is previous radius
                return max(0.0, r - step_r)
            last_free_r = r
            r += step_r
        return min(last_free_r, max_radius)
        
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

        print(f"Selected front cell: {best_cell} with rotation: {min_rotation}")
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
        """
        Append free cells to BTP if they aren’t already there.
        """
        for cell in free_cells:
            if self.is_initial_position and cell == [-1, 0]:  # Avoid cell behind robot to be BTP
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
