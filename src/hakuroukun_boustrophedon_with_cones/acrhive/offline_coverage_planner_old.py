#!/usr/bin/env python3
# =============================================================================
#  offline_coverage_planner.py
#
#  Offline coverage path planner for Hakuroukun, using
#  Boustrophedon Cellular Decomposition (BCD).
# =============================================================================
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import math
import rospy
import numpy as np
from scipy.ndimage import distance_transform_edt

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401

from planning.simple_astar import astar_plan, SimpleOccupancyGrid


# -----------------------------------------------------------------------------
#  Boustrophedon cell
# -----------------------------------------------------------------------------
class Cell:
    def __init__(self, cell_id):
        self.id = cell_id
        self.cols = {}          # gx -> (y_lo, y_hi)
        self.mask = None        
        self.area = 0.0         

    def add_column(self, gx, y_lo, y_hi):
        self.cols[gx] = (y_lo, y_hi)

    def finalize(self, shape, resolution):
        self.mask = np.zeros(shape, dtype=bool)
        for gx, (lo, hi) in self.cols.items():
            self.mask[lo:hi + 1, gx] = True
        self.area = float(self.mask.sum()) * resolution * resolution


# -----------------------------------------------------------------------------
#  Planner node
# -----------------------------------------------------------------------------
class OfflineCoveragePlanner:
    def __init__(self):
        rospy.init_node('offline_coverage_planner')

        # ---- parameters -----------------------------------------------------
        # robot_radius: Lowered from 0.55 to 0.45 to prevent narrow corridor blockages
        self.robot_radius   = rospy.get_param("robot_radius", 0.45)
        self.lane_spacing   = rospy.get_param("lane_spacing", 1.0)
        self.turn_margin    = rospy.get_param("turn_margin", 0.30)
        self.min_cell_area  = rospy.get_param("min_cell_area", 1.0)
        self.astar_conn     = rospy.get_param("astar_connectivity", 8)
        self.densify_step   = rospy.get_param("densify_step", 0.20)
        self.crop_margin_m  = rospy.get_param("crop_margin", 1.0)
        
        # Settling time for AMCL particles on startup
        self.startup_delay  = rospy.get_param("startup_delay", 5.0)

        # ---- ROS I/O --------------------------------------------------------
        self.path_pub = rospy.Publisher('/desired_path', Path, queue_size=1, latch=True)
        rospy.Subscriber('/map', OccupancyGrid, self.map_cb)
        rospy.Subscriber('/hakuroukun_pose/rear_wheel_odometry', Odometry, self.odom_cb)

        self.map_data = None
        self.start_pose = None
        self.first_pose_time = None
        self.path_generated = False

        self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf)

        # Polling rate increased to check delay timing accurately
        self.timer = rospy.Timer(rospy.Duration(0.5), self.check_and_plan)

        rospy.loginfo("[BCD] Planner ready. robot_radius=%.2f lane_spacing=%.2f "
                      "turn_margin=%.2f" % (self.robot_radius, self.lane_spacing,
                                            self.turn_margin))

    # ------------------------------------------------------------------ I/O
    def map_cb(self, msg):
        self.map_data = msg

    def odom_cb(self, msg):
        robot_frame = msg.child_frame_id if msg.child_frame_id else "base_link"
        try:
            transform = self.tf_buf.lookup_transform("map", robot_frame, rospy.Time(0), rospy.Duration(0.2))
            self.start_pose = (transform.transform.translation.x, transform.transform.translation.y)
            if self.first_pose_time is None:
                self.first_pose_time = rospy.Time.now().to_sec()
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[BCD] TF lookup map -> {robot_frame} failed: {e}. Falling back to odom message transform.")
            try:
                ps = PoseStamped()
                ps.header = msg.header
                ps.pose = msg.pose.pose
                ps.header.stamp = rospy.Time(0)
                ps_map = self.tf_buf.transform(ps, "map", rospy.Duration(0.2))
                self.start_pose = (ps_map.pose.position.x, ps_map.pose.position.y)
                if self.first_pose_time is None:
                    self.first_pose_time = rospy.Time.now().to_sec()
            except Exception:
                self.start_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
                if self.first_pose_time is None:
                    self.first_pose_time = rospy.Time.now().to_sec()

    def check_and_plan(self, event):
        if self.path_generated:
            return
        if self.map_data is None:
            rospy.loginfo_throttle(5, "[BCD] waiting for /map ...")
            return
        if self.start_pose is None or self.first_pose_time is None:
            rospy.loginfo_throttle(5, "[BCD] waiting for /hakuroukun_pose/rear_wheel_odometry ...")
            return
        
        # Defer planning until AMCL localization has settled
        elapsed = rospy.Time.now().to_sec() - self.first_pose_time
        if elapsed < self.startup_delay:
            rospy.loginfo_throttle(1, "[BCD] letting localization settle... %.1fs/%.1fs" 
                                   % (elapsed, self.startup_delay))
            return

        try:
            self.plan_coverage()
            self.path_generated = True   # plan once
        except Exception as e:
            rospy.logerr("[BCD] planning failed: %s" % e)

    # =====================================================================
    #  MAIN
    # =====================================================================
    def plan_coverage(self):
        t0 = rospy.Time.now()
        rospy.loginfo("[BCD] start. robot at %s" % str(self.start_pose))

        grid, crop_ox, crop_oy, res = self._prepare_grid(self.map_data)
        free = grid == 0                                  
        rospy.loginfo("[BCD] cropped grid %dx%d, %d free cells"
                      % (free.shape[1], free.shape[0], int(free.sum())))

        raw_start = self.start_pose
        snapped = self._snap_to_free(grid, crop_ox, crop_oy, res, raw_start)
        if math.hypot(snapped[0] - raw_start[0],
                      snapped[1] - raw_start[1]) > 1e-3:
            rospy.logwarn("[BCD] robot start %s is inside the inflation band; "
                          "planning from nearest free cell %s with a straight "
                          "lead-in." % (str(raw_start), str(snapped)))
        self.start_pose = snapped

        astar_grid = SimpleOccupancyGrid(
            free.shape[1], free.shape[0], res, crop_ox, crop_oy,
            np.where(free, 0, 100).astype(np.int16).reshape(-1).tolist())

        # --- 1) decomposition ---
        cells = self._bcd_decompose(free)
        for c in cells:
            c.finalize(free.shape, res)
        cells = [c for c in cells if c.area >= self.min_cell_area]
        if not cells:
            rospy.logerr("[BCD] no cells produced. Is the map empty / too inflated?")
            return
        rospy.loginfo("[BCD] %d coverage cells" % len(cells))

        # --- 2) per-cell coverage waypoints (in world coords) ---
        cell_paths = []
        for c in cells:
            wp = self._cover_cell(c, res, crop_ox, crop_oy)
            if len(wp) >= 2:
                cell_paths.append(wp)
        if not cell_paths:
            rospy.logerr("[BCD] cells produced no lanes. lane_spacing too large?")
            return

        # --- 3) order cells, greedy nearest-neighbour from the robot ---
        ordered = self._order_cells(cell_paths, self.start_pose)

        # --- 4) assemble: robot -> cell -> cell, A* only for transits ---
        global_wpts = [self.start_pose]
        for wp in ordered:
            global_wpts.extend(wp)

        full = []
        for i in range(1, len(global_wpts)):
            seg = self._connect(astar_grid, global_wpts[i - 1], global_wpts[i])
            if full:
                full.extend(seg[1:])
            else:
                full.extend(seg)

        full = self._dedup(full)
        if len(full) < 2:
            rospy.logerr("[BCD] assembled path is empty.")
            return

        if math.hypot(raw_start[0] - snapped[0],
                      raw_start[1] - snapped[1]) > 1e-3:
            full = self._densify_straight(raw_start, snapped)[:-1] + full
            full = self._dedup(full)

        self.publish_path(full)
        dt = (rospy.Time.now() - t0).to_sec()
        rospy.loginfo("[BCD] done in %.2fs : %d cells, %d path points"
                      % (dt, len(ordered), len(full)))

    # =====================================================================
    #  MAP PREP
    # =====================================================================
    def _prepare_grid(self, msg):
        w, h, res = msg.info.width, msg.info.height, msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y

        raw = np.array(msg.data, dtype=np.int16).reshape((h, w))
        free = (raw == 0)

        ys, xs = np.where(free)
        if xs.size == 0:
            raise RuntimeError("map has no free cells")
        m = int(math.ceil(self.robot_radius / res)) + int(self.crop_margin_m / res) + 3
        x0 = max(0, xs.min() - m); x1 = min(w, xs.max() + m + 1)
        y0 = max(0, ys.min() - m); y1 = min(h, ys.max() + m + 1)

        sub = raw[y0:y1, x0:x1]
        free_sub = (sub == 0)

        dist_m = distance_transform_edt(free_sub) * res
        inflated_free = free_sub & (dist_m > self.robot_radius)

        grid = np.where(inflated_free, 0, 100).astype(np.int16)
        crop_ox = ox + x0 * res
        crop_oy = oy + y0 * res
        return grid, crop_ox, crop_oy, res

    def _snap_to_free(self, grid, crop_ox, crop_oy, res, pose):
        from collections import deque
        free = grid == 0
        H, W = grid.shape
        gx = min(max(int((pose[0] - crop_ox) / res), 0), W - 1)
        gy = min(max(int((pose[1] - crop_oy) / res), 0), H - 1)
        if free[gy, gx]:
            return pose
        seen = {(gx, gy)}
        q = deque([(gx, gy)])
        while q:
            cx, cy = q.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < W and 0 <= ny < H and (nx, ny) not in seen:
                    if free[ny, nx]:
                        return (crop_ox + (nx + 0.5) * res,
                                crop_oy + (ny + 0.5) * res)
                    seen.add((nx, ny))
                    q.append((nx, ny))
        return pose   

    # =====================================================================
    #  BOUSTROPHEDON CELLULAR DECOMPOSITION
    # =====================================================================
    @staticmethod
    def _runs(col_bool):
        runs = []
        in_run = False
        lo = 0
        for i, v in enumerate(col_bool):
            if v and not in_run:
                lo = i; in_run = True
            elif not v and in_run:
                runs.append((lo, i - 1)); in_run = False
        if in_run:
            runs.append((lo, len(col_bool) - 1))
        return runs

    def _bcd_decompose(self, free):
        H, W = free.shape
        cells = []
        next_id = 0
        prev_runs = []          
        prev_cell = []          

        for x in range(W):
            cur_runs = self._runs(free[:, x])

            cur_to_prev = [[] for _ in cur_runs]
            prev_to_cur = [[] for _ in prev_runs]
            for ci, (clo, chi) in enumerate(cur_runs):
                for pi, (plo, phi) in enumerate(prev_runs):
                    if plo <= chi and clo <= phi:        
                        cur_to_prev[ci].append(pi)
                        prev_to_cur[pi].append(ci)

            cur_cell = [None] * len(cur_runs)
            for ci, (clo, chi) in enumerate(cur_runs):
                prevs = cur_to_prev[ci]
                if len(prevs) == 1 and len(prev_to_cur[prevs[0]]) == 1:
                    cell = prev_cell[prevs[0]]
                else:
                    cell = Cell(next_id); next_id += 1
                    cells.append(cell)
                cell.add_column(x, clo, chi)
                cur_cell[ci] = cell

            prev_runs = cur_runs
            prev_cell = cur_cell

        return cells

    # =====================================================================
    #  PER-CELL COVERAGE  
    # =====================================================================
    def _cover_cell(self, cell, res, crop_ox, crop_oy):
        mask = cell.mask
        ys, xs = np.where(mask)
        x0, x1 = xs.min(), xs.max()
        y0, y1 = ys.min(), ys.max()

        step = max(1, int(round(self.lane_spacing / res)))
        trim = max(0, int(round(self.turn_margin / res)))

        v_lanes = self._lanes_vertical(mask, x0, x1, step, trim)
        h_lanes = self._lanes_horizontal(mask, y0, y1, step, trim)

        if v_lanes and (not h_lanes or len(v_lanes) <= len(h_lanes)):
            lanes, axis = v_lanes, "vertical"
        elif h_lanes:
            lanes, axis = h_lanes, "horizontal"
        else:
            return []

        wpts = []
        for i, (a, b) in enumerate(lanes):
            if i % 2 == 1:
                a, b = b, a
            wpts.append(a)
            wpts.append(b)

        rospy.loginfo("[BCD]  cell %d : %d %s lanes (area %.1f m^2)"
                      % (cell.id, len(lanes), axis, cell.area))

        return [(crop_ox + (gx + 0.5) * res, crop_oy + (gy + 0.5) * res)
                for (gx, gy) in wpts]

    def _lanes_vertical(self, mask, x0, x1, step, trim):
        lanes = []
        for gx in range(x0 + step // 2, x1 + 1, step):
            for lo, hi in self._runs(mask[:, gx]):
                if hi - lo + 1 > 2 * trim:
                    lanes.append(((gx, lo + trim), (gx, hi - trim)))
        return lanes

    def _lanes_horizontal(self, mask, y0, y1, step, trim):
        lanes = []
        for gy in range(y0 + step // 2, y1 + 1, step):
            for lo, hi in self._runs(mask[gy, :]):
                if hi - lo + 1 > 2 * trim:
                    lanes.append(((lo + trim, gy), (hi - trim, gy)))
        return lanes

    # =====================================================================
    #  CELL ORDERING
    # =====================================================================
    def _order_cells(self, cell_paths, start):
        remaining = list(cell_paths)
        ordered = []
        cur = start
        while remaining:
            best_i, best_d, best_rev = 0, float('inf'), False
            for i, wp in enumerate(remaining):
                d_head = math.hypot(wp[0][0] - cur[0],  wp[0][1] - cur[1])
                d_tail = math.hypot(wp[-1][0] - cur[0], wp[-1][1] - cur[1])
                if d_head < best_d:
                    best_d, best_i, best_rev = d_head, i, False
                if d_tail < best_d:
                    best_d, best_i, best_rev = d_tail, i, True
            wp = remaining.pop(best_i)
            if best_rev:
                wp = wp[::-1]
            ordered.append(wp)
            cur = wp[-1]
        return ordered

    # =====================================================================
    #  SEGMENT CONNECTION  
    # =====================================================================
    def _connect(self, grid, p1, p2):
        if self._segment_free(grid, p1, p2):
            return self._densify_straight(p1, p2)

        path = astar_plan(grid, p1[0], p1[1], p2[0], p2[1],
                          connectivity=self.astar_conn)
        if not path or path == "GOAL_OCCUPIED":
            rospy.logwarn("[BCD] A* transit failed (%.1f,%.1f)->(%.1f,%.1f); "
                          "using straight line." % (p1[0], p1[1], p2[0], p2[1]))
            return self._densify_straight(p1, p2)

        out = [path[0]]
        for i in range(1, len(path)):
            out.extend(self._densify_straight(path[i - 1], path[i])[1:])
        return out

    def _segment_free(self, grid, p1, p2, step=0.05):
        d = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        n = max(2, int(d / step))
        for i in range(n + 1):
            a = i / float(n)
            x = p1[0] * (1 - a) + p2[0] * a
            y = p1[1] * (1 - a) + p2[1] * a
            gx = int((x - grid.origin_x) / grid.resolution)
            gy = int((y - grid.origin_y) / grid.resolution)
            if gx < 0 or gx >= grid.width or gy < 0 or gy >= grid.height:
                return False
            if grid.data[gy * grid.width + gx] >= 50:
                return False
        return True

    def _densify_straight(self, p1, p2):
        d = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if d <= self.densify_step:
            return [p1, p2]
        n = int(d / self.densify_step)
        pts = [p1]
        for j in range(1, n + 1):
            a = j / float(n + 1)
            pts.append((p1[0] * (1 - a) + p2[0] * a,
                        p1[1] * (1 - a) + p2[1] * a))
        pts.append(p2)
        return pts

    @staticmethod
    def _dedup(pts, eps=1e-4):
        out = [pts[0]]
        for p in pts[1:]:
            if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > eps:
                out.append(p)
        return out

    # =====================================================================
    #  OUTPUT
    # =====================================================================
    def publish_path(self, points):
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        for x, y in points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)
        rospy.loginfo("[BCD] published /desired_path with %d points" % len(points))


if __name__ == '__main__':
    try:
        OfflineCoveragePlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass