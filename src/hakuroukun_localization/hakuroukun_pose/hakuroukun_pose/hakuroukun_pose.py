#!/usr/bin/env python3
##
# @file local_replanner.py
#
# @brief Provide implementation of the online path modification layer.
#
# @section author_doxygen_example Author(s)
# - Created by Dinh Ngoc Duc on 24/10/2024.
# - Modified by Fadli Due Ramandavito on 29/06/2026.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard Libraries
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import math
import threading

# External Libraries
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from scipy.ndimage import distance_transform_edt
import tf2_ros
import tf2_geometry_msgs  # noqa: F401

# Internal Libraries
from planning.simple_astar import astar_plan, SimpleOccupancyGrid


class LocalReplanner:
    """! LocalReplanner class
    The class provides online path modification for the BCD coverage planner.

    Two-layer safety architecture:
    - Layer 1 (path_follower.py): Reflex stop. LiDAR FORWARD VETO at ~0.45 m.
    - Layer 2 (this node): Persistence-gated A* detour. Fires only when an
      obstacle occupies the upcoming path for >= persistence_threshold seconds,
      distinguishing static objects from transient ones (e.g. pedestrians).
    """

    # ==========================================================================
    # PUBLIC METHODS
    # ==========================================================================

    def __init__(self):
        """! Constructor
        """
        rospy.init_node("local_replanner")

        self._register_parameters()

        self._register_state()

        self._register_publishers()

        self._register_subscribers()

        self._register_timers()

        rospy.loginfo(
            "[local_replanner] up. persistence=%.1fs clear=%.1fs window=%.1fm "
            "lookahead=%.1fm inflate=%.2fm",
            self._persistence_threshold, self._clear_time, self._window_size_m,
            self._lookahead_check_m, self._obstacle_inflate_m)

    def run(self):
        """! Start ros node
        """
        rospy.spin()

    # ==========================================================================
    # PRIVATE METHODS
    # ==========================================================================

    def _register_parameters(self):
        """! Register ROS parameters method
        """
        ns = "local_replanner"

        self._persistence_threshold = rospy.get_param(
            f"{ns}/persistence_threshold", 7.0)

        self._clear_time = rospy.get_param(
            f"{ns}/clear_time", 1.0)

        self._window_size_m = rospy.get_param(
            f"{ns}/window_size_m", 15.0)

        self._eval_rate = rospy.get_param(
            f"{ns}/eval_rate", 2.0)

        self._lookahead_check_m = rospy.get_param(
            f"{ns}/lookahead_check_m", 6.0)

        self._obstacle_inflate_m = rospy.get_param(
            f"{ns}/obstacle_inflate_m", 1.0)

        # Must equal boustrophedon_config robot_radius so planned path points
        # always lie inside the A* free space.
        self._static_wall_inflate_m = rospy.get_param(
            "robot_radius", 1.0)

        self._scan_min_range = rospy.get_param(
            f"{ns}/scan_min_range", 0.30)

        self._scan_max_range = rospy.get_param(
            f"{ns}/scan_max_range", 15.0)

        self._densify_step = rospy.get_param(
            "densify_step", 0.20)

        self._rejoin_margin_m = rospy.get_param(
            f"{ns}/rejoin_margin_m", 1.5)

        self._map_frame = rospy.get_param(
            f"{ns}/map_frame", "map")

        self._laser_frame = rospy.get_param(
            f"{ns}/laser_frame", "laser_link")

    def _register_state(self):
        """! Register internal state variables method
        """
        self._lock = threading.Lock()

        # Static map
        self._static_inflated = None
        self._map_res = None
        self._map_ox  = None
        self._map_oy  = None
        self._map_w   = None
        self._map_h   = None

        # Path state
        self._baseline_path      = []
        self._current_path       = []
        self._detour_active      = False
        self._detour_end_index   = None
        self._last_detour_points = []
        self._last_i_now         = None
        self._return_appended    = False

        # Robot pose
        self._robot_xy  = None
        self._robot_yaw = 0.0

        # Local obstacle grid (last-seen timestamp per cell).
        # Allocated in _map_cb once map resolution is known.
        self._obs_grid   = None
        self._obs_grid_w = None
        self._obs_grid_h = None
        # FIX (2026-06-29): initialise to None so _maybe_recenter_obs_grid
        # snaps to the robot's actual position on the first call, rather than
        # computing a ~20 m shift from (0,0) that wipes the entire grid.
        self._obs_ox = None
        self._obs_oy = None
        self._obs_grid_origin_initialized = False

        # TF
        self._tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_lst = tf2_ros.TransformListener(self._tf_buf)

    def _register_publishers(self):
        """! Register ROS publishers method
        """
        self._path_pub = rospy.Publisher(
            "/desired_path", Path, queue_size=1, latch=True)

        self._viz_grid_pub = rospy.Publisher(
            "/local_replanner/obstacle_grid", OccupancyGrid, queue_size=1, latch=True)

        self._viz_marker_pub = rospy.Publisher(
            "/local_replanner/markers", MarkerArray, queue_size=1, latch=True)

    def _register_subscribers(self):
        """! Register ROS subscribers method
        """
        rospy.Subscriber("/map", OccupancyGrid, self._map_cb, queue_size=1)

        rospy.Subscriber("/planned_path", Path, self._baseline_cb, queue_size=1)

        rospy.Subscriber("/scan_multi", LaserScan, self._scan_cb, queue_size=1)

        rospy.Subscriber(
            "/hakuroukun_pose/rear_wheel_odometry", Odometry,
            self._odom_cb, queue_size=10)

        rospy.Subscriber("/return_path", Path, self._return_path_cb, queue_size=1)

    def _register_timers(self):
        """! Register ROS timers method
        """
        rospy.Timer(rospy.Duration(1.0 / self._eval_rate), self._evaluate)

    def _map_cb(self, data: OccupancyGrid):
        """! Map callback method
        @param data: OccupancyGrid message
        """
        with self._lock:
            self._map_res = data.info.resolution
            self._map_ox  = data.info.origin.position.x
            self._map_oy  = data.info.origin.position.y
            self._map_w   = data.info.width
            self._map_h   = data.info.height

            raw  = np.array(data.data, dtype=np.int16).reshape((self._map_h, self._map_w))
            free = (raw == 0)
            dist = distance_transform_edt(free) * self._map_res
            self._static_inflated = free & (dist > self._static_wall_inflate_m)

            side_cells       = int(math.ceil(self._window_size_m / self._map_res))
            self._obs_grid_w = side_cells
            self._obs_grid_h = side_cells
            self._obs_grid   = np.zeros((side_cells, side_cells), dtype=np.float32)

            rospy.loginfo(
                "[local_replanner] map received (%dx%d cells, res=%.3fm), "
                "obstacle grid %dx%d cells.",
                self._map_w, self._map_h, self._map_res, side_cells, side_cells)

    def _baseline_cb(self, data: Path):
        """! Baseline path callback method
        @param data: Path message from offline_coverage_planner
        """
        with self._lock:
            self._baseline_path = [
                (p.pose.position.x, p.pose.position.y) for p in data.poses]
            self._current_path       = list(self._baseline_path)
            self._detour_active      = False
            self._detour_end_index   = None
            self._last_i_now         = None
            self._return_appended    = False

        self._publish_path(self._current_path)

        rospy.loginfo("[local_replanner] baseline path received: %d points",
                      len(self._baseline_path))

    def _return_path_cb(self, data: Path):
        """! Return path callback method
        Appends an A* return-to-home leg from return_to_start.py onto the
        tail of current_path. One-shot per coverage session.
        @param data: Path message from return_to_start node
        """
        with self._lock:
            if self._return_appended:
                rospy.logwarn("[local_replanner] return leg already appended — ignoring.")
                return
            if not self._current_path:
                rospy.logwarn("[local_replanner] /return_path received before baseline — ignoring.")
                return

            return_pts = [(p.pose.position.x, p.pose.position.y) for p in data.poses]
            if not return_pts:
                rospy.logwarn("[local_replanner] /return_path is empty — ignoring.")
                return

            self._current_path    = list(self._current_path) + return_pts
            self._return_appended = True

        self._publish_path(self._current_path)

        rospy.loginfo(
            "[local_replanner] return leg appended: %d points, total path %d points.",
            len(return_pts), len(self._current_path))

    def _odom_cb(self, data: Odometry):
        """! Odometry callback method
        @param data: Odometry message from hakuroukun_pose
        """
        try:
            ps = PoseStamped()
            ps.header       = data.header
            ps.pose         = data.pose.pose
            ps.header.stamp = rospy.Time(0)
            ps_map = self._tf_buf.transform(ps, self._map_frame, rospy.Duration(0.1))
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[local_replanner] odom->map TF: {e}")
            return

        from scipy.spatial.transform import Rotation
        q = ps_map.pose.orientation
        self._robot_xy  = (ps_map.pose.position.x, ps_map.pose.position.y)
        self._robot_yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("zyx")[0]

    def _scan_cb(self, data: LaserScan):
        """! LiDAR scan callback method
        Stamps obstacle grid cells with the current ROS time for each valid
        LiDAR return, using a vectorised laser_frame -> map_frame transform.
        @param data: LaserScan message from /scan_multi
        """
        if self._obs_grid is None or self._robot_xy is None:
            return

        self._maybe_recenter_obs_grid()

        try:
            tr = self._tf_buf.lookup_transform(
                self._map_frame, data.header.frame_id or self._laser_frame,
                rospy.Time(0), rospy.Duration(0.1))
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[local_replanner] laser->map TF: {e}")
            return

        t = tr.transform.translation
        q = tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)

        n = len(data.ranges)
        if n == 0:
            return

        angles = data.angle_min + np.arange(n, dtype=np.float32) * data.angle_increment
        ranges = np.asarray(data.ranges, dtype=np.float32)

        valid = (np.isfinite(ranges) &
                 (ranges >= max(data.range_min, self._scan_min_range)) &
                 (ranges <= min(data.range_max, self._scan_max_range)))
        if not np.any(valid):
            return

        r  = ranges[valid]
        a  = angles[valid]
        xl = r * np.cos(a)
        yl = r * np.sin(a)
        xm = c * xl - s * yl + t.x
        ym = s * xl + c * yl + t.y

        gx = ((xm - self._obs_ox) / self._map_res).astype(np.int32)
        gy = ((ym - self._obs_oy) / self._map_res).astype(np.int32)
        in_bounds = ((gx >= 0) & (gx < self._obs_grid_w) &
                     (gy >= 0) & (gy < self._obs_grid_h))
        if not np.any(in_bounds):
            return

        now = rospy.Time.now().to_sec()
        with self._lock:
            self._obs_grid[gy[in_bounds], gx[in_bounds]] = now

    def _maybe_recenter_obs_grid(self):
        """! Recenter obstacle grid method
        Slides the local obstacle window to keep the robot roughly centred.
        Only shifts when the robot has moved more than 1/4 of the window width.

        FIX (2026-06-29):
        - First-call snap: obs_ox/obs_oy were initialised to (0,0), causing
          a ~20 m shift on the first call that wiped the entire grid.
          Fix: snap the origin directly to the robot position on first call.
        - obs_first lockstep: obs_first (persistence timestamps) was not
          shifted together with obs_grid, causing timestamps to desync from
          their cells and the 7 s gate to never fire on long runs.
          Fix: apply the identical shift to obs_first in the same lock.
        """
        if self._obs_grid is None or self._robot_xy is None:
            return

        half       = (self._obs_grid_w * self._map_res) / 2.0
        desired_ox = self._robot_xy[0] - half
        desired_oy = self._robot_xy[1] - half

        if not self._obs_grid_origin_initialized:
            with self._lock:
                self._obs_ox = desired_ox
                self._obs_oy = desired_oy
                if not hasattr(self, "_obs_first") or self._obs_first is None \
                        or self._obs_first.shape != self._obs_grid.shape:
                    self._obs_first = np.zeros_like(self._obs_grid)
                self._obs_grid_origin_initialized = True
            rospy.loginfo(
                "[local_replanner] obs_grid origin snapped to (%.2f, %.2f)",
                desired_ox, desired_oy)
            return

        dx_cells = int(round((desired_ox - self._obs_ox) / self._map_res))
        dy_cells = int(round((desired_oy - self._obs_oy) / self._map_res))

        if abs(dx_cells) < self._obs_grid_w // 4 and abs(dy_cells) < self._obs_grid_h // 4:
            return

        with self._lock:
            new       = np.zeros_like(self._obs_grid)
            new_first = np.zeros_like(self._obs_grid)

            if not hasattr(self, "_obs_first") or self._obs_first is None \
                    or self._obs_first.shape != self._obs_grid.shape:
                self._obs_first = np.zeros_like(self._obs_grid)

            src_x0 = max(0, dx_cells)
            src_y0 = max(0, dy_cells)
            src_x1 = min(self._obs_grid_w, self._obs_grid_w + dx_cells)
            src_y1 = min(self._obs_grid_h, self._obs_grid_h + dy_cells)
            dst_x0 = src_x0 - dx_cells
            dst_y0 = src_y0 - dy_cells
            dst_x1 = src_x1 - dx_cells
            dst_y1 = src_y1 - dy_cells

            if src_x1 > src_x0 and src_y1 > src_y0:
                new[dst_y0:dst_y1, dst_x0:dst_x1] = \
                    self._obs_grid[src_y0:src_y1, src_x0:src_x1]
                new_first[dst_y0:dst_y1, dst_x0:dst_x1] = \
                    self._obs_first[src_y0:src_y1, src_x0:src_x1]

            self._obs_grid  = new
            self._obs_first = new_first
            self._obs_ox    = self._obs_ox + dx_cells * self._map_res
            self._obs_oy    = self._obs_oy + dy_cells * self._map_res

    def _evaluate(self, timer):
        """! Evaluate detour method
        Checks for persistent obstacles ahead on the current path and
        triggers an A* detour if one is found. Runs at eval_rate Hz.
        @param timer: Timer (unused)
        """
        if self._static_inflated is None:
            return
        if not self._baseline_path or self._robot_xy is None:
            return

        now = rospy.Time.now().to_sec()

        with self._lock:
            obs = self._obs_grid
            persistent_mask = self._compute_persistent_mask(obs, now)

        i_now = self._closest_path_index_windowed(
            self._current_path, self._robot_xy, self._last_i_now)
        self._last_i_now = i_now

        i_block_start, i_block_end = self._find_blocked_span(
            self._current_path, i_now, persistent_mask)

        if i_block_start is not None:
            rospy.loginfo(
                "[local_replanner] blockage on path [%d..%d] (robot at %d). "
                "Computing detour.", i_block_start, i_block_end, i_now)
            self._compute_and_apply_detour(
                i_now, i_block_start, i_block_end, persistent_mask)

        self._publish_viz(persistent_mask)

    def _compute_persistent_mask(self, obs, now):
        """! Compute persistent obstacle mask method
        Returns a boolean mask of cells that have been continuously observed
        for at least persistence_threshold seconds.
        @param obs: Current obstacle grid (float32, last-hit timestamp per cell)
        @param now: Current ROS time in seconds
        @return: Boolean mask, True = persistent obstacle
        """
        if not hasattr(self, "_obs_first") or self._obs_first is None \
                or self._obs_first.shape != obs.shape:
            self._obs_first = np.zeros_like(obs)

        newly_seen = (obs > 0) & (self._obs_first == 0)
        self._obs_first[newly_seen] = obs[newly_seen]

        decayed = (obs > 0) & ((now - obs) > self._clear_time)
        if np.any(decayed):
            obs[decayed] = 0.0
            self._obs_first[decayed] = 0.0

        persistent = ((self._obs_first > 0) &
                      ((now - self._obs_first) >= self._persistence_threshold) &
                      ((now - obs) <= self._clear_time))
        return persistent

    def _closest_path_index(self, path, xy):
        """! Closest path index method
        Returns the index of the nearest point on path to xy (global search).
        @param path: list of (x, y) tuples
        @param xy: (x, y) query point
        @return: index into path
        """
        if not path:
            return 0
        px = np.array([p[0] for p in path])
        py = np.array([p[1] for p in path])
        d2 = (px - xy[0]) ** 2 + (py - xy[1]) ** 2
        return int(np.argmin(d2))

    def _closest_path_index_windowed(self, path, xy, last_i,
                                     back_window=10, fwd_window=50):
        """! Windowed closest path index method
        Same as _closest_path_index but restricted to a forward window around
        last_i. Prevents argmin from jumping to a geometrically close but
        physically distant section of the path (e.g. post-rejoin baseline
        when the robot is still on the detour arc).
        @param path: list of (x, y) tuples
        @param xy: (x, y) query point
        @param last_i: last known index (None triggers global search)
        @param back_window: how many steps behind last_i to allow
        @param fwd_window: how many steps ahead of last_i to search
        @return: index into path
        """
        if not path:
            return 0
        if last_i is None:
            return self._closest_path_index(path, xy)

        last_i = max(0, min(last_i, len(path) - 1))
        lo = max(0, last_i - back_window)
        hi = min(len(path), last_i + fwd_window + 1)
        if lo >= hi:
            return last_i

        px = np.array([p[0] for p in path[lo:hi]])
        py = np.array([p[1] for p in path[lo:hi]])
        d2 = (px - xy[0]) ** 2 + (py - xy[1]) ** 2
        return lo + int(np.argmin(d2))

    def _find_blocked_span(self, path, i_start, persistent_mask):
        """! Find blocked span method
        Scans forward from i_start for path points inside the persistent
        obstacle mask, up to lookahead_check_m metres ahead.
        @param path: list of (x, y) tuples
        @param i_start: starting index on path
        @param persistent_mask: boolean obstacle mask
        @return: (first_blocked_index, last_blocked_index) or (None, None)
        """
        if not path:
            return None, None

        max_steps = int(self._lookahead_check_m / self._densify_step)
        i_end     = min(len(path), i_start + max_steps)

        blocked = [i for i in range(i_start, i_end)
                   if self._point_in_mask(path[i][0], path[i][1], persistent_mask)]

        if not blocked:
            return None, None
        return blocked[0], blocked[-1]

    def _point_in_mask(self, x, y, mask):
        """! Point in mask check method
        @param x: x coordinate in map frame
        @param y: y coordinate in map frame
        @param mask: boolean obstacle mask
        @return: True if (x, y) falls inside a masked cell
        """
        gx = int((x - self._obs_ox) / self._map_res)
        gy = int((y - self._obs_oy) / self._map_res)
        if gx < 0 or gx >= self._obs_grid_w or gy < 0 or gy >= self._obs_grid_h:
            return False
        return bool(mask[gy, gx])

    def _compute_and_apply_detour(self, i_now, i_block_start, i_block_end,
                                  persistent_mask):
        """! Compute and apply detour method
        Runs A* around the blocked span and splices the result into
        current_path. Detours are permanent — the baseline is never restored
        after an obstacle is classified as static.
        @param i_now: current robot index on current_path
        @param i_block_start: first blocked path index
        @param i_block_end: last blocked path index
        @param persistent_mask: boolean obstacle mask
        """
        path = self._current_path

        # Backstep 15 steps (3.0 m) before the block so the A* start cell
        # is safely outside the obstacle inflation radius.
        backstep      = max(i_now, max(0, i_block_start - 15))
        rejoin_steps  = int(self._rejoin_margin_m / self._densify_step)
        rejoin        = min(len(path) - 1, i_block_end + rejoin_steps)

        while rejoin < len(path) - 1 and self._point_in_mask(
                path[rejoin][0], path[rejoin][1], persistent_mask):
            rejoin += 1

        astar_grid = self._build_astar_grid(persistent_mask)
        if astar_grid is None:
            return

        p_start = path[backstep]
        p_goal  = path[rejoin]
        detour  = astar_plan(astar_grid, p_start[0], p_start[1],
                             p_goal[0], p_goal[1], connectivity=8)

        if not detour or detour == "GOAL_OCCUPIED":
            rospy.logwarn(
                "[local_replanner] A* detour failed (start=%s goal=%s). "
                "Holding current path.", p_start, p_goal)
            return

        densified = []
        for i in range(1, len(detour)):
            seg = self._densify_segment(detour[i - 1], detour[i], self._densify_step)
            densified.extend(seg[1:] if densified else seg)

        new_path                  = path[:backstep] + densified + path[rejoin:]
        self._current_path        = new_path
        self._detour_active       = True
        self._detour_end_index    = backstep + len(densified) - 1
        self._last_i_now          = backstep
        self._last_detour_points  = list(densified)

        self._publish_path(new_path)

        rospy.loginfo(
            "[local_replanner] DETOUR applied: backstep=%d rejoin=%d "
            "detour_pts=%d new_path_len=%d",
            backstep, rejoin, len(densified), len(new_path))

    def _build_astar_grid(self, persistent_mask):
        """! Build A* occupancy grid method
        Combines the inflated static map with inflated persistent obstacles
        to produce the grid used for detour planning.
        @param persistent_mask: boolean obstacle mask in obs_grid frame
        @return: SimpleOccupancyGrid, or None if the static map is not yet received
        """
        if self._static_inflated is None:
            return None

        free = self._static_inflated.copy()

        if np.any(persistent_mask):
            ox_cells = int(round((self._obs_ox - self._map_ox) / self._map_res))
            oy_cells = int(round((self._obs_oy - self._map_oy) / self._map_res))

            inflated_obs         = ~persistent_mask
            d                    = distance_transform_edt(inflated_obs) * self._map_res
            inflated_obs_blocked = d <= self._obstacle_inflate_m

            H, W = free.shape
            x0 = max(0, ox_cells);  x1 = min(W, ox_cells + self._obs_grid_w)
            y0 = max(0, oy_cells);  y1 = min(H, oy_cells + self._obs_grid_h)
            sx0 = x0 - ox_cells;    sx1 = sx0 + (x1 - x0)
            sy0 = y0 - oy_cells;    sy1 = sy0 + (y1 - y0)
            if x1 > x0 and y1 > y0:
                free[y0:y1, x0:x1] &= ~inflated_obs_blocked[sy0:sy1, sx0:sx1]

        data = np.where(free, 0, 100).astype(np.int16).reshape(-1).tolist()
        return SimpleOccupancyGrid(
            self._map_w, self._map_h, self._map_res,
            self._map_ox, self._map_oy, data)

    @staticmethod
    def _densify_segment(p1, p2, step=0.20):
        """! Densify segment method
        Interpolates intermediate points between p1 and p2 at spacing step.
        @param p1: start point (x, y)
        @param p2: end point (x, y)
        @param step: point spacing in metres
        @return: list of (x, y) tuples from p1 to p2
        """
        d = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if d <= step:
            return [p1, p2]
        n   = int(d / step)
        pts = [p1]
        for j in range(1, n + 1):
            a = j / float(n + 1)
            pts.append((p1[0] * (1 - a) + p2[0] * a,
                        p1[1] * (1 - a) + p2[1] * a))
        pts.append(p2)
        return pts

    def _publish_path(self, points):
        """! Publish path method
        @param points: list of (x, y) tuples to publish as nav_msgs/Path
        """
        msg = Path()
        msg.header.stamp    = rospy.Time.now()
        msg.header.frame_id = self._map_frame
        for x, y in points:
            ps = PoseStamped()
            ps.header             = msg.header
            ps.pose.position.x    = x
            ps.pose.position.y    = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._path_pub.publish(msg)

    def _publish_viz(self, persistent_mask):
        """! Publish visualization method
        Publishes the persistent obstacle grid and baseline/detour markers
        for RViz inspection.
        @param persistent_mask: boolean obstacle mask
        """
        if self._obs_grid is None:
            return

        now = rospy.Time.now()

        og = OccupancyGrid()
        og.header.stamp               = now
        og.header.frame_id            = self._map_frame
        og.info.resolution            = self._map_res
        og.info.width                 = self._obs_grid_w
        og.info.height                = self._obs_grid_h
        og.info.origin.position.x     = self._obs_ox
        og.info.origin.position.y     = self._obs_oy
        og.info.origin.orientation.w  = 1.0
        og.data = np.where(persistent_mask, 100, 0).astype(np.int8).reshape(-1).tolist()
        self._viz_grid_pub.publish(og)

        ma = MarkerArray()

        m_base = Marker()
        m_base.header.stamp       = now
        m_base.header.frame_id    = self._map_frame
        m_base.ns                 = "baseline"
        m_base.id                 = 0
        m_base.type               = Marker.LINE_STRIP
        m_base.action             = Marker.ADD
        m_base.scale.x            = 0.06
        m_base.color              = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.7)
        m_base.pose.orientation.w = 1.0
        for x, y in self._baseline_path:
            p = Point(); p.x = x; p.y = y; p.z = 0.02
            m_base.points.append(p)
        ma.markers.append(m_base)

        m_det = Marker()
        m_det.header.stamp       = now
        m_det.header.frame_id    = self._map_frame
        m_det.ns                 = "detour"
        m_det.id                 = 1
        m_det.type               = Marker.LINE_STRIP
        m_det.action             = Marker.ADD if self._last_detour_points else Marker.DELETE
        m_det.scale.x            = 0.12
        m_det.color              = ColorRGBA(r=1.0, g=0.4, b=0.0, a=0.95)
        m_det.pose.orientation.w = 1.0
        for x, y in self._last_detour_points:
            p = Point(); p.x = x; p.y = y; p.z = 0.05
            m_det.points.append(p)
        ma.markers.append(m_det)

        self._viz_marker_pub.publish(ma)


if __name__ == "__main__":
    node = LocalReplanner()
    node.run()