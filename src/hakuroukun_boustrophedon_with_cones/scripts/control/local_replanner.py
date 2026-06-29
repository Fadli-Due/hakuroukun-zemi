#!/usr/bin/env python3
# =============================================================================
#  local_replanner.py
#
#  Online path modification layer for the BCD coverage planner.
#  Sensei's brief: "generate optimal path online from the map, and basically
#  the robot follows it. If obstacle exists, modify it online by some method."
#
#  Architecture (two safety layers):
#    Layer 1 (in path_follower.py):  Reflex stop.  LiDAR FORWARD VETO at ~0.45m
#                                    fires in milliseconds. Handles sudden
#                                    intrusions (a child, etc.) — never depends
#                                    on this node.
#    Layer 2 (this node):            Deliberate detour.  Only kicks in when an
#                                    obstacle persists on the upcoming path for
#                                    >= persistence_threshold seconds. Then it
#                                    A*-routes around the obstacle and splices
#                                    the detour into the path.
#
#  Topic ownership:
#    /planned_path   (input, latched)  — original BCD path from
#                                        offline_coverage_planner.py
#    /desired_path   (output)          — modified path the follower tracks
#
#  Data flow per scan:
#    /scan_multi  ->  vectorised transform to map frame  ->  stamp obstacle
#    grid cells with `now`. A cell becomes "persistent" once it has been hit
#    continuously for `persistence_threshold` seconds. Cells not seen for
#    `clear_time` seconds decay back to free. The persistent grid is OR'd
#    onto the inflated static map to form the A* grid used for detours.
# =============================================================================
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import math
import threading
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

from planning.simple_astar import astar_plan, SimpleOccupancyGrid


# -----------------------------------------------------------------------------
class LocalReplanner:
    def __init__(self):
        rospy.init_node("local_replanner")

        ns = "local_replanner"
        # ---- persistence / decay -------------------------------------------
        # An obstacle must occupy the same cell continuously for this many
        # seconds before we route around it. Set high enough that a person
        # standing still briefly does NOT cause the robot to drive around them.
        self.persistence_threshold = rospy.get_param(f"{ns}/persistence_threshold", 7.0)
        # A cell is "cleared" once nothing has been seen there for this long.
        self.clear_time            = rospy.get_param(f"{ns}/clear_time", 1.0)
        # Local obstacle window centred on the robot, full side length in metres.
        self.window_size_m         = rospy.get_param(f"{ns}/window_size_m", 15.0)
        # How often we check whether a detour is needed (Hz). Independent of
        # the LiDAR rate -- scan stamping happens in scan_cb, evaluation runs
        # on this timer to keep A* off the scan callback's thread.
        self.eval_rate             = rospy.get_param(f"{ns}/eval_rate", 2.0)
        # How far ahead along the path we look for blockage (metres).
        self.lookahead_check_m     = rospy.get_param(f"{ns}/lookahead_check_m", 6.0)
        # Inflation applied to detected DYNAMIC obstacles when building the A*
        # detour grid (m). FIX (2026-06-25): was rospy.get_param("robot_radius", 1.0)
        # — global namespace so YAML's local_replanner/obstacle_inflate_m was silently
        # ignored. Now correctly namespaced.
        self.obstacle_inflate_m    = rospy.get_param(f"{ns}/obstacle_inflate_m", 1.0)
        # Inflation used for STATIC walls in the pre-computed A* free mask (m).
        # Must equal the BCD planner's robot_radius so every planned path point
        # lies inside the A* free space. Keeping this separate from obstacle_inflate_m
        # prevents baseline path points near walls from being falsely blocked when
        # obstacle_inflate_m > robot_radius (would cause "Goal cell occupied" errors).
        self.static_wall_inflate_m = rospy.get_param("robot_radius", 1.0)
        # LiDAR points closer than this are dropped (the robot's own footprint).
        self.scan_min_range        = rospy.get_param(f"{ns}/scan_min_range", 0.30)
        # LiDAR points farther than this are dropped (noise / out of useful range).
        self.scan_max_range        = rospy.get_param(f"{ns}/scan_max_range", 15.0)
        self.densify_step          = rospy.get_param("densify_step", 0.20)
        # When a detour is required, A* aims to rejoin the baseline this far
        # past the blocked span (metres) so the rejoin point is comfortably clear.
        self.rejoin_margin_m       = rospy.get_param(f"{ns}/rejoin_margin_m", 1.5)
        # Frame names
        self.map_frame             = rospy.get_param(f"{ns}/map_frame", "map")
        self.laser_frame           = rospy.get_param(f"{ns}/laser_frame", "laser_link")

        # ---- internal state ------------------------------------------------
        self.lock = threading.Lock()
        self.static_grid_msg = None       # nav_msgs/OccupancyGrid (raw /map)
        self.static_inflated = None       # bool array, True = free (after inflation)
        self.map_res = None
        self.map_ox = None
        self.map_oy = None
        self.map_w = self.map_h = None

        self.baseline_path = []           # list[(x,y)] from /planned_path
        self.current_path  = []           # list[(x,y)] currently being executed
        self.detour_active = False        # is the live path a modified one?
        self.last_detour_points = []      # for viz: the active detour polyline
        # Index in current_path where the detour arc ends (i.e. the rejoin point).
        # Used to keep the detour active until the robot has physically completed it,
        # regardless of whether the obstacle is still visible to the LiDAR.
        self.detour_end_index = None
        # Last computed index of the robot on current_path. Used to bound the
        # closest-point search to a forward window (mirrors path_follower's
        # monotonic tracking). Without this, np.argmin can jump from the detour
        # arc to the post-rejoin baseline tail when they pass geometrically close,
        # falsely indicating the arc is completed.
        self.last_i_now = None

        self.robot_xy = None              # (x, y) in map frame
        self.robot_yaw = 0.0

        # Local obstacle grid (last-seen-time grid). Allocated once the static
        # map arrives so we know the resolution.
        self.obs_grid = None              # float32 array, value = last-hit time (sec); 0 = never
        self.obs_grid_w = None
        self.obs_grid_h = None
        # Origin of obs_grid in map frame; updated as robot moves.
        self.obs_ox = 0.0
        self.obs_oy = 0.0

        # ---- TF ------------------------------------------------------------
        self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf)

        # ---- ROS I/O -------------------------------------------------------
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber("/planned_path", Path, self.baseline_cb, queue_size=1)
        rospy.Subscriber("/scan_multi", LaserScan, self.scan_cb, queue_size=1)
        rospy.Subscriber("/hakuroukun_pose/rear_wheel_odometry", Odometry,
                         self.odom_cb, queue_size=10)
        # /return_path: A* return-to-home leg computed by return_to_start.py
        # after coverage is done. When received, it is appended to current_path
        # and republished on /desired_path so the path_follower drives it.
        # Persistence-gated obstacle avoidance keeps working on the return leg
        # because evaluate() runs on current_path regardless of its origin.
        rospy.Subscriber("/return_path", Path, self.return_path_cb, queue_size=1)
        self.return_appended = False   # one-shot guard

        self.path_pub = rospy.Publisher("/desired_path", Path, queue_size=1, latch=True)

        # ---- Visualization publishers (for RViz debugging) ----------------
        # /local_replanner/obstacle_grid:  OccupancyGrid showing the persistent
        #                                  obstacle mask (occupied = blocked).
        # /local_replanner/baseline_marker: LINE_STRIP of the original BCD path.
        # /local_replanner/detour_marker:   LINE_STRIP of the active detour (if any).
        self.viz_grid_pub  = rospy.Publisher("/local_replanner/obstacle_grid",
                                             OccupancyGrid, queue_size=1, latch=True)
        self.viz_marker_pub = rospy.Publisher("/local_replanner/markers",
                                              MarkerArray, queue_size=1, latch=True)

        rospy.Timer(rospy.Duration(1.0 / self.eval_rate), self.evaluate)

        rospy.loginfo(
            "[local_replanner] up. persistence=%.1fs clear=%.1fs window=%.1fm "
            "lookahead_check=%.1fm inflate=%.2fm",
            self.persistence_threshold, self.clear_time, self.window_size_m,
            self.lookahead_check_m, self.obstacle_inflate_m)

    # ====================================================================
    #  CALLBACKS
    # ====================================================================
    def map_cb(self, msg):
        """Cache the static map and pre-compute its inflated free mask."""
        with self.lock:
            self.static_grid_msg = msg
            self.map_res = msg.info.resolution
            self.map_ox  = msg.info.origin.position.x
            self.map_oy  = msg.info.origin.position.y
            self.map_w   = msg.info.width
            self.map_h   = msg.info.height

            raw = np.array(msg.data, dtype=np.int16).reshape((self.map_h, self.map_w))
            free = (raw == 0)
            dist = distance_transform_edt(free) * self.map_res
            # Use static_wall_inflate_m (= robot_radius) for wall clearance so
            # BCD-planned path points are always inside the A* free space.
            # obstacle_inflate_m is reserved for dynamic obstacles only.
            self.static_inflated = free & (dist > self.static_wall_inflate_m)

            # Allocate the local obstacle grid (last-hit time per cell).
            side_cells = int(math.ceil(self.window_size_m / self.map_res))
            self.obs_grid_w = side_cells
            self.obs_grid_h = side_cells
            self.obs_grid = np.zeros((side_cells, side_cells), dtype=np.float32)
            rospy.loginfo(
                "[local_replanner] static map ready (%dx%d cells, res=%.3fm), "
                "obstacle grid %dx%d cells.",
                self.map_w, self.map_h, self.map_res, side_cells, side_cells)

    def baseline_cb(self, msg):
        """Latched baseline path from offline_coverage_planner."""
        with self.lock:
            self.baseline_path = [(p.pose.position.x, p.pose.position.y)
                                  for p in msg.poses]
            # On first reception or when no detour is active, publish baseline
            # straight through. If a detour is active when a new baseline
            # arrives (re-plan), drop the detour -- the new baseline wins.
            self.current_path = list(self.baseline_path)
            self.detour_active = False
            self.detour_end_index = None
            self.last_i_now = None
            # A new baseline starts a fresh coverage run — allow another
            # return-leg append at the end of it.
            self.return_appended = False
        self._publish(self.current_path)
        rospy.loginfo("[local_replanner] baseline path received: %d points",
                      len(self.baseline_path))

    def return_path_cb(self, msg):
        """Append an A* return-to-home leg from return_to_start.py.

        The return path is appended onto the tail of current_path so the
        path_follower drives from the end of coverage straight into the
        return leg with no gap. Persistence-gated obstacle avoidance keeps
        running on this combined path, so a person walking into the return
        corridor still triggers a detour.

        One-shot: subsequent /return_path messages within the same baseline
        session are ignored. A new /planned_path resets the guard.
        """
        with self.lock:
            if self.return_appended:
                rospy.logwarn(
                    "[local_replanner] /return_path received but a return leg "
                    "was already appended this session — ignoring.")
                return
            if not self.current_path:
                rospy.logwarn(
                    "[local_replanner] /return_path received before any "
                    "baseline — ignoring.")
                return

            return_pts = [(p.pose.position.x, p.pose.position.y)
                          for p in msg.poses]
            if not return_pts:
                rospy.logwarn("[local_replanner] /return_path is empty — ignoring.")
                return

            # Append. current_path may contain detours already; that's fine —
            # the return leg sits after whatever the existing tail is.
            self.current_path = list(self.current_path) + return_pts
            self.return_appended = True
            # last_i_now refers to current_path; appending doesn't invalidate
            # the existing index. Leave it untouched.

        self._publish(self.current_path)
        rospy.loginfo(
            "[local_replanner] return leg appended: %d points, "
            "new current_path length = %d.",
            len(return_pts), len(self.current_path))

    def odom_cb(self, msg):
        try:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
            ps.header.stamp = rospy.Time(0)
            ps_map = self.tf_buf.transform(ps, self.map_frame, rospy.Duration(0.1))
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[local_replanner] odom->map TF: {e}")
            return
        from scipy.spatial.transform import Rotation
        q = ps_map.pose.orientation
        self.robot_xy = (ps_map.pose.position.x, ps_map.pose.position.y)
        self.robot_yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("zyx")[0]

    def scan_cb(self, msg):
        """Stamp the local obstacle grid with the current scan.

        Vectorised laser_link -> map transform. Cost: ~150 points per scan,
        one TF lookup + one matrix multiply + one indexed assignment. ~1ms.
        """
        if self.obs_grid is None or self.robot_xy is None:
            return

        # 1) Re-centre the obstacle grid window on the robot (cheap if no shift).
        self._maybe_recenter_obs_grid()

        # 2) Lookup laser_frame -> map_frame ONCE.
        try:
            tr = self.tf_buf.lookup_transform(
                self.map_frame, msg.header.frame_id or self.laser_frame,
                rospy.Time(0), rospy.Duration(0.1))
        except Exception as e:
            rospy.logwarn_throttle(2.0,
                f"[local_replanner] laser->map TF: {e}")
            return

        t = tr.transform.translation
        q = tr.transform.rotation
        # Build 2x2 rotation from quaternion (yaw only — flat ground assumption)
        # yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)

        # 3) Build vectorised point cloud.
        n = len(msg.ranges)
        if n == 0:
            return
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        ranges = np.asarray(msg.ranges, dtype=np.float32)

        valid = (np.isfinite(ranges) &
                 (ranges >= max(msg.range_min, self.scan_min_range)) &
                 (ranges <= min(msg.range_max, self.scan_max_range)))
        if not np.any(valid):
            return
        r = ranges[valid]
        a = angles[valid]

        # Points in laser frame.
        xl = r * np.cos(a)
        yl = r * np.sin(a)

        # Rotate + translate into map frame.
        xm = c * xl - s * yl + t.x
        ym = s * xl + c * yl + t.y

        # 4) Stamp grid cells.
        gx = ((xm - self.obs_ox) / self.map_res).astype(np.int32)
        gy = ((ym - self.obs_oy) / self.map_res).astype(np.int32)
        in_bounds = ((gx >= 0) & (gx < self.obs_grid_w) &
                     (gy >= 0) & (gy < self.obs_grid_h))
        if not np.any(in_bounds):
            return
        now = rospy.Time.now().to_sec()
        with self.lock:
            self.obs_grid[gy[in_bounds], gx[in_bounds]] = now

    def _maybe_recenter_obs_grid(self):
        """Slide the obstacle window so the robot stays roughly centred.

        We only shift when the robot has moved by more than ~1/4 of the window;
        each shift translates the array (cells leaving the window are dropped,
        new cells enter as zero = never seen). This keeps the grid local and
        cheap regardless of how big the static map is.
        """
        if self.obs_grid is None or self.robot_xy is None:
            return
        half = (self.obs_grid_w * self.map_res) / 2.0
        # Desired bottom-left so the robot sits at the centre.
        desired_ox = self.robot_xy[0] - half
        desired_oy = self.robot_xy[1] - half
        dx_cells = int(round((desired_ox - self.obs_ox) / self.map_res))
        dy_cells = int(round((desired_oy - self.obs_oy) / self.map_res))
        # Only bother shifting if the drift exceeds 1/4 window.
        if abs(dx_cells) < self.obs_grid_w // 4 and abs(dy_cells) < self.obs_grid_h // 4:
            return

        with self.lock:
            new = np.zeros_like(self.obs_grid)
            # Copy overlap region from old grid into new grid (shifted).
            src_x0 = max(0, dx_cells)
            src_y0 = max(0, dy_cells)
            src_x1 = min(self.obs_grid_w, self.obs_grid_w + dx_cells)
            src_y1 = min(self.obs_grid_h, self.obs_grid_h + dy_cells)
            dst_x0 = src_x0 - dx_cells
            dst_y0 = src_y0 - dy_cells
            dst_x1 = src_x1 - dx_cells
            dst_y1 = src_y1 - dy_cells
            if src_x1 > src_x0 and src_y1 > src_y0:
                new[dst_y0:dst_y1, dst_x0:dst_x1] = \
                    self.obs_grid[src_y0:src_y1, src_x0:src_x1]
            self.obs_grid = new
            self.obs_ox = self.obs_ox + dx_cells * self.map_res
            self.obs_oy = self.obs_oy + dy_cells * self.map_res

    # ====================================================================
    #  EVALUATION TIMER
    # ====================================================================
    def evaluate(self, _evt):
        """Decide whether to detour. Runs at eval_rate Hz."""
        if self.static_inflated is None:
            return
        if not self.baseline_path or self.robot_xy is None:
            return

        now = rospy.Time.now().to_sec()

        # 1) Snapshot the persistent obstacle mask: cells continuously hit for
        #    >= persistence_threshold, AND seen within clear_time.
        with self.lock:
            obs = self.obs_grid
            persistent_mask = self._compute_persistent_mask(obs, now)

        # 2) Find the robot's index on the current path using monotonic
        # windowed search. This prevents argmin from jumping to the
        # post-rejoin baseline tail when the detour arc passes geometrically
        # near it (which would falsely signal arc completion).
        i_now = self._closest_path_index_windowed(
            self.current_path, self.robot_xy, self.last_i_now)
        self.last_i_now = i_now

        # 3) Find the first blocked index ahead within lookahead_check_m.
        i_block_start, i_block_end = self._find_blocked_span(
            self.current_path, i_now, persistent_mask)

        if i_block_start is None:
            # No persistent obstacle ahead on current_path. Do nothing.
            #
            # Design choice (2026-06-25): detours are PERMANENT once applied.
            # The persistence-gated logic only fires a detour after an obstacle
            # has remained in place for >= persistence_threshold seconds, at
            # which point it is classified as static and assumed to stay.
            # Reverting to the pristine baseline would contradict this model —
            # losing sight of a static obstacle (e.g. it left the LiDAR FOV
            # as the robot turned) does NOT mean the obstacle is gone, and
            # restoring the baseline path can send the robot back toward it.
            #
            # If another persistent obstacle appears later, _compute_and_apply_detour
            # will splice a new detour onto the already-modified current_path
            # — the algorithm always works on self.current_path, never on the
            # pristine baseline, so multiple detours stack correctly.
            pass
        else:
            # 4) A blockage exists. Compute and splice a detour.
            rospy.loginfo(
                "[local_replanner] blockage on path [%d..%d] (robot at %d). "
                "Computing detour.", i_block_start, i_block_end, i_now)

            self._compute_and_apply_detour(i_now, i_block_start, i_block_end,
                                           persistent_mask)

        # 5) Publish visualization (always, so RViz sees the mask even when clear).
        self._publish_viz(persistent_mask)

    def _compute_persistent_mask(self, obs, now):
        """Return a bool mask of the obs grid where cells count as obstacles.

        Approximation: a cell is treated as a real obstacle if it was last
        seen within `clear_time` AND the time since `now - persistence_threshold`
        the cell has been continuously stamped. Since we don't track first-seen,
        we approximate: stamped recently (live) AND the timestamp is at least
        `persistence_threshold` seconds in the past relative to a sliding
        "established" line. In practice this means we test:
            obs > 0 AND (now - obs) < clear_time AND obs has existed
            long enough — see persistence_history below.
        For simplicity and robustness in the first version, the rule is:
            cell is obstacle if it has been stamped recently AND the
            FIRST stamp is older than persistence_threshold seconds.

        We track first-stamp times in `obs_first` (allocated lazily).
        """
        if not hasattr(self, "obs_first") or self.obs_first is None \
                or self.obs_first.shape != obs.shape:
            self.obs_first = np.zeros_like(obs)

        # Update first-seen: cells newly stamped (first ~= 0 but obs > 0) → set first = obs.
        newly_seen = (obs > 0) & (self.obs_first == 0)
        self.obs_first[newly_seen] = obs[newly_seen]

        # Cells that have decayed away (not seen for > clear_time): forget them.
        decayed = (obs > 0) & ((now - obs) > self.clear_time)
        if np.any(decayed):
            obs[decayed] = 0.0
            self.obs_first[decayed] = 0.0

        # Persistent = first-seen long ago, AND still being stamped.
        persistent = ((self.obs_first > 0) &
                      ((now - self.obs_first) >= self.persistence_threshold) &
                      ((now - obs) <= self.clear_time))
        return persistent

    def _closest_path_index(self, path, xy):
        if not path:
            return 0
        px = np.array([p[0] for p in path])
        py = np.array([p[1] for p in path])
        d2 = (px - xy[0]) ** 2 + (py - xy[1]) ** 2
        return int(np.argmin(d2))

    def _closest_path_index_windowed(self, path, xy, last_i,
                                     back_window=10, fwd_window=50):
        """Closest path index restricted to a forward window around last_i.

        Mirrors path_follower's monotonic tracking. Prevents 'argmin jumps'
        when the path doubles back near itself — e.g., a detour arc that
        loops around an obstacle and rejoins the baseline. In that case a
        global argmin can return a post-rejoin baseline point even though
        the robot is physically still on the arc, leading to a premature
        'arc completed' decision and an unwanted baseline restore.

        - back_window allows small reverse motion (REVERSE mode) without
          locking the index in place.
        - fwd_window must be large enough to track forward motion between
          eval ticks (eval_rate=2 Hz, robot ~0.35 m/s, densify=0.20 m
          → ~1 index per tick; 50 is comfortably generous).
        - Falls back to global argmin when last_i is None (first call or
          immediately after a path swap)."""
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
        """Scan forward from i_start. Return (first_blocked, last_blocked)
        in path-index space, or (None, None) if no blockage within lookahead.

        Looks ahead at most lookahead_check_m metres along the path."""
        if not path:
            return None, None
        max_steps = int(self.lookahead_check_m / self.densify_step)
        i_end = min(len(path), i_start + max_steps)

        blocked = []
        for i in range(i_start, i_end):
            x, y = path[i]
            if self._point_in_mask(x, y, persistent_mask):
                blocked.append(i)

        if not blocked:
            return None, None
        return blocked[0], blocked[-1]

    def _point_in_mask(self, x, y, mask):
        """Is (x,y) in map frame inside the persistent obstacle mask?"""
        gx = int((x - self.obs_ox) / self.map_res)
        gy = int((y - self.obs_oy) / self.map_res)
        if gx < 0 or gx >= self.obs_grid_w or gy < 0 or gy >= self.obs_grid_h:
            return False
        return bool(mask[gy, gx])

    # ====================================================================
    #  DETOUR
    # ====================================================================
    def _compute_and_apply_detour(self, i_now, i_block_start, i_block_end,
                                  persistent_mask):
        """A* around the blocked span and splice into the path."""
        path = self.current_path

        # Choose detour endpoints: backstep far enough before the block that the
        # A* start cell is outside the obstacle inflation zone, rejoin past the block.
        #
        # FIX (2026-06-25): was `i_block_start - 5` (5 steps × 0.2m = 1.0m backstep).
        # With obstacle_inflate_m = 1.0–1.2m that placed the A* start cell exactly inside
        # the inflation bubble → "[A*] Start cell is occupied/unknown!" on every call.
        # 15 steps (3.0m) >> obstacle_inflate_m (1.2m), giving a comfortably clear start.
        # max(i_now, …) ensures we never backstep behind the robot's current position —
        # path[i_now] was physically visited so it is guaranteed free in the dynamic grid.
        backstep = max(i_now, max(0, i_block_start - 15))
        rejoin_steps = int(self.rejoin_margin_m / self.densify_step)
        rejoin = min(len(path) - 1, i_block_end + rejoin_steps)

        # Walk further forward until the rejoin point is itself clear of the
        # mask (don't rejoin into an obstacle).
        while rejoin < len(path) - 1 and self._point_in_mask(
                path[rejoin][0], path[rejoin][1], persistent_mask):
            rejoin += 1

        # Build the inflated A* grid: static_inflated AND-NOT persistent_obs.
        astar_grid = self._build_astar_grid(persistent_mask)
        if astar_grid is None:
            return

        p_start = path[backstep]
        p_goal  = path[rejoin]
        detour = astar_plan(astar_grid, p_start[0], p_start[1],
                            p_goal[0], p_goal[1], connectivity=8)

        if not detour or detour == "GOAL_OCCUPIED":
            rospy.logwarn(
                "[local_replanner] A* detour failed (start=%s goal=%s). "
                "Holding current path; reflex-stop will keep robot safe.",
                p_start, p_goal)
            return

        # Densify the A* polyline to the same step the baseline uses (0.20m).
        densified = []
        for i in range(1, len(detour)):
            seg = self._densify(detour[i - 1], detour[i], step=self.densify_step)
            if densified:
                densified.extend(seg[1:])
            else:
                densified.extend(seg)

        # Splice: [baseline up to backstep] + [detour] + [baseline from rejoin].
        new_path = path[:backstep] + densified + path[rejoin:]
        self.current_path = new_path
        self.detour_active = True
        # Record the index in current_path where the detour arc ENDS.
        # current_path layout: [0 .. backstep-1] baseline | [backstep .. backstep+len(densified)-1] detour | rest baseline
        # The robot must reach this index before it is safe to restore the baseline.
        self.detour_end_index = backstep + len(densified) - 1
        # Seed the windowed index at backstep — the robot is physically at the
        # start of the detour arc when the detour is applied.
        self.last_i_now = backstep
        self.last_detour_points = list(densified)   # for visualization
        self._publish(new_path)
        rospy.loginfo(
            "[local_replanner] DETOUR applied: backstep=%d rejoin=%d, "
            "detour points=%d, new path length=%d",
            backstep, rejoin, len(densified), len(new_path))

    def _maybe_restore_baseline(self, i_now):
        """If the blocked region is gone, swap the live path back to baseline."""
        if not self.baseline_path:
            return
        self.current_path = list(self.baseline_path)
        self.detour_active = False
        self.detour_end_index = None
        # Path identity changed → invalidate cached index. The next evaluate()
        # will do one global argmin to relocate the robot in the new path.
        self.last_i_now = None
        self.last_detour_points = []  # clear viz
        self._publish(self.baseline_path)
        rospy.loginfo("[local_replanner] obstacle cleared — baseline restored.")

    def _build_astar_grid(self, persistent_mask):
        """Build a SimpleOccupancyGrid combining the inflated static map with
        the inflated persistent obstacles. Both are inflated by robot_radius."""
        if self.static_inflated is None:
            return None

        # Start with the inflated static map (True = free).
        free = self.static_inflated.copy()

        # Now subtract the persistent obstacles, also inflated.
        if np.any(persistent_mask):
            # The persistent mask is in the local obs_grid frame; project it
            # into the global map grid by computing the offset.
            ox_cells = int(round((self.obs_ox - self.map_ox) / self.map_res))
            oy_cells = int(round((self.obs_oy - self.map_oy) / self.map_res))

            # Inflate the obstacle mask (cells within obstacle_inflate_m of
            # any persistent cell are treated as blocked).
            inflated_obs = ~persistent_mask  # True = clear-of-obstacle
            d = distance_transform_edt(inflated_obs) * self.map_res
            inflated_obs_blocked = d <= self.obstacle_inflate_m  # True = within infl

            # OR the inflated obstacles into the full-map free grid.
            H, W = free.shape
            # Determine overlap region between obs window and full map.
            x0 = max(0, ox_cells); x1 = min(W, ox_cells + self.obs_grid_w)
            y0 = max(0, oy_cells); y1 = min(H, oy_cells + self.obs_grid_h)
            sx0 = x0 - ox_cells; sx1 = sx0 + (x1 - x0)
            sy0 = y0 - oy_cells; sy1 = sy0 + (y1 - y0)
            if x1 > x0 and y1 > y0:
                free[y0:y1, x0:x1] &= ~inflated_obs_blocked[sy0:sy1, sx0:sx1]

        data = np.where(free, 0, 100).astype(np.int16).reshape(-1).tolist()
        return SimpleOccupancyGrid(
            self.map_w, self.map_h, self.map_res,
            self.map_ox, self.map_oy, data)

    # ====================================================================
    #  HELPERS
    # ====================================================================
    @staticmethod
    def _densify(p1, p2, step=0.20):
        d = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if d <= step:
            return [p1, p2]
        n = int(d / step)
        pts = [p1]
        for j in range(1, n + 1):
            a = j / float(n + 1)
            pts.append((p1[0] * (1 - a) + p2[0] * a,
                        p1[1] * (1 - a) + p2[1] * a))
        pts.append(p2)
        return pts

    def _publish(self, points):
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        for x, y in points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    # ====================================================================
    #  VISUALIZATION  (RViz debugging — purely informational)
    # ====================================================================
    def _publish_viz(self, persistent_mask):
        """Publish the persistent-obstacle grid and detour/baseline markers."""
        if self.obs_grid is None:
            return
        now = rospy.Time.now()

        # 1) Persistent mask as an OccupancyGrid (100 = occupied, 0 = free).
        og = OccupancyGrid()
        og.header.stamp = now
        og.header.frame_id = self.map_frame
        og.info.resolution = self.map_res
        og.info.width      = self.obs_grid_w
        og.info.height     = self.obs_grid_h
        og.info.origin.position.x = self.obs_ox
        og.info.origin.position.y = self.obs_oy
        og.info.origin.orientation.w = 1.0
        data = np.where(persistent_mask, 100, 0).astype(np.int8)
        og.data = data.reshape(-1).tolist()
        self.viz_grid_pub.publish(og)

        # 2) Baseline and detour LINE_STRIP markers.
        ma = MarkerArray()

        m_base = Marker()
        m_base.header.stamp = now
        m_base.header.frame_id = self.map_frame
        m_base.ns = "baseline"
        m_base.id = 0
        m_base.type = Marker.LINE_STRIP
        m_base.action = Marker.ADD
        m_base.scale.x = 0.06
        m_base.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.7)  # blue
        m_base.pose.orientation.w = 1.0
        for x, y in self.baseline_path:
            p = Point(); p.x = x; p.y = y; p.z = 0.02
            m_base.points.append(p)
        ma.markers.append(m_base)

        m_det = Marker()
        m_det.header.stamp = now
        m_det.header.frame_id = self.map_frame
        m_det.ns = "detour"
        m_det.id = 1
        m_det.type = Marker.LINE_STRIP
        m_det.action = Marker.ADD if self.last_detour_points else Marker.DELETE
        m_det.scale.x = 0.12
        m_det.color = ColorRGBA(r=1.0, g=0.4, b=0.0, a=0.95)  # orange
        m_det.pose.orientation.w = 1.0
        for x, y in self.last_detour_points:
            p = Point(); p.x = x; p.y = y; p.z = 0.05
            m_det.points.append(p)
        ma.markers.append(m_det)

        self.viz_marker_pub.publish(ma)


if __name__ == "__main__":
    LocalReplanner()
    rospy.spin()