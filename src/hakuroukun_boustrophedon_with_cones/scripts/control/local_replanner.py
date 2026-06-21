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
        # Inflation applied to detected obstacles for A* (matches the
        # planner's robot_radius by default).
        self.obstacle_inflate_m    = rospy.get_param("robot_radius", 1.0)
        # LiDAR points closer than this are dropped (the robot's own footprint).
        self.scan_min_range        = rospy.get_param(f"{ns}/scan_min_range", 0.30)
        # LiDAR points farther than this are dropped (noise / out of useful range).
        self.scan_max_range        = rospy.get_param(f"{ns}/scan_max_range", 15.0)
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

        # Cached "free" boolean array from the last _build_astar_grid() call.
        # Used by _compute_and_apply_detour() to validate backstep/rejoin
        # against the actual occupancy A* will plan on, instead of guessing
        # with a fixed offset (see 2026-06-21 debug session).
        self._last_astar_free = None

        # ---- TF ------------------------------------------------------------
        self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf)

        # ---- ROS I/O -------------------------------------------------------
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber("/planned_path", Path, self.baseline_cb, queue_size=1)
        rospy.Subscriber("/scan_multi", LaserScan, self.scan_cb, queue_size=1)
        rospy.Subscriber("/hakuroukun_pose/rear_wheel_odometry", Odometry,
                         self.odom_cb, queue_size=10)

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
            self.static_inflated = free & (dist > self.obstacle_inflate_m)

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
        self._publish(self.current_path)
        rospy.loginfo("[local_replanner] baseline path received: %d points",
                      len(self.baseline_path))

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

        FIX (2026-06-21): this now shifts `obs_first` in lockstep with
        `obs_grid`. Previously only obs_grid was translated -- obs_first kept
        its old, now-misaligned contents, so after any recenter the "first
        seen" timer for a physical obstacle location was desynced from the
        live obs grid. In practice `newly_seen` (in _compute_persistent_mask)
        fired again right after every recenter -- the box's cells looked
        "never seen" at their new array indices even though they'd been
        tracked for seconds under the old indices -- so persistence_threshold
        could never be reached if recenters happened more often than
        persistence_threshold seconds apart.
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
            new_first = (np.zeros_like(self.obs_first)
                         if getattr(self, "obs_first", None) is not None else None)

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
                if new_first is not None:
                    new_first[dst_y0:dst_y1, dst_x0:dst_x1] = \
                        self.obs_first[src_y0:src_y1, src_x0:src_x1]

            # --- [lr-debug] diagnostic: confirm recenter frequency ---------
            rospy.loginfo(
                "[lr-debug] RECENTER fired: dx=%d dy=%d cells "
                "(threshold=1/4 window=%.2fm)",
                dx_cells, dy_cells, self.obs_grid_w * self.map_res / 4.0)
            # -----------------------------------------------------------------

            self.obs_grid = new
            if new_first is not None:
                self.obs_first = new_first
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

        # 2) Find the robot's index on the current path (search forward only).
        i_now = self._closest_path_index(self.current_path, self.robot_xy)

        # 3) Find the first blocked index ahead within lookahead_check_m.
        #
        #    IMPORTANT: once a detour is active, self.current_path IS the
        #    detour -- it was built specifically to route around the
        #    obstacle, so checking it for blockage will (correctly) always
        #    come back clear and immediately look like "obstacle is gone".
        #    That caused a detour-apply / baseline-restore flap every
        #    ~0.5s (see local_replanner debug session, 2026-06-21): the
        #    obstacle never actually moved, only the path being checked did.
        #
        #    So while a detour is active, check persistence against the
        #    ORIGINAL baseline path instead -- that's the only path whose
        #    "is the obstacle still here" status is meaningful. We still
        #    track i_now on current_path (for splicing/backstep math), but
        #    we evaluate blockage against baseline_path.
        check_path = self.baseline_path if self.detour_active else self.current_path
        i_now_baseline = (self._closest_path_index(self.baseline_path, self.robot_xy)
                          if self.detour_active else i_now)
        i_block_start, i_block_end = self._find_blocked_span(
            check_path, i_now_baseline, persistent_mask)

        if i_block_start is None:
            # No persistent obstacle ahead on the relevant path. If a detour
            # was active, it's now safe to restore the baseline.
            if self.detour_active:
                self._maybe_restore_baseline(i_now)
        else:
            # 4) A blockage exists. Compute and splice a detour.
            #    Skip if a detour is already active and still covers this
            #    span -- avoids recomputing/resplicing every eval tick.
            if self.detour_active:
                rospy.logdebug_throttle(
                    2.0,
                    "[local_replanner] obstacle still blocks baseline "
                    "[%d..%d] -- detour remains active.",
                    i_block_start, i_block_end)
            else:
                rospy.loginfo(
                    "[local_replanner] blockage on path [%d..%d] (robot at %d). "
                    "Computing detour.", i_block_start, i_block_end, i_now)

                self._compute_and_apply_detour(i_now, i_block_start, i_block_end,
                                               persistent_mask)

        # 5) Publish visualization (always, so RViz sees the mask even when clear).
        self._publish_viz(persistent_mask)

    def _compute_persistent_mask(self, obs, now):
        """Return a bool mask of the obs grid where cells count as obstacles.

        Two timers per cell:
          * obs        — last time the cell was hit by LiDAR.
          * obs_first  — first time the cell entered continuous tracking.

        Decay rules (revised 2026-06-21 after the obstacle_grid stayed empty
        even with the box physically in front of the robot):

          * clear_time  = brief-gap tolerance for *live tracking*.
                          A cell only counts as a persistent obstacle if it
                          has been hit within clear_time.

          * persistence_reset_time = how long without ANY hit before we
                          erase obs_first too. Must be MUCH longer than
                          clear_time, otherwise any brief laser gap wipes
                          seconds of accumulated persistence.

        NOTE: this timer logic only holds if obs_first stays spatially
        aligned with obs across grid recenters -- see the fix in
        _maybe_recenter_obs_grid above.
        """
        if not hasattr(self, "obs_first") or self.obs_first is None \
                or self.obs_first.shape != obs.shape:
            self.obs_first = np.zeros_like(obs)

        # First-seen: set the moment a cell becomes tracked.
        newly_seen = (obs > 0) & (self.obs_first == 0)
        self.obs_first[newly_seen] = obs[newly_seen]

        # Fully forget a cell only after a long absence.
        persistence_reset_time = max(self.clear_time * 5.0, 5.0)
        fully_gone = (obs > 0) & ((now - obs) > persistence_reset_time)
        if np.any(fully_gone):
            obs[fully_gone] = 0.0
            self.obs_first[fully_gone] = 0.0

        # Persistent = first-seen long enough ago AND still live.
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

    def _find_blocked_span(self, path, i_start, persistent_mask):
        """Scan forward from i_start. Return (first_blocked, last_blocked)
        in path-index space, or (None, None) if no blockage within lookahead.

        Looks ahead at most lookahead_check_m metres along the path."""
        if not path:
            return None, None
        max_steps = int(self.lookahead_check_m / 0.20)  # densify_step is 0.20m
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
        """A* around the blocked span and splice into the path.

        FIX (2026-06-21, second debug session): backstep and rejoin are now
        validated against the actual A* occupancy grid we plan on, not a
        fixed offset (backstep) or the raw uninflated mask (rejoin).

        Previously: backstep = i_block_start - 5 = 1.0m before the block
        (5 points * 0.20m densify). Coincidentally equal to obstacle_inflate_m
        (= robot_radius = 1.0m by default), so the backstep landed almost
        exactly on the inflation boundary -- frequently inside it after
        grid rounding. A* then rejected the start with "Start cell is
        occupied/unknown!" every eval tick. Rejoin had a walk-forward loop
        but checked persistent_mask (raw cells) instead of the inflated grid,
        so even rejoin's validation was looking at the wrong thing.

        Now: build the A* grid first, cache its `free` boolean, then walk
        backstep backward and rejoin forward against that same array until
        each is genuinely free. Removes the 1.0m-vs-1.0m coincidence and
        adapts to whatever inflation radius is actually in use.
        """
        path = self.current_path

        # Build the inflated A* grid FIRST so we can validate endpoints
        # against the same occupancy A* will plan on.
        astar_grid = self._build_astar_grid(persistent_mask)
        if astar_grid is None:
            return

        free = self._last_astar_free
        if free is None:
            rospy.logwarn("[local_replanner] no cached free grid -- skipping detour.")
            return

        def _is_free(idx):
            x, y = path[idx]
            gx = int(round((x - self.map_ox) / self.map_res))
            gy = int(round((y - self.map_oy) / self.map_res))
            if gx < 0 or gx >= self.map_w or gy < 0 or gy >= self.map_h:
                return False
            return bool(free[gy, gx])

        # Backstep: start at i_block_start - 5 and walk BACKWARD until we
        # find a cell that's genuinely free of inflated obstacles.
        backstep = max(0, i_block_start - 5)
        while backstep > 0 and not _is_free(backstep):
            backstep -= 1

        # Rejoin: start rejoin_margin past the block and walk FORWARD until
        # we find a cell that's genuinely free.
        rejoin_steps = int(self.rejoin_margin_m / 0.20)
        rejoin = min(len(path) - 1, i_block_end + rejoin_steps)
        while rejoin < len(path) - 1 and not _is_free(rejoin):
            rejoin += 1

        # If we walked off either end without finding a free cell, give up
        # cleanly -- A* would fail anyway and reflex-stop keeps us safe.
        if not _is_free(backstep) or not _is_free(rejoin):
            rospy.logwarn(
                "[local_replanner] could not find free backstep/rejoin "
                "(backstep=%d free=%s, rejoin=%d free=%s) -- holding path.",
                backstep, _is_free(backstep), rejoin, _is_free(rejoin))
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
            seg = self._densify(detour[i - 1], detour[i], step=0.20)
            if densified:
                densified.extend(seg[1:])
            else:
                densified.extend(seg)

        # Splice: [baseline up to backstep] + [detour] + [baseline from rejoin].
        new_path = path[:backstep] + densified + path[rejoin:]
        self.current_path = new_path
        self.detour_active = True
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
        self.last_detour_points = []  # clear viz
        self._publish(self.baseline_path)
        rospy.loginfo("[local_replanner] obstacle cleared — baseline restored.")

    def _build_astar_grid(self, persistent_mask):
        """Build a SimpleOccupancyGrid combining the inflated static map with
        the inflated persistent obstacles. Both are inflated by robot_radius.

        Also caches the final `free` boolean array on self._last_astar_free
        so _compute_and_apply_detour() can validate its backstep/rejoin
        endpoints against the same occupancy A* plans on.
        """
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

        # Cache for endpoint validation in _compute_and_apply_detour.
        self._last_astar_free = free

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