#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
return_to_start.py

One-shot node that triggers a return-to-home leg after coverage is complete.

Workflow
--------
1. Latches the BCD baseline path from /planned_path (first point = home).
2. Builds a SimpleOccupancyGrid from /map, inflated with robot_radius so
   the A* return path keeps the robot off the walls.
3. Tracks the robot's pose from /hakuroukun_pose/rear_wheel_odometry.
4. Waits for /path_follower/done (latched Bool published once when the
   path_follower has finished the coverage path).
5. On True, runs A* from the robot's current pose to baseline[0].
6. Publishes the A* result on /return_path (nav_msgs/Path, latched).
   local_replanner subscribes to that topic, splices the return path
   onto the tail of current_path, and republishes /desired_path so the
   path_follower drives it. Persistence-gated obstacle avoidance works
   on the return leg for free because the replanner evaluates whatever
   is in current_path.

Soft-failure behaviour
----------------------
If A* fails (no path found, start cell occupied, etc.), the node logs a
warning and exits without publishing /return_path. Coverage is already
complete so the robot just stops — no crash, no undefined behaviour.

Notes
-----
- The home pose is hard-coded to baseline_path[0]. If you later want a
  configurable dock pose, expose it as a ROS param.
- The node is one-shot: once it publishes /return_path it sets a guard
  flag and ignores further /path_follower/done messages.
"""

import threading
import numpy as np

# Make the `planning` package importable when this script is launched directly
# by rosrun/roslaunch. Same pattern as local_replanner.py: go up two levels
# (from scripts/planning/ to scripts/), then `from planning.simple_astar ...`
# resolves correctly.
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped transform)

from std_msgs.msg import Bool, Header
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped

from scipy.ndimage import distance_transform_edt

from planning.simple_astar import astar_plan, SimpleOccupancyGrid


class ReturnToStart:
    def __init__(self):
        rospy.init_node('return_to_start')

        # ---- parameters ---------------------------------------------------
        # robot_radius: clearance from walls for the return A* path.
        # Matches the BCD planner's value so the return path uses the same
        # inflation model as the coverage path.
        self.robot_radius = rospy.get_param("robot_radius", 1.0)
        # Densification step (m) when interpolating between A* waypoints.
        # 0.20 matches local_replanner's detour densification so the
        # path_follower sees the same point spacing on coverage, detour,
        # and return legs.
        self.densify_step = rospy.get_param("~densify_step", 0.20)
        # A* max expansions — large maps need this to be generous.
        self.astar_max_expansions = rospy.get_param(
            "~astar_max_expansions", 200000)
        # A* connectivity (4 or 8). 8 gives smoother diagonals.
        self.astar_connectivity = rospy.get_param("~astar_connectivity", 8)

        # ---- state --------------------------------------------------------
        self.lock = threading.Lock()
        self.baseline_path = []      # list[(x, y)] from /planned_path
        self.static_grid = None      # SimpleOccupancyGrid built from /map
        self.robot_xy = None         # (x, y) in map frame
        self.return_published = False  # one-shot guard

        # ---- tf for odom → map transform ---------------------------------
        self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf)

        # ---- ROS I/O ------------------------------------------------------
        # /return_path is latched so local_replanner can subscribe at any time
        # — even if it starts late — and still receive the return path.
        self.return_pub = rospy.Publisher(
            '/return_path', Path, queue_size=1, latch=True)

        rospy.Subscriber('/planned_path', Path, self.baseline_cb)
        rospy.Subscriber('/map', OccupancyGrid, self.map_cb)
        rospy.Subscriber('/hakuroukun_pose/rear_wheel_odometry',
                         Odometry, self.odom_cb)
        rospy.Subscriber('/path_follower/done', Bool, self.done_cb)

        rospy.loginfo("[return_to_start] ready. robot_radius=%.2f densify=%.2f",
                      self.robot_radius, self.densify_step)

    # ------------------------------------------------------------------ I/O
    def baseline_cb(self, msg):
        """Cache the BCD baseline. First point is the home pose."""
        with self.lock:
            self.baseline_path = [(p.pose.position.x, p.pose.position.y)
                                  for p in msg.poses]
        if self.baseline_path:
            rospy.loginfo(
                "[return_to_start] baseline received: home = (%.2f, %.2f)",
                self.baseline_path[0][0], self.baseline_path[0][1])

    def map_cb(self, msg):
        """Build a wall-inflated SimpleOccupancyGrid for A*."""
        with self.lock:
            w, h, res = msg.info.width, msg.info.height, msg.info.resolution
            ox, oy = msg.info.origin.position.x, msg.info.origin.position.y

            raw = np.array(msg.data, dtype=np.int16).reshape((h, w))
            free = (raw == 0)

            # Inflate walls by robot_radius using the same Euclidean distance
            # transform the planner and replanner use. Result: every grid
            # cell within robot_radius of a wall is marked occupied for A*.
            dist = distance_transform_edt(free) * res
            inflated = free & (dist > self.robot_radius)

            # Build SimpleOccupancyGrid's flat data list:
            # 0 = free, 100 = occupied. astar_plan reads from .data.
            data_flat = np.where(inflated, 0, 100).astype(np.int8).flatten().tolist()

            self.static_grid = SimpleOccupancyGrid(
                width=w, height=h, resolution=res,
                origin_x=ox, origin_y=oy, data=data_flat)
        rospy.loginfo_once(
            "[return_to_start] static map ready (%dx%d, res=%.3fm)", w, h, res)

    def odom_cb(self, msg):
        """Track robot position in the map frame via TF."""
        try:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
            ps_map = self.tf_buf.transform(ps, "map", rospy.Duration(0.1))
            self.robot_xy = (ps_map.pose.position.x, ps_map.pose.position.y)
        except Exception:
            # TF not ready yet — try again on next message.
            pass

    def done_cb(self, msg):
        """Path-follower says coverage is done. Compute and publish return."""
        if not msg.data:
            return
        if self.return_published:
            return  # one-shot guard

        with self.lock:
            baseline_ready = bool(self.baseline_path)
            grid_ready = self.static_grid is not None
            pose_ready = self.robot_xy is not None

        if not (baseline_ready and grid_ready and pose_ready):
            rospy.logwarn(
                "[return_to_start] /path_follower/done received but state "
                "not ready (baseline=%s, grid=%s, pose=%s). Skipping.",
                baseline_ready, grid_ready, pose_ready)
            return

        self._compute_and_publish_return()

    # -------------------------------------------------------------- planning
    def _compute_and_publish_return(self):
        """Run A* from robot pose to baseline[0], densify, publish."""
        with self.lock:
            sx, sy = self.robot_xy
            gx, gy = self.baseline_path[0]
            grid = self.static_grid

        rospy.loginfo(
            "[return_to_start] computing return: (%.2f, %.2f) -> (%.2f, %.2f)",
            sx, sy, gx, gy)

        result = astar_plan(grid, sx, sy, gx, gy,
                            connectivity=self.astar_connectivity,
                            max_expansions=self.astar_max_expansions)

        if not result or result == "GOAL_OCCUPIED":
            rospy.logwarn(
                "[return_to_start] A* failed (start=(%.2f, %.2f), "
                "goal=(%.2f, %.2f)). Robot will stay at coverage end pose.",
                sx, sy, gx, gy)
            return

        # Densify so point spacing matches the rest of current_path
        # (path_follower's lookahead behaviour is tuned for ~0.20 m spacing).
        densified = self._densify(result, self.densify_step)

        self._publish(densified)
        self.return_published = True
        rospy.loginfo(
            "[return_to_start] return path published: %d A* points, "
            "%d densified points.", len(result), len(densified))

    def _densify(self, pts, step):
        """Insert intermediate points so consecutive points are <= step apart."""
        if len(pts) < 2:
            return list(pts)
        out = [pts[0]]
        for i in range(1, len(pts)):
            x0, y0 = out[-1]
            x1, y1 = pts[i]
            d = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
            if d <= step:
                out.append((x1, y1))
                continue
            n_intervals = int(d / step)
            for k in range(1, n_intervals + 1):
                t = k / float(n_intervals + 1)
                out.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
            out.append((x1, y1))
        return out

    def _publish(self, points):
        path = Path()
        path.header = Header()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        for (x, y) in points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.return_pub.publish(path)


if __name__ == "__main__":
    try:
        ReturnToStart()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass