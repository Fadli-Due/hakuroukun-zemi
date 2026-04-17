#!/usr/bin/env python3
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import math
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
from planning.simple_astar import astar_plan, SimpleOccupancyGrid
from scipy.ndimage import distance_transform_edt
import tf2_ros
import tf2_geometry_msgs


class OfflineCoveragePlanner:
    def __init__(self):
        rospy.init_node('offline_coverage_planner')

        # ---------------- Parameters ----------------
        self.tasp_cell_size = rospy.get_param("tasp_cell_size", 0.7)
        self.inflated_tasp_cell = rospy.get_param("inflated_tasp_cell", 2.0)
        self.sampling_resolution = rospy.get_param("sampling_resolution", 0.1)
        self.inflation_radius_m = rospy.get_param("inflation_radius_m", 0.8)

        # New practical turning proxy params
        self.use_turn_trim = rospy.get_param("use_turn_trim", True)
        self.turn_margin_cells = rospy.get_param("turn_margin_cells", 2)

        # Path shaping
        self.astar_connectivity = rospy.get_param("astar_connectivity", 8)
        self.densify_step = rospy.get_param("densify_step", 0.25)
        self.smoothing_iterations = rospy.get_param("smoothing_iterations", 2)
        self.corner_lookahead_pts = rospy.get_param("corner_lookahead_pts", 2)
        self.enable_smoothing = rospy.get_param("enable_smoothing", True)

        # TASP-like body envelope
        self.body_half_size = max(
            self.inflation_radius_m,
            0.5 * self.inflated_tasp_cell * self.tasp_cell_size
        )

        rospy.loginfo(
            f"[Planner] Params: "
            f"tasp_cell_size={self.tasp_cell_size}, "
            f"inflated_tasp_cell={self.inflated_tasp_cell}, "
            f"body_half_size={self.body_half_size:.2f}, "
            f"turn_margin_cells={self.turn_margin_cells}, "
            f"astar_connectivity={self.astar_connectivity}"
        )

        # ---------------- ROS I/O ----------------
        self.path_pub = rospy.Publisher('/desired_path', Path, queue_size=1, latch=True)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb)
        self.odom_sub = rospy.Subscriber('/hakuroukun_pose/rear_wheel_odometry', Odometry, self.odom_cb)

        self.map_data = None
        self.start_pose = None
        self.path_generated = False

        self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf)

        self.timer = rospy.Timer(rospy.Duration(1.0), self.check_and_plan)

    # --------------------------------------------------------------------------
    # MAP / ODOM
    # --------------------------------------------------------------------------
    def map_cb(self, msg):
        self.map_data = msg

    def odom_cb(self, msg):
        try:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
            ps.header.stamp = rospy.Time(0)
            ps_map = self.tf_buf.transform(ps, "map", rospy.Duration(0.2))
            self.start_pose = (ps_map.pose.position.x, ps_map.pose.position.y)
        except Exception:
            # fallback if TF is unavailable
            self.start_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def check_and_plan(self, event):
        if self.path_generated:
            return

        if self.map_data is None:
            rospy.loginfo_throttle(5, "[Planner] Waiting for /map topic...")
            return

        if self.start_pose is None:
            rospy.loginfo_throttle(5, "[Planner] Waiting for /hakuroukun_pose/rear_wheel_odometry...")
            return

        self.plan_coverage()

    # --------------------------------------------------------------------------
    # GRID HELPERS
    # --------------------------------------------------------------------------
    def world_to_grid(self, ogrid, x, y):
        gx = int((x - ogrid.origin_x) / ogrid.resolution)
        gy = int((y - ogrid.origin_y) / ogrid.resolution)
        return gx, gy

    def is_point_free(self, ogrid, p):
        x, y = p
        gx, gy = self.world_to_grid(ogrid, x, y)

        if gx < 0 or gx >= ogrid.width or gy < 0 or gy >= ogrid.height:
            return False

        idx = gy * ogrid.width + gx
        val = ogrid.data[idx]
        return not (val >= 50 or val == -1)

    def inflate_occupancy_data(self, map_msg, inflation_radius_m: float):
        w = map_msg.info.width
        h = map_msg.info.height
        res = map_msg.info.resolution

        data = np.array(map_msg.data, dtype=np.int16).reshape((h, w))

        # Treat unknown as obstacle
        obstacle = (data >= 50) | (data < 0)

        # distance to nearest obstacle (meters)
        dist = distance_transform_edt(~obstacle) * res
        inflated = dist <= inflation_radius_m

        out = data.copy()
        out[inflated] = 100

        # keep unknown where it survives
        out[(data < 0) & (~inflated)] = -1

        return out.reshape(-1).astype(np.int16).tolist()

    # --------------------------------------------------------------------------
    # COVERAGE PLANNING
    # --------------------------------------------------------------------------
    def plan_coverage(self):
        rospy.loginfo(f"[Planner] Starting plan. Robot at: {self.start_pose}")

        # Inflate map using practical body clearance
        effective_inflation = max(self.inflation_radius_m, self.body_half_size)
        inflated_data = self.inflate_occupancy_data(self.map_data, effective_inflation)

        ogrid = SimpleOccupancyGrid(
            self.map_data.info.width,
            self.map_data.info.height,
            self.map_data.info.resolution,
            self.map_data.info.origin.position.x,
            self.map_data.info.origin.position.y,
            inflated_data
        )

        # 1) Generate coarse boustrophedon points
        waypoints = self.generate_boustrophedon_points(ogrid)
        waypoints = self.downsample_by_spacing(waypoints, min_dist=self.tasp_cell_size)

        if not waypoints:
            rospy.logerr("[Planner] No valid coverage points found. Parameters may be too strict.")
            self.path_generated = True
            return

        # 2) Start from nearest valid waypoint
        closest_idx = self.find_closest_index(self.start_pose, waypoints)
        rospy.loginfo(f"[Planner] Closest waypoint index: {closest_idx}/{len(waypoints)}")

        sorted_waypoints = waypoints[closest_idx:] + waypoints[:closest_idx]
        first_wp = sorted_waypoints[0]

        rospy.loginfo(f"[Planner] start_pose free? {self.is_point_free(ogrid, self.start_pose)}")
        rospy.loginfo(f"[Planner] first waypoint free? {self.is_point_free(ogrid, first_wp)}")
        rospy.loginfo(f"[Planner] First waypoint: {first_wp}")

        # 3) Connect start -> first waypoint
        full_path_points = []

        first_segment = astar_plan(
            ogrid,
            self.start_pose[0], self.start_pose[1],
            first_wp[0], first_wp[1],
            connectivity=self.astar_connectivity
        )

        if first_segment and first_segment != "GOAL_OCCUPIED":
            dense_first = self.densify_path(ogrid, first_segment, step=self.densify_step)
            full_path_points.extend(dense_first)
            current_pos = dense_first[-1]
            start_index = 1
            rospy.loginfo("[Planner] Connected robot start pose to first waypoint.")
        else:
            rospy.logwarn("[Planner] Could not connect robot pose to first waypoint. Starting from first waypoint directly.")
            full_path_points.append(first_wp)
            current_pos = first_wp
            start_index = 1

        # 4) Connect remaining sweep waypoints
        total = len(sorted_waypoints)
        rospy.loginfo(f"[Planner] Connecting remaining {total - start_index} waypoints...")

        for i, target in enumerate(sorted_waypoints[start_index:], start=start_index):
            path_segment = astar_plan(
                ogrid,
                current_pos[0], current_pos[1],
                target[0], target[1],
                connectivity=self.astar_connectivity
            )

            if not path_segment or path_segment == "GOAL_OCCUPIED":
                rospy.logwarn(f"[Planner] Skipping unreachable target {i}/{total}: {target}")
                continue

            dense_segment = self.densify_path(ogrid, path_segment, step=self.densify_step)

            if full_path_points:
                full_path_points.extend(dense_segment[1:])
            else:
                full_path_points.extend(dense_segment)

            current_pos = dense_segment[-1]

            if i % 10 == 0:
                sys.stdout.write(f"\r[Planner] Progress: {i}/{total}")
                sys.stdout.flush()

        print("")

        if not full_path_points:
            rospy.logwarn("[Planner] Could not build any valid path.")
            self.path_generated = True
            return

        # 5) Clean up and smooth
        full_path_points = self.remove_duplicate_points(full_path_points)
        full_path_points = self.downsample_by_spacing(full_path_points, min_dist=0.08)

        #if self.enable_smoothing:
            #full_path_points = self.smooth_path(ogrid, full_path_points)

        # one more light cleanup after smoothing
        full_path_points = self.remove_duplicate_points(full_path_points)
        full_path_points = self.downsample_by_spacing(full_path_points, min_dist=0.05)

        self.publish_path(full_path_points)
        self.path_generated = True

    def generate_boustrophedon_points(self, ogrid):
        points = []
        min_x, min_y = ogrid.origin_x, ogrid.origin_y
        max_x = min_x + (ogrid.width * ogrid.resolution)
        max_y = min_y + (ogrid.height * ogrid.resolution)

        x = min_x + self.tasp_cell_size / 2.0
        col_idx = 0

        valid_count = 0
        total_checks = 0

        while x < max_x:
            col_points = []
            y = min_y + self.tasp_cell_size / 2.0

            while y < max_y:
                total_checks += 1
                if self.is_area_free(ogrid, (x, y)):
                    col_points.append((x, y))
                    valid_count += 1
                y += self.tasp_cell_size

            # Trim lane ends to leave room for turning
            if self.use_turn_trim:
                trim_n = int(self.turn_margin_cells)
                if len(col_points) > 2 * trim_n:
                    col_points = col_points[trim_n:-trim_n]
                else:
                    col_points = []

            if col_idx % 2 == 1:
                col_points.reverse()

            points.extend(col_points)
            x += self.tasp_cell_size
            col_idx += 1

        rospy.loginfo(f"[Planner] Snake Gen: Found {valid_count} valid raw points out of {total_checks} checks.")
        rospy.loginfo(f"[Planner] Snake Gen: Produced {len(points)} trimmed coverage points.")
        return points

    def is_area_free(self, ogrid, center_cell):
        cx, cy = center_cell

        # TASP-style body envelope
        half_size = self.body_half_size

        x_min = cx - half_size
        x_max = cx + half_size
        y_min = cy - half_size
        y_max = cy + half_size

        for sx in np.arange(x_min, x_max + 1e-9, self.sampling_resolution):
            for sy in np.arange(y_min, y_max + 1e-9, self.sampling_resolution):
                gx, gy = self.world_to_grid(ogrid, sx, sy)

                if gx < 0 or gx >= ogrid.width or gy < 0 or gy >= ogrid.height:
                    return False

                idx = gy * ogrid.width + gx
                val = ogrid.data[idx]
                if val >= 50 or val == -1:
                    return False

        return True

    # --------------------------------------------------------------------------
    # PATH POST-PROCESSING
    # --------------------------------------------------------------------------
    def segment_is_free(self, ogrid, p1, p2, step=0.05):
        x1, y1 = p1
        x2, y2 = p2
        dist = math.hypot(x2 - x1, y2 - y1)
        n = max(2, int(dist / step))

        for i in range(n + 1):
            a = i / float(n)
            x = x1 * (1 - a) + x2 * a
            y = y1 * (1 - a) + y2 * a

            gx, gy = self.world_to_grid(ogrid, x, y)

            if gx < 0 or gx >= ogrid.width or gy < 0 or gy >= ogrid.height:
                return False

            idx = gy * ogrid.width + gx
            val = ogrid.data[idx]
            if val >= 50 or val == -1:
                return False

        return True

    def densify_path(self, ogrid, points, step=0.1):
        if len(points) < 2:
            return points

        new_points = [points[0]]

        for i in range(1, len(points)):
            p1 = points[i - 1]
            p2 = points[i]

            if not self.segment_is_free(ogrid, p1, p2, step=0.05):
                new_points.append(p2)
                continue

            dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            if dist > step:
                num_inserts = int(dist / step)
                for j in range(1, num_inserts + 1):
                    alpha = j / float(num_inserts + 1)
                    mx = p1[0] * (1 - alpha) + p2[0] * alpha
                    my = p1[1] * (1 - alpha) + p2[1] * alpha
                    new_points.append((mx, my))

            new_points.append(p2)

        return new_points

    def remove_duplicate_points(self, pts, eps=1e-6):
        if not pts:
            return pts

        out = [pts[0]]
        for p in pts[1:]:
            if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > eps:
                out.append(p)
        return out

    def line_of_sight_simplify(self, ogrid, pts):
        if len(pts) < 3:
            return pts

        simplified = [pts[0]]
        anchor = pts[0]
        i = 1

        while i < len(pts):
            furthest = i
            for j in range(i, len(pts)):
                if self.segment_is_free(ogrid, anchor, pts[j], step=0.05):
                    furthest = j
                else:
                    break

            simplified.append(pts[furthest])
            anchor = pts[furthest]
            i = furthest + 1

        return simplified

    def angle_between(self, p_prev, p_curr, p_next):
        v1x = p_curr[0] - p_prev[0]
        v1y = p_curr[1] - p_prev[1]
        v2x = p_next[0] - p_curr[0]
        v2y = p_next[1] - p_curr[1]

        n1 = math.hypot(v1x, v1y)
        n2 = math.hypot(v2x, v2y)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0

        c = (v1x * v2x + v1y * v2y) / (n1 * n2)
        c = max(-1.0, min(1.0, c))
        return math.acos(c)

    def smooth_path(self, ogrid, pts):
        if len(pts) < 5:
            return pts

        # first simplify obvious zig-zags while keeping safety
        pts = self.line_of_sight_simplify(ogrid, pts)
        pts = self.densify_path(ogrid, pts, step=self.densify_step)

        for _ in range(self.smoothing_iterations):
            new_pts = [pts[0]]

            for i in range(1, len(pts) - 1):
                p_prev = pts[i - 1]
                p_curr = pts[i]
                p_next = pts[i + 1]

                angle = self.angle_between(p_prev, p_curr, p_next)

                # only smooth noticeable corners
                if angle < math.radians(20):
                    new_pts.append(p_curr)
                    continue

                mx1 = (
                    0.75 * p_curr[0] + 0.25 * p_prev[0],
                    0.75 * p_curr[1] + 0.25 * p_prev[1]
                )
                mx2 = (
                    0.75 * p_curr[0] + 0.25 * p_next[0],
                    0.75 * p_curr[1] + 0.25 * p_next[1]
                )

                # keep smoothing only if safe
                if self.segment_is_free(ogrid, new_pts[-1], mx1) and self.segment_is_free(ogrid, mx1, mx2):
                    new_pts.append(mx1)
                    new_pts.append(mx2)
                else:
                    new_pts.append(p_curr)

            new_pts.append(pts[-1])
            pts = self.remove_duplicate_points(new_pts)
            pts = self.densify_path(ogrid, pts, step=self.densify_step)

        return pts

    # --------------------------------------------------------------------------
    # UTILS
    # --------------------------------------------------------------------------
    def find_closest_index(self, curr, points):
        min_d = float('inf')
        idx = 0
        for i, p in enumerate(points):
            d = math.hypot(p[0] - curr[0], p[1] - curr[1])
            if d < min_d:
                min_d = d
                idx = i
        return idx

    def downsample_by_spacing(self, pts, min_dist=0.6):
        if not pts:
            return pts

        out = [pts[0]]
        last = pts[0]

        for p in pts[1:]:
            if math.hypot(p[0] - last[0], p[1] - last[1]) >= min_dist:
                out.append(p)
                last = p

        # always keep final point if it got dropped
        if math.hypot(pts[-1][0] - out[-1][0], pts[-1][1] - out[-1][1]) > 1e-6:
            out.append(pts[-1])

        return out

    def publish_path(self, points):
        msg = Path()
        stamp = rospy.Time.now()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"

        for p in points:
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = "map"
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)
        rospy.loginfo(f"[Planner] PUBLISHED PATH with {len(points)} points to /desired_path")


if __name__ == '__main__':
    try:
        OfflineCoveragePlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass