#!/usr/bin/env python3
import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rospy
import math
import sys
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

        # --- Parameters ---
        self.tasp_cell_size = rospy.get_param("tasp_cell_size", 0.7)
        self.inflated_tasp_cell = rospy.get_param("inflated_tasp_cell", 1.5)
        self.sampling_resolution = rospy.get_param("sampling_resolution", 0.1)
        self.inflation_radius_m = rospy.get_param("inflation_radius_m", 0.6)

        # --- Publishers/Subscribers ---
        self.path_pub = rospy.Publisher('/desired_path', Path, queue_size=1, latch=True)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb)
        self.odom_sub = rospy.Subscriber('/hakuroukun_pose/rear_wheel_odometry', Odometry, self.odom_cb)

        self.map_data = None
        self.start_pose = None
        self.path_generated = False

        self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf)

        # Check for readiness every 1.0 second
        self.timer = rospy.Timer(rospy.Duration(1.0), self.check_and_plan)

    def inflate_occupancy_data(self, map_msg, inflation_radius_m: float):
        w = map_msg.info.width
        h = map_msg.info.height
        res = map_msg.info.resolution

        data = np.array(map_msg.data, dtype=np.int16).reshape((h, w))

        # Treat unknown as obstacle for safety
        obstacle = (data >= 50) | (data < 0)

        # Distance (in meters) to nearest obstacle cell
        dist = distance_transform_edt(~obstacle) * res

        inflated = dist <= inflation_radius_m

        out = data.copy()
        out[inflated] = 100

        # Keep unknown as unknown if not swallowed by inflation
        out[(data < 0) & (~inflated)] = -1

        return out.reshape(-1).astype(np.int16).tolist()

    def odom_cb(self, msg):
        try:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
            ps.header.stamp = rospy.Time(0)
            ps_map = self.tf_buf.transform(ps, "map", rospy.Duration(0.2))
            self.start_pose = (ps_map.pose.position.x, ps_map.pose.position.y)
        except Exception:
            # Fallback if TF is unavailable
            self.start_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def map_cb(self, msg):
        self.map_data = msg

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

    def is_point_free(self, ogrid, p):
        x, y = p
        gx = int((x - ogrid.origin_x) / ogrid.resolution)
        gy = int((y - ogrid.origin_y) / ogrid.resolution)

        if gx < 0 or gx >= ogrid.width or gy < 0 or gy >= ogrid.height:
            return False

        idx = gy * ogrid.width + gx
        val = ogrid.data[idx]
        return not (val >= 50 or val == -1)

    def plan_coverage(self):
        rospy.loginfo(f"[Planner] Starting Plan. Robot at: {self.start_pose}")

        inflated_data = self.inflate_occupancy_data(self.map_data, self.inflation_radius_m)

        ogrid = SimpleOccupancyGrid(
            self.map_data.info.width,
            self.map_data.info.height,
            self.map_data.info.resolution,
            self.map_data.info.origin.position.x,
            self.map_data.info.origin.position.y,
            inflated_data
        )

        # 1. Generate coarse coverage waypoints
        waypoints = self.generate_boustrophedon_points(ogrid)

        # Downsample to remove points that are too close
        waypoints = self.downsample_by_spacing(waypoints, min_dist=self.tasp_cell_size)

        if not waypoints:
            rospy.logerr("[Planner] No valid coverage points found! Check if parameters are too strict for the corridor width.")
            self.path_generated = True
            return

        # 2. Find closest waypoint to robot
        closest_idx = self.find_closest_index(self.start_pose, waypoints)
        rospy.loginfo(f"[Planner] Closest waypoint index: {closest_idx}/{len(waypoints)}")

        # Reorder waypoint list to start from nearest waypoint
        sorted_waypoints = waypoints[closest_idx:] + waypoints[:closest_idx]
        first_wp = sorted_waypoints[0]

        rospy.loginfo(f"[Planner] start_pose free? {self.is_point_free(ogrid, self.start_pose)}")
        rospy.loginfo(f"[Planner] first waypoint free? {self.is_point_free(ogrid, first_wp)}")
        rospy.loginfo(f"[Planner] First waypoint: {first_wp}")

        # 3. Build path
        full_path_points = []
        total = len(sorted_waypoints)

        # First: try to connect robot pose to nearest valid waypoint
        first_segment = astar_plan(
            ogrid,
            self.start_pose[0], self.start_pose[1],
            first_wp[0], first_wp[1],
            connectivity=4
        )

        if first_segment and first_segment != "GOAL_OCCUPIED":
            dense_first = self.densify_path(ogrid, first_segment, step=0.3)
            full_path_points.extend(dense_first)
            current_pos = dense_first[-1]
            start_index = 1
            rospy.loginfo("[Planner] Connected robot start pose to first waypoint.")
        else:
            rospy.logwarn("[Planner] Could not connect raw robot pose to first waypoint. Starting directly from nearest valid waypoint.")
            full_path_points.append(first_wp)
            current_pos = first_wp
            start_index = 1

        rospy.loginfo(f"[Planner] Connecting remaining {total - start_index} waypoints...")

        for i, target in enumerate(sorted_waypoints[start_index:], start=start_index):
            path_segment = astar_plan(
                ogrid,
                current_pos[0], current_pos[1],
                target[0], target[1],
                connectivity=4
            )

            if not path_segment or path_segment == "GOAL_OCCUPIED":
                rospy.logwarn(f"[Planner] Skipping unreachable target {i}/{total}: {target}")
                continue

            dense_segment = self.densify_path(ogrid, path_segment, step=0.3)

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
            rospy.logwarn("[Planner] A* could not connect any points. Is the robot starting in an invalid region?")
        else:
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

            if col_idx % 2 == 1:
                col_points.reverse()

            points.extend(col_points)
            x += self.tasp_cell_size
            col_idx += 1

        rospy.loginfo(f"[Planner] Snake Gen: Found {valid_count} valid points out of {total_checks} checks.")
        return points

    def is_area_free(self, ogrid, center_cell):
        cx, cy = center_cell
        half_size = max(self.inflation_radius_m, 0.5 * self.tasp_cell_size)

        x_min = cx - half_size
        x_max = cx + half_size
        y_min = cy - half_size
        y_max = cy + half_size

        for sx in np.arange(x_min, x_max, self.sampling_resolution):
            for sy in np.arange(y_min, y_max, self.sampling_resolution):
                gx = int((sx - ogrid.origin_x) / ogrid.resolution)
                gy = int((sy - ogrid.origin_y) / ogrid.resolution)

                if gx < 0 or gx >= ogrid.width or gy < 0 or gy >= ogrid.height:
                    return False

                idx = gy * ogrid.width + gx
                val = ogrid.data[idx]
                if val >= 50 or val == -1:
                    return False
        return True

    def segment_is_free(self, ogrid, p1, p2, step=0.05):
        x1, y1 = p1
        x2, y2 = p2
        dist = math.hypot(x2 - x1, y2 - y1)
        n = max(2, int(dist / step))

        for i in range(n + 1):
            a = i / float(n)
            x = x1 * (1 - a) + x2 * a
            y = y1 * (1 - a) + y2 * a

            gx = int((x - ogrid.origin_x) / ogrid.resolution)
            gy = int((y - ogrid.origin_y) / ogrid.resolution)

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