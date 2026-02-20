#!/usr/bin/env python3
import math, time, collections
import numpy as np
import rospy
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, Bool
from scipy.spatial.transform import Rotation
import tf2_ros
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs  # IMPORTANT: registers PoseStamped support for tf2 Buffer.transform()


class PurePursuitNode:
    def __init__(self):
        rospy.init_node('pure_pursuit_hakuroukun', anonymous=True)

        # ========== Forward-drive parameters ==========
        pp_ns = "pure_pursuit_hakuroukun"
        self.MAX_SPEED        = rospy.get_param(f"{pp_ns}/max_speed", 0.4)
        self.MIN_SPEED        = rospy.get_param(f"{pp_ns}/min_speed", 0.25)
        self.MAX_ACCEL        = rospy.get_param("/hakuroukun_steering_controller/linear/x/max_acceleration", 2.5)
        self.MAX_STEERING     = rospy.get_param("/hakuroukun_steering_controller/angular/z/max_position", 0.78)
        self.MIN_STEERING     = rospy.get_param("/hakuroukun_steering_controller/angular/z/min_position", -0.78)
        self.lookahead_dist   = rospy.get_param(f"{pp_ns}/lookahead_distance", 0.6)
        self.wheelbase        = rospy.get_param(f"{pp_ns}/wheelbase", 1.1)
        self.ctrl_rate        = rospy.get_param(f"{pp_ns}/control_rate", 10)

        # ========== Reverse parameters ==========
        rp = rospy.get_param("reverse", {})
        self.rev_enable   = rp.get("enable", True)
        self.rev_speed    = abs(rp.get("speed", 0.25))
        self.rev_max_t    = rp.get("max_duration", 2.0)
        self.front_stop   = rp.get("front_stop_range", 0.8)
        self.front_clear  = rp.get("front_clear_range", 1.2)
        self.front_fov    = math.radians(rp.get("front_fov_deg", 90))
        self.ang_thresh   = math.radians(rp.get("angle_threshold_deg", 100))
        self.stuck_time   = rp.get("stuck_time", 1.5)
        self.progress_min = rp.get("progress_min", 0.05)
        
        # recovery bookkeeping
        self.stuck_start = None
        self.cooldown_until = 0.0
        self.rev_cooldown = rp.get("cooldown", 3.0)   # seconds

        # extra tuning params
        self.goal_tol = rospy.get_param(f"{pp_ns}/goal_tolerance", 0.4)
        self.steer_lpf = rospy.get_param(f"{pp_ns}/steer_lpf", 0.35)  # weight on previous
        self.steer_prev = 0.0
        self.closest_i = 0
        self.max_search_ahead = rospy.get_param(f"{pp_ns}/max_search_ahead", 80)

        # -------- internal state --------
        self.current_pose = None        # (x, y, yaw) in odom
        self.path_points  = []          # list[(x,y)] in target_frame
        self.path_available = False
        self.previous_speed = 0.0
        self.last_alpha = 0.0

        self.mode = "FORWARD"
        self.rev_start = None
        self.min_front = float('inf')
        self.prog_hist = collections.deque(maxlen=200)

        # -------- TF Buffer --------
        self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf)
        self.path_frame = "map"
        self.target_frame = "odom"

        # -------- ROS I/O --------
        rospy.Subscriber('/hakuroukun_pose/rear_wheel_odometry', Odometry, self.odom_cb)
        rospy.Subscriber('/desired_path', Path, self.path_cb)
        rospy.Subscriber('/scan_multi', LaserScan, self.scan_cb)
        rospy.Subscriber('/stop_signal', Bool, self.stop_cb)

        cmd_topic = rospy.get_param("~cmd_topic", "/hakuroukun_steering_controller/cmd_controller")
        self.cmd_pub = rospy.Publisher(cmd_topic, Float64MultiArray, queue_size=10)

        rospy.on_shutdown(self.stop_robot)

    # ---------- Callbacks ----------
    def stop_cb(self, msg):
        if msg.data:
            self.stop_robot()

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = (msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w)
        yaw = Rotation.from_quat(q).as_euler("zyx")[0]
        self.current_pose = (x, y, yaw)

        # progress tracking
        now = rospy.Time.now().to_sec()
        if hasattr(self, 'last_pos'):
            d = math.hypot(x - self.last_pos[0], y - self.last_pos[1])
            self.prog_hist.append((now, d))
            while self.prog_hist and now - self.prog_hist[0][0] > self.stuck_time:
                self.prog_hist.popleft()
        self.last_pos = (x, y)

    def path_cb(self, msg):
        self.path_frame = msg.header.frame_id if msg.header.frame_id else self.path_frame

        pts = []
        for ps in msg.poses:
            try:
                ps_in = PoseStamped()
                ps_in.header = msg.header
                ps_in.pose = ps.pose
                ps_in.header.stamp = rospy.Time(0)  # latest TF

                ps_out = self.tf_buf.transform(ps_in, self.target_frame, rospy.Duration(0.2))
                pts.append((ps_out.pose.position.x, ps_out.pose.position.y))
            except Exception as e:
                rospy.logwarn_throttle(
                    1.0,
                    f"Path TF transform failed ({self.path_frame}->{self.target_frame}): {e}"
                )
                self.path_available = False
                return

        self.path_points = pts
        self.closest_i = 0
        self.path_available = bool(self.path_points)

    def scan_cb(self, msg):
        n = len(msg.ranges)
        if n == 0:
            self.min_front = float('inf')
            return

        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        mask   = np.abs(angles) <= self.front_fov/2.0
        if not np.any(mask):
            self.min_front = float('inf')
            return

        rng = np.asarray(msg.ranges)[mask]

        # reject invalid + too-small values
        rng = rng[np.isfinite(rng)]
        if msg.range_min > 0:
            rng = rng[rng >= msg.range_min]
        rng = rng[rng > 0.02]

        self.min_front = float(np.min(rng)) if rng.size else float('inf')

    # ---------- Main loop ----------
    def run(self):
        rate = rospy.Rate(self.ctrl_rate)

        while not rospy.is_shutdown():
            # Gate 1: pose
            if not self.current_pose:
                rospy.logwarn_throttle(2.0, "[path_follower] waiting for odom...")
                rate.sleep()
                continue

            # Gate 2: path
            if not self.path_available or not self.path_points:
                rospy.logwarn_throttle(2.0, "[path_follower] waiting for /desired_path...")
                rate.sleep()
                continue

            # Emergency stop (always active safety)
            if self.min_front < 0.45:
                rospy.logwarn_throttle(1.0, f"[path_follower] EMERGENCY STOP min_front={self.min_front:.2f}")
                self.publish_cmd(0.0, 0.0)
                rate.sleep()
                continue

            # Normal PP
            v_fwd, steer_fwd = self.compute_pp()
            now = time.time()

            moved = sum(d for _, d in self.prog_hist)
            stuck = moved < self.progress_min
            heading_bad = abs(self.last_alpha) > self.ang_thresh

            # Start / reset stuck timer
            if stuck:
                if self.stuck_start is None:
                    self.stuck_start = now
            else:
                self.stuck_start = None

            stuck_dur = 0.0 if self.stuck_start is None else (now - self.stuck_start)

            # Trigger reverse only when:
            #  A) stuck for a while, OR
            #  B) front is blocked AND (stuck a bit OR heading is bad)
            want_recover_reverse = (
                (stuck_dur > self.stuck_time) or
                ((self.min_front < self.front_stop) and (stuck_dur > 0.3 or heading_bad))
            )

            # FSM with cooldown to prevent oscillation
            if self.mode == "FORWARD":
                if self.rev_enable and (now > self.cooldown_until) and want_recover_reverse:
                    self.mode = "REVERSE"
                    self.rev_start = now
                    rospy.loginfo(f"MODE → REVERSE (recover) front={self.min_front:.2f} stuck={stuck_dur:.2f} heading_bad={heading_bad}")
            else:  # REVERSE
                duration = now - (self.rev_start if self.rev_start is not None else now)

                # Stop reversing when we have space again OR we've reversed long enough
                if (self.min_front > self.front_clear) or (duration > self.rev_max_t):
                    self.mode = "FORWARD"
                    self.cooldown_until = now + self.rev_cooldown
                    self.stuck_start = None
                    rospy.loginfo("MODE → FORWARD (recovered)")

            # Command selection
            if self.mode == "REVERSE":
                v = -self.rev_speed
                steer = self.reverse_steering()
            else:
                v, steer = v_fwd, steer_fwd
                
             # 1Hz status heartbeat (great for demo)
            rospy.loginfo_throttle(
                1.0,
                f"[pf] mode={self.mode} v={v:.2f} steer={steer:.2f} "
                f"min_front={self.min_front:.2f} alpha={self.last_alpha:.2f} "
                f"closest_i={self.closest_i}/{len(self.path_points)} stuck_dur={stuck_dur:.1f}"
            )

            self.publish_cmd(v, steer)
            rate.sleep()

    # ---------- Helper functions ----------
    def publish_cmd(self, v, steer):
        msg = Float64MultiArray()
        msg.data = [float(v), float(steer)]
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def wrap_pi(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    # ---- returns (speed, steering) ----
    def compute_pp(self):
        x, y, yaw = self.current_pose

        gx, gy = self.path_points[-1]
        dist_to_final = math.hypot(gx - x, gy - y)

        end_window = 120
        near_end = (len(self.path_points) - self.closest_i) < end_window

        if near_end and dist_to_final < self.goal_tol:
            self.last_alpha = 0.0
            self.previous_speed = 0.0
            return (0.0, 0.0)

        Lp = self.lookahead_point(x, y, yaw)
        if Lp is None:
            fb = min(self.closest_i + 10, len(self.path_points) - 1)
            Lp = self.path_points[fb]

        dx, dy = Lp[0] - x, Lp[1] - y
        alpha = self.wrap_pi(math.atan2(dy, dx) - yaw)
        self.last_alpha = alpha

        steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha), self.lookahead_dist)
        steering = max(self.MIN_STEERING, min(self.MAX_STEERING, steering))

        steering = (1.0 - self.steer_lpf) * steering + self.steer_lpf * self.steer_prev
        self.steer_prev = steering

        curvature_penalty = max(0.25, 1.0 - abs(steering) / self.MAX_STEERING)
        desired_speed = (self.MAX_SPEED - self.MIN_SPEED) * curvature_penalty + self.MIN_SPEED

        dt = 1.0 / self.ctrl_rate
        max_dv = self.MAX_ACCEL * dt
        speed_diff = desired_speed - self.previous_speed
        speed_diff = max(-max_dv, min(max_dv, speed_diff))
        new_speed = self.previous_speed + speed_diff
        self.previous_speed = new_speed

        if new_speed < 0.05:
            new_speed = 0.05

        return new_speed, steering

    def lookahead_point(self, rx, ry, yaw):
        if not self.path_points:
            return None

        def to_robot(dx, dy, yaw):
            c = math.cos(-yaw)
            s = math.sin(-yaw)
            return (c*dx - s*dy, s*dx + c*dy)

        # --- search only forward from cursor to avoid lane switching ---
        start = max(0, self.closest_i)
        end = min(len(self.path_points), start + self.max_search_ahead)

        best_i = start
        best_d2 = float('inf')
        for i in range(start, end):
            px, py = self.path_points[i]
            d2 = (px - rx)**2 + (py - ry)**2
            if d2 < best_d2:
                best_d2 = d2
                best_i = i

        self.closest_i = best_i

        # --- walk forward by arc length until lookahead distance ---
        dist_acc = 0.0
        lastx, lasty = self.path_points[best_i]

        for j in range(best_i + 1, len(self.path_points)):
            x, y = self.path_points[j]
            dist_acc += math.hypot(x - lastx, y - lasty)
            lastx, lasty = x, y

            if dist_acc >= self.lookahead_dist:
                dx, dy = x - rx, y - ry
                xr, _ = to_robot(dx, dy, yaw)
                if xr > 0.0:     # slightly softer than 0.10
                    return (x, y)

            # fallback: pick a future point that is not (too) behind in robot frame
            for k in (10, 20, 30, 40, 60, 80):
                fb = min(best_i + k, len(self.path_points) - 1)
                xfb, yfb = self.path_points[fb]
                dx, dy = xfb - rx, yfb - ry
                xr, _ = to_robot(dx, dy, yaw)
                if xr > -0.05:   # allow slightly behind, but avoid "go backwards" targets
                    return (xfb, yfb)

            # last resort: just head to final point
            return self.path_points[-1]


    # ---- Steering while reversing ----
    def reverse_steering(self):
        x, y, yaw = self.current_pose

        # choose target "in front" of the reversed heading
        Lp = self.lookahead_point(x, y, yaw + math.pi)
        if Lp is None:
            return 0.0

        dx, dy = Lp[0] - x, Lp[1] - y
        alpha_rev = self.wrap_pi(math.atan2(dy, dx) - (yaw + math.pi))
        steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha_rev), self.lookahead_dist)
        return max(self.MIN_STEERING, min(self.MAX_STEERING, steer))


if __name__ == '__main__':
    PurePursuitNode().run()