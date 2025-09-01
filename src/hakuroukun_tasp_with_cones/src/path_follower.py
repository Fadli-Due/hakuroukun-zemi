#!/usr/bin/env python3
import math, time, collections
import numpy as np
import rospy
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, Bool, Float32, String

# SciPy optional (fallback math used if missing)
try:
    from scipy.spatial.transform import Rotation
    _USE_SCIPY = True
except Exception:
    _USE_SCIPY = False


class PurePursuitNode:
    def __init__(self):
        rospy.init_node('pure_pursuit_hakuroukun', anonymous=True)

        pp_ns = "pure_pursuit_hakuroukun"
        # ===== Forward-drive =====
        self.MAX_SPEED        = rospy.get_param(f"{pp_ns}/max_speed", 0.6)
        self.MIN_SPEED        = rospy.get_param(f"{pp_ns}/min_speed", 0.4)
        self.MAX_ACCEL        = rospy.get_param("/hakuroukun_steering_controller/linear/x/max_acceleration", 2.5)
        self.MAX_STEERING     = rospy.get_param("/hakuroukun_steering_controller/angular/z/max_position", 0.78)
        self.MIN_STEERING     = rospy.get_param("/hakuroukun_steering_controller/angular/z/min_position", -0.78)
        self.lookahead_dist   = rospy.get_param(f"{pp_ns}/lookahead_distance", 0.8)
        self.wheelbase        = rospy.get_param(f"{pp_ns}/wheelbase", 1.1)
        self.ctrl_rate        = rospy.get_param(f"{pp_ns}/control_rate", 10)

        # ===== Reverse / safety =====
        rp = rospy.get_param("reverse", {})
        self.rev_enable   = rp.get("enable", True)
        self.rev_speed    = abs(rp.get("speed", 0.25))
        self.rev_max_t    = rp.get("max_duration", 2.0)
        self.rev_min_t    = rp.get("min_duration", 0.8)          # NEW: must reverse at least this long
        self.rev_min_back = rp.get("min_backoff_m", 0.35)        # NEW: and at least this far
        self.front_stop   = rp.get("front_stop_range", 1.2)
        self.front_clear  = rp.get("front_clear_range", 1.4)
        self.front_fov    = math.radians(rp.get("front_fov_deg", 140))
        self.ang_thresh   = math.radians(rp.get("angle_threshold_deg", 80))
        self.stuck_time   = rp.get("stuck_time", 1.0)
        self.progress_min = rp.get("progress_min", 0.02)

        # Laser cleanup / smoothing
        self.clip_add_min = float(rp.get("front_clip_add_min", 0.03))
        self.clip_sub_max = float(rp.get("front_clip_sub_max", 0.05))
        self.front_beta   = float(rp.get("front_smooth_beta", 0.5))  # 0..1 EMA

        # Lookahead shaping
        self.rev_la_factor = rospy.get_param(f"{pp_ns}/reverse_lookahead_factor", 0.6)
        self.la_min        = rospy.get_param(f"{pp_ns}/lookahead_min", 0.5)
        self.la_max        = rospy.get_param(f"{pp_ns}/lookahead_max", 1.2)

        # ---- state ----
        self.current_pose   = None
        self.last_pos       = None
        self.path_points    = []
        self.path_available = False
        self.previous_speed = 0.0
        self.last_alpha     = 0.0
        self.mode           = "FORWARD"
        self.rev_start      = None
        self.rev_start_pos  = None
        self.min_front_raw  = float('inf')
        self.min_front      = float('inf')  # smoothed
        self.prog_hist      = collections.deque(maxlen=200)
        self.external_stop  = False

        # I/O
        self.cmd_pub       = rospy.Publisher('cmd_controller', Float64MultiArray, queue_size=10)
        self.dbg_front_pub = rospy.Publisher('/pp/debug/min_front', Float32, queue_size=10)
        self.dbg_mode_pub  = rospy.Publisher('/pp/debug/mode', String, queue_size=10)
        self.dbg_alpha_pub = rospy.Publisher('/pp/debug/alpha', Float32, queue_size=10)

        rospy.Subscriber('/hakuroukun_pose/rear_wheel_odometry', Odometry, self.odom_cb)
        rospy.Subscriber('/desired_path', Path, self.path_cb)
        rospy.Subscriber('/scan_multi', LaserScan, self.scan_cb)
        rospy.Subscriber('/stop_signal', Bool, self.stop_cb)

        rospy.on_shutdown(self.stop_robot)

    # ---------- Callbacks ----------
    def stop_cb(self, msg):
        self.external_stop = bool(msg.data)
        # If bumper condition active, let emergency reverse take over
        if self.external_stop and not (self.rev_enable and self.min_front < self.front_stop):
            self.stop_robot()

    def odom_cb(self, msg: Odometry):
        px = msg.pose.pose.position.x; py = msg.pose.pose.position.y
        ox = msg.pose.pose.orientation.x; oy = msg.pose.pose.orientation.y
        oz = msg.pose.pose.orientation.z; ow = msg.pose.pose.orientation.w
        if _USE_SCIPY:
            yaw = Rotation.from_quat((ox, oy, oz, ow)).as_euler("zyx")[0]
        else:
            siny_cosp = 2.0 * (ow * oz + ox * oy)
            cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = (px, py, yaw)

        now = rospy.Time.now().to_sec()
        if self.last_pos is not None:
            d = math.hypot(px - self.last_pos[0], py - self.last_pos[1])
            self.prog_hist.append((now, d))
            while self.prog_hist and now - self.prog_hist[0][0] > self.stuck_time:
                self.prog_hist.popleft()
        self.last_pos = (px, py)

    def path_cb(self, msg: Path):
        self.path_points = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.path_available = bool(self.path_points)

    def scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            self.min_front_raw = float('inf')
        else:
            angles = msg.angle_min + (np.arange(n, dtype=np.float32) * msg.angle_increment)
            mask   = np.abs(angles) <= (self.front_fov / 2.0)
            rng = np.asarray(msg.ranges, dtype=np.float32)[mask] if np.any(mask) else np.array([])
            rng[~np.isfinite(rng)] = np.inf
            rng[rng <= 0.0] = np.inf
            rmin = msg.range_min + self.clip_add_min if msg.range_min > 0.0 else self.clip_add_min
            rmax = msg.range_max - self.clip_sub_max if msg.range_max > 0.0 else np.inf
            rng = rng[(rng >= rmin) & (rng <= rmax)]
            self.min_front_raw = float(np.min(rng)) if rng.size else float('inf')

        # EMA smoothing (prevents flicker)
        beta = max(0.0, min(1.0, self.front_beta))
        if not math.isfinite(self.min_front):
            self.min_front = self.min_front_raw
        else:
            self.min_front = (1.0 - beta) * self.min_front + beta * self.min_front_raw

        try: self.dbg_front_pub.publish(Float32(self.min_front))
        except Exception: pass

    # ---------- Main loop ----------
    def run(self):
        rate = rospy.Rate(self.ctrl_rate)
        while not rospy.is_shutdown():
            # If external STOP but no bumper condition -> hold
            if self.external_stop and not (self.rev_enable and self.min_front < self.front_stop):
                self.publish_cmd(0.0, 0.0); rate.sleep(); continue

            # Emergency reverse even with NO path
            if self.current_pose and (not self.path_available) and self.rev_enable and self.min_front < self.front_stop:
                v = self._ramp_to(-self.rev_speed)
                self.publish_cmd(v, 0.0)
                rate.sleep(); continue

            if self.current_pose and self.path_available:
                v_fwd, steer_fwd = self.compute_pp()
                now = time.time()
                moved = sum(d for _, d in self.prog_hist)
                stuck = moved < self.progress_min
                heading_bad = abs(self.last_alpha) > self.ang_thresh

                # ----- FSM with hysteresis -----
                if self.mode == "FORWARD":
                    if self.rev_enable and (self.min_front < self.front_stop or heading_bad or stuck):
                        reason = ("front" if self.min_front < self.front_stop else
                                  "heading" if heading_bad else "stuck")
                        self.mode = "REVERSE"
                        self.rev_start = now
                        self.rev_start_pos = (self.current_pose[0], self.current_pose[1])
                        rospy.loginfo(f"MODE → REVERSE ({reason})")
                        try: self.dbg_mode_pub.publish(String(self.mode))
                        except Exception: pass

                elif self.mode == "REVERSE":
                    duration = now - (self.rev_start if self.rev_start is not None else now)
                    backoff  = 0.0
                    if self.rev_start_pos is not None:
                        dx = self.current_pose[0] - self.rev_start_pos[0]
                        dy = self.current_pose[1] - self.rev_start_pos[1]
                        backoff = math.hypot(dx, dy)

                    # Exit REVERSE only after min time AND min backoff AND clearance
                    can_exit = (duration >= self.rev_min_t and
                                backoff  >= self.rev_min_back and
                                self.min_front > self.front_clear and
                                not heading_bad)

                    if can_exit or duration > self.rev_max_t:
                        self.mode = "FORWARD"
                        rospy.loginfo("MODE → FORWARD")
                        try: self.dbg_mode_pub.publish(String(self.mode))
                        except Exception: pass

                # ----- Command selection -----
                if self.mode == "REVERSE":
                    v = self._ramp_to(-self.rev_speed)
                    steer = self.reverse_steering()
                else:
                    v, steer = v_fwd, steer_fwd

                self.publish_cmd(v, steer)
            else:
                self.publish_cmd(0.0, 0.0)
            rate.sleep()

    # ---------- Helpers ----------
    def publish_cmd(self, v, steer):
        msg = Float64MultiArray(); msg.data = [v, steer]
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def compute_pp(self):
        x, y, yaw = self.current_pose
        Lp = self.lookahead_point(x, y, self.lookahead_dist)
        if Lp is None: return (0.0, 0.0)
        dx, dy = Lp[0] - x, Lp[1] - y
        alpha = math.atan2(dy, dx) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        self.last_alpha = alpha
        try: self.dbg_alpha_pub.publish(Float32(alpha))
        except Exception: pass
        if abs(alpha) > math.pi/2:
            steering = math.copysign(self.MAX_STEERING, alpha)
        else:
            steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha), self.lookahead_dist)
        steering = max(self.MIN_STEERING, min(self.MAX_STEERING, steering))
        curvature_penalty = max(0.2, 1 - abs(steering) / self.MAX_STEERING)
        desired_speed = (self.MAX_SPEED - self.MIN_SPEED) * curvature_penalty + self.MIN_SPEED
        dt = 1.0 / max(1, self.ctrl_rate)
        max_dv = self.MAX_ACCEL * dt
        dv = max(-max_dv, min(max_dv, desired_speed - self.previous_speed))
        self.previous_speed += dv
        return self.previous_speed, steering

    def lookahead_point(self, rx, ry, dist):
        for px, py in self.path_points:
            if math.hypot(px - rx, py - ry) >= dist:
                return (px, py)
        return None

    def _ramp_to(self, target):
        dt = 1.0 / max(1, self.ctrl_rate)
        max_dv = self.MAX_ACCEL * dt
        dv = max(-max_dv, min(max_dv, target - self.previous_speed))
        self.previous_speed += dv
        return self.previous_speed

    def reverse_steering(self):
        x, y, yaw = self.current_pose
        la_rev = max(self.la_min, min(self.la_max, self.lookahead_dist * self.rev_la_factor))
        Lp = self.lookahead_point(x, y, la_rev)
        if Lp is None: return 0.0
        dx, dy = Lp[0] - x, Lp[1] - y
        alpha_rev = math.atan2(dy, dx) - (yaw + math.pi)
        alpha_rev = math.atan2(math.sin(alpha_rev), math.cos(alpha_rev))
        steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha_rev), la_rev)
        return max(self.MIN_STEERING, min(self.MAX_STEERING, steer))


if __name__ == '__main__':
    PurePursuitNode().run()
