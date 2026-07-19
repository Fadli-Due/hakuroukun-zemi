#!/usr/bin/env python3
import math, time, collections
import numpy as np
import rospy
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, Bool, Int8MultiArray
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
        self.lookahead_dist   = rospy.get_param(f"{pp_ns}/lookahead_distance", 1.5)
        self.wheelbase        = rospy.get_param(f"{pp_ns}/wheelbase", 1.1)
        self.ctrl_rate        = rospy.get_param(f"{pp_ns}/control_rate", 10)
        self.skip_on_timeout  = rospy.get_param(f"{pp_ns}/skip_on_timeout", 40)
        self.obstacle_stop_range = rospy.get_param(f"{pp_ns}/obstacle_stop_range", 0.45)

        # ========== Reverse parameters ==========
        rp = rospy.get_param("reverse", {})
        self.rev_enable    = rp.get("enable", True)
        self.rev_speed     = abs(rp.get("speed", 0.25))
        self.rev_max_t     = rp.get("max_duration", 4.0)
        self.min_rev_dwell = rp.get("min_dwell", 1.5)
        self.front_stop    = rp.get("front_stop_range", 0.8)
        self.front_clear   = rp.get("front_clear_range", 1.2)
        self.front_fov     = math.radians(rp.get("front_fov_deg", 90))
        self.ang_thresh    = math.radians(rp.get("angle_threshold_deg", 100))
        self.stuck_time    = rp.get("stuck_time", 3.0)
        self.progress_min  = rp.get("progress_min", 0.05)

        # How long to let FORWARD-only steering keep trying before giving up
        # on a tight corner (nothing blocking, just geometry).
        self.fwd_only_grace = rp.get("fwd_only_grace_time", 15.0)
        # Tracks how long FORWARD has been stuck with nothing blocking.
        self.fwd_only_stuck_start = None

        # Gear engagement tracking.
        self.gear_engage_timeout = rp.get("gear_engage_timeout", 3.0)
        self.gear_actual = 0
        self.gear_pend   = 0
        self.rev_gear_engaged_at = None

        # recovery bookkeeping
        self.stuck_start = None
        self.cooldown_until = 0.0
        self.rev_cooldown = rp.get("cooldown", 3.0)

        # ========== HOLD-mode parameters ==========
        self.max_hold_time = rp.get("max_hold_time", 15.0)
        self.hold_start = None
        self.hold_entry_path_version = 0
        self.path_version = 0

        # extra tuning params
        self.goal_tol = rospy.get_param(f"{pp_ns}/goal_tolerance", 0.4)
        self.steer_lpf = rospy.get_param(f"{pp_ns}/steer_lpf", 0.35)
        self.steer_prev = 0.0
        self.closest_i = 0
        self.max_search_ahead = rospy.get_param(f"{pp_ns}/max_search_ahead", 80)

        # -------- internal state --------
        self.current_pose = None
        self.path_points  = []
        self.path_available = False
        self.previous_speed = 0.0
        self.last_alpha = 0.0

        self.mode = "FORWARD"
        self.rev_reason = "BLOCKED"  # Can be "BLOCKED" or "GEOMETRY"
        self.rev_start = None
        self.min_front = float('inf')
        self.prog_hist = collections.deque(maxlen=200)
        self.pose_hist = collections.deque(maxlen=200)

        # Map->odom calibration gate.
        self.map_odom_ready = False

        # -------- TF Buffer --------
        self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf)
        self.path_frame = "map"
        self.target_frame = "map"

        # -------- ROS I/O --------
        rospy.Subscriber('/hakuroukun_pose/rear_wheel_odometry', Odometry, self.odom_cb)
        rospy.Subscriber('/desired_path', Path, self.path_cb)
        rospy.Subscriber('/scan_multi', LaserScan, self.scan_cb)
        rospy.Subscriber('/stop_signal', Bool, self.stop_cb)
        rospy.Subscriber('/hakuroukun/gear_state', Int8MultiArray, self.gear_state_cb)
        rospy.Subscriber('/map_odom_calibrator/initialized', Bool, self.map_odom_init_cb)

        cmd_topic = rospy.get_param("~cmd_topic", "/cmd_controller")
        self.cmd_pub = rospy.Publisher(cmd_topic, Float64MultiArray, queue_size=10)

        self.done_pub = rospy.Publisher(
            '/path_follower/done', Bool, queue_size=1, latch=True)
        self.done_dwell_time = rospy.get_param(f"{pp_ns}/done_dwell_time", 2.0)
        self.done_at_goal_since = None
        self.done_published = False

        rospy.on_shutdown(self.stop_robot)

    # ---------- Callbacks ----------
    def stop_cb(self, msg):
        if msg.data:
            self.stop_robot()

    def gear_state_cb(self, msg):
        if len(msg.data) >= 2:
            self.gear_actual = int(msg.data[0])
            self.gear_pend   = int(msg.data[1])

    def map_odom_init_cb(self, msg):
        if msg.data and not self.map_odom_ready:
            self.map_odom_ready = True
            rospy.loginfo(
                "[path_follower] map->odom calibrated — motion gate released.")
        self.map_odom_ready = bool(msg.data)

    def odom_cb(self, msg):
        try:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
            ps.header.stamp = rospy.Time(0)
            ps_map = self.tf_buf.transform(ps, "map", rospy.Duration(0.1))
        except Exception as e:
            rospy.logwarn_throttle(2.0,
                f"[path_follower] odom→map TF failed: {e}")
            return

        x = ps_map.pose.position.x
        y = ps_map.pose.position.y
        q = ps_map.pose.orientation
        yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("zyx")[0]
        self.current_pose = (x, y, yaw)

        now = rospy.Time.now().to_sec()
        if hasattr(self, 'last_pos'):
            d = math.hypot(x - self.last_pos[0], y - self.last_pos[1])
            self.prog_hist.append((now, d))
            while self.prog_hist and now - self.prog_hist[0][0] > self.stuck_time:
                self.prog_hist.popleft()
        self.last_pos = (x, y)

        self.pose_hist.append((now, x, y))
        while self.pose_hist and now - self.pose_hist[0][0] > self.stuck_time:
            self.pose_hist.popleft()

    def path_cb(self, msg):
        self.path_frame = msg.header.frame_id if msg.header.frame_id else "map"
        if self.path_frame != "map":
            rospy.logwarn_throttle(2.0,
                f"[path_follower] expected path in 'map', got '{self.path_frame}'")
        self.path_points = [(p.pose.position.x, p.pose.position.y)
                            for p in msg.poses]

        if self.mode == "HOLD" and self.path_points:
            self.closest_i = min(self.closest_i, len(self.path_points) - 1)
            rospy.loginfo(
                f"[path_follower] new path ({len(self.path_points)} pts): "
                f"closest_i preserved at {self.closest_i} (HOLD — detour splice)")
        elif self.current_pose and self.path_points:
            rx, ry, _ = self.current_pose
            best_i, best_d2 = 0, float('inf')
            for i, (px, py) in enumerate(self.path_points):
                d2 = (px - rx)**2 + (py - ry)**2
                if d2 < best_d2:
                    best_d2, best_i = d2, i
            self.closest_i = best_i
            rospy.loginfo(
                f"[path_follower] new path ({len(self.path_points)} pts): "
                f"closest_i seeded at {self.closest_i} "
                f"(dist to robot = {math.sqrt(best_d2):.2f}m)")
        else:
            self.closest_i = 0
            rospy.loginfo(
                f"[path_follower] new path ({len(self.path_points)} pts): "
                f"closest_i = 0 (no pose yet)")

        self.path_available = bool(self.path_points)
        self.path_version += 1
        self.done_at_goal_since = None

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
        rng = rng[np.isfinite(rng)]
        if msg.range_min > 0:
            rng = rng[rng >= msg.range_min]
        rng = rng[rng > 0.02]

        self.min_front = float(np.min(rng)) if rng.size else float('inf')

    # ---------- Main loop ----------
    def run(self):
        rate = rospy.Rate(self.ctrl_rate)

        while not rospy.is_shutdown():
            # Gate 0: map->odom calibration
            if not self.map_odom_ready:
                rospy.logwarn_throttle(
                    2.0,
                    "[path_follower] waiting for map->odom calibration — "
                    "click '2D Pose Estimate' in RViz...")
                rate.sleep()
                continue

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

            v_fwd, steer_fwd = self.compute_pp()
            now = rospy.Time.now().to_sec()

            # Net displacement over the stuck window
            if len(self.pose_hist) >= 2:
                _, x0, y0 = self.pose_hist[0]
                _, x1, y1 = self.pose_hist[-1]
                net_disp = math.hypot(x1 - x0, y1 - y0)
            else:
                net_disp = float('inf')
            stuck = net_disp < self.progress_min
            heading_bad = abs(self.last_alpha) > self.ang_thresh

            # Stuck timer
            if stuck:
                if self.stuck_start is None:
                    self.stuck_start = now
            else:
                self.stuck_start = None
            stuck_dur = 0.0 if self.stuck_start is None else (now - self.stuck_start)

            # ── Triggers ─────────────────────────────────────────────────
            want_hold = (self.min_front < self.obstacle_stop_range)

            # Something physically blocking — reverse immediately once stuck.
            want_recover_reverse_blocked = (
                self.min_front < self.front_stop and
                (stuck_dur > 0.3 or heading_bad)
            )

            # Geometry-stuck only (clear path ahead, just can't complete the corner).
            geometry_stuck = (
                stuck_dur > self.stuck_time and
                self.min_front >= self.front_stop
            )
            if geometry_stuck:
                if self.fwd_only_stuck_start is None:
                    self.fwd_only_stuck_start = now
            else:
                self.fwd_only_stuck_start = None
            fwd_only_dur = (
                0.0 if self.fwd_only_stuck_start is None
                else (now - self.fwd_only_stuck_start)
            )
            want_skip_corner = (fwd_only_dur > self.fwd_only_grace)

            # Stage 3 Recovery Trigger: If stuck for more than 8.0s, and front is clear,
            # we perform a brief reverse nudge to relieve binding tension and reposition.
            want_recover_reverse_geometry = (
                stuck_dur > 8.0 and
                self.min_front >= self.front_stop
            )

            # ── FSM ──────────────────────────────────────────────────────
            if self.mode == "FORWARD":
                if want_hold:
                    self.mode = "HOLD"
                    self.hold_start = now
                    self.hold_entry_path_version = self.path_version
                    self.stuck_start = None
                    self.fwd_only_stuck_start = None
                    rospy.loginfo(
                        f"MODE → HOLD (obstacle ahead) min_front={self.min_front:.2f} "
                        f"— waiting for replanner (max {self.max_hold_time:.0f}s)")

                elif self.rev_enable and (now > self.cooldown_until) and want_recover_reverse_blocked:
                    self.mode = "REVERSE"
                    self.rev_reason = "BLOCKED"
                    self.rev_start = now
                    self.rev_gear_engaged_at = None
                    self.fwd_only_stuck_start = None
                    rospy.loginfo(
                        f"MODE → REVERSE (obstacle blocking) front={self.min_front:.2f} "
                        f"stuck={stuck_dur:.2f} heading_bad={heading_bad}")

                elif self.rev_enable and (now > self.cooldown_until) and want_recover_reverse_geometry:
                    # Switch to reverse due to geometry bind (no obstacle in front)
                    self.mode = "REVERSE"
                    self.rev_reason = "GEOMETRY"
                    self.rev_start = now
                    self.rev_gear_engaged_at = None
                    self.fwd_only_stuck_start = None
                    rospy.logwarn(
                        f"MODE → REVERSE (geometry bind relief) stuck={stuck_dur:.1f}s, "
                        f"clear path ahead. Initiating reverse wiggle nudge.")

                elif want_skip_corner:
                    # Geometry-stuck fallback: skip forward past the tight corner
                    old_i = self.closest_i
                    self.closest_i = min(
                        self.closest_i + self.skip_on_timeout,
                        len(self.path_points) - 1)
                    self.stuck_start = None
                    self.fwd_only_stuck_start = None
                    rospy.logwarn(
                        f"MODE stays FORWARD — geometry corner skip: "
                        f"path index {old_i} → {self.closest_i} "
                        f"(fwd_only_dur={fwd_only_dur:.1f}s, min_front={self.min_front:.2f})")

            elif self.mode == "HOLD":
                hold_dur = now - (self.hold_start if self.hold_start is not None else now)

                if self.path_version > self.hold_entry_path_version:
                    self.mode = "REVERSE"
                    self.rev_reason = "BLOCKED"
                    self.rev_start = now
                    self.rev_gear_engaged_at = None
                    rospy.loginfo(
                        f"MODE → REVERSE (new path received, clearing space) "
                        f"hold_dur={hold_dur:.1f}s")

                elif hold_dur > self.max_hold_time:
                    old_i = self.closest_i
                    self.closest_i = min(
                        self.closest_i + self.skip_on_timeout,
                        len(self.path_points) - 1)
                    self.mode = "REVERSE"
                    self.rev_reason = "BLOCKED"
                    self.rev_start = now
                    self.rev_gear_engaged_at = None
                    rospy.logwarn(
                        f"MODE → REVERSE (HOLD timed out at {self.max_hold_time:.0f}s "
                        f"— replanner failed, skipping path index "
                        f"{old_i} → {self.closest_i})")

                elif self.min_front > self.front_clear:
                    self.mode = "FORWARD"
                    self.stuck_start = None
                    rospy.loginfo(
                        f"MODE → FORWARD (obstacle cleared during hold, "
                        f"hold_dur={hold_dur:.1f}s)")

            else:  # REVERSE
                duration = now - (self.rev_start if self.rev_start is not None else now)
                gear_engaged_now = (self.gear_actual == 1 and self.gear_pend == 0)

                if self.rev_gear_engaged_at is None:
                    if gear_engaged_now:
                        self.rev_gear_engaged_at = now
                        rospy.loginfo(
                            f"[pf] REVERSE gear engaged after {duration:.2f}s "
                            f"— dwell clock starts now")
                    elif duration > self.gear_engage_timeout:
                        rospy.logwarn(
                            f"[pf] REVERSE aborted: gear did not engage within "
                            f"{self.gear_engage_timeout:.1f}s (gear={self.gear_actual} "
                            f"pend={self.gear_pend})")
                        self.mode = "FORWARD"
                        self.cooldown_until = now + self.rev_cooldown
                        self.rev_gear_engaged_at = None
                        self.stuck_start = None
                else:
                    rev_motion_dur = now - self.rev_gear_engaged_at
                    if rev_motion_dur < self.min_rev_dwell:
                        pass
                    else:
                        # Determine exit conditions depending on why we reversed:
                        if self.rev_reason == "GEOMETRY":
                            # No obstacle exists, so reverse for a fixed reposition period
                            should_exit = (rev_motion_dur >= min(2.5, self.rev_max_t))
                        else:
                            # Obstacle present: exit when front is clear OR duration exceeded
                            should_exit = (self.min_front > self.front_clear) or (rev_motion_dur > self.rev_max_t)

                        if should_exit:
                            self.mode = "FORWARD"
                            self.cooldown_until = now + self.rev_cooldown
                            self.rev_gear_engaged_at = None
                            self.stuck_start = None
                            
                            # Skip the index aggressively if recovering from geometry stall
                            if self.rev_reason == "GEOMETRY":
                                old_i = self.closest_i
                                self.closest_i = min(
                                    self.closest_i + int(self.skip_on_timeout * 1.5),
                                    len(self.path_points) - 1)
                                rospy.loginfo(
                                    f"[pf] Geometry reverse recovery complete. "
                                    f"Skipping index {old_i} → {self.closest_i}")
                            else:
                                rospy.loginfo(
                                    f"MODE → FORWARD (recovered, rev_motion_dur={rev_motion_dur:.2f}s, "
                                    f"total_rev_dur={duration:.2f}s)")

            # ── Command selection ────────────────────────────────────────
            if self.mode == "REVERSE":
                v = -self.rev_speed
                steer = self.reverse_steering()
            elif self.mode == "HOLD":
                v = 0.0
                steer = 0.0
            else:
                v, steer = v_fwd, steer_fwd

            # 1Hz heartbeat
            rospy.loginfo_throttle(
                1.0,
                f"[pf] mode={self.mode} v={v:.2f} steer={steer:.2f} "
                f"min_front={self.min_front:.2f} alpha={self.last_alpha:.2f} "
                f"closest_i={self.closest_i}/{len(self.path_points)} "
                f"stuck_dur={stuck_dur:.1f} fwd_only_dur={fwd_only_dur:.1f} "
                f"net_disp={net_disp:.2f} gear={self.gear_actual} pend={self.gear_pend}"
            )

            # SAFETY VETO
            if v > 0.0 and self.min_front < self.obstacle_stop_range:
                rospy.logwarn_throttle(1.0,
                    f"[path_follower] FORWARD VETO min_front={self.min_front:.2f}")
                v = 0.0

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

    def compute_pp(self):
        x, y, yaw = self.current_pose

        gx, gy = self.path_points[-1]
        dist_to_final = math.hypot(gx - x, gy - y)

        end_window = 120
        near_end = (len(self.path_points) - self.closest_i) < end_window

        if near_end and dist_to_final < self.goal_tol:
            self.last_alpha = 0.0
            self.previous_speed = 0.0

            now = rospy.Time.now()
            if self.done_at_goal_since is None:
                self.done_at_goal_since = now
            elif (not self.done_published and
                  (now - self.done_at_goal_since).to_sec() >= self.done_dwell_time):
                self.done_pub.publish(Bool(data=True))
                self.done_published = True
                rospy.loginfo(
                    "[path_follower] /path_follower/done published "
                    "(dist_to_final=%.2fm, dwell=%.1fs).",
                    dist_to_final, self.done_dwell_time)

            return (0.0, 0.0)
        else:
            self.done_at_goal_since = None

        # ─── Dynamic Stuck Detection Recovery ───
        is_stuck_now = False
        wiggle_active = False
        stuck_elapsed = 0.0
        
        if self.stuck_start is not None:
            stuck_elapsed = rospy.Time.now().to_sec() - self.stuck_start
            if stuck_elapsed > 2.0:
                is_stuck_now = True
            if stuck_elapsed > 4.0:
                wiggle_active = True

        active_lookahead = self.lookahead_dist
        if is_stuck_now:
            active_lookahead = self.lookahead_dist * 1.8
            rospy.logwarn_throttle(
                2.0,
                f"[path_follower] Stall protection active: widening lookahead to {active_lookahead:.2f}m."
            )

        Lp = self.lookahead_point(x, y, yaw, active_lookahead)
        if Lp is None:
            fb = min(self.closest_i + 10, len(self.path_points) - 1)
            Lp = self.path_points[fb]

        dx, dy = Lp[0] - x, Lp[1] - y
        alpha = self.wrap_pi(math.atan2(dy, dx) - yaw)
        self.last_alpha = alpha

        steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha), active_lookahead)
        steering = max(self.MIN_STEERING, min(self.MAX_STEERING, steering))

        steering = (1.0 - self.steer_lpf) * steering + self.steer_lpf * self.steer_prev
        self.steer_prev = steering

        # Wiggle steering if progress remains bound up for more than 4 seconds
        if wiggle_active:
            # Generate a 1.5Hz oscillation of +/- 0.15 radians (~8.5 degrees) to break static friction
            wiggle_amplitude = 0.15
            wiggle_frequency = 1.5
            wiggle_offset = wiggle_amplitude * math.sin(2.0 * math.pi * wiggle_frequency * rospy.Time.now().to_sec())
            steering = steering + wiggle_offset
            steering = max(self.MIN_STEERING, min(self.MAX_STEERING, steering))
            rospy.logwarn_throttle(
                1.5,
                f"[path_follower] Steering wiggle active (stuck {stuck_elapsed:.1f}s) to break friction."
            )

        if is_stuck_now:
            # Bypass curvature limitations when stuck to generate maximum break-free torque
            desired_speed = self.MAX_SPEED
            rospy.logwarn_throttle(
                2.0,
                f"[path_follower] Stall protection speed boost active: v={desired_speed:.2f}m/s"
            )
        else:
            curvature_penalty = max(0.25, 1.0 - abs(steering) / self.MAX_STEERING)
            desired_speed = (self.MAX_SPEED - self.MIN_SPEED) * curvature_penalty + self.MIN_SPEED

        dt = 1.0 / self.ctrl_rate
        max_dv = self.MAX_ACCEL * dt
        speed_diff = desired_speed - self.previous_speed
        speed_diff = max(-max_dv, min(max_dv, speed_diff))
        new_speed = self.previous_speed + speed_diff
        self.previous_speed = new_speed

        if abs(new_speed) < 0.05:
            new_speed = 0.05 if new_speed >= 0 else -0.05

        return new_speed, steering

    def lookahead_point(self, rx, ry, yaw, l_dist=None):
        if not self.path_points:
            return None

        # Fallback to standard lookahead distance if no override is supplied
        if l_dist is None:
            l_dist = self.lookahead_dist

        def to_robot(dx, dy, yaw):
            c = math.cos(-yaw)
            s = math.sin(-yaw)
            return (c * dx - s * dy, s * dx + c * dy)

        start = max(0, self.closest_i)
        end = min(len(self.path_points), start + self.max_search_ahead)
        best_i, best_d2 = start, float('inf')
        for i in range(start, end):
            px, py = self.path_points[i]
            d2 = (px - rx) ** 2 + (py - ry) ** 2
            if d2 < best_d2:
                best_d2, best_i = d2, i
        self.closest_i = best_i

        dist_acc = 0.0
        lastx, lasty = self.path_points[best_i]
        for j in range(best_i + 1, len(self.path_points)):
            x, y = self.path_points[j]
            dist_acc += math.hypot(x - lastx, y - lasty)
            lastx, lasty = x, y
            if dist_acc >= l_dist:
                dx, dy = x - rx, y - ry
                xr, _ = to_robot(dx, dy, yaw)
                if xr > 0.0:
                    return (x, y)

        for k in (10, 20, 30, 40, 60, 80):
            fb = min(best_i + k, len(self.path_points) - 1)
            xfb, yfb = self.path_points[fb]
            dx, dy = xfb - rx, yfb - ry
            xr, _ = to_robot(dx, dy, yaw)
            if xr > -0.05:
                return (xfb, yfb)
        return self.path_points[-1]

    def reverse_steering(self):
        """Guided reverse: negated pure-pursuit toward the upcoming path direction.

        WHY NEGATED vs. FORWARD PP:
        Ackermann kinematics invert the relationship between wheel angle and
        trajectory direction during backward motion:
          FORWARD  + left steer  (positive) → robot nose turns left
          BACKWARD + left steer  (positive) → rear (direction of travel) goes
                                              RIGHT → robot effectively turns right
        So to back away AND rotate the nose toward a target on the left (where
        the path continues), we need NEGATIVE wheel angle.
        """
        if not self.current_pose or not self.path_points:
            return 0.0

        x, y, yaw = self.current_pose

        target_i = min(self.closest_i + 20, len(self.path_points) - 1)
        tx, ty = self.path_points[target_i]

        dx, dy = tx - x, ty - y
        if math.hypot(dx, dy) < 0.1:
            return 0.0

        alpha = self.wrap_pi(math.atan2(dy, dx) - yaw)
        steer_fwd = math.atan2(2.0 * self.wheelbase * math.sin(alpha),
                               self.lookahead_dist)
        steer_rev = -steer_fwd

        return max(self.MIN_STEERING, min(self.MAX_STEERING, steer_rev))


if __name__ == '__main__':
    PurePursuitNode().run()