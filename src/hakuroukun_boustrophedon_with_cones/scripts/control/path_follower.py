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
        self.lookahead_dist   = rospy.get_param(f"{pp_ns}/lookahead_distance", 1.5)
        self.wheelbase        = rospy.get_param(f"{pp_ns}/wheelbase", 1.1)
        self.ctrl_rate        = rospy.get_param(f"{pp_ns}/control_rate", 10)
        # How many path steps to skip forward when HOLD times out and the
        # replanner failed to produce a new path (static wall / map boundary).
        # 40 steps × 0.20m densify = 8m skip — enough to jump past a stuck
        # corner turn onto the next coverage lane.
        self.skip_on_timeout  = rospy.get_param(f"{pp_ns}/skip_on_timeout", 40)
        self.obstacle_stop_range = rospy.get_param(f"{pp_ns}/obstacle_stop_range", 0.45)

        # ========== Reverse parameters ==========
        rp = rospy.get_param("reverse", {})
        self.rev_enable   = rp.get("enable", True)
        self.rev_speed    = abs(rp.get("speed", 0.25))
        self.rev_max_t    = rp.get("max_duration", 2.0)
        self.front_stop   = rp.get("front_stop_range", 0.8)
        self.front_clear  = rp.get("front_clear_range", 1.2)
        self.front_fov    = math.radians(rp.get("front_fov_deg", 90))
        self.ang_thresh   = math.radians(rp.get("angle_threshold_deg", 100))
        self.stuck_time   = rp.get("stuck_time", 3.0)
        self.progress_min = rp.get("progress_min", 0.05)

        # recovery bookkeeping
        self.stuck_start = None
        self.cooldown_until = 0.0
        self.rev_cooldown = rp.get("cooldown", 3.0)   # seconds

        # ========== HOLD-mode parameters (2026-06-22) ==========
        # When the robot detects an obstacle ahead it now enters HOLD instead
        # of immediately reversing. Staying stationary lets the replanner's
        # persistence timer accumulate cleanly (the obs_grid stops shifting
        # under recentering) so a detour can fire within persistence_threshold
        # seconds. Once a new path arrives, the robot reverses briefly to
        # clear the obstacle, then follows the new (detour-spliced) path.
        self.max_hold_time = rp.get("max_hold_time", 15.0)
        self.hold_start = None
        self.hold_entry_path_version = 0
        self.path_version = 0

        # extra tuning params
        self.goal_tol = rospy.get_param(f"{pp_ns}/goal_tolerance", 0.4)
        self.steer_lpf = rospy.get_param(f"{pp_ns}/steer_lpf", 0.35)  # weight on previous
        self.steer_prev = 0.0
        self.closest_i = 0
        self.max_search_ahead = rospy.get_param(f"{pp_ns}/max_search_ahead", 80)

        # -------- internal state --------
        self.current_pose = None        # (x, y, yaw) in map frame
        self.path_points  = []          # list[(x,y)] in map frame
        self.path_available = False
        self.previous_speed = 0.0
        self.last_alpha = 0.0

        self.mode = "FORWARD"
        self.rev_start = None
        self.min_front = float('inf')
        self.prog_hist = collections.deque(maxlen=200)
        self.pose_hist = collections.deque(maxlen=200)

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

        cmd_topic = rospy.get_param("~cmd_topic", "/hakuroukun_steering_controller/cmd_controller")
        self.cmd_pub = rospy.Publisher(cmd_topic, Float64MultiArray, queue_size=10)

        # /path_follower/done: latched Bool published exactly once when the
        # robot has completed the current /desired_path (near final point AND
        # stopped for done_dwell_time seconds). Subscribed by return_to_start.py
        # which then computes the A* return-to-home leg.
        self.done_pub = rospy.Publisher(
            '/path_follower/done', Bool, queue_size=1, latch=True)
        # Dwell time (s) the robot must stay near goal & stopped before /done fires.
        # Prevents firing during end-of-path wiggle (REVERSE/FORWARD recovery
        # oscillation seen near closest_i ~= len(path)-2).
        self.done_dwell_time = rospy.get_param(f"{pp_ns}/done_dwell_time", 2.0)
        self.done_at_goal_since = None  # rospy.Time when goal-arrival began
        self.done_published = False     # one-shot guard

        rospy.on_shutdown(self.stop_robot)

    # ---------- Callbacks ----------
    def stop_cb(self, msg):
        if msg.data:
            self.stop_robot()

    def odom_cb(self, msg):
        try:
            ps = PoseStamped()
            ps.header = msg.header                # frame_id should be "odom"
            ps.pose = msg.pose.pose
            ps.header.stamp = rospy.Time(0)       # use latest TF
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

        # progress tracking (now in map frame — wheel slip can't fake it)
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

        # FIX (2026-06-23): smart closest_i seeding.
        #
        # The old code reset closest_i = 0 unconditionally. After a detour
        # splice, the robot is physically at backstep (somewhere mid-path,
        # e.g. index 150). With max_search_ahead=25 (5m window), lookahead_
        # point()'s closest-index scan [0..25] never reaches the robot's real
        # position. It falls through all fallbacks and returns path_points[-1]
        # — the literal end of the coverage route — as the PP target. The
        # robot then steers directly toward the far end of the map, cutting
        # across restricted zones. (Observed: robot entered cone zone after
        # first detour fired, 2026-06-23 Case 3.)
        #
        # Fix: seed closest_i at the path point nearest the robot's current
        # pose. O(N) in path length, but only runs when a new path arrives
        # (at most a few times per run). For the initial path (no prior pose),
        # falls back to 0 as before.
        if self.current_pose and self.path_points:
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
        # Bump on every new path. The HOLD state watches this to detect that
        # the replanner has spliced in a detour, which is the cue to reverse
        # briefly and start following the new route.
        self.path_version += 1

        # Reset the goal-arrival dwell timer. A new path is incoming — the
        # robot is no longer "at goal" w.r.t. the old path. The /done flag
        # stays True (latched) for any subscriber that already consumed it
        # (e.g. return_to_start.py), but the dwell-tracking state is fresh
        # so it doesn't immediately re-fire when the robot reaches the end
        # of the new (return) path.
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

        # reject invalid + too-small values
        rng = rng[np.isfinite(rng)]
        if msg.range_min > 0:
            rng = rng[rng >= msg.range_min]
        rng = rng[rng > 0.02]

        self.min_front = float(np.min(rng)) if rng.size else float('inf')

    # ---------- Main loop ----------
    def run(self):
        rate = rospy.Rate(self.ctrl_rate)
        pp_ns = "pure_pursuit_hakuroukun"

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

            # Normal PP — compute desired forward command
            v_fwd, steer_fwd = self.compute_pp()
            # ROS sim time so timers stay in lockstep with the replanner's
            # persistence_threshold (which uses rospy.Time). With wall-clock
            # time.time(), HOLD could time out before persistence had even
            # accumulated — observed 2026-06-22 when sim ran at ~50% real.
            now = rospy.Time.now().to_sec()

            # Net displacement over the stuck window (immune to jitter)
            if len(self.pose_hist) >= 2:
                _, x0, y0 = self.pose_hist[0]
                _, x1, y1 = self.pose_hist[-1]
                net_disp = math.hypot(x1 - x0, y1 - y0)
            else:
                net_disp = float('inf')   # not enough history yet → not stuck
            stuck = net_disp < self.progress_min
            heading_bad = abs(self.last_alpha) > self.ang_thresh

            # Start / reset stuck timer
            if stuck:
                if self.stuck_start is None:
                    self.stuck_start = now
            else:
                self.stuck_start = None
            stuck_dur = 0.0 if self.stuck_start is None else (now - self.stuck_start)

            # ── Triggers ─────────────────────────────────────────────────
            # Obstacle imminent → HOLD (stop, wait for the replanner).
            # Holding stationary lets persistence build up cleanly.
            want_hold = (self.min_front < self.obstacle_stop_range)

            # Legacy stuck-recovery: only fires when the robot is stuck
            # WITHOUT an immediate obstacle (e.g. U-turn binding, steering
            # geometry edge case). Obstacles are handled by want_hold.
            want_recover_reverse = (
                (stuck_dur > self.stuck_time) or
                ((self.min_front < self.front_stop) and
                 (stuck_dur > 0.3 or heading_bad))
            )

            # ── FSM with cooldown to prevent oscillation ─────────────────
            if self.mode == "FORWARD":
                if want_hold:
                    self.mode = "HOLD"
                    self.hold_start = now
                    self.hold_entry_path_version = self.path_version
                    self.stuck_start = None
                    rospy.loginfo(
                        f"MODE → HOLD (obstacle ahead) min_front={self.min_front:.2f} "
                        f"— waiting for replanner (max {self.max_hold_time:.0f}s)")
                elif self.rev_enable and (now > self.cooldown_until) and want_recover_reverse:
                    self.mode = "REVERSE"
                    self.rev_start = now
                    rospy.loginfo(f"MODE → REVERSE (recover) front={self.min_front:.2f} "
                                f"stuck={stuck_dur:.2f} heading_bad={heading_bad}")

            elif self.mode == "HOLD":
                hold_dur = now - (self.hold_start if self.hold_start is not None else now)

                # A) New path arrived — replanner did its job, time to clear
                #    space by reversing, then follow the new (detoured) path.
                if self.path_version > self.hold_entry_path_version:
                    self.mode = "REVERSE"
                    self.rev_start = now
                    rospy.loginfo(
                        f"MODE → REVERSE (new path received, clearing space) "
                        f"hold_dur={hold_dur:.1f}s")

                # B) Held too long with no detour — replanner couldn't route
                #    around a static wall / map boundary (A* failed). Skip
                #    forward on the path to escape the stuck waypoints, then
                #    guided-reverse to realign with the new heading.
                elif hold_dur > self.max_hold_time:
                    old_i = self.closest_i
                    self.closest_i = min(
                        self.closest_i + self.skip_on_timeout,
                        len(self.path_points) - 1)
                    self.mode = "REVERSE"
                    self.rev_start = now
                    rospy.logwarn(
                        f"MODE → REVERSE (HOLD timed out at {self.max_hold_time:.0f}s "
                        f"— replanner failed, skipping path index "
                        f"{old_i} → {self.closest_i})")

                # C) Obstacle cleared on its own (pedestrian walked away).
                elif self.min_front > self.front_clear:
                    self.mode = "FORWARD"
                    self.stuck_start = None
                    rospy.loginfo(
                        f"MODE → FORWARD (obstacle cleared during hold, "
                        f"hold_dur={hold_dur:.1f}s)")

            else:  # REVERSE
                duration = now - (self.rev_start if self.rev_start is not None else now)

                # Stop reversing when we have space again OR we've reversed long enough
                if (self.min_front > self.front_clear) or (duration > self.rev_max_t):
                    self.mode = "FORWARD"
                    self.cooldown_until = now + self.rev_cooldown
                    self.stuck_start = None
                    rospy.loginfo("MODE → FORWARD (recovered)")

            # ── Command selection ────────────────────────────────────────
            if self.mode == "REVERSE":
                v = -self.rev_speed
                steer = self.reverse_steering()
            elif self.mode == "HOLD":
                v = 0.0
                steer = 0.0
            else:
                v, steer = v_fwd, steer_fwd

            # 1Hz status heartbeat
            rospy.loginfo_throttle(
                1.0,
                f"[pf] mode={self.mode} v={v:.2f} steer={steer:.2f} "
                f"min_front={self.min_front:.2f} alpha={self.last_alpha:.2f} "
                f"closest_i={self.closest_i}/{len(self.path_points)} "
                f"stuck_dur={stuck_dur:.1f} net_disp={net_disp:.2f}"
            )

            # SAFETY VETO: block forward motion into close obstacles, but
            # never block reverse — that's how we recover.
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

            # Goal reached — start (or continue) the dwell timer. Only
            # publish /path_follower/done once the robot has held its
            # arrival pose for done_dwell_time seconds. This filters out
            # the end-of-path wiggle (REVERSE/FORWARD recovery oscillation
            # near the final point) — without the dwell, /done could fire
            # in the middle of a recovery transition and trigger the
            # return leg before the robot is actually settled.
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
            # Not at goal — clear the dwell timer so it restarts cleanly
            # if/when the robot re-enters the goal region.
            self.done_at_goal_since = None

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
            return (c * dx - s * dy, s * dx + c * dy)

        # closest path index, searching only forward from the cursor
        start = max(0, self.closest_i)
        end = min(len(self.path_points), start + self.max_search_ahead)
        best_i, best_d2 = start, float('inf')
        for i in range(start, end):
            px, py = self.path_points[i]
            d2 = (px - rx) ** 2 + (py - ry) ** 2
            if d2 < best_d2:
                best_d2, best_i = d2, i
        self.closest_i = best_i

        # walk forward by arc length until we pass the lookahead distance
        dist_acc = 0.0
        lastx, lasty = self.path_points[best_i]
        for j in range(best_i + 1, len(self.path_points)):
            x, y = self.path_points[j]
            dist_acc += math.hypot(x - lastx, y - lasty)
            lastx, lasty = x, y
            if dist_acc >= self.lookahead_dist:
                dx, dy = x - rx, y - ry
                xr, _ = to_robot(dx, dy, yaw)
                if xr > 0.0:            # point is in front of the robot
                    return (x, y)
                # behind us — keep walking forward

        # fallback: nearest future non-behind point
        for k in (10, 20, 30, 40, 60, 80):
            fb = min(best_i + k, len(self.path_points) - 1)
            xfb, yfb = self.path_points[fb]
            dx, dy = xfb - rx, yfb - ry
            xr, _ = to_robot(dx, dy, yaw)
            if xr > -0.05:
                return (xfb, yfb)
        return self.path_points[-1]

    # ---- Steering while reversing ----
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

        The result: during REVERSE, the robot simultaneously backs away from
        the obstacle and rotates to face the upcoming path direction. When
        FORWARD resumes, heading error is already reduced — fewer PP
        corrections and less overshoot.

        WHY NOT THE OLD APPROACH (lookahead_point with flipped yaw):
        The original code called lookahead_point(x, y, yaw + π). That function
        walks path_points FORWARD from closest_i and only accepts candidates
        "in front of" the queried heading. With heading flipped by π, nothing
        on the forward path qualifies, so it fell through all fallbacks and
        returned path_points[-1] — the end of the entire coverage route. The
        resulting alpha was large and arbitrary, saturating steering to
        MAX_STEERING every time. With Ackermann, max-angle reverse just
        retraces the mirrored forward arc, yielding near-zero net displacement.

        NOTE: this uses self.closest_i as the look-ahead base. When called
        after path_skip_on_timeout has advanced closest_i, the target
        automatically points toward the skipped-ahead section of the path —
        exactly where we want the robot to go after it recovers.
        """
        if not self.current_pose or not self.path_points:
            return 0.0

        x, y, yaw = self.current_pose

        # Look 20 steps (4m) ahead from closest_i: stable heading target
        # that's far enough to be meaningful but not in a different lane.
        target_i = min(self.closest_i + 20, len(self.path_points) - 1)
        tx, ty = self.path_points[target_i]

        dx, dy = tx - x, ty - y
        if math.hypot(dx, dy) < 0.1:
            return 0.0   # target coincides with robot — no meaningful angle

        alpha = self.wrap_pi(math.atan2(dy, dx) - yaw)

        # Standard PP formula for forward motion…
        steer_fwd = math.atan2(2.0 * self.wheelbase * math.sin(alpha),
                               self.lookahead_dist)
        # …negated for Ackermann reverse kinematics.
        steer_rev = -steer_fwd

        return max(self.MIN_STEERING, min(self.MAX_STEERING, steer_rev))


if __name__ == '__main__':
    PurePursuitNode().run()