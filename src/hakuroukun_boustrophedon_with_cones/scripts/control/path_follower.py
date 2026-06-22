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
        self.lookahead_dist   = rospy.get_param(f"{pp_ns}/lookahead_distance", 1.5)  # Raised from 0.6 to 1.5
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
        self.current_pose = None        # (x, y, yaw) in odom
        self.path_points  = []          # list[(x,y)] in target_frame
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
        self.closest_i = 0
        self.path_available = bool(self.path_points)
        # Bump on every new path. The HOLD state watches this to detect that
        # the replanner has spliced in a detour, which is the cue to reverse
        # briefly and start following the new route.
        self.path_version += 1

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
            # NEW (2026-06-22): obstacle imminent → HOLD (stop, wait for the
            # replanner). The robot used to reverse the moment min_front
            # dropped under 0.45, but that kept moving the LiDAR window and
            # prevented persistence from accumulating cleanly, so a detour
            # could never fire. Holding stationary lets persistence build up.
            want_hold = (self.min_front < 0.45)

            # Legacy stuck-recovery: only fires when the robot is stuck WITHOUT
            # an immediate obstacle in front (e.g. U-turn binding, steering
            # geometry edge case). Obstacles are now handled by want_hold.
            want_recover_reverse = (
                (stuck_dur > self.stuck_time) or
                ((self.min_front < self.front_stop) and
                 (stuck_dur > 0.3 or heading_bad))
            )

            # ── FSM with cooldown to prevent oscillation ────────────────
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
                # B) Held too long with no detour — fall back to legacy reverse
                #    so the robot at least tries something.
                elif hold_dur > self.max_hold_time:
                    self.mode = "REVERSE"
                    self.rev_start = now
                    rospy.logwarn(
                        f"MODE → REVERSE (HOLD timed out at {self.max_hold_time:.0f}s "
                        f"with no new path — replanner may have failed)")
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

            # ── Command selection ───────────────────────────────────────
            if self.mode == "REVERSE":
                v = -self.rev_speed
                steer = self.reverse_steering()
            elif self.mode == "HOLD":
                v = 0.0
                steer = 0.0
            else:
                v, steer = v_fwd, steer_fwd

            # 1Hz status heartbeat (great for demo)
            rospy.loginfo_throttle(
                1.0,
                f"[pf] mode={self.mode} v={v:.2f} steer={steer:.2f} "
                f"min_front={self.min_front:.2f} alpha={self.last_alpha:.2f} "
                f"closest_i={self.closest_i}/{len(self.path_points)} "
                f"stuck_dur={stuck_dur:.1f} net_disp={net_disp:.2f}"
            )

            # SAFETY VETO: block forward motion into close obstacles, but
            # never block reverse — that's how we recover.
            if v > 0.0 and self.min_front < 0.45:
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

        # fallback (runs only AFTER the loop): nearest future non-behind point
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
        # FIX (2026-06-21, path_follower debug session): this used to call
        # self.lookahead_point(x, y, yaw + math.pi) to find a "target behind
        # the robot." That doesn't work -- lookahead_point() is a forward-
        # path search: it only walks self.path_points forward from
        # self.closest_i (toward wherever the route continues, i.e. toward
        # the obstacle that just triggered the veto), and only accepts a
        # candidate if it tests as "in front of" the queried heading. Points
        # that are actually ahead on the route essentially never satisfy
        # "behind the robot" once you flip yaw by pi, so the search fell
        # through every fallback and frequently returned self.path_points[-1]
        # (the literal end of the whole coverage route) as the "reverse
        # target." That produced a large, essentially arbitrary alpha that
        # saturated against MAX_STEERING -- coincidentally the same ceiling
        # FORWARD mode was also saturating against while fighting the
        # obstacle, which is why logs showed identical steer values in both
        # FORWARD and REVERSE. With Ackermann steering, reversing at max
        # wheel angle just retraces the mirrored arc of whatever FORWARD was
        # doing, so net displacement per recovery cycle was ~0.
        #
        # REVERSE mode's actual job is just "open clearance from whatever
        # tripped the veto," not "track the path backward." Centered
        # steering while reversing translates the robot straight back along
        # its current heading -- away from the obstacle -- which guarantees
        # real distance gain regardless of path geometry, instead of
        # depending on a search that was never designed to run backward.
        return 0.0


if __name__ == '__main__':
    PurePursuitNode().run()