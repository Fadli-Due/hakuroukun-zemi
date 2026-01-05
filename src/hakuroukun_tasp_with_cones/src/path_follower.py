#!/usr/bin/env python3
import math, time, collections
import numpy as np
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, Bool
from scipy.spatial.transform import Rotation

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
        self.rev_speed    = abs(rp.get("speed", 0.25))          # magnitude
        self.rev_max_t    = rp.get("max_duration", 2.0)
        self.front_stop   = rp.get("front_stop_range", 0.8)
        self.front_clear  = rp.get("front_clear_range", 1.2)
        self.front_fov    = math.radians(rp.get("front_fov_deg", 90))
        self.ang_thresh   = math.radians(rp.get("angle_threshold_deg", 100))
        self.stuck_time   = rp.get("stuck_time", 1.5)
        self.progress_min = rp.get("progress_min", 0.05)

        # -------- internal state --------
        self.current_pose = None        # (x, y, yaw)
        self.path_points  = []          # list[(x,y)]
        self.path_available = False
        self.previous_speed = 0.0
        self.last_alpha = 0.0           # heading error from PP
        self.mode = "FORWARD"           # or "REVERSE"
        self.rev_start = None
        self.min_front = float('inf')
        self.prog_hist = collections.deque(maxlen=200)  # (time, dist) tuples

        self.velocity_cmd = 0.0
        self.steering_cmd = 0.0

        # -------- ROS I/O --------
        rospy.Subscriber('/hakuroukun_pose/rear_wheel_odometry', Odometry, self.odom_cb)
        rospy.Subscriber('/desired_path', Path, self.path_cb)
        rospy.Subscriber('/scan_multi', LaserScan, self.scan_cb)
        rospy.Subscriber('/stop_signal', Bool, self.stop_cb)

        self.cmd_pub = rospy.Publisher('/cmd_controller', Float64MultiArray, queue_size=10)

        # register shutdown once (not every loop)
        rospy.on_shutdown(self.stop_robot)


    # ---------- Callbacks ----------
    def stop_cb(self, msg):  # external stop signal
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
            # purge old
            while self.prog_hist and now - self.prog_hist[0][0] > self.stuck_time:
                self.prog_hist.popleft()
        self.last_pos = (x, y)

    def path_cb(self, msg):
        self.path_points = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.path_available = bool(self.path_points)

    def scan_cb(self, msg):
        #handling for weird scans (all inf/NaN or empty)
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
        self.min_front = float(np.min(rng)) if rng.size else float('inf')

    # ---------- Main loop ----------
    def run(self):
        rate = rospy.Rate(self.ctrl_rate)
        while not rospy.is_shutdown():
            if self.current_pose and self.path_available:
                v_fwd, steer_fwd = self.compute_pp()  # also updates self.last_alpha
                now = time.time()
                moved = sum(d for _, d in self.prog_hist)
                stuck = moved < self.progress_min
                heading_bad = abs(self.last_alpha) > self.ang_thresh

                # ----- FSM -----
                if self.mode == "FORWARD":
                    if self.rev_enable and (self.min_front < self.front_stop or heading_bad or stuck):
                        #reason logging
                        reason = ("front" if self.min_front < self.front_stop
                                  else "heading" if heading_bad
                                  else "stuck")
                        self.mode = "REVERSE"
                        self.rev_start = now
                        rospy.loginfo(f"MODE → REVERSE ({reason})")
                elif self.mode == "REVERSE":
                    duration = now - (self.rev_start if self.rev_start is not None else now)
                    if (self.min_front > self.front_clear and not heading_bad) or duration > self.rev_max_t:
                        self.mode = "FORWARD"
                        rospy.loginfo("MODE → FORWARD")

                # ----- Command selection -----
                if self.mode == "REVERSE":
                    v = -self.rev_speed
                    steer = self.reverse_steering()
                else:
                    v, steer = v_fwd, steer_fwd

                self.publish_cmd(v, steer)
            rate.sleep()

    # ---------- Helper functions ----------
    def publish_cmd(self, v, steer):
        self.velocity_cmd = v
        self.steering_cmd = steer
        msg = Float64MultiArray()
        msg.data = [v, steer]
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, self.steering_cmd)

    # ---- returns alpha ----
    def compute_pp(self):
        x, y, yaw = self.current_pose
        Lp = self.lookahead_point(x, y)
        if Lp is None:
            return (0.0, 0.0)

        dx, dy = Lp[0] - x, Lp[1] - y
        alpha = math.atan2(dy, dx) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        self.last_alpha = alpha  # save for FSM

        if abs(alpha) > math.pi/2:
            steering = math.copysign(self.MAX_STEERING, alpha)
        else:
            steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha), self.lookahead_dist)
        steering = max(self.MIN_STEERING, min(self.MAX_STEERING, steering))

        curvature_penalty = max(0.2, 1 - abs(steering) / self.MAX_STEERING)
        desired_speed = (self.MAX_SPEED - self.MIN_SPEED) * curvature_penalty + self.MIN_SPEED

        dt = 1.0 / self.ctrl_rate
        max_dv = self.MAX_ACCEL * dt
        speed_diff = desired_speed - self.previous_speed
        speed_diff = max(-max_dv, min(max_dv, speed_diff))
        new_speed = self.previous_speed + speed_diff
        self.previous_speed = new_speed

        return new_speed, steering

    def lookahead_point(self, rx, ry):
        for px, py in self.path_points:
            if math.hypot(px - rx, py - ry) >= self.lookahead_dist:
                return (px, py)
        return None

    # ---- Steering while reversing ----
    def reverse_steering(self):
        x, y, yaw = self.current_pose
        Lp = self.lookahead_point(x, y)
        if Lp is None:
            return 0.0
        dx, dy = Lp[0] - x, Lp[1] - y
        alpha_rev = math.atan2(dy, dx) - (yaw + math.pi)
        alpha_rev = math.atan2(math.sin(alpha_rev), math.cos(alpha_rev))
        steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha_rev), self.lookahead_dist)
        return max(self.MIN_STEERING, min(self.MAX_STEERING, steer))

if __name__ == '__main__':
    PurePursuitNode().run()