#!/usr/bin/env python3
import math
from datetime import datetime
import pytz
import os
import yaml

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import geonav_transform.geonav_conversions as gc
import tf.transformations as tf


class HakuroukunPose:

    def __init__(self):
        self.log = True
        rospy.init_node("robot_localization", anonymous=True)
        self._register_parameters()
        self._register_log_file()
        self._get_initial_pose()
        self._get_initial_orientation()
        rospy.sleep(1)
        self._register_publishers()
        self._register_subscribers()
        rospy.sleep(1)
        self._register_timers()

    def run(self):
        rospy.spin()

    def _register_parameters(self):
        self.publish_rate = rospy.get_param("~publish_rate", 0.1) 
        # Initial yaw in odom frame. The actual initial heading in the map
        # frame is set by the 2D Pose Estimate click, which
        # map_odom_calibrator.py converts into the map->odom yaw offset.
        # path_follower.py reads pose in the map frame, so this value is
        # compensated for by the calibrator regardless of what it starts at.
        # Kept at 0.0 for a clean odom origin.
        #
        # (Previously hardcoded to math.pi on 2026-07-05 as a workaround
        # for the "robot facing West at D-F" case. Replaced by the init
        # handshake on 2026-07-06 — the calibrator now owns initial heading.)
        self.yaw = 0.0
        # Safe defaults until first GPS/IMU message arrives
        self.x_rear = 0.0
        self.y_rear = 0.0
        self.quaternion_x = 0.0
        self.quaternion_y = 0.0
        self.quaternion_z = 0.0
        self.quaternion_w = 1.0
        self._last_imu_time = 0.0
        # Gyro bias calibration
        self._bias_samples = []
        self._gyro_bias_z = 0.0
        self._bias_calibrated = False

        self._load_rotation_angle()

    def _load_rotation_angle(self):
        """
        Loads the GPS rotation angle from a calibration file written by
        gps_rotation_calibrator.py (run during the manual E-building -> D-F
        drive). Falls back to the old hardcoded 172.1 deg if the file is
        missing, unreadable, or malformed -- a missing calibration should
        never crash the node or silently use 0 deg.

        FIX (2026-07-12): rotation_angle was previously hardcoded, dated
        2026-06-30. Since the GPS unit gets borrowed/remounted by other lab
        members, that constant goes silently stale with no error signal --
        producing confident-but-wrong path tracking. This reads a fresh,
        per-drive-session value instead.
        """
        default_path = os.path.expanduser(
            "~/catkin_ws/src/hakuroukun_localization/hakuroukun_pose/"
            "config/gps_rotation_calibration.yaml")
        calib_path = rospy.get_param("~gps_calibration_file", default_path)

        fallback_deg = 172.1
        try:
            with open(calib_path) as f:
                calib = yaml.safe_load(f)
            angle_deg = calib["rotation_angle_deg"]
            self.rotation_angle_deg = angle_deg
            # Offset that aligns IMU yaw zero-reference with GPS local frame +X.
            # Without this, self.yaw and (x_gps, y_gps) live in different frames
            # and the rear-axle projection (and downstream steering) go wrong
            # whenever the robot boots facing anything other than the local +X
            # direction. Estimated 2026-07-14 from empirical rotation test.
            self.imu_yaw_offset_deg = calib.get("imu_yaw_offset_deg", 0.0)
            rospy.loginfo(
                f"[hakuroukun_pose] Loaded GPS rotation angle "
                f"{angle_deg:.2f} deg, IMU yaw offset "
                f"{self.imu_yaw_offset_deg:.2f} deg from {calib_path} "
                f"(calibrated {calib.get('calibrated_at', 'unknown')}, "
                f"{calib.get('segments_used', '?')} segments, "
                f"agreement={calib.get('segment_agreement_deg', '?')} deg)")
        except Exception as e:
            self.rotation_angle_deg = fallback_deg
            rospy.logwarn(
                f"[hakuroukun_pose] Could not load GPS calibration file "
                f"({calib_path}): {e}. Falling back to hardcoded "
                f"{fallback_deg} deg. If the GPS was recently moved or "
                f"remounted, this fallback may be stale -- run "
                f"gps_rotation_calibrator.py on the next drive.")

    def _register_subscribers(self):
        self.gps_sub = rospy.Subscriber(
            "/fix", NavSatFix, self._gps_callback)
        self.imu_sub = rospy.Subscriber(
            "/imu", Imu, self._imu_callback)

    def _register_publishers(self):
        self.rear_pose_pub = rospy.Publisher(
            "/hakuroukun_pose/rear_wheel_odometry", Odometry, queue_size=10)
        self.orientation_pub = rospy.Publisher(
            "/hakuroukun_pose/orientation", Float64, queue_size=1)

    def _register_timers(self):
        rospy.Timer(rospy.Duration(self.publish_rate),
                    self._publish_rear_wheel_pose)
        if self.log:
            rospy.Timer(rospy.Duration(self.publish_rate),
                        self._log_pose)

    def _register_log_file(self):
        current_folder = os.path.dirname(os.path.abspath(__file__))
        new_folder = os.path.join(current_folder, '..', '..', 'log_data')
        new_folder = os.path.normpath(new_folder)
        os.makedirs(new_folder, exist_ok=True)
        japan_timezone = pytz.timezone('Asia/Tokyo')
        current_time = datetime.now(japan_timezone).strftime(
            "position_log_%Y%m%d_%H-%M")
        self.file_name = os.path.join(
            new_folder, current_time + ".csv")

    def _get_initial_pose(self):
        first_gps_mess = rospy.wait_for_message(
            '/fix', NavSatFix, timeout=10)
        rospy.loginfo("GPS Data Received")
        self.initial_lat = first_gps_mess.latitude
        self.initial_lon = first_gps_mess.longitude

    def _get_initial_orientation(self):
        rospy.wait_for_message('/imu', Imu, timeout=10)
        rospy.loginfo("IMU Data Received")

    def _gps_callback(self, data: NavSatFix):
        if not self._bias_calibrated:
            return
        self.x_gps, self.y_gps = self._get_xy_from_latlon(
            data.latitude, data.longitude, self.initial_lat, self.initial_lon)
        gps_to_rear_axis = 0.6
        self.x_rear = self.x_gps - \
            gps_to_rear_axis * math.cos(self.yaw)
        self.y_rear = self.y_gps - \
            gps_to_rear_axis * math.sin(self.yaw)

    def _imu_callback(self, data: Imu):
            raw_z = data.angular_velocity.z

            # Keep gyro bias calibration as a stationary-wait safety period,
            # even though we no longer integrate gyro for yaw. The wait gives
            # GPS time to stabilize and prevents motion during startup.
            if not self._bias_calibrated:
                self._bias_samples.append(raw_z)
                if len(self._bias_samples) % 500 == 0:
                    rospy.loginfo(
                        f"Gyro bias calibration: {len(self._bias_samples)}/3000 "
                        f"samples — keep robot stationary")
                if len(self._bias_samples) >= 3000:
                    self._gyro_bias_z = sum(self._bias_samples) / len(self._bias_samples)
                    self._bias_calibrated = True
                    self._last_imu_time = rospy.get_time()
                    rospy.loginfo(f"Gyro Z bias calibrated: {self._gyro_bias_z:.4f} deg/s")
                return

            # Extract absolute yaw from IMU quaternion, then apply calibrated
            # offset to align with the GPS local frame. Replaces gyro integration
            # (which produced a yaw that had no relation to the local frame,
            # causing 90-deg decoupling between heading and position motion).
            q = [data.orientation.x, data.orientation.y,
                data.orientation.z, data.orientation.w]
            _, _, imu_yaw = tf.euler_from_quaternion(q)
            yaw_unwrapped = imu_yaw + math.radians(self.imu_yaw_offset_deg)
            # Normalize to [-pi, pi]
            self.yaw = math.atan2(math.sin(yaw_unwrapped), math.cos(yaw_unwrapped))

            # Reconstruct flat (yaw-only) output quaternion
            self.quaternion_x = 0.0
            self.quaternion_y = 0.0
            self.quaternion_z = math.sin(self.yaw / 2.0)
            self.quaternion_w = math.cos(self.yaw / 2.0)

    def _publish_rear_wheel_pose(self, timer):
        rear_wheel_msg = Odometry()
        rear_wheel_msg.header.frame_id = "odom"
        rear_wheel_msg.child_frame_id = "base_link"
        rear_wheel_msg.header.stamp = rospy.get_rostime()
        rear_wheel_msg.pose.pose.position.x = self.x_rear
        rear_wheel_msg.pose.pose.position.y = self.y_rear
        rear_wheel_msg.pose.pose.position.z = 0.0
        rear_wheel_msg.pose.pose.orientation.x = self.quaternion_x
        rear_wheel_msg.pose.pose.orientation.y = self.quaternion_y
        rear_wheel_msg.pose.pose.orientation.z = self.quaternion_z
        rear_wheel_msg.pose.pose.orientation.w = self.quaternion_w
        rospy.loginfo_throttle(
            2.0,
            f"[hakuroukun_pose] x: {rear_wheel_msg.pose.pose.position.x:.3f}, "
            f"y: {rear_wheel_msg.pose.pose.position.y:.3f}, yaw: {self.yaw:.3f}")
        self.rear_pose_pub.publish(rear_wheel_msg)   # ← add this line
        
    def _log_pose(self, timer):
        pose = f"{self.x_rear}, {self.y_rear}, {(self.yaw)}" + "\n"
        with open(self.file_name, mode="a") as f:
            f.write(pose)

    def _get_xy_from_latlon(self, latitude, longitude, initial_lat, initial_lon):
        # Rotation angle now loaded once at startup by _load_rotation_angle()
        # -- see that method for the file path and fallback behavior. Do NOT
        # hardcode a value here again; edit gps_rotation_calibration.yaml
        # (or re-run gps_rotation_calibrator.py) instead.
        rotation_angle = math.radians(self.rotation_angle_deg)
        x_gps, y_gps = gc.ll2xy(latitude, longitude, initial_lat, initial_lon)
        x_gps_local = x_gps*math.cos(rotation_angle) - \
            y_gps*math.sin(rotation_angle)
        y_gps_local = x_gps*math.sin(rotation_angle) + \
            y_gps*math.cos(rotation_angle)
        return x_gps_local, y_gps_local

    def _get_euler_from_quaternion(self, quad):
        quaternion = (quad[0], quad[1], quad[2], quad[3])
        euler = tf.euler_from_quaternion(quaternion)
        return euler

    def _integrate_yaw(self, current_orientation, angular_rate, dt):
        current_orientation += math.radians(angular_rate) * dt
        return current_orientation


if __name__ == "__main__":
    node = HakuroukunPose()
    node.run()