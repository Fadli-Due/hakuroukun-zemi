import math
from datetime import datetime
import pytz
import os

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
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
        self.publish_rate = 0.1
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
        rospy.loginfo("IMU Data Received")

    def _gps_callback(self, data: NavSatFix):
        self.x_gps, self.y_gps = self._get_xy_from_latlon(
            data.latitude, data.longitude, self.initial_lat, self.initial_lon)
        gps_to_rear_axis = 0.6
        self.x_rear = self.x_gps - \
            gps_to_rear_axis * math.cos(self.yaw)
        self.y_rear = self.y_gps - \
            gps_to_rear_axis * math.sin(self.yaw)

    def _imu_callback(self, data: Imu):
        raw_z = data.angular_velocity.z

        # Bias calibration: collect first 50 samples while stationary
        if not self._bias_calibrated:
            self._bias_samples.append(raw_z)
            if len(self._bias_samples) >= 50:
                self._gyro_bias_z = sum(self._bias_samples) / len(self._bias_samples)
                self._bias_calibrated = True
                rospy.loginfo(f"Gyro Z bias calibrated: {self._gyro_bias_z:.4f} deg/s")
            return

        corrected_z = raw_z - self._gyro_bias_z
        now = rospy.get_time()
        dt = now - self._last_imu_time
        self._last_imu_time = now
        if dt > 0.0 and dt < 1.0:
            self.yaw = self._integrate_yaw(self.yaw, corrected_z, dt)
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
        rospy.loginfo(
            f"x: {rear_wheel_msg.pose.pose.position.x}, y: {rear_wheel_msg.pose.pose.position.y}")
        self.rear_pose_pub.publish(rear_wheel_msg)

    def _log_pose(self, timer):
        pose = f"{self.x_rear}, {self.y_rear}, {(self.yaw)}" + "\n"
        with open(self.file_name, mode="a") as f:
            f.write(pose)

    def _get_xy_from_latlon(self, latitude, longitude, initial_lat, initial_lon):
        # Measured 2026-06-30 via GPS straight-line drive (rotation_angle calibration)
        rotation_angle = math.radians(172.1)
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