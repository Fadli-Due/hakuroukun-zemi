#!/usr/bin/env python3
"""
map_odom_calibrator.py

Broadcasts the `map -> odom` transform for the real-robot setup (no AMCL).
Initially identity. Updates whenever a pose arrives on /initialpose, which
is the topic RViz's "2D Pose Estimate" tool publishes to.

Workflow:
  1. Launch this node (typically via offline_path_planning_real.launch).
  2. Robot icon appears in RViz at the wrong place — expected.
  3. Click "2D Pose Estimate" in RViz, click on the robot's actual position
     on the map, drag to set heading.
  4. This node receives /initialpose, computes the map->odom offset that
     reconciles the click with the current odom pose, and starts broadcasting
     that offset.
  5. Click again any time the icon drifts. No relaunch needed.

The math: given the current odom-frame pose P_odom and the user-supplied
map-frame pose P_map, we want a transform T such that T @ P_odom = P_map.
For 2D (x, y, yaw):
    T_yaw = P_map.yaw - P_odom.yaw
    T_x   = P_map.x - (cos(T_yaw) * P_odom.x - sin(T_yaw) * P_odom.y)
    T_y   = P_map.y - (sin(T_yaw) * P_odom.x + cos(T_yaw) * P_odom.y)
"""
import math
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MapOdomCalibrator:
    def __init__(self):
        rospy.init_node("map_odom_calibrator")

        # Frames
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.odom_topic = rospy.get_param(
            "~odom_topic", "/hakuroukun_pose/rear_wheel_odometry"
        )
        self.broadcast_rate = rospy.get_param("~broadcast_rate", 30.0)

        # Optional initial offset (useful if you want to start from a saved value)
        self.tx = float(rospy.get_param("~init_x", 0.0))
        self.ty = float(rospy.get_param("~init_y", 0.0))
        self.tyaw = float(rospy.get_param("~init_yaw", 0.0))

        # Latest odom-frame pose (cached from odom_topic)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_received = False

        self.br = tf2_ros.TransformBroadcaster()

        rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, self.initialpose_cb, queue_size=1
        )
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)

        rospy.Timer(rospy.Duration(1.0 / self.broadcast_rate), self.broadcast)

        rospy.loginfo(
            "[map_odom_calibrator] Ready. Initial %s->%s = (x=%.3f y=%.3f yaw=%.4f rad).",
            self.map_frame, self.odom_frame, self.tx, self.ty, self.tyaw,
        )
        rospy.loginfo(
            "[map_odom_calibrator] Listening on /initialpose. Use the "
            "'2D Pose Estimate' tool in RViz to calibrate."
        )

    def odom_cb(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.odom_yaw = yaw
        self.odom_received = True

    def initialpose_cb(self, msg):
        if not self.odom_received:
            rospy.logwarn(
                "[map_odom_calibrator] No odom message received yet on %s; "
                "ignoring /initialpose. Wait for hakuroukun_pose_node to start "
                "publishing.",
                self.odom_topic,
            )
            return

        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            rospy.logwarn(
                "[map_odom_calibrator] /initialpose has frame_id='%s' but "
                "expected '%s'. Check RViz Fixed Frame.",
                msg.header.frame_id, self.map_frame,
            )

        # Pose the user clicked, expressed in the map frame
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, pyaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Snapshot current odom pose so the math is consistent
        ox, oy, oyaw = self.odom_x, self.odom_y, self.odom_yaw

        # Solve T such that T @ (ox, oy, oyaw) == (px, py, pyaw)
        new_yaw = self._wrap(pyaw - oyaw)
        c = math.cos(new_yaw)
        s = math.sin(new_yaw)
        new_x = px - (c * ox - s * oy)
        new_y = py - (s * ox + c * oy)

        self.tx, self.ty, self.tyaw = new_x, new_y, new_yaw

        rospy.loginfo(
            "[map_odom_calibrator] %s->%s updated: x=%.3f y=%.3f yaw=%.4f rad "
            "(odom snapshot: x=%.3f y=%.3f yaw=%.4f; click: x=%.3f y=%.3f yaw=%.4f)",
            self.map_frame, self.odom_frame, self.tx, self.ty, self.tyaw,
            ox, oy, oyaw, px, py, pyaw,
        )

    def broadcast(self, _event):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = self.tx
        t.transform.translation.y = self.ty
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, self.tyaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)

    @staticmethod
    def _wrap(a):
        # Wrap to (-pi, pi]
        while a > math.pi:
            a -= 2.0 * math.pi
        while a <= -math.pi:
            a += 2.0 * math.pi
        return a


if __name__ == "__main__":
    try:
        MapOdomCalibrator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass