#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomTFBroadcaster:
    def __init__(self):
        rospy.init_node("odom_tf_broadcaster", anonymous=True)

        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.odom_topic = rospy.get_param("~odom_topic", "/hakuroukun_pose/rear_wheel_odometry")

        self.br = tf2_ros.TransformBroadcaster()
        rospy.Subscriber(self.odom_topic, Odometry, self.cb, queue_size=10)

        rospy.loginfo(f"[odom_tf_broadcaster] Listening: {self.odom_topic}")
        rospy.loginfo(f"[odom_tf_broadcaster] Publishing TF: {self.odom_frame} -> {self.base_frame}")

    def cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

if __name__ == "__main__":
    OdomTFBroadcaster()
    rospy.spin()