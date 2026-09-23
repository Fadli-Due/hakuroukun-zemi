#!/usr/bin/env python3
"""publish_map_odom_calibrated.py

In the real-robot launch, map_odom_calibrator.py publishes a latched
std_msgs/Bool on /map_odom_calibrator/initialized once the user clicks
"2D Pose Estimate" in RViz. path_follower.py (and offline_coverage_planner.py)
block on that topic before starting.

The D-F sim launch uses AMCL instead of map_odom_calibrator.py, so that topic
never gets published and the nodes wait forever. This script publishes it
once, after a short delay, so the sim run starts automatically without a
manual RViz click.

Params:
  delay  -- seconds to wait before publishing (default 2.0), long enough
            for AMCL / map_server / TF to be up.
"""
import rospy
from std_msgs.msg import Bool


def main():
    rospy.init_node('publish_map_odom_calibrated', anonymous=False)
    delay = rospy.get_param('~delay', 2.0)

    # latch=True so nodes that subscribe slightly later still get it
    pub = rospy.Publisher('/map_odom_calibrator/initialized', Bool, queue_size=1, latch=True)

    rospy.loginfo(f"[publish_map_odom_calibrated] waiting {delay:.1f}s then "
                   f"publishing True on /map_odom_calibrator/initialized ...")
    rospy.sleep(delay)

    msg = Bool(data=True)
    # publish a few times in case a subscriber wasn't fully connected yet
    rate = rospy.Rate(5)
    for _ in range(5):
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo("[publish_map_odom_calibrated] published. Exiting.")


if __name__ == '__main__':
    main()