#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool

# NEW TASP PLANNER
from hakuroukun_tasp_with_cones.src.tasp_path_planner_simple import TASPPathPlannerSimple

from simple_astar import SimpleOccupancyGrid, astar_plan
from visualizer import TASPVisualizer

current_pose = None
inflated_grid_msg = None

def send_stop_signal():
    stop_msg = Bool()
    stop_msg.data = True
    stop_signal_pub.publish(stop_msg)

def odom_callback(data):
    global current_pose
    p = data.pose.pose.position
    o = data.pose.pose.orientation
    _,_,yaw = euler_from_quaternion([o.x,o.y,o.z,o.w])
    current_pose = [p.x, p.y, yaw]

def costmap_callback(msg):
    global inflated_grid_msg
    inflated_grid_msg = msg

def robot_has_reached_goal(cur, goal, tol):
    return math.hypot(goal[0]-cur[0], goal[1]-cur[1]) <= tol

def plan_path(occ, start, goal):
    grid = SimpleOccupancyGrid(
        occ.info.width, occ.info.height,
        occ.info.resolution,
        occ.info.origin.position.x,
        occ.info.origin.position.y,
        occ.data
    )
    return astar_plan(grid, start[0], start[1], goal[0], goal[1], connectivity=8)

def publish_path(path_points):
    msg = Path()
    msg.header.frame_id = "odom"
    msg.header.stamp = rospy.Time.now()
    for px, py in path_points:
        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = px
        ps.pose.position.y = py
        ps.pose.orientation.w = 1.0
        msg.poses.append(ps)
    path_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("tasp_with_cones")

    start_pose = rospy.get_param("start_pose", [0,0,0])
    tasp_goal_tolerance = rospy.get_param("tasp_goal_tolerance", 1.0)
    wait_duration = rospy.Duration(rospy.get_param("wait_duration", 5.0))
    rate_hz = rospy.get_param("rate_hz", 2)

    # NEW:
    tasp_planner = TASPPathPlannerSimple()
    vis = TASPVisualizer(frame_id="odom")

    rospy.Subscriber("/hakuroukun_pose/rear_wheel_odometry", Odometry, odom_callback)
    rospy.Subscriber("/costmap_node/costmap/costmap", OccupancyGrid, costmap_callback)

    path_pub = rospy.Publisher("/desired_path", Path, queue_size=10)
    stop_signal_pub = rospy.Publisher("/stop_signal", Bool, queue_size=10)

    rate = rospy.Rate(rate_hz)
    start_time = rospy.Time.now()

    current_goal = None
    current_path = []

    while not rospy.is_shutdown():
        vis.publish_start(start_pose)
        vis.publish_camera_fov()

        if rospy.Time.now() - start_time < wait_duration:
            rate.sleep()
            continue

        if inflated_grid_msg is None:
            rospy.logwarn_throttle(5, "Waiting for inflated costmap...")
            rate.sleep()
            continue

        if current_pose is None:
            rospy.logwarn_throttle(5.0, "[TASP] Waiting for odometry (current_pose is None)...")
            rate.sleep()
            continue        

        # 1. Get new TASP goal if needed
        if current_goal is None or robot_has_reached_goal(current_pose, current_goal, tasp_goal_tolerance):
            current_goal = tasp_planner.tasp_path_planning(current_pose, inflated_grid_msg, start_pose)

            if current_goal:
                rospy.loginfo(f"[TASP] New goal: {current_goal}")
                vis.publish_goal(current_goal)
            else:
                rospy.loginfo("[TASP] No more goals. Sending stop...")
                send_stop_signal()
                break

        # 2. A* path to goal
        current_path = plan_path(inflated_grid_msg, current_pose, current_goal)

        if current_path == "GOAL_OCCUPIED":
            rospy.logwarn("[TASP] Goal occupied → skipping")
            current_goal = None
            continue

        if not current_path:
            rospy.logwarn("[TASP] No A* path found → skipping")
            send_stop_signal()
            continue

        publish_path(current_path)
        vis.publish_astar_path(current_path)

        rate.sleep()
