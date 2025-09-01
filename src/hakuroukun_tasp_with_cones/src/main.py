#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool, Float32

from tasp_path_planner import TASPPathPlanner
from simple_astar import SimpleOccupancyGrid, astar_plan
from visualizer import TASPVisualizer

current_pose = None
inflated_grid_msg = None
path_pub = None
stop_signal_pub = None

def send_stop_signal(stop=True):
    msg = "STOP" if stop else "RESUME"
    rospy.loginfo_throttle(2.0, f"[main] Sending {msg} signal to follower")
    stop_msg = Bool()
    stop_msg.data = bool(stop)
    stop_signal_pub.publish(stop_msg)

def odom_callback(data):
    global current_pose
    p = data.pose.pose.position
    q = data.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    current_pose = [p.x, p.y, yaw]

def costmap_callback(msg):
    global inflated_grid_msg
    inflated_grid_msg = msg

def robot_has_reached_goal(cur, goal, tol):
    dx = goal[0] - cur[0]; dy = goal[1] - cur[1]
    return math.hypot(dx, dy) <= tol

def plan_path(occupancy_grid_msg, start, goal, occ_threshold=50):
    """A* with start-cell nudge; unknown treated as blocked."""
    width  = occupancy_grid_msg.info.width
    height = occupancy_grid_msg.info.height
    res    = occupancy_grid_msg.info.resolution
    ox     = occupancy_grid_msg.info.origin.position.x
    oy     = occupancy_grid_msg.info.origin.position.y
    data   = occupancy_grid_msg.data

    def world_to_index(wx, wy):
        cx = int((wx - ox) / res); cy = int((wy - oy) / res)
        return cx, cy

    def in_bounds(cx, cy):
        return 0 <= cx < width and 0 <= cy < height

    def cell_value(wx, wy):
        cx, cy = world_to_index(wx, wy)
        if not in_bounds(cx, cy):
            return 100
        v = data[cy * width + cx]
        return 100 if v < 0 else v

    def blocked(wx, wy):
        return cell_value(wx, wy) >= occ_threshold

    if blocked(goal[0], goal[1]):
        return "GOAL_OCCUPIED"

    sx, sy = start[0], start[1]
    if blocked(sx, sy):
        step_r = max(res, 0.05); max_r = 0.6; found = None; r = step_r
        while r <= max_r and found is None:
            for k in range(16):
                a = 2.0 * math.pi * (k / 16.0)
                tx = sx + r * math.cos(a); ty = sy + r * math.sin(a)
                if not blocked(tx, ty):
                    found = (tx, ty); break
            r += step_r
        if found: sx, sy = found
        else: return []

    ogrid = SimpleOccupancyGrid(width, height, res, ox, oy, data)
    return astar_plan(ogrid, sx, sy, goal[0], goal[1], connectivity=8)

def publish_path(path_points, frame_id="odom"):
    msg = Path()
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()
    now = rospy.Time.now()
    for px, py in path_points:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = now
        ps.pose.position.x = px
        ps.pose.position.y = py
        ps.pose.orientation.w = 1.0
        msg.poses.append(ps)
    path_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("tasp_with_cones")

    start_pose          = rospy.get_param("start_pose", [0.0, 0.0, 0.0])
    tasp_goal_tolerance = float(rospy.get_param("tasp_goal_tolerance", 1.0))
    wait_duration       = rospy.Duration(rospy.get_param("wait_duration", 5.0))
    rate_hz             = int(rospy.get_param("rate_hz", 2))

    pause_on_cell_change = rospy.get_param("pause_on_cell_change", True)
    cell_change_delta_m  = float(rospy.get_param("cell_change_delta_m", 0.15))
    cell_change_pause_s  = float(rospy.get_param("cell_change_pause_s", 0.8))

    skip_goal_on_plan_fail = rospy.get_param("skip_goal_on_plan_fail", True)
    max_plan_failures      = int(rospy.get_param("max_plan_failures", 2))
    halt_on_plan_fail      = rospy.get_param("halt_on_plan_fail", False)  # <— NEW

    tasp_planner = TASPPathPlanner()
    vis = TASPVisualizer(frame_id="odom")

    rospy.Subscriber("/hakuroukun_pose/rear_wheel_odometry", Odometry, odom_callback)
    rospy.Subscriber("/costmap_node/costmap/costmap", OccupancyGrid, costmap_callback)
    path_pub = rospy.Publisher("/desired_path", Path, queue_size=10)
    stop_signal_pub = rospy.Publisher("/stop_signal", Bool, queue_size=10)

    state = {"last_cell_size": None, "pause_until": rospy.Time(0), "force_new_goal": False}
    def _cell_size_cb(msg: Float32):
        if not pause_on_cell_change: return
        new = float(msg.data); prev = state["last_cell_size"]
        if prev is None: state["last_cell_size"] = new; return
        if abs(new - prev) >= cell_change_delta_m:
            rospy.loginfo(f"[replan] cell size {prev:.2f}→{new:.2f} m; pausing {cell_change_pause_s:.1f}s")
            state["last_cell_size"] = new
            state["pause_until"] = rospy.Time.now() + rospy.Duration(cell_change_pause_s)
            state["force_new_goal"] = True
            send_stop_signal(True)
        else:
            state["last_cell_size"] = new
    rospy.Subscriber("/tasp/debug/cell_size", Float32, _cell_size_cb, queue_size=1)

    rate = rospy.Rate(rate_hz)
    start_time = rospy.Time.now()
    current_goal = None
    plan_fail_count = 0

    while not rospy.is_shutdown():
        if pause_on_cell_change and rospy.Time.now() < state["pause_until"]:
            rate.sleep(); continue
        if state.get("force_new_goal", False):
            current_goal = None
            state["force_new_goal"] = False

        if current_pose is None:
            rospy.logwarn_throttle(5.0, "[main] Waiting for odom (current_pose is None)...")
            rate.sleep(); continue

        vis.publish_start(start_pose)
        vis.publish_camera_fov()

        if rospy.Time.now() - start_time < wait_duration:
            rate.sleep(); continue

        if inflated_grid_msg is None:
            rospy.logwarn_throttle(5.0, "[main] Waiting for inflated costmap data...")
            rate.sleep(); continue

        # 1) Pick/refresh TASP goal
        if current_goal is None or robot_has_reached_goal(current_pose, current_goal, tasp_goal_tolerance):
            new_goal = tasp_planner.tasp_path_planning(current_pose, inflated_grid_msg, start_pose)
            vis.publish_btp(tasp_planner.BTP)
            if new_goal:
                rospy.loginfo(f"[TASP] New goal: {new_goal}")
                current_goal = new_goal
                vis.publish_goal(current_goal)
            else:
                rospy.loginfo("[TASP] No more goals.")
                send_stop_signal(True)
                rate.sleep(); continue

        # 2) Plan A*
        if current_goal is None:
            rate.sleep(); continue
        current_path = plan_path(inflated_grid_msg, current_pose, current_goal)

        if current_path == "GOAL_OCCUPIED":
            rospy.logwarn("[TASP] Goal cell occupied, requesting new goal ...")
            current_goal = None; plan_fail_count = 0
            rate.sleep(); continue

        if not current_path:
            plan_fail_count += 1
            rospy.logwarn_throttle(2.0, f"[TASP] A* failed (no route). Fail count={plan_fail_count}")
            # <— ONLY halt if explicitly requested
            if halt_on_plan_fail:
                send_stop_signal(True)
            else:
                send_stop_signal(False)
            if skip_goal_on_plan_fail and plan_fail_count >= max_plan_failures:
                rospy.logwarn("[TASP] Skipping unroutable goal after repeated failures.")
                if current_goal in tasp_planner.BTP:
                    try: tasp_planner.BTP.remove(current_goal)
                    except ValueError: pass
                current_goal = None; plan_fail_count = 0
            rate.sleep(); continue

        # 3) Publish path + resume follower
        frame_id = inflated_grid_msg.header.frame_id or "odom"
        publish_path(current_path, frame_id=frame_id)
        plan_fail_count = 0
        send_stop_signal(False)
        vis.publish_astar_path(current_path)

        rate.sleep()
