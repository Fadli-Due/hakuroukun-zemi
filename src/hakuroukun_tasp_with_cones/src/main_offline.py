#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

grid_msg = None

def send_stop_signal(pub):
    msg = Bool()
    msg.data = True
    pub.publish(msg)

def grid_cb(msg):
    global grid_msg
    grid_msg = msg

def is_free(v):
    # costmap/occupancy conventions:
    # -1 unknown, 0 free, 100 occupied; costmap may be 0..100
    return v == 0

def grid_to_world(ix, iy, info):
    wx = info.origin.position.x + (ix + 0.5) * info.resolution
    wy = info.origin.position.y + (iy + 0.5) * info.resolution
    return wx, wy

def boustrophedon_sweep(msg, step_cells):
    w = msg.info.width
    h = msg.info.height
    data = msg.data
    info = msg.info

    pts = []
    left_to_right = True

    for y in range(0, h, step_cells):
        xs = range(0, w, step_cells) if left_to_right else range(w - 1, -1, -step_cells)
        row_pts = []
        for x in xs:
            idx = y * w + x
            if is_free(data[idx]):
                row_pts.append(grid_to_world(x, y, info))
        pts.extend(row_pts)
        left_to_right = not left_to_right

    return pts

def publish_path(path_pub, frame_id, pts):
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = rospy.Time.now()

    now = rospy.Time.now()
    for (x, y) in pts:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = now
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)

    path_pub.publish(path)

if __name__ == "__main__":
    rospy.init_node("boustrophedon_offline")

    # Params
    source_topic = rospy.get_param("~grid_topic", "/costmap_node/costmap/costmap")
    frame_id     = rospy.get_param("~frame_id", "map")
    step_cells   = int(rospy.get_param("~step_cells", 2))
    republish_hz = float(rospy.get_param("~republish_hz", 0.5))  # 0.5 = every 2s

    path_pub = rospy.Publisher("/desired_path", Path, queue_size=1, latch=True)
    stop_pub = rospy.Publisher("/stop_signal", Bool, queue_size=1)

    rospy.Subscriber(source_topic, OccupancyGrid, grid_cb)

    rate = rospy.Rate(max(republish_hz, 0.1))

    planned_pts = None

    while not rospy.is_shutdown():
        if grid_msg is None:
            rate.sleep()
            continue

        if planned_pts is None:
            planned_pts = boustrophedon_sweep(grid_msg, step_cells)
            if not planned_pts:
                send_stop_signal(stop_pub)
                planned_pts = []
            publish_path(path_pub, frame_id, planned_pts)

        # republish (latch already helps, but this keeps follower refreshed)
        publish_path(path_pub, frame_id, planned_pts)
        rate.sleep()