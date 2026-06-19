import rospy, numpy as np
from nav_msgs.msg import OccupancyGrid
def cb(msg):
    data = np.array(msg.data, dtype=np.int16)
    known = np.sum(data != -1)
    total = data.size
    occ = np.sum(data > 50)
    free = np.sum(data == 0)
    print("known_ratio:", known/total, "free:", free, "occ:", occ, "total:", total)
    raise SystemExit
rospy.init_node("map_stat", anonymous=True)
rospy.Subscriber("/map", OccupancyGrid, cb)
rospy.spin()