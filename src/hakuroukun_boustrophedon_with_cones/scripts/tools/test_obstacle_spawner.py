#!/usr/bin/env python3
# =============================================================================
#  test_obstacle_spawner.py
#
#  Drop / move / delete test obstacles in Gazebo to exercise the local replanner.
#
#  Two scenarios:
#    1. Static box   — spawns a 0.6 x 0.6 x 0.8 m box at (x, y). Stays put,
#                       trips the >=7s persistence threshold, replanner should
#                       carve a detour around it.
#    2. Pedestrian   — spawns a thin "person" cylinder, then optionally moves
#                       it away after `dwell` seconds. Use dwell=3-5 (less than
#                       persistence threshold) to verify the reflex stop fires
#                       but NO detour is computed. Use dwell=10+ to confirm a
#                       detour DOES eventually trigger.
#
#  Usage examples:
#    # Drop a static box at (5, 2) on the path:
#    rosrun hakuroukun_boustrophedon_with_cones test_obstacle_spawner.py \
#        static --x 5.0 --y 2.0
#
#    # Drop a pedestrian that stands for 5s then walks away:
#    rosrun hakuroukun_boustrophedon_with_cones test_obstacle_spawner.py \
#        pedestrian --x 5.0 --y 2.0 --dwell 5.0
#
#    # Delete by name:
#    rosrun hakuroukun_boustrophedon_with_cones test_obstacle_spawner.py \
#        delete --name test_box_0
# =============================================================================
import argparse
import sys
import time
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist


STATIC_BOX_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="c"><geometry><box><size>0.6 0.6 0.8</size></box></geometry></collision>
      <visual name="v">
        <geometry><box><size>0.6 0.6 0.8</size></box></geometry>
        <material>
          <ambient>0.9 0.2 0.2 1</ambient>
          <diffuse>0.9 0.2 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

# Pedestrian = a slim cylinder roughly person-shaped. NOT static so we can
# physically move it with SetModelState.
PEDESTRIAN_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial><mass>10</mass>
        <inertia>
          <ixx>1</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1</iyy><iyz>0</iyz><izz>1</izz>
        </inertia>
      </inertial>
      <collision name="c">
        <geometry><cylinder><radius>0.25</radius><length>1.7</length></cylinder></geometry>
      </collision>
      <visual name="v">
        <geometry><cylinder><radius>0.25</radius><length>1.7</length></cylinder></geometry>
        <material>
          <ambient>0.1 0.4 0.9 1</ambient>
          <diffuse>0.1 0.4 0.9 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def spawn(name, sdf_xml, x, y, z=0.4):
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=10.0)
    spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    pose = Pose(position=Point(x=x, y=y, z=z),
                orientation=Quaternion(x=0, y=0, z=0, w=1))
    resp = spawn_srv(model_name=name, model_xml=sdf_xml,
                     robot_namespace="", initial_pose=pose, reference_frame="world")
    if not resp.success:
        rospy.logerr("[spawner] spawn failed: %s", resp.status_message)
        return False
    rospy.loginfo("[spawner] spawned '%s' at (%.2f, %.2f)", name, x, y)
    return True


def delete(name):
    rospy.wait_for_service("/gazebo/delete_model", timeout=5.0)
    del_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    resp = del_srv(model_name=name)
    if not resp.success:
        rospy.logwarn("[spawner] delete failed: %s", resp.status_message)
        return False
    rospy.loginfo("[spawner] deleted '%s'", name)
    return True


def move(name, x, y, z=0.85):
    rospy.wait_for_service("/gazebo/set_model_state", timeout=5.0)
    set_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    ms = ModelState()
    ms.model_name = name
    ms.pose = Pose(position=Point(x=x, y=y, z=z),
                   orientation=Quaternion(x=0, y=0, z=0, w=1))
    ms.twist = Twist()
    ms.reference_frame = "world"
    resp = set_srv(ms)
    if not resp.success:
        rospy.logwarn("[spawner] move failed: %s", resp.status_message)
        return False
    return True


def cmd_static(args):
    return spawn(args.name, STATIC_BOX_SDF.format(name=args.name),
                 args.x, args.y, z=0.4)


def cmd_pedestrian(args):
    if not spawn(args.name, PEDESTRIAN_SDF.format(name=args.name),
                 args.x, args.y, z=0.85):
        return False
    if args.dwell <= 0:
        return True
    rospy.loginfo("[spawner] pedestrian standing for %.1fs then walking off "
                  "to (%.1f, %.1f) ...", args.dwell, args.exit_x, args.exit_y)
    rospy.sleep(args.dwell)
    # Smooth "walk" in ~1s by interpolating 10 steps.
    steps = 10
    for i in range(1, steps + 1):
        a = i / float(steps)
        xi = (1 - a) * args.x + a * args.exit_x
        yi = (1 - a) * args.y + a * args.exit_y
        move(args.name, xi, yi)
        rospy.sleep(0.1)
    rospy.loginfo("[spawner] pedestrian moved away.")
    return True


def cmd_delete(args):
    return delete(args.name)


def main():
    rospy.init_node("test_obstacle_spawner", anonymous=True)

    p = argparse.ArgumentParser(description="Drop test obstacles into Gazebo "
                                            "to exercise the local replanner.")
    sub = p.add_subparsers(dest="cmd")
    sub.required = True

    sp = sub.add_parser("static", help="Spawn a static box at (x, y).")
    sp.add_argument("--x", type=float, required=True)
    sp.add_argument("--y", type=float, required=True)
    sp.add_argument("--name", default="test_box_0")
    sp.set_defaults(func=cmd_static)

    pp = sub.add_parser("pedestrian", help="Spawn a pedestrian; optionally walk off.")
    pp.add_argument("--x", type=float, required=True)
    pp.add_argument("--y", type=float, required=True)
    pp.add_argument("--dwell", type=float, default=0.0,
                    help="Seconds to stand still before walking away "
                         "(0 = stay forever). Use <persistence_threshold to "
                         "test reflex-stop only, >persistence_threshold to "
                         "verify detour.")
    pp.add_argument("--exit-x", type=float, default=20.0,
                    help="X coord to walk to.")
    pp.add_argument("--exit-y", type=float, default=20.0,
                    help="Y coord to walk to.")
    pp.add_argument("--name", default="test_pedestrian_0")
    pp.set_defaults(func=cmd_pedestrian)

    dl = sub.add_parser("delete", help="Delete a named model.")
    dl.add_argument("--name", required=True)
    dl.set_defaults(func=cmd_delete)

    args = p.parse_args(rospy.myargv()[1:])
    ok = args.func(args)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
