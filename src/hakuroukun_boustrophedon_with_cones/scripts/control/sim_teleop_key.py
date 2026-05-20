#!/usr/bin/env python3
"""
sim_teleop_key.py
-----------------
Keyboard teleop for the Hakuroukun Gazebo sim, used for manually driving
the robot during gmapping.

Publishes Float64MultiArray([velocity_m_s, steering_angle_rad]) on
/hakuroukun_steering_controller/cmd_controller — same format the
path_follower uses.

Controls (single key press; state persists):
    w : nudge forward velocity   ( +V_STEP )
    s : nudge reverse velocity   ( -V_STEP )
    a : steer left               ( +S_STEP )
    d : steer right              ( -S_STEP )
  space: zero velocity AND steering (stop, wheels straight)
    x : zero velocity only (coast, keep steering)
    z : zero steering only
    , : halve current velocity (fine slow-down)
    . : halve current steering
    q : quit

Notes:
- Limits: |v| <= V_MAX (m/s), |steer| <= S_MAX (rad).
- The script publishes at 10 Hz continuously, regardless of input.
- Run this in its OWN terminal (it grabs stdin).
"""
import sys
import termios
import tty
import select
import rospy
from std_msgs.msg import Float64MultiArray

# ---- Tunables ----
V_MAX = 0.50           # m/s, capped for safe sim driving
S_MAX = 0.45           # rad, ~26 deg, reasonable Ackermann limit
V_STEP = 0.05
S_STEP = 0.05
RATE_HZ = 10
CMD_TOPIC = "/hakuroukun_steering_controller/cmd_controller"

HELP = """
[sim_teleop_key]
 w/s : +/- velocity   (V_STEP=%.2f, |v| <= %.2f m/s)
 a/d : +/- steer left/right  (S_STEP=%.2f, |steer| <= %.2f rad)
 space: full stop (v=0, steer=0)
 x   : v=0 (keep steer)
 z   : steer=0 (keep v)
 , . : halve v / halve steer
 q   : quit
""" % (V_STEP, V_MAX, S_STEP, S_MAX)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def get_key(timeout=0.05):
    """Non-blocking single-character read from stdin."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rospy.init_node("sim_teleop_key", anonymous=False)
    pub = rospy.Publisher(CMD_TOPIC, Float64MultiArray, queue_size=10)
    rate = rospy.Rate(RATE_HZ)

    v = 0.0
    steer = 0.0

    sys.stdout.write(HELP)
    sys.stdout.flush()

    last_status = ""
    while not rospy.is_shutdown():
        k = get_key(timeout=0.05)
        if k:
            if k == "w":
                v = clamp(v + V_STEP, -V_MAX, V_MAX)
            elif k == "s":
                v = clamp(v - V_STEP, -V_MAX, V_MAX)
            elif k == "a":
                steer = clamp(steer + S_STEP, -S_MAX, S_MAX)
            elif k == "d":
                steer = clamp(steer - S_STEP, -S_MAX, S_MAX)
            elif k == " ":
                v = 0.0
                steer = 0.0
            elif k == "x":
                v = 0.0
            elif k == "z":
                steer = 0.0
            elif k == ",":
                v *= 0.5
            elif k == ".":
                steer *= 0.5
            elif k == "q" or k == "\x03":   # q or Ctrl-C
                break

        msg = Float64MultiArray()
        msg.data = [float(v), float(steer)]
        pub.publish(msg)

        status = "  v=%+.2f m/s   steer=%+.2f rad (%+5.1f deg)" % (
            v, steer, steer * 180.0 / 3.14159265,
        )
        if status != last_status:
            # Re-print on the same console line.
            sys.stdout.write("\r" + status + "   ")
            sys.stdout.flush()
            last_status = status

        rate.sleep()

    # Stop the robot on exit.
    stop = Float64MultiArray()
    stop.data = [0.0, 0.0]
    for _ in range(5):
        pub.publish(stop)
        rospy.sleep(0.02)
    sys.stdout.write("\n[sim_teleop_key] stopped.\n")
    sys.stdout.flush()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass