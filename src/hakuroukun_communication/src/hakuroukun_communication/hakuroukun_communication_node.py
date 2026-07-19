#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright 2024 - ISE Mobile Robot Group. All Rights Reserved.
# Modified by Fadli Due Ramandavito (2026)
import serial
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Int8MultiArray
import time


class HakuroukunCommunicationNode(object):
    """
    ROS Node for controlling the Hakuroukun robot via serial motor control.
    Applies balanced quadratic mapping for steering.
    """

    def __init__(self) -> None:
        rospy.init_node("hakuroukun_communication_node", anonymous=True)

        # Get parameters
        port = rospy.get_param("/hakuroukun_communication_node/port")
        baud_rate = rospy.get_param("/hakuroukun_communication_node/baud_rate")
        controller_rate = rospy.get_param(
            "/hakuroukun_communication_node/controller_rate", 10)

        # Open serial connection
        self.connection = serial.Serial(port, int(baud_rate), timeout=0.05)
        time.sleep(2)  # Give Arduino time to reset after serial connect
        # Drain any startup messages (e.g. "bcd_automode ready")
        while self.connection.in_waiting:
            self.connection.readline()
        rospy.loginfo(f"Connected to {port} at {baud_rate} baud, "
                      f"controller_rate={controller_rate} Hz")

        # Subscribers
        self.cmd_controller_subscriber = rospy.Subscriber(
            "/cmd_controller", Float64MultiArray, self._cmd_controller_callback
        )

        # Publisher for gear state feedback (2026-07-19).
        # Data layout: [gear_actual (0=fwd, 1=rev), pend (0=idle, 1=pending)].
        # path_follower.py uses this to gate the reverse-mode timer so
        # the min_rev_dwell only starts counting after the gear is
        # physically engaged (gear matches commanded, pend=0). Without
        # this, reverse dwell burns during mechanical gear shift latency
        # and the robot never actually moves backward.
        self.gear_state_pub = rospy.Publisher(
            "/hakuroukun/gear_state", Int8MultiArray, queue_size=1)

        # Timer for sending commands at a fixed rate
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / float(controller_rate)),
            self._timer_callback
        )

        # Internal variables
        self.sequence_id = 0
        self.cmd_vel_msg = Twist()
        self.cmd_controller_msg = Float64MultiArray()
        self.cmd_controller_msg.data = [0.0, 0.0]
        self.cumulative_steering_angle = 0.0
        self.cmd_vel_flag = False
        self.cmd_controller_flag = False
        self.direction = 0  # 0 = forward, 1 = reverse
        self.previous_steering_angle = 0.0

        # Hysteresis state for curve-switching (see _corrected_steering_command).
        self.committed_direction_ccw = None
        self.committed_goal_angle = 0.0
        self.STEERING_HYSTERESIS_RAD = 0.03  # tune if oscillation persists

    def run(self) -> None:
        rospy.spin()

    def _timer_callback(self, event) -> None:
        if not self.cmd_vel_flag and not self.cmd_controller_flag:
            return
        acceleration_command, steering_command = self._apply_indentification()

        # Format: "0" + dir(1) + steering(3) + acceleration(3) + "00" = 10 chars
        command = f"0{self.direction}{steering_command:03d}{acceleration_command:03d}00"
        rospy.loginfo(command)

        self.connection.write(bytes(command + "\r\n", encoding='ascii'))
        self.connection.flush()

        # Read diagnostic response from Arduino
        # bcd_automode.ino sends: "<status><dir><steer><accel>,pm_st=N,pm_ac=N,gear=N,pend=N\n"
        try:
            data = self.connection.readline()
            if data:
                response = data.decode('ascii', errors='replace').strip()
                if response and len(response) > 0:
                    status_char = response[0]
                    if status_char == '2':
                        rospy.logwarn(f"Arduino motor watchdog tripped: {response}")
                    elif status_char == '1':
                        rospy.logwarn(f"Arduino rejected command: {response}")
                    else:
                        rospy.logdebug(f"PM feedback: {response}")

                    # Parse gear + pending state from the diagnostic tail.
                    # Firmware format: "...,gear=<0|1>,pend=<0|1>"
                    if ',gear=' in response:
                        try:
                            gear_actual = int(response.split(',gear=')[1][0])
                            pend_int = int(response.split(',pend=')[1][0]) \
                                if ',pend=' in response else 0

                            # Publish gear state for path_follower's reverse timer.
                            gear_msg = Int8MultiArray()
                            gear_msg.data = [gear_actual, pend_int]
                            self.gear_state_pub.publish(gear_msg)

                            if gear_actual != self.direction:
                                rospy.logwarn_throttle(
                                    1.0,
                                    f"[comm] GEAR MISMATCH: "
                                    f"commanded={self.direction} "
                                    f"actual={gear_actual} pending={pend_int}")
                        except (IndexError, ValueError):
                            pass
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Serial read error: {e}")

    def _cmd_controller_callback(self, msg: Float64MultiArray) -> None:
        self.cmd_controller_msg = msg
        self.cmd_controller_flag = True

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.cmd_vel_msg = msg
        self.cmd_vel_flag = True

    def _apply_indentification(self):
        linear_velocity = 0.0
        steering_angle = 0.0

        if self.cmd_vel_flag:
            self.cmd_vel_flag = False
            if self.cmd_vel_msg.linear.x < 0:
                self.direction = 1
            else:
                self.direction = 0
            linear_velocity = abs(self.cmd_vel_msg.linear.x)
            steering_angle_delta = self.cmd_vel_msg.angular.z * 0.5
            self.cumulative_steering_angle += steering_angle_delta
            steering_angle = self.cumulative_steering_angle

        elif self.cmd_controller_flag:
            if self.cmd_controller_msg.data[0] < 0:
                self.direction = 1
            else:
                self.direction = 0
            linear_velocity = abs(self.cmd_controller_msg.data[0])
            steering_angle = self.cmd_controller_msg.data[1]

        # Acceleration command
        if linear_velocity == 0:
            acceleration_command = 193   # neutral, pedal released
        else:
            raw_accel = 193 + linear_velocity * 1428
            acceleration_command = max(raw_accel, 550)
        acceleration_command = max(193, min(767, acceleration_command))

        # Steering command
        steering_val = self._corrected_steering_command(steering_angle)
        steering_command = round(steering_val)
        steering_command = max(287, min(690, steering_command))

        return int(acceleration_command), int(steering_command)

    def _corrected_steering_command(self, goal_angle_rad: float) -> float:
        delta = goal_angle_rad - self.committed_goal_angle
        if (self.committed_direction_ccw is None
                or abs(delta) > self.STEERING_HYSTERESIS_RAD):
            self.committed_direction_ccw = (delta > 0)
            self.committed_goal_angle = goal_angle_rad

        self.previous_steering_angle = goal_angle_rad

        if self.committed_direction_ccw:
            a_ccw = -86.29
            b_ccw = 317.31
            c_ccw = 567
            return a_ccw * (goal_angle_rad ** 2) + b_ccw * goal_angle_rad + c_ccw
        else:
            a_cw = 69.86
            b_cw = 317.31
            c_cw = 567
            return a_cw * (goal_angle_rad ** 2) + b_cw * goal_angle_rad + c_cw


if __name__ == "__main__":
    node = HakuroukunCommunicationNode()
    node.run()