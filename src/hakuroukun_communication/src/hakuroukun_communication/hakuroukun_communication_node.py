#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright 2024 - ISE Mobile Robot Group. All Rights Reserved.
# Modified by Fadli Due Ramandavito (2026)
#
# Changelog vs previous version:
#   1. Serial command now ends with \r\n — Arduino reads with readStringUntil('\n')
#      instead of relying on a 2ms timeout.  Eliminates partial-read failures.
#   2. Default controller_rate raised from 1 Hz to 10 Hz to match path_follower's
#      control_rate.  (Override via communication_bringup.launch param.)
#   3. Serial read uses readline() with a short timeout and logs motor watchdog
#      warnings ('2' status) from bcd_automode.ino.
#   4. Original three fixes preserved (10-char pad, direction not reset, flag handling).

import serial
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
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
        # If you want to use /cmd_vel, uncomment below:
        # self.cmd_vel_subscriber = rospy.Subscriber(
        #     "/cmd_vel", Twist, self._cmd_vel_callback
        # )

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

    def run(self) -> None:
        rospy.spin()

    def _timer_callback(self, event) -> None:
        acceleration_command, steering_command = self._apply_indentification()

        # Format: "0" + dir(1) + steering(3) + acceleration(3) + "00" = 10 chars
        # Then append \r\n so the Arduino can use readStringUntil('\n')
        command = f"0{self.direction}{steering_command:03d}{acceleration_command:03d}00"
        rospy.loginfo(command)

        self.connection.write(bytes(command + "\r\n", encoding='ascii'))
        self.connection.flush()

        # Read diagnostic response from Arduino
        # bcd_automode.ino sends: "<status><dir><steer><accel>,pm_st=N,pm_ac=N\n"
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
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Serial read error: {e}")

    def _cmd_controller_callback(self, msg: Float64MultiArray) -> None:
        self.cmd_controller_msg = msg
        self.cmd_controller_flag = True

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.cmd_vel_msg = msg
        self.cmd_vel_flag = True

    def _apply_indentification(self):
        """
        Determines appropriate motor acceleration and steering commands.
        Uses an asymmetric quadratic mapping for steering power correction.
        """
        linear_velocity = 0.0
        steering_angle = 0.0
        # Direction is only updated when a new message actually arrives.
        # (FIX 2: do NOT reset self.direction = 0 unconditionally.)

        # Check /cmd_vel
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

        # Check /cmd_controller
        elif self.cmd_controller_flag:
            # Keep applying last cmd — don't reset flag so the last command
            # persists across timer ticks even if no new message arrives.
            if self.cmd_controller_msg.data[0] < 0:
                self.direction = 1
            else:
                self.direction = 0
            linear_velocity = abs(self.cmd_controller_msg.data[0])
            steering_angle = self.cmd_controller_msg.data[1]

        # Acceleration command
        if linear_velocity == 0:
            acceleration_command = 290
        else:
            acceleration_command = (linear_velocity + 1) * 500
        acceleration_command = max(290, min(680, acceleration_command))

        # Steering command (with asymmetric correction)
        steering_val = self._corrected_steering_command(steering_angle)
        steering_command = round(steering_val)
        steering_command = max(390, min(850, steering_command))

        return int(acceleration_command), int(steering_command)

    def _corrected_steering_command(self, goal_angle_rad: float) -> float:
        """
        Applies asymmetric quadratic correction.
        - CW follows:    p(theta) =  69.86 theta^2 + 317.31 theta + 525
        - CCW follows:   p(theta) = -86.29 theta^2 + 317.31 theta + 610
        """
        try:
            current_angle_rad = self.previous_steering_angle
        except AttributeError:
            self.previous_steering_angle = 0.0
            current_angle_rad = 0.0

        is_counterclockwise = goal_angle_rad > current_angle_rad

        self.previous_steering_angle = goal_angle_rad

        if is_counterclockwise:
            a_ccw = -86.29
            b_ccw = 317.31
            c_ccw = 610
            return a_ccw * (goal_angle_rad ** 2) + b_ccw * goal_angle_rad + c_ccw
        else:
            a_cw = 69.86
            b_cw = 317.31
            c_cw = 525
            return a_cw * (goal_angle_rad ** 2) + b_cw * goal_angle_rad + c_cw


if __name__ == "__main__":
    node = HakuroukunCommunicationNode()
    node.run()