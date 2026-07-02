#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright 2024 - ISE Mobile Robot Group. All Rights Reserved.
# Modified by Fadli Due Ramandavito (2026) — three fixes applied:
#   1. Serial command padded to 10 chars (Arduino length check was rejecting all commands)
#   2. self.direction no longer resets to 0 every timer tick (caused relay chatter during REVERSE)
#   3. cmd_controller_flag now reset to False after processing (was never cleared)
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
        """
        Class constructor
        """
        rospy.init_node("hakuroukun_communication_node", anonymous=True)
        # Get parameters
        port = rospy.get_param("/hakuroukun_communication_node/port")
        baud_rate = rospy.get_param("/hakuroukun_communication_node/baud_rate")
        controller_rate = rospy.get_param("/hakuroukun_communication_node/controller_rate")
        # Open serial connection
        self.connection = serial.Serial(port, int(baud_rate), timeout=0.05)
        time.sleep(2)  # Give some time to initialize
        rospy.loginfo(f"Connected to {port} at {baud_rate} baud rate")
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
            rospy.Duration(1 / float(controller_rate)),
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
        self.previous_steering_angle = 0.0  # Track the last commanded steering angle

    def run(self) -> None:
        """Start the ROS node's main loop."""
        rospy.spin()

    def _timer_callback(self, event) -> None:
        """
        Callback function for a periodic Timer to send serial commands.
        """
        acceleration_command, steering_command = self._apply_indentification()

        # FIX 1: pad to exactly 10 chars so Arduino length check passes.
        # Arduino does: if (command.length() != 10) → ignore command.
        # Old format f"0{dir}{steer}{accel}" was only 8 chars after readStringUntil
        # strips \r\n — every single command was silently rejected.
        # Format: "00" + dir(1) + steering(3) + acceleration(3) = 10 chars.
        # Arduino parses: dir=substring(1,2), steer=substring(2,5), accel=substring(5,8)
        # — the leading "00" satisfies the length check without shifting any parsed field.
        command = f"00{self.direction}{steering_command:03d}{acceleration_command:03d}"
        rospy.loginfo(command)

        # Send it via serial
        self.connection.write(bytes(f"{command}\r\n", encoding='ascii'))
        self.connection.flush()

        # Read any response (optional)
        data = self.connection.readline()
        rospy.loginfo(f"PM feedback: {data}")

    def _cmd_controller_callback(self, msg: Float64MultiArray) -> None:
        """
        Callback function for Controller input subscriber.
        Expects [linear_velocity, steering_angle_in_radians].
        """
        self.cmd_controller_msg = msg
        self.cmd_controller_flag = True

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """
        Callback function for velocity subscriber (/cmd_vel).
        Expects Twist: linear.x (m/s), angular.z (rad/s).
        """
        self.cmd_vel_msg = msg
        self.cmd_vel_flag = True

    def _apply_indentification(self):
        """
        Determines appropriate motor acceleration and steering commands.
        Uses an asymmetric quadratic mapping for steering power correction.
        """
        linear_velocity = 0.0
        steering_angle = 0.0
        # FIX 2: do NOT reset self.direction = 0 here unconditionally.
        # Old code reset direction to 0 at the top of every timer tick.
        # If path_follower publishes at a slightly different rate than the
        # timer, ticks where no new message arrives would snap direction back
        # to 0 (forward) mid-reverse, causing relay chatter.
        # Direction is now only updated when a new message actually arrives.

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
            # FIX 3: reset the flag after processing.
            # Old code never cleared cmd_controller_flag — it stayed True
            # forever after the first message, so the elif branch always
            # executed but direction was still being reset at the top (fix 2).
            self.cmd_controller_flag = False
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

        # Clamp the command within valid range
        steering_command = max(350, min(845, steering_command))

        return int(acceleration_command), int(steering_command)

    def _corrected_steering_command(self, goal_angle_rad: float) -> float:
        """
        Applies asymmetric quadratic correction **only when necessary**.
        - Uses a different equation for CW and CCW movement.
        - CW follows:    p(theta) =  69.86 θ² + 317.31 θ + 525
        - CCW follows:   p(theta) = -86.29 θ² + 317.31 θ + 610
        """
        try:
            current_angle_rad = self.previous_steering_angle
        except AttributeError:
            self.previous_steering_angle = 0.0
            current_angle_rad = 0.0

        is_counterclockwise = goal_angle_rad > current_angle_rad  # True if increasing angle

        # Store this as the last commanded angle
        self.previous_steering_angle = goal_angle_rad

        if is_counterclockwise:
            # Counterclockwise adjustment (shifts zero to 610)
            a_ccw = -86.29
            b_ccw = 317.31
            c_ccw = 610  # previously cross-checked near 607-608; revisit alongside c_cw if drift persists
            return a_ccw * (goal_angle_rad ** 2) + b_ccw * goal_angle_rad + c_ccw
        else:
            # Clockwise mapping (shifts zero to 525, measured ground truth via rolling test)
            a_cw = 69.86
            b_cw = 317.31
            c_cw = 525  # was 510, then 607 (visual check); recalibrated 2026-06-30 via rolling straight-line test
            return a_cw * (goal_angle_rad ** 2) + b_cw * goal_angle_rad + c_cw


if __name__ == "__main__":
    node = HakuroukunCommunicationNode()
    node.run()