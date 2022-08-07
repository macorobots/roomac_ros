#!/usr/bin/env python

import math

import rospy

import utils
from servo_controller import AnalogServo, ServoController


class GripperController:
    def __init__(self):
        zero_angle_signal_gripper_finger = rospy.get_param(
            "~zero_angle_signal_gripper_finger", 650
        )

        analog_lower_signal_bound = rospy.get_param("~analog_lower_signal_bound", 500)
        analog_upper_signal_bound = rospy.get_param("~analog_upper_signal_bound", 2500)

        analog_update_delay = rospy.get_param("~analog_update_delay", 0.7)
        max_speed = rospy.get_param("~max_speed", 0.005)

        analog_scale_factor = utils.calculate_scale_factor(
            analog_upper_signal_bound,
            analog_lower_signal_bound,
            math.radians(180.0),
        )

        servos = {}

        initial_analog_speed = 2.0

        servos["gripper_finger_l_right_joint"] = AnalogServo(
            "gripper",
            zero_angle_signal_gripper_finger,
            analog_lower_signal_bound,
            analog_upper_signal_bound,
            analog_scale_factor,
            max_speed,
            analog_update_delay,
        )

        servos["gripper_finger_l_right_joint"].init_servo(0.0, initial_analog_speed)
        self._servo_controller = ServoController(servos)

        # Linear approximation using these two points
        # 0.2 -> 0.005m
        # 1.0 -> -0.0045m
        self.a = -0.011875
        self.b = 0.007375

    def go_to_point(self, joint_names, dists, duration=0.0):
        angles = []
        for dist in dists:
            angles.append(utils.linear_transform_dist_to_angle(self.a, self.b, dist))

        return self._servo_controller.go_to_point(joint_names, angles, duration)
