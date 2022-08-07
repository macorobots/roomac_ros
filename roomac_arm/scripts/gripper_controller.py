#!/usr/bin/env python

import math

import rospy

import utils
from servo_controller import AnalogServo, ServoController


class GripperController:
    def __init__(self):
        self._servo_name = "gripper_finger_l_right_joint"

        self._load_parameters()
        self._calculate_scale_factors()

        self._servos = {}
        self._add_servos()

        initial_analog_position = 0.0
        initial_analog_speed = 2.0
        self._init_servos(
            initial_analog_position,
            initial_analog_speed,
        )

        self._servo_controller = ServoController(self._servos)

    def go_to_point(self, joint_names, dists, duration=0.0):
        angles = []
        for dist in dists:
            angles.append(utils.linear_transform_dist_to_angle(self._a, self._b, dist))

        return self._servo_controller.go_to_point(joint_names, angles, duration)

    def _init_servos(
        self,
        initial_analog_position,
        initial_analog_speed,
    ):
        self._servos[self._servo_name].init_servo(
            initial_analog_position, initial_analog_speed
        )

    def _add_servos(self):
        self._servos[self._servo_name] = AnalogServo(
            "gripper",
            self._zero_angle_signal_gripper_finger,
            self._analog_lower_signal_bound,
            self._analog_upper_signal_bound,
            self._analog_scale_factor,
            self._max_speed,
            self._analog_update_delay,
        )

    def _calculate_scale_factors(self):
        self._analog_scale_factor = utils.calculate_scale_factor(
            self._analog_upper_signal_bound,
            self._analog_lower_signal_bound,
            math.radians(self._analog_angle_diff),
        )

    def _load_parameters(self):
        self._zero_angle_signal_gripper_finger = rospy.get_param(
            "~zero_angle_signal_gripper_finger", 650
        )

        self._analog_lower_signal_bound = rospy.get_param(
            "~analog_lower_signal_bound", 500
        )
        self._analog_upper_signal_bound = rospy.get_param(
            "~analog_upper_signal_bound", 2500
        )
        self._analog_angle_diff = rospy.get_param("~analog_angle_diff", 180.0)

        self._analog_update_delay = rospy.get_param("~analog_update_delay", 0.7)
        self._max_speed = rospy.get_param("~max_speed", 0.005)

        self._a = rospy.get_param("~gripper_angle_to_distance_a", -0.011875)
        self._b = rospy.get_param("~gripper_angle_to_distance_b", 0.007375)
