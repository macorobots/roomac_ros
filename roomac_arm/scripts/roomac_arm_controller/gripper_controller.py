#!/usr/bin/env python

import rospy

import utils
from servo_controller import ServoController


class GripperController:
    def __init__(self):
        self._a = rospy.get_param("~gripper_angle_to_distance_a", -0.011875)
        self._b = rospy.get_param("~gripper_angle_to_distance_b", 0.007375)

        self._analog_update_delay = rospy.get_param("~analog_update_delay", 0.7)
        self._max_speed = rospy.get_param("~max_speed", 0.005)
        self._initial_angle = rospy.get_param("~initial_angle", 0.0)
        self._initial_analog_speed = rospy.get_param("~initial_analog_speed", 2.0)
        self._initial_digital_playtime = rospy.get_param(
            "~initial_digital_playtime", 255
        )

        common_params = {
            "analog_update_delay": self._analog_update_delay,
            "max_speed": self._max_speed,
            "initial_angle": self._initial_angle,
            "initial_digital_playtime": self._initial_digital_playtime,
            "initial_analog_speed": self._initial_analog_speed,
        }

        self._servos = {}
        gripper_servo_joint = rospy.get_param("~gripper_controller/gripper_servo_joint")

        parameter_ns = "~gripper_controller/" + gripper_servo_joint + "/"
        params = utils.parse_servo(parameter_ns)
        params.update(common_params)
        self._servos[gripper_servo_joint] = utils.create_servo(**params)

        self._servo_controller = ServoController(self._servos)

    def go_to_point(self, joint_names, dists, duration=0.0):
        angles = []
        for dist in dists:
            angles.append(utils.linear_transform_dist_to_angle(self._a, self._b, dist))

        return self._servo_controller.go_to_point(joint_names, angles, duration)
