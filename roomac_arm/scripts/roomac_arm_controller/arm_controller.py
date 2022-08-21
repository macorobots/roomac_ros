#!/usr/bin/env python

import rospy

import utils
from servo_controller import ServoController


class ArmController:
    def __init__(self):
        self._analog_update_delay = rospy.get_param("~analog_update_delay", 0.7)
        self._max_speed = rospy.get_param("~max_speed", 0.005)
        self._initial_angle = rospy.get_param("~initial_angle", 0.0)
        self._initial_digital_playtime = rospy.get_param(
            "~initial_digital_playtime", 255
        )
        self._initial_analog_speed = rospy.get_param("~initial_analog_speed", 2.0)

        common_params = {
            "analog_update_delay": self._analog_update_delay,
            "max_speed": self._max_speed,
            "initial_angle": self._initial_angle,
            "initial_digital_playtime": self._initial_digital_playtime,
            "initial_analog_speed": self._initial_analog_speed,
        }

        self._servos = {}

        arm_servo_joints = rospy.get_param("~arm_controller/arm_servo_joints")

        for servo_joint in arm_servo_joints:
            parameter_ns = "~arm_controller/" + servo_joint + "/"
            params = utils.parse_servo(parameter_ns)
            params.update(common_params)
            self._servos[servo_joint] = utils.create_servo(**params)

        self._servo_controller = ServoController(self._servos)

    def go_to_point(self, joint_names, angles, duration=0.0):
        return self._servo_controller.go_to_point(joint_names, angles, duration)
