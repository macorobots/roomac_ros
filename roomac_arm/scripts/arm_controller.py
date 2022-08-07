#!/usr/bin/env python

import math

import rospy

import utils
from servo import AnalogServo, DigitalServo
from servo_controller import ServoController

class ArmController:
    def __init__(self):
        self._load_parameters()
        self._calculate_scale_factors()

        self._servos = {}
        self._add_servos()

        initial_digital_position = 0.0
        initial_digital_playtime = 255
        initial_analog_position = 0.0
        initial_analog_speed = 2.0
        self._init_servos(
            initial_digital_position,
            initial_digital_playtime,
            initial_analog_position,
            initial_analog_speed,
        )

        self._servo_controller = ServoController(self._servos)

    def go_to_point(self, joint_names, angles, duration=0.0):
        return self._servo_controller.go_to_point(joint_names, angles, duration)

    def _init_servos(
        self,
        initial_digital_position,
        initial_digital_playtime,
        initial_analog_position,
        initial_analog_speed,
    ):
        for x in [
            "shoulder_pitch_right_joint",
            "shoulder_roll_right_joint",
            "elbow_right_joint",
        ]:
            self._servos[x].init_servo(
                initial_digital_position, initial_digital_playtime
            )

        for x in [
            "wrist_right_joint",
            "gripper_twist_right_joint",
        ]:
            self._servos[x].init_servo(initial_analog_position, initial_analog_speed)

    def _add_servos(self):
        self._servos["shoulder_pitch_right_joint"] = DigitalServo(
            "shoulder_pan",
            self._zero_angle_signal_shoulder_pitch,
            self._digital_lower_signal_bound,
            self._digital_upper_signal_bound,
            self._digital_scale_factor_geared,
            self._max_speed,
        )
        self._servos["shoulder_roll_right_joint"] = DigitalServo(
            "shoulder_lift",
            self._zero_angle_signal_sholder_roll,
            self._digital_lower_signal_bound,
            self._digital_upper_signal_bound,
            self._digital_scale_factor_geared,
            self._max_speed,
        )
        self._servos["elbow_right_joint"] = DigitalServo(
            "elbow",
            self._zero_angle_signal_elbow,
            self._digital_lower_signal_bound,
            self._digital_upper_signal_bound,
            self._digital_scale_factor,
            self._max_speed,
        )

        self._servos["wrist_right_joint"] = AnalogServo(
            "wrist",
            self._zero_angle_signal_wrist,
            self._wrist_analog_lower_signal_bound,
            self._wrist_analog_upper_signal_bound,
            self._wrist_analog_scale_factor,
            self._max_speed,
            self._analog_update_delay,
        )
        self._servos["gripper_twist_right_joint"] = AnalogServo(
            "wrist_twist",
            self._zero_angle_signal_gripper_twist,
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
        self._wrist_analog_scale_factor = utils.calculate_scale_factor(
            self._wrist_signal_zero_position,
            self._wrist_signal_90_degrees,
            math.radians(self._wrist_analog_angle_diff),
        )
        self._digital_scale_factor = utils.calculate_scale_factor(
            self._digital_upper_signal_bound,
            self._digital_lower_signal_bound,
            math.radians(self._digital_angle_diff),
        )
        self._digital_scale_factor_geared = utils.calculate_scale_factor(
            self._digital_upper_signal_bound,
            self._digital_lower_signal_bound,
            math.radians(self._digital_angle_diff_geared),
        )

    def _load_parameters(self):
        self._zero_angle_signal_shoulder_pitch = rospy.get_param(
            "~zero_angle_signal_shoulder_pitch", 860
        )
        self._zero_angle_signal_sholder_roll = rospy.get_param(
            "~zero_angle_signal_sholder_roll", 280
        )
        self._zero_angle_signal_elbow = rospy.get_param("~zero_angle_signal_elbow", 512)
        self._zero_angle_signal_wrist = rospy.get_param(
            "~zero_angle_signal_wrist", 1450
        )
        self._zero_angle_signal_gripper_twist = rospy.get_param(
            "~zero_angle_signal_gripper_twist", 1590
        )

        self._analog_lower_signal_bound = rospy.get_param(
            "~analog_lower_signal_bound", 500
        )
        self._analog_upper_signal_bound = rospy.get_param(
            "~analog_upper_signal_bound", 2500
        )
        self._analog_angle_diff = rospy.get_param("~analog_angle_diff", 180.0)

        self._wrist_analog_lower_signal_bound = rospy.get_param(
            "~wrist_analog_lower_signal_bound", 600
        )
        self._wrist_analog_upper_signal_bound = rospy.get_param(
            "~wrist_analog_upper_signal_bound", 2400
        )
        self._wrist_analog_angle_diff = rospy.get_param(
            "~wrist_analog_angle_diff", 90.0
        )

        self._digital_lower_signal_bound = rospy.get_param(
            "~digital_lower_signal_bound", 0
        )
        self._digital_upper_signal_bound = rospy.get_param(
            "~digital_upper_signal_bound", 1024
        )
        self._digital_angle_diff = rospy.get_param("~digital_upper_signal_bound", 330.0)
        self._digital_angle_diff_geared = rospy.get_param(
            "~digital_upper_signal_bound", 165.0
        )

        self._wrist_signal_zero_position = rospy.get_param(
            "~wrist_signal_zero_position", 1509
        )
        self._wrist_signal_90_degrees = rospy.get_param("~wrist_signal_90_degrees", 697)

        self._analog_update_delay = rospy.get_param("~analog_update_delay", 0.7)
        self._max_speed = rospy.get_param("~max_speed", 0.005)
