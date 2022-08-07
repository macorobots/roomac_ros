#!/usr/bin/env python

import math

import rospy

import utils
from servo_controller import AnalogServo, DigitalServo, ServoController


class ArmController:
    def __init__(self):
        zero_angle_signal_shoulder_pitch = rospy.get_param(
            "~zero_angle_signal_shoulder_pitch", 860
        )
        zero_angle_signal_sholder_roll = rospy.get_param(
            "~zero_angle_signal_sholder_roll", 280
        )
        zero_angle_signal_elbow = rospy.get_param("~zero_angle_signal_elbow", 512)
        zero_angle_signal_wrist = rospy.get_param("~zero_angle_signal_wrist", 1450)
        zero_angle_signal_gripper_twist = rospy.get_param(
            "~zero_angle_signal_gripper_twist", 1590
        )

        analog_lower_signal_bound = rospy.get_param("~analog_lower_signal_bound", 500)
        analog_upper_signal_bound = rospy.get_param("~analog_upper_signal_bound", 2500)
        analog_angle_diff = rospy.get_param("~analog_angle_diff", 180.0)

        wrist_analog_lower_signal_bound = rospy.get_param(
            "~wrist_analog_lower_signal_bound", 600
        )
        wrist_analog_upper_signal_bound = rospy.get_param(
            "~wrist_analog_upper_signal_bound", 2400
        )
        wrist_analog_angle_diff = rospy.get_param("~wrist_analog_angle_diff", 90.0)

        digital_lower_signal_bound = rospy.get_param("~digital_lower_signal_bound", 0)
        digital_upper_signal_bound = rospy.get_param(
            "~digital_upper_signal_bound", 1024
        )
        digital_angle_diff = rospy.get_param("~digital_upper_signal_bound", 330.0)
        digital_angle_diff_geared = rospy.get_param(
            "~digital_upper_signal_bound", 165.0
        )

        wrist_signal_zero_position = rospy.get_param(
            "~wrist_signal_zero_position", 1509
        )
        wrist_signal_90_degrees = rospy.get_param("~wrist_signal_90_degrees", 697)

        analog_update_delay = rospy.get_param("~analog_update_delay", 0.7)
        max_speed = rospy.get_param("~max_speed", 0.005)

        analog_scale_factor = utils.calculate_scale_factor(
            analog_upper_signal_bound,
            analog_lower_signal_bound,
            math.radians(analog_angle_diff),
        )
        wrist_analog_scale_factor = utils.calculate_scale_factor(
            wrist_signal_zero_position,
            wrist_signal_90_degrees,
            math.radians(wrist_analog_angle_diff),
        )
        digital_scale_factor = utils.calculate_scale_factor(
            digital_upper_signal_bound,
            digital_lower_signal_bound,
            math.radians(digital_angle_diff),
        )
        digital_scale_factor_geared = utils.calculate_scale_factor(
            digital_upper_signal_bound,
            digital_lower_signal_bound,
            math.radians(digital_angle_diff_geared),
        )

        servos = {}

        initial_playtime = 255
        initial_analog_speed = 2.0

        servos["shoulder_pitch_right_joint"] = DigitalServo(
            "shoulder_pan",
            zero_angle_signal_shoulder_pitch,
            digital_lower_signal_bound,
            digital_upper_signal_bound,
            digital_scale_factor_geared,
            max_speed,
        )
        servos["shoulder_roll_right_joint"] = DigitalServo(
            "shoulder_lift",
            zero_angle_signal_sholder_roll,
            digital_lower_signal_bound,
            digital_upper_signal_bound,
            digital_scale_factor_geared,
            max_speed,
        )
        servos["elbow_right_joint"] = DigitalServo(
            "elbow",
            zero_angle_signal_elbow,
            digital_lower_signal_bound,
            digital_upper_signal_bound,
            digital_scale_factor,
            max_speed,
        )

        servos["wrist_right_joint"] = AnalogServo(
            "wrist",
            zero_angle_signal_wrist,
            wrist_analog_lower_signal_bound,
            wrist_analog_upper_signal_bound,
            wrist_analog_scale_factor,
            max_speed,
            analog_update_delay,
        )
        servos["gripper_twist_right_joint"] = AnalogServo(
            "wrist_twist",
            zero_angle_signal_gripper_twist,
            analog_lower_signal_bound,
            analog_upper_signal_bound,
            analog_scale_factor,
            max_speed,
            analog_update_delay,
        )
        for x in [
            "shoulder_pitch_right_joint",
            "shoulder_roll_right_joint",
            "elbow_right_joint",
        ]:
            servos[x].init_servo(0.0, initial_playtime)

        for x in [
            "wrist_right_joint",
            "gripper_twist_right_joint",
        ]:
            servos[x].init_servo(0.0, initial_analog_speed)

        self._servo_controller = ServoController(servos)

    def go_to_point(self, joint_names, angles, duration=0.0):
        return self._servo_controller.go_to_point(joint_names, angles, duration)
