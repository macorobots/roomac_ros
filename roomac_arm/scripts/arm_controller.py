#!/usr/bin/env python

import rospy
import math
from servo_controller import AnalogServo, DigitalServo

import itertools


class ArmController:
    def __init__(self):
        zero_angle_signal = rospy.get_param(
            "~zero_angle_signal", [850, 320, 512, 1509, 1500, 650]
        )
        analog_lower_signal_bound = rospy.get_param("~analog_lower_signal_bound", 500)
        analog_upper_signal_bound = rospy.get_param("~analog_upper_signal_bound", 2500)
        analog_lower_signal_bound_wrist = rospy.get_param(
            "~analog_lower_signal_bound_wrist", 600
        )
        analog_upper_signal_bound_wrist = rospy.get_param(
            "~analog_upper_signal_bound_wrist", 2400
        )
        digital_lower_signal_bound = rospy.get_param("~digital_lower_signal_bound", 0)
        digital_upper_signal_bound = rospy.get_param(
            "~digital_upper_signal_bound", 1024
        )
        wrist_signal_zero_position = rospy.get_param(
            "~wrist_signal_zero_position", 1509
        )
        wrist_signal_90_degrees = rospy.get_param("~wrist_signal_90_degrees", 697)
        analog_speed = rospy.get_param("~analog_speed", 2.0)
        wrist_speed = rospy.get_param("~wrist_speed", 4.0)
        analog_update_delay = rospy.get_param("~analog_update_delay", 0.7)
        self._min_change_threshold = rospy.get_param("~min_change_threshold", 0.01)
        max_speed = rospy.get_param("~max_speed", 0.005)

        analog_scale_factor_wrist = (
            wrist_signal_zero_position - wrist_signal_90_degrees
        ) / (90.0 * math.pi / 180.0)
        digital_scale_factor = (
            digital_upper_signal_bound - digital_lower_signal_bound
        ) / ((330.0 / 2.0) * math.pi / 180.0)
        analog_scale_factor = (
            analog_upper_signal_bound - analog_lower_signal_bound
        ) / (180.0 * math.pi / 180.0)

        self._servos = {}

        initial_playtime = 255

        self._servos["right_shoulder_pan"] = DigitalServo(
            "shoulder_pan",
            zero_angle_signal[0],
            digital_lower_signal_bound,
            digital_upper_signal_bound,
            digital_scale_factor,
            max_speed,
            initial_playtime,
        )
        self._servos["right_shoulder_lift"] = DigitalServo(
            "shoulder_lift",
            zero_angle_signal[1],
            digital_lower_signal_bound,
            digital_upper_signal_bound,
            digital_scale_factor,
            max_speed,
            initial_playtime,
        )
        self._servos["right_elbow"] = DigitalServo(
            "elbow",
            zero_angle_signal[2],
            digital_lower_signal_bound,
            digital_upper_signal_bound,
            digital_scale_factor / 2.0,  # no gear reduction
            max_speed,
            initial_playtime,
        )

        self._servos["right_wrist"] = AnalogServo(
            "wrist",
            zero_angle_signal[3],
            analog_lower_signal_bound_wrist,
            analog_upper_signal_bound_wrist,
            analog_scale_factor_wrist,
            max_speed,
            wrist_speed,
            analog_update_delay,
        )
        self._servos["right_gripper_twist"] = AnalogServo(
            "wrist_twist",
            zero_angle_signal[4],
            analog_lower_signal_bound,
            analog_upper_signal_bound,
            analog_scale_factor,
            max_speed,
            analog_speed,
            analog_update_delay,
        )
        self._servos["right_gripper"] = AnalogServo(
            "gripper",
            zero_angle_signal[5],
            analog_lower_signal_bound,
            analog_upper_signal_bound,
            analog_scale_factor,
            max_speed,
            analog_speed,
            analog_update_delay,
        )

        for x in self._servos:
            x.set_angle(0.0)

    def go_to_point(self, joint_names, angles, duration):
        joint_names, angles = self._get_valid_joints(joint_names, angles)

        if not self._angle_change_above_min_threshold(joint_names, angles):
            return

        min_movement_duration = self._calculate_min_movement_duration(
            joint_names, angles
        )
        movement_duration = max(duration, min_movement_duration)

        for joint_name, angle in itertools.izip(joint_names, angles):
            self._servos[joint_name].execute_motion(angle, movement_duration)

        return movement_duration

    def go_to_point(self, joint_names, angles):
        joint_names, angles = self._get_valid_joints(joint_names, angles)

        if not self._angle_change_above_min_threshold(joint_names, angles):
            return

        min_movement_duration = self._calculate_min_movement_duration(
            joint_names, angles
        )
        movement_duration = min_movement_duration

        for joint_name, angle in itertools.izip(joint_names, angles):
            self._servos[joint_name].execute_motion(angle, movement_duration)

        return movement_duration

    def _get_valid_joints(self, joint_names, angles):
        valid_joint_names = []
        valid_angles = []

        for joint_name, angle in itertools.izip(joint_names, angles):
            if not joint_name in self._servos:
                rospy.logwarn(joint_name + " not valid joint name")
                continue

            if not self._servos[joint_name].is_initialized():
                rospy.logwarn(joint_name + " not initialized")
                continue

            valid_joint_names.append(joint_name)
            valid_angles.append(angle)

        return valid_joint_names, valid_angles

    def _angle_change_above_min_threshold(self, joint_names, angles):
        angle_diffs = []

        # Python3
        # for joint_name, angle in zip(joint_names, angles):
        for joint_name, angle in itertools.izip(joint_names, angles):
            angle_diffs.append(self._servos[joint_name].calculate_angle_diff(angle))

        max_angle_diff = max(angle_diffs)

        if max_angle_diff < self.self._min_change_threshold:
            return False
        else:
            return True

    def _calculate_min_movement_duration(self, joint_names, angles):
        min_movement_duration = 0
        for joint_name, angle in itertools.izip(joint_names, angles):
            min_duration = self._servos[joint_name].calculate_min_movement_duration(
                angle
            )
            if min_duration > min_movement_duration:
                min_movement_duration = min_duration

        return min_movement_duration
