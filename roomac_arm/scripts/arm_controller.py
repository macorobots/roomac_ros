#!/usr/bin/env python

import rospy
import math
from servo_controller import AnalogServo, DigitalServo

import itertools


class ArmController:
    def __init__(self):
        self.zero_angle_signal = rospy.get_param(
            "~zero_angle_signal", [850, 320, 512, 1509, 1500, 650]
        )
        self.analog_lower_signal_bound = rospy.get_param(
            "~analog_lower_signal_bound", 500
        )
        self.analog_upper_signal_bound = rospy.get_param(
            "~analog_upper_signal_bound", 2500
        )
        self.analog_lower_signal_bound_wrist = rospy.get_param(
            "~analog_lower_signal_bound_wrist", 600
        )
        self.analog_upper_signal_bound_wrist = rospy.get_param(
            "~analog_upper_signal_bound_wrist", 2400
        )
        self.digital_lower_signal_bound = rospy.get_param(
            "~digital_lower_signal_bound", 0
        )
        self.digital_upper_signal_bound = rospy.get_param(
            "~digital_upper_signal_bound", 1024
        )
        self.wrist_signal_zero_position = rospy.get_param(
            "~wrist_signal_zero_position", 1509
        )
        self.wrist_signal_90_degrees = rospy.get_param("~wrist_signal_90_degrees", 697)
        self.analog_speed = rospy.get_param("~analog_speed", 2.0)
        self.wrist_speed = rospy.get_param("~wrist_speed", 4.0)
        self.analog_update_delay = rospy.get_param("~analog_update_delay", 0.7)
        self.min_change_threshold = rospy.get_param("~min_change_threshold", 0.01)
        self.max_speed = rospy.get_param("~max_speed", 0.005)

        self.joint_names = [
            "right_shoulder_pan",
            "right_shoulder_lift",
            "right_elbow",
            "right_wrist",
            "right_gripper_twist",
            "right_gripper",
        ]

        self.digital_joint_names = [
            self.joint_names[0],
            self.joint_names[1],
            self.joint_names[2],
        ]
        self.analog_joint_names = [
            self.joint_names[3],
            self.joint_names[4],
            self.joint_names[5],
        ]

        self.analog_scale_factor_wrist = (
            self.wrist_signal_zero_position - self.wrist_signal_90_degrees
        ) / (90.0 * math.pi / 180.0)
        self.digital_scale_factor = (
            self.digital_upper_signal_bound - self.digital_lower_signal_bound
        ) / ((330.0 / 2.0) * math.pi / 180.0)
        self.analog_scale_factor = (
            self.analog_upper_signal_bound - self.analog_lower_signal_bound
        ) / (180.0 * math.pi / 180.0)

        self.servos = {}

        initial_playtime = 255

        self.servos["right_shoulder_pan"] = DigitalServo(
            "shoulder_pan",
            self.zero_angle_signal[0],
            self.digital_lower_signal_bound,
            self.digital_upper_signal_bound,
            self.digital_scale_factor,
            self.max_speed,
            initial_playtime,
        )
        self.servos["right_shoulder_lift"] = DigitalServo(
            "shoulder_lift",
            self.zero_angle_signal[1],
            self.digital_lower_signal_bound,
            self.digital_upper_signal_bound,
            self.digital_scale_factor,
            self.max_speed,
            initial_playtime,
        )
        self.servos["right_elbow"] = DigitalServo(
            "elbow",
            self.zero_angle_signal[2],
            self.digital_lower_signal_bound,
            self.digital_upper_signal_bound,
            self.digital_scale_factor / 2.0,  # no gear reduction
            self.max_speed,
            initial_playtime,
        )

        self.servos["right_wrist"] = AnalogServo(
            "wrist",
            self.zero_angle_signal[3],
            self.analog_lower_signal_bound_wrist,
            self.analog_upper_signal_bound_wrist,
            self.analog_scale_factor_wrist,
            self.max_speed,
            self.wrist_speed,
            self.analog_update_delay,
        )
        self.servos["right_gripper_twist"] = AnalogServo(
            "wrist_twist",
            self.zero_angle_signal[4],
            self.analog_lower_signal_bound,
            self.analog_upper_signal_bound,
            self.analog_scale_factor,
            self.max_speed,
            self.analog_speed,
            self.analog_update_delay,
        )
        self.servos["right_gripper"] = AnalogServo(
            "gripper",
            self.zero_angle_signal[5],
            self.analog_lower_signal_bound,
            self.analog_upper_signal_bound,
            self.analog_scale_factor,
            self.max_speed,
            self.analog_speed,
            self.analog_update_delay,
        )

        for x in self.servos:
            x.set_angle(0.0)

    def get_valid_joints(self, joint_names, angles):
        valid_joint_names = []
        valid_angles = []

        for joint_name, angle in itertools.izip(joint_names, angles):
            if not joint_name in self.servos:
                rospy.logwarn(joint_name + " not valid joint name")
                continue

            if not self.servos[joint_name].is_initialized():
                rospy.logwarn(joint_name + " not initialized")
                continue

            valid_joint_names.append(joint_name)
            valid_angles.append(angle)

        return valid_joint_names, valid_angles

    def angle_change_above_min_threshold(self, joint_names, angles):
        angle_diffs = []

        # Python3
        # for joint_name, angle in zip(joint_names, angles):
        for joint_name, angle in itertools.izip(joint_names, angles):
            angle_diffs.append(self.servos[joint_name].calculate_angle_diff(angle))

        max_angle_diff = max(angle_diffs)

        if max_angle_diff < self.min_change_threshold:
            return False
        else:
            return True

    def calculate_min_movement_duration(self, joint_names, angles):
        min_movement_duration = 0
        for joint_name, angle in itertools.izip(joint_names, angles):
            min_duration = self.servos[joint_name].calculate_min_movement_duration(
                angle
            )
            if min_duration > min_movement_duration:
                min_movement_duration = min_duration

        return min_movement_duration

    def go_to_point(self, joint_names, angles, duration):
        joint_names, angles = self.get_valid_joints(joint_names, angles)

        if not self.angle_change_above_min_threshold(joint_names, angles):
            return

        min_movement_duration = self.calculate_min_movement_duration(
            joint_names, angles
        )
        movement_duration = max(duration, min_movement_duration)

        for joint_name, angle in itertools.izip(joint_names, angles):
            self.servos[joint_name].execute_motion(angle, movement_duration)

        return movement_duration

    def go_to_point(self, joint_names, angles):
        joint_names, angles = self.get_valid_joints(joint_names, angles)

        if not self.angle_change_above_min_threshold(joint_names, angles):
            return

        min_movement_duration = self.calculate_min_movement_duration(
            joint_names, angles
        )
        movement_duration = min_movement_duration

        for joint_name, angle in itertools.izip(joint_names, angles):
            self.servos[joint_name].execute_motion(angle, movement_duration)

        return movement_duration
