#!/usr/bin/env python

import rospy
import math
from servo_controller import AnalogServo, DigitalServo


class ArmController:
    def __init__(self):
        self.zero_positions = rospy.get_param(
            "~zero_positions", [850, 320, 512, 1509, 1500, 650]
        )
        self.max_movement_time = rospy.get_param("~max_movement_time", 1000)
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
        self.analog_speed = rospy.get_param("~analog_speed", 10)
        self.wrist_speed = rospy.get_param("~wrist_speed", 4.0)
        self.analog_update_delay = rospy.get_param("~analog_update_delay", 0.7)
        self.change_threshold = rospy.get_param("~change_threshold", 0.01)
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

        self.servos["right_shoulder_pan"] = DigitalServo(
            "shoulder_pan",
            self.zero_positions[0],
            self.digital_lower_signal_bound,
            self.digital_upper_signal_bound,
            self.digital_scale_factor,
            self.max_speed,
        )
        self.servos["right_shoulder_lift"] = DigitalServo(
            "shoulder_lift",
            self.zero_positions[1],
            self.digital_lower_signal_bound,
            self.digital_upper_signal_bound,
            self.digital_scale_factor,
            self.max_speed,
        )
        self.servos["right_elbow"] = DigitalServo(
            "elbow",
            self.zero_positions[2],
            self.digital_lower_signal_bound,
            self.digital_upper_signal_bound,
            self.digital_scale_factor / 2.0,  # no gear reduction
            self.max_speed,
        )

        self.servos["right_wrist"] = AnalogServo(
            "wrist",
            self.zero_positions[3],
            self.analog_lower_signal_bound_wrist,
            self.analog_upper_signal_bound_wrist,
            self.analog_scale_factor_wrist,
            self.wrist_speed,
            self.analog_update_delay,
        )
        self.servos["right_gripper_twist"] = AnalogServo(
            "wrist_twist",
            self.zero_positions[4],
            self.analog_lower_signal_bound,
            self.analog_upper_signal_bound,
            self.analog_scale_factor,
            self.analog_speed,
            self.analog_update_delay,
        )
        self.servos["right_gripper"] = AnalogServo(
            "gripper",
            self.zero_positions[5],
            self.analog_lower_signal_bound,
            self.analog_upper_signal_bound,
            self.analog_scale_factor,
            self.analog_speed,
            self.analog_update_delay,
        )

        for x in self.servos:
            x.set_angle(0.0)

        self.servos["right_wrist"].publish_speed(self.wrist_speed)

    def go_to_point(self, joint_names, positions):
        max_angle_diff = 0
        movement_times = []

        for id in range(len(joint_names)):
            joint_name = joint_names[id]

            if not joint_name in self.servos:
                continue

            if not self.servos[joint_name].is_initialized():
                rospy.logerr(joint_name + " not initialized")
                continue

            position_diff = self.servos[joint_name].calculate_position_diff(
                positions[id]
            )

            if position_diff > max_angle_diff:
                max_angle_diff = position_diff

            movement_times.append(
                self.servos[joint_name].calculate_movement_time(positions[id])
            )

        if max_angle_diff < self.change_threshold:
            return

        movement_time = self.calculate_movement_time(movement_times)

        for joint_name in self.digital_joint_names:
            self.servos[joint_name].publish_playtime(movement_time)

        for id in range(len(joint_names)):

            joint_name = joint_names[id]

            if not joint_name in self.servos:
                continue

            self.servos[joint_name].set_angle(positions[id])

        return movement_time

    def calculate_movement_time(self, movement_times):

        movement_time = max(movement_times)

        if movement_time > self.max_movement_time:
            movement_time = self.max_movement_time

        return movement_time
