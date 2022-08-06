#!/usr/bin/env python

import math

import rospy

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
        zero_angle_signal_gripper_finger = rospy.get_param(
            "~zero_angle_signal_gripper_finger", 650
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

        analog_update_delay = rospy.get_param("~analog_update_delay", 0.7)
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

        servos = {}

        initial_playtime = 255
        initial_analog_speed = 2.0

        servos["shoulder_pitch_right_joint"] = DigitalServo(
            "shoulder_pan",
            zero_angle_signal_shoulder_pitch,
            digital_lower_signal_bound,
            digital_upper_signal_bound,
            digital_scale_factor,
            max_speed,
        )
        servos["shoulder_roll_right_joint"] = DigitalServo(
            "shoulder_lift",
            zero_angle_signal_sholder_roll,
            digital_lower_signal_bound,
            digital_upper_signal_bound,
            digital_scale_factor,
            max_speed,
        )
        servos["elbow_right_joint"] = DigitalServo(
            "elbow",
            zero_angle_signal_elbow,
            digital_lower_signal_bound,
            digital_upper_signal_bound,
            digital_scale_factor / 2.0,  # no gear reduction
            max_speed,
        )

        servos["wrist_right_joint"] = AnalogServo(
            "wrist",
            zero_angle_signal_wrist,
            analog_lower_signal_bound_wrist,
            analog_upper_signal_bound_wrist,
            analog_scale_factor_wrist,
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
        servos["right_gripper_joint"] = AnalogServo(
            "gripper",
            zero_angle_signal_gripper_finger,
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
            "right_gripper_joint",
        ]:
            servos[x].init_servo(0.0, initial_analog_speed)

        self._servo_controller = ServoController(servos)

    def go_to_point(self, joint_names, angles, duration=0.0):
        return self._servo_controller.go_to_point(joint_names, angles, duration)
