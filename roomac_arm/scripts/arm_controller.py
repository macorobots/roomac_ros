#!/usr/bin/env python

import math
import itertools
import numpy as np

import rospy
from sensor_msgs.msg import JointState

from dynamic_reconfigure.server import Server
from roomac_arm.cfg import ArmControllerConfig

from servo_controller import AnalogServo, DigitalServo


class ArmController(object):
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

        self._interpolate_movement = rospy.get_param("~interpolate_movement", True)
        self._interpolation_frequency = rospy.get_param(
            "~interpolation_frequency", 10.0
        )
        self._interpolation_duration_type = rospy.get_param(
            "~interpolation_duration_type", "exact"
        )

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
            zero_angle_signal[0],
            digital_lower_signal_bound,
            digital_upper_signal_bound,
            digital_scale_factor,
            max_speed,
            initial_playtime,
        )
        self._servos["right_shoulder_lift"] = DigitalServo(
            zero_angle_signal[1],
            digital_lower_signal_bound,
            digital_upper_signal_bound,
            digital_scale_factor,
            max_speed,
            initial_playtime,
        )
        self._servos["right_elbow"] = DigitalServo(
            zero_angle_signal[2],
            digital_lower_signal_bound,
            digital_upper_signal_bound,
            digital_scale_factor / 2.0,  # no gear reduction
            max_speed,
            initial_playtime,
        )

        self._servos["right_wrist"] = AnalogServo(
            zero_angle_signal[3],
            analog_lower_signal_bound_wrist,
            analog_upper_signal_bound_wrist,
            analog_scale_factor_wrist,
            max_speed,
            wrist_speed,
            analog_update_delay,
        )
        self._servos["right_gripper_twist"] = AnalogServo(
            zero_angle_signal[4],
            analog_lower_signal_bound,
            analog_upper_signal_bound,
            analog_scale_factor,
            max_speed,
            analog_speed,
            analog_update_delay,
        )
        self._servos["right_gripper"] = AnalogServo(
            zero_angle_signal[5],
            analog_lower_signal_bound,
            analog_upper_signal_bound,
            analog_scale_factor,
            max_speed,
            analog_speed,
            analog_update_delay,
        )

        self._publish_joint_states = rospy.get_param("~publish_joint_states", True)
        self._joint_state_pub = rospy.Publisher(
            "joint_states_from_controller", JointState, queue_size=10, latch=True
        )

        self._servo_position_cmd_pub = rospy.Publisher(
            "joint_state_cmd", JointState, queue_size=10
        )

        self._dynamic_reconfigure_srv = Server(
            ArmControllerConfig, self._dynamic_reconfigure_cb
        )

        self._init_servos()

    def _init_servos(self):
        joint_names = []
        position_signals = []
        velocity_signals = []
        for x in self._servos:
            # initial speeds set when creating objects
            self._servos[x].set_angle(0.0)
            joint_names.append(x)
            position_signals.append(self._servos[x].get_position_signal())
            velocity_signals.append(self._servos[x].get_velocity_signal())

        self._send_joint_state_cmd(joint_names, position_signals, velocity_signals)

    def go_to_point(self, joint_names, angles, duration=0.0):
        joint_names, angles = self._get_valid_joints(joint_names, angles)

        if not self._angle_change_above_min_threshold(joint_names, angles):
            return

        min_movement_duration = self._calculate_min_movement_duration(
            joint_names, angles
        )
        movement_duration = max(duration, min_movement_duration)

        if self._interpolate_movement:
            # in interpolation movement_duration is rounded to be multiple of interpolaction frequency
            movement_duration = self._execute_motion_with_interpolation(
                joint_names, angles, movement_duration
            )
        else:
            self._execute_motion(joint_names, angles, movement_duration)

        return movement_duration

    def _execute_motion_with_interpolation(
        self, joint_names, angles, movement_duration
    ):
        num_of_steps = int(math.ceil(movement_duration * self._interpolation_frequency))

        if self._interpolation_duration_type == "exact":
            time_step = movement_duration / num_of_steps
            interpolated_movement_duration = movement_duration
        elif self._interpolation_duration_type == "approximated":
            time_step = 1.0 / self._interpolation_frequency
            interpolated_movement_duration = num_of_steps * (
                1.0 / self._interpolation_frequency
            )

        angle_diffs = []
        for joint_name, angle in itertools.izip(joint_names, angles):
            angle_diffs.append(self._servos[joint_name].calculate_angle_diff(angle))

        angle_steps = [x / num_of_steps for x in angle_diffs]

        current_angles = [
            self._servos[joint_name].get_current_angle() for joint_name in joint_names
        ]
        for i in range(num_of_steps):
            current_angles = np.add(current_angles, angle_steps)
            self._execute_motion(joint_names, current_angles, time_step)

        return interpolated_movement_duration

    def _execute_motion(self, joint_names, angles, movement_duration):
        position_signals = []
        velocity_signals = []
        for joint_name, angle in itertools.izip(joint_names, angles):
            self._servos[joint_name].set_angle_and_duration(angle, movement_duration)
            position_signals.append(self._servos[joint_name].get_position_signal())
            velocity_signals.append(self._servos[joint_name].get_velocity_signal())

        self._send_joint_state_cmd(joint_names, position_signals, velocity_signals)

        if self._publish_joint_states:
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = joint_names
            joint_state_msg.position = angles
            self._joint_state_pub.publish(joint_state_msg)

        rospy.sleep(rospy.Duration(movement_duration))

    def _send_joint_state_cmd(self, joint_names, position_signals, velocity_signals):
        joint_state_cmd_msg = JointState()
        joint_state_cmd_msg.header.stamp = rospy.Time.now()

        for joint_name, position, velocity in itertools.izip(
            joint_names, position_signals, velocity_signals
        ):
            joint_state_cmd_msg.name = [joint_name]
            joint_state_cmd_msg.position = [position]
            joint_state_cmd_msg.velocity = [velocity]
            self._servo_position_cmd_pub.publish(joint_state_cmd_msg)

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
            angle_diffs.append(
                abs(self._servos[joint_name].calculate_angle_diff(angle))
            )

        max_angle_diff = max(angle_diffs)

        if max_angle_diff < self._min_change_threshold:
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

    def _dynamic_reconfigure_cb(self, config, level):

        self._min_change_threshold = config.min_change_threshold

        for x in self._servos:
            self._servos[x].set_max_speed(config.max_speed)

        self._interpolate_movement = config.interpolate_movement
        self._interpolation_frequency = config.interpolation_frequency

        return config
