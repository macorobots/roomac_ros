#!/usr/bin/env python

import math
import itertools
import numpy as np

import rospy

from dynamic_reconfigure.server import Server
from sensor_msgs.msg import JointState

from roomac_msgs.msg import DigitalServoCmd, AnalogServoCmd
from roomac_arm.cfg import ArmControllerConfig


class ServoController:
    def __init__(self, servos):

        self._servos = servos

        self._min_change_threshold = rospy.get_param("~min_change_threshold", 0.01)

        self._interpolate_movement = rospy.get_param("~interpolate_movement", True)
        self._interpolation_frequency = rospy.get_param(
            "~interpolation_frequency", 10.0
        )
        self._interpolation_duration_type = rospy.get_param(
            "~interpolation_duration_type", "exact"
        )

        self._publish_joint_states = rospy.get_param("~publish_joint_states", True)
        self._joint_state_pub = rospy.Publisher(
            "joint_states_from_controller", JointState, queue_size=10
        )

        self._dynamic_reconfigure_srv = Server(
            ArmControllerConfig, self._dynamic_reconfigure_cb
        )

    def go_to_point(self, joint_names, angles, duration=0.0):
        joint_names, angles = self._get_valid_joints(joint_names, angles)

        if not self._angle_change_above_min_threshold(joint_names, angles):
            rospy.logwarn("Change below threshold")
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
        for joint_name, angle in itertools.izip(joint_names, angles):
            self._servos[joint_name].execute_motion(angle, movement_duration)

        if self._publish_joint_states:
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = joint_names
            joint_state_msg.position = angles
            self._joint_state_pub.publish(joint_state_msg)

        rospy.sleep(rospy.Duration(movement_duration))

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


class Servo(object):
    def __init__(
        self,
        name,
        zero_angle_signal,
        lower_signal_bound,
        upper_signal_bound,
        angle_to_signal_scale_factor,
        max_speed,
    ):
        self._name = name

        self._zero_angle_signal = zero_angle_signal
        self._current_angle = None

        self._lower_signal_bound = lower_signal_bound
        self._upper_signal_bound = upper_signal_bound

        self._angle_to_signal_scale_factor = angle_to_signal_scale_factor

        self._max_speed = max_speed

    def execute_motion(self, new_angle, movement_duration):
        raise NotImplementedError()

    def is_initialized(self):
        return not (self._current_angle is None)

    def calculate_angle_diff(self, new_angle):
        return new_angle - self._current_angle

    def calculate_min_movement_duration(self, new_angle):
        return abs(self.calculate_angle_diff(new_angle)) / self._max_speed

    def get_current_angle(self):
        return self._current_angle

    def set_max_speed(self, max_speed):
        self._max_speed = max_speed

    def _transform_angle_to_signal(self, angle):
        return round(
            angle * self._angle_to_signal_scale_factor + self._zero_angle_signal
        )

    def _bound_signal(self, signal):
        return max(self._lower_signal_bound, min(self._upper_signal_bound, signal))


class AnalogServo(Servo):
    def __init__(
        self,
        name,
        zero_angle_signal,
        lower_signal_bound,
        upper_signal_bound,
        angle_to_signal_scale_factor,
        max_speed,
        analog_update_delay,
    ):
        super(AnalogServo, self).__init__(
            name,
            zero_angle_signal,
            lower_signal_bound,
            upper_signal_bound,
            angle_to_signal_scale_factor,
            max_speed,
        )
        self._cmd_pub = rospy.Publisher(
            self._name + "_cmd", AnalogServoCmd, queue_size=10
        )

        self._analog_update_delay = analog_update_delay

    def execute_motion(self, new_angle, movement_duration):
        analog_speed = self._calculate_analog_speed(new_angle, movement_duration)
        signal = self._bound_signal(self._transform_angle_to_signal(new_angle))
        self._current_angle = new_angle

        self._publish_cmd(signal, analog_speed)

    def _publish_cmd(self, signal, analog_speed):
        cmd = AnalogServoCmd()
        cmd.signal = signal
        cmd.signal_change_step = analog_speed
        self._cmd_pub.publish(cmd)

    def _calculate_analog_speed(self, new_angle, movement_duration):
        return (
            abs(self.calculate_angle_diff(new_angle))
            * self._angle_to_signal_scale_factor
        ) * (self._analog_update_delay / movement_duration)

    def init_servo(self, angle, analog_speed):
        signal = self._bound_signal(self._transform_angle_to_signal(angle))
        self._current_angle = angle
        self._publish_cmd(signal, analog_speed)


class DigitalServo(Servo):
    def __init__(
        self,
        name,
        zero_angle_signal,
        lower_signal_bound,
        upper_signal_bound,
        angle_to_signal_scale_factor,
        max_speed,
    ):
        super(DigitalServo, self).__init__(
            name,
            zero_angle_signal,
            lower_signal_bound,
            upper_signal_bound,
            angle_to_signal_scale_factor,
            max_speed,
        )

        self._cmd_pub = rospy.Publisher(
            self._name + "_cmd", DigitalServoCmd, queue_size=10
        )

    def execute_motion(self, new_angle, movement_duration):
        playtime = self._calculate_playtime(movement_duration)
        signal = self._bound_signal(self._transform_angle_to_signal(new_angle))
        self._current_angle = new_angle

        self._publish_cmd(signal, playtime)

    def _publish_cmd(self, signal, playtime):
        cmd = DigitalServoCmd()
        cmd.signal = signal
        cmd.playtime = playtime
        self._cmd_pub.publish(cmd)

    def _bound_playtime(self, value):
        return max(0, min(255, value))

    def _calculate_playtime(self, movement_duration):
        # units of 10 ms
        return self._bound_playtime(round(movement_duration * 100.0))

    def init_servo(self, angle, playtime):
        signal = self._bound_signal(self._transform_angle_to_signal(angle))
        self._current_angle = angle
        self._publish_cmd(signal, playtime)
