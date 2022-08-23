#!/usr/bin/env python

import math
import itertools
import numpy as np

import rospy

from dynamic_reconfigure.server import Server
from sensor_msgs.msg import JointState

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
        self._last_execution_finish_time = rospy.Time.now()
        self._wait_time_scaling = 0.9

    def go_to_point(self, joint_names, angles, duration=0.0):
        joint_names, angles = self._get_valid_joints(joint_names, angles)

        if not joint_names:
            rospy.logwarn("No valid joints found, aborting go_to_point execution")
            return

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
        time_to_finish_last_command = (
            self._last_execution_finish_time - rospy.Time.now()
        )
        if time_to_finish_last_command > rospy.Duration(0):
            # wait time scaling isn't the best solution, but with it the movement is smoother
            # todo: maybe change it to flat wait time
            rospy.sleep(
                rospy.Duration(
                    time_to_finish_last_command.to_sec() * self._wait_time_scaling
                )
            )
            # rospy.loginfo("Sleeping time: " + str(time_to_finish_last_command.to_sec()))

        send_time_start = rospy.Time.now()
        for joint_name, angle in itertools.izip(joint_names, angles):
            self._servos[joint_name].execute_motion(angle, movement_duration)
        sending_time_duration = rospy.Time.now() - send_time_start
        # rospy.loginfo("Sending time duration: " + str(sending_time_duration.to_sec()))

        self._last_execution_finish_time = (
            rospy.Time.now() + rospy.Duration(movement_duration) - sending_time_duration
        )
        # rospy.loginfo("Movement duration: " + str(movement_duration))

        if self._publish_joint_states:
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = joint_names
            joint_state_msg.position = angles
            self._joint_state_pub.publish(joint_state_msg)

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
        self._wait_time_scaling = config.wait_time_scaling

        return config
