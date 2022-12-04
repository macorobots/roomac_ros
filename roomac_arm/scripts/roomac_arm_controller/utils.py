#!/usr/bin/env python

#  Software License Agreement
#
#  Copyright (C) 2022, Maciej Stepien, All rights reserved.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
#  Authors: Maciej Stepien

import math

import rospy

from servo import AnalogServo, DigitalServo


def create_servo(
    servo_type,
    command_topic_name,
    zero_angle_signal,
    lower_signal_bound,
    upper_signal_bound,
    scale_factor,
    max_speed,
    analog_update_delay,
    initial_angle,
    initial_analog_speed,
    initial_digital_playtime,
):
    if servo_type == "analog":
        servo = AnalogServo(
            command_topic_name,
            zero_angle_signal,
            lower_signal_bound,
            upper_signal_bound,
            scale_factor,
            max_speed,
            analog_update_delay,
        )
        servo.init_servo(initial_angle, initial_analog_speed)
    elif servo_type == "digital":
        servo = DigitalServo(
            command_topic_name,
            zero_angle_signal,
            lower_signal_bound,
            upper_signal_bound,
            scale_factor,
            max_speed,
        )
        servo.init_servo(initial_angle, initial_digital_playtime)

    return servo


def parse_servo(parameter_ns):
    try:
        servo_type = rospy.get_param(parameter_ns + "servo_type")
        if servo_type not in ["digital", "analog"]:
            rospy.logerr(
                "Servo type "
                + servo_type
                + " not supported, available servo types: (analog, digital)"
            )

        command_topic_name = rospy.get_param(parameter_ns + "command_topic_name")
        zero_angle_signal = rospy.get_param(parameter_ns + "zero_angle_signal")
        lower_signal_bound = rospy.get_param(parameter_ns + "lower_signal_bound")
        upper_signal_bound = rospy.get_param(parameter_ns + "upper_signal_bound")
        use_signal_bounds_for_scaling = rospy.get_param(
            parameter_ns + "use_signal_bounds_for_scaling"
        )
        angle_diff = rospy.get_param(parameter_ns + "angle_diff")
    except KeyError as e:
        rospy.logerr("Required parameter not defined: " + str(e))
        raise

    if use_signal_bounds_for_scaling:
        scaling_lower_signal = lower_signal_bound
        scaling_upper_signal = upper_signal_bound
    else:
        try:
            scaling_lower_signal = rospy.get_param(
                parameter_ns + "scaling_lower_signal", 0
            )
            scaling_upper_signal = rospy.get_param(
                parameter_ns + "scaling_upper_signal", 0
            )
        except KeyError as e:
            rospy.logerr("Required parameter not defined: " + str(e))
            raise

    scale_factor = calculate_scale_factor(
        scaling_upper_signal,
        scaling_lower_signal,
        math.radians(angle_diff),
    )

    return {
        "servo_type": servo_type,
        "command_topic_name": command_topic_name,
        "zero_angle_signal": zero_angle_signal,
        "lower_signal_bound": lower_signal_bound,
        "upper_signal_bound": upper_signal_bound,
        "scale_factor": scale_factor,
    }


def calculate_scale_factor(upper_signal, lower_signal, angle_difference):
    return (upper_signal - lower_signal) / angle_difference


def linear_transform_angle_to_dist(a, b, angle):
    return -(a * angle + b)


def linear_transform_dist_to_angle(a, b, dist):
    return (-dist - b) / a
