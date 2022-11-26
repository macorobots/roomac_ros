#!/usr/bin/env python

#  Software License Agreement
#
#  Copyright (C) 2022, Maciej Stępień, All rights reserved.
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
#  Authors: Maciej Stępień

import rospy

from roomac_msgs.msg import DigitalServoCmd, AnalogServoCmd


class Servo(object):
    def __init__(
        self,
        zero_angle_signal,
        lower_signal_bound,
        upper_signal_bound,
        angle_to_signal_scale_factor,
        max_speed,
    ):
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
        command_topic_name,
        zero_angle_signal,
        lower_signal_bound,
        upper_signal_bound,
        angle_to_signal_scale_factor,
        max_speed,
        analog_update_delay,
    ):
        super(AnalogServo, self).__init__(
            zero_angle_signal,
            lower_signal_bound,
            upper_signal_bound,
            angle_to_signal_scale_factor,
            max_speed,
        )
        # latching is necessary to ensure that homming will be performed on the start
        # todo: instead of latching it will be safer to implement eg homing service in
        # the upper microcontroller
        self._cmd_pub = rospy.Publisher(
            command_topic_name, AnalogServoCmd, queue_size=10, latch=True
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
        command_topic_name,
        zero_angle_signal,
        lower_signal_bound,
        upper_signal_bound,
        angle_to_signal_scale_factor,
        max_speed,
    ):
        super(DigitalServo, self).__init__(
            zero_angle_signal,
            lower_signal_bound,
            upper_signal_bound,
            angle_to_signal_scale_factor,
            max_speed,
        )

        # latching is necessary to ensure that homming will be performed on the start
        # todo: instead of latching it will be safer to implement eg homing service in
        # the upper microcontroller
        self._cmd_pub = rospy.Publisher(
            command_topic_name, DigitalServoCmd, queue_size=10, latch=True
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
