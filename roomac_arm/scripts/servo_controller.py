#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16, UInt8, Float32
import math


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

    def set_angle_and_duration(self, new_angle, movement_duration):
        self.set_movement_time(new_angle, movement_duration)
        self.set_angle(new_angle)

    def set_angle(self, angle):
        self._current_angle = angle

    def get_position_signal(self):
        signal = self._bound_signal(
            self._transform_angle_to_signal(self._current_angle)
        )
        return signal

    def set_movement_time(self, new_angle, movement_duration):
        raise NotImplementedError()

    def get_velocity_signal(self):
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
        zero_angle_signal,
        lower_signal_bound,
        upper_signal_bound,
        angle_to_signal_scale_factor,
        max_speed,
        analog_speed,
        analog_update_delay,
    ):
        super(AnalogServo, self).__init__(
            zero_angle_signal,
            lower_signal_bound,
            upper_signal_bound,
            angle_to_signal_scale_factor,
            max_speed,
        )
        self._analog_speed = analog_speed
        self._analog_update_delay = analog_update_delay

    def _calculate_analog_speed(self, new_angle, movement_duration):
        return (
            abs(self.calculate_angle_diff(new_angle))
            * self._angle_to_signal_scale_factor
        ) * (self._analog_update_delay / movement_duration)

    def set_movement_time(self, new_angle, movement_duration):
        self._analog_speed = self._calculate_analog_speed(new_angle, movement_duration)

    def get_velocity_signal(self):
        return self._analog_speed


class DigitalServo(Servo):
    def __init__(
        self,
        zero_angle_signal,
        lower_signal_bound,
        upper_signal_bound,
        angle_to_signal_scale_factor,
        max_speed,
        playtime,
    ):
        super(DigitalServo, self).__init__(
            zero_angle_signal,
            lower_signal_bound,
            upper_signal_bound,
            angle_to_signal_scale_factor,
            max_speed,
        )
        self._playtime = playtime

    def _bound_playtime(self, value):
        return max(0, min(255, value))

    def _calculate_playtime(self, movement_duration):
        # units of 10 ms
        return self._bound_playtime(round(movement_duration * 100.0))

    def set_movement_time(self, new_angle, movement_duration):
        self._playtime = self._calculate_playtime(movement_duration)

    def get_velocity_signal(self):
        return self._playtime
