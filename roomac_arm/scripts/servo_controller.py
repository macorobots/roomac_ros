#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16, UInt8, Float32
import math


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
        # latching???
        self._signal_pub = rospy.Publisher(
            self._name + "_position", UInt16, queue_size=5, latch=True
        )

        self._zero_angle_signal = zero_angle_signal
        self._current_angle = None

        self._lower_signal_bound = lower_signal_bound
        self._upper_signal_bound = upper_signal_bound

        self._angle_to_signal_scale_factor = angle_to_signal_scale_factor

        self._max_speed = max_speed

    def execute_motion(self, new_angle, movement_duration):
        self._set_movement_time(new_angle, movement_duration)
        self.set_angle(new_angle)

    def set_angle(self, angle):
        signal = self._bound_signal(self._transform_angle_to_signal(angle))
        self._publish_signal(signal)
        self._current_angle = angle

    def is_initialized(self):
        return not (self._current_angle is None)

    def calculate_angle_diff(self, new_angle):
        return new_angle - self._current_angle

    def calculate_min_movement_duration(self, new_angle):
        return abs(self.calculate_angle_diff(new_angle)) / self._max_speed

    def current_angle(self):
        return self._current_angle

    def _transform_angle_to_signal(self, angle):
        return angle * self._angle_to_signal_scale_factor + self._zero_angle_signal

    def _bound_signal(self, signal):
        return max(self._lower_signal_bound, min(self._upper_signal_bound, signal))

    def _publish_signal(self, signal):
        signal_msg = UInt16()
        signal_msg.data = signal
        self._signal_pub.publish(signal_msg)

    def _set_movement_time(self, new_angle, movement_duration):
        raise NotImplementedError()


class AnalogServo(Servo):
    def __init__(
        self,
        name,
        zero_angle_signal,
        lower_signal_bound,
        upper_signal_bound,
        angle_to_signal_scale_factor,
        max_speed,
        analog_speed,
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
        self._speeds_pub = rospy.Publisher(
            self._name + "_speed", Float32, queue_size=5, latch=True
        )

        self._analog_speed = analog_speed
        self._analog_update_delay = analog_update_delay

        self._publish_analog_speed(self._analog_speed)

    def _publish_analog_speed(self, analog_speed):
        analog_speed_msg = Float32()
        analog_speed_msg.data = analog_speed
        self._speeds_pub.publish(analog_speed_msg)

    def _calculate_analog_speed(self, new_angle, movement_duration):
        return (
            abs(self.calculate_angle_diff(new_angle))
            * self._angle_to_signal_scale_factor
        ) * (self._analog_update_delay / movement_duration)

    def _set_movement_time(self, new_angle, movement_duration):
        self._analog_speed = self._calculate_analog_speed(new_angle, movement_duration)
        self._publish_analog_speed(self._analog_speed)


class DigitalServo(Servo):
    def __init__(
        self,
        name,
        zero_angle_signal,
        lower_signal_bound,
        upper_signal_bound,
        angle_to_signal_scale_factor,
        max_speed,
        playtime,
    ):
        super(DigitalServo, self).__init__(
            name,
            zero_angle_signal,
            lower_signal_bound,
            upper_signal_bound,
            angle_to_signal_scale_factor,
            max_speed,
        )

        self._speeds_pub = rospy.Publisher(
            self._name + "_playtime", UInt8, queue_size=5, latch=True
        )
        self._playtime = playtime
        self._publish_playtime(self._playtime)

    def _bound_playtime(self, value):
        return max(0, min(255, value))

    def _publish_playtime(self, playtime):
        playtime = self._bound_playtime(playtime)
        playtime_msg = UInt8()
        playtime_msg.data = playtime
        self._speeds_pub.publish(playtime_msg)

    def _calculate_playtime(self, movement_duration):
        # units of 10 ms
        return self._bound_playtime(movement_duration * 100.0)

    def _set_movement_time(self, new_angle, movement_duration):
        self._playtime = self._calculate_playtime(movement_duration)
        self._publish_playtime(self._playtime)
