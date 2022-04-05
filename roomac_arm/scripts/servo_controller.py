#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16, UInt8, Float32
import math


class Servo:
    def __init__(
        self,
        name,
        zero_angle_signal,
        lower_signal_bound,
        upper_signal_bound,
        angle_to_signal_scale_factor,
        max_speed,
    ):
        self.name = name
        # latching???
        self.signal_pub = rospy.Publisher(
            name + "_position", UInt16, queue_size=5, latch=True
        )

        self.zero_angle_signal = zero_angle_signal
        self.current_angle = None

        self.lower_signal_bound = lower_signal_bound
        self.upper_signal_bound = upper_signal_bound

        self.angle_to_signal_scale_factor = angle_to_signal_scale_factor

        self.max_speed = max_speed

    def calculate_angle_diff(self, new_angle):
        return abs(new_angle - self.current_angle)

    def calculate_min_movement_duration(self, new_angle):
        return self.calculate_angle_diff(new_angle) / self.max_speed

    def transform_angle_to_signal(self, angle):
        return angle * self.angle_to_signal_scale_factor + self.zero_angle_signal

    def is_initialized(self):
        return not (self.current_angle is None)

    def bound_signal(self, signal):
        return max(self.lower_signal_bound, min(self.upper_signal_bound, signal))

    def publish_signal(self, signal):
        signal_msg = UInt16()
        signal_msg.data = signal
        self.signal_pub.publish(signal_msg)

    def set_angle(self, angle):
        signal = self.bound_signal(self.transform_angle_to_signal(angle))
        self.publish_signal(signal)
        self.current_angle = angle

    def set_speed(self, new_angle, movement_duration):
        raise NotImplementedError()

    def execute_motion(self, new_angle, movement_duration):
        self.set_speed(new_angle, movement_duration)
        self.set_angle(new_angle)


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
        Servo.__init__(
            self,
            name,
            zero_angle_signal,
            lower_signal_bound,
            upper_signal_bound,
            angle_to_signal_scale_factor,
            max_speed,
        )
        self.speeds_pub = rospy.Publisher(
            self.name + "_speed", Float32, queue_size=5, latch=True
        )

        self.analog_speed = analog_speed
        self.analog_update_delay = analog_update_delay

        self.publish_analog_speed(self.analog_speed)

    def publish_analog_speed(self, analog_speed):
        analog_speed_msg = Float32()
        analog_speed_msg.data = analog_speed
        self.speeds_pub.publish(analog_speed_msg)

    def calculate_analog_speed(self, new_angle, movement_duration):
        return (
            self.calculate_angle_diff(new_angle) * self.angle_to_signal_scale_factor
        ) * (self.analog_update_delay / movement_duration)

    def set_speed(self, new_angle, movement_duration):
        self.analog_speed = self.calculate_analog_speed(new_angle, movement_duration)
        self.publish_analog_speed(self.analog_speed)


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
        Servo.__init__(
            self,
            name,
            zero_angle_signal,
            lower_signal_bound,
            upper_signal_bound,
            angle_to_signal_scale_factor,
            max_speed,
        )

        self.speeds_pub = rospy.Publisher(
            self.name + "_playtime", UInt8, queue_size=5, latch=True
        )
        self.playtime = playtime
        self.publish_playtime(self.playtime)

    def bound_playtime(self, value):
        return max(0, min(255, value))

    def publish_playtime(self, playtime):
        playtime = self.bound_playtime(playtime)
        playtime_msg = UInt8()
        playtime_msg.data = playtime
        self.speeds_pub.publish(playtime_msg)

    def calculate_playtime(self, movement_duration):
        # units of 10 ms
        return self.bound_playtime(movement_duration * 100.0)

    def set_speed(self, new_angle, movement_duration):
        self.playtime = self.calculate_playtime(movement_duration)
        self.publish_playtime(self.playtime)
