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

    def calculate_angle_diff(self, new_angle):
        return abs(new_angle - self.current_angle)

    def calculate_movement_time(self, new_angle):
        raise NotImplementedError()

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


class AnalogServo(Servo):
    def __init__(
        self,
        name,
        zero_angle_signal,
        lower_signal_bound,
        upper_signal_bound,
        angle_to_signal_scale_factor,
        speed,
        analog_update_delay,
    ):
        Servo.__init__(
            self,
            name,
            zero_angle_signal,
            lower_signal_bound,
            upper_signal_bound,
            angle_to_signal_scale_factor,
        )
        self.speeds_pub = rospy.Publisher(
            self.name + "_speed", Float32, queue_size=5, latch=True
        )
        self.speed = speed
        self.analog_update_delay = analog_update_delay

        self.set_speed(self.speed)

    def publish_speed(self, speed):
        speed_msg = Float32()
        speed_msg.data = speed
        self.speeds_pub.publish(speed_msg)

    def set_speed(self, speed):
        self.speed = speed
        self.publish_speed(speed)

    def calculate_movement_time(self, new_angle):
        angle_diff = self.calculate_angle_diff(new_angle)
        movement_time = (
            (angle_diff * self.angle_to_signal_scale_factor) / self.speed
        ) * self.analog_update_delay
        return movement_time


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
        Servo.__init__(
            self,
            name,
            zero_angle_signal,
            lower_signal_bound,
            upper_signal_bound,
            angle_to_signal_scale_factor,
        )

        self.speeds_pub = rospy.Publisher(
            self.name + "_playtime", UInt8, queue_size=5, latch=True
        )
        self.publish_playtime(255)
        self.max_speed = max_speed

    def bound_playtime(self, value):
        return max(0, min(255, value))

    def publish_playtime(self, playtime):
        playtime = self.bound_playtime(playtime)
        playtime_msg = UInt8()
        playtime_msg.data = playtime
        self.speeds_pub.publish(playtime_msg)

    def calculate_movement_time(self, new_angle):
        angle_diff = self.calculate_angle_diff(new_angle)
        movement_time = (angle_diff / (2 * math.pi)) / self.max_speed
        return movement_time
