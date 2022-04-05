#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16, UInt8, Float32
import math


class Servo:
    def __init__(
        self, name, zero_position, lower_signal_bound, upper_signal_bound, scale_factor
    ):
        self.name = name
        self.position_pub = rospy.Publisher(
            name + "_position", UInt16, queue_size=5, latch=True
        )

        self.zero_position = zero_position
        self.last_position = None

        self.lower_signal_bound = lower_signal_bound
        self.upper_signal_bound = upper_signal_bound

        self.scale_factor = scale_factor

    def calculate_position_diff(self, new_position):
        return abs(new_position - self.last_position)

    def calculate_movement_time(self, new_position):
        raise NotImplementedError()

    def publish_position(self, position):
        position_msg = UInt16()
        position_msg.data = position
        self.position_pub.publish(position_msg)

    def transform_angle_to_signal(self, angle):
        return angle * self.scale_factor + self.zero_position

    def is_initialized(self):
        return not (self.last_position is None)

    def bound_angle(self, value):
        return max(self.lower_signal_bound, min(self.upper_signal_bound, value))

    def set_angle(self, angle):
        signal = self.bound_angle(self.transform_angle_to_signal(angle))
        self.publish_position(signal)
        self.last_position = angle


class AnalogServo(Servo):
    def __init__(
        self,
        name,
        zero_position,
        lower_signal_bound,
        upper_signal_bound,
        scale_factor,
        speed,
        analog_update_delay,
    ):
        Servo.__init__(
            self,
            name,
            zero_position,
            lower_signal_bound,
            upper_signal_bound,
            scale_factor,
        )
        self.speeds_pub = rospy.Publisher(
            self.name + "_speed", Float32, queue_size=5, latch=True
        )
        self.speed = speed
        self.analog_update_delay = analog_update_delay

    def publish_speed(self, speed):
        self.speed = speed
        speed_msg = Float32()
        speed_msg.data = speed
        self.speeds_pub.publish(speed_msg)

    def calculate_movement_time(self, new_position):
        angle_diff = self.calculate_position_diff(new_position)

        movement_time = (
            (angle_diff * self.scale_factor) / self.speed
        ) * self.analog_update_delay

        return movement_time


class DigitalServo(Servo):
    def __init__(
        self,
        name,
        zero_position,
        lower_signal_bound,
        upper_signal_bound,
        scale_factor,
        max_speed,
    ):
        Servo.__init__(
            self,
            name,
            zero_position,
            lower_signal_bound,
            upper_signal_bound,
            scale_factor,
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

    def calculate_movement_time(self, new_position):
        angle_diff = self.calculate_position_diff(new_position)

        movement_time = (angle_diff / (2 * math.pi)) / self.max_speed

        return movement_time
