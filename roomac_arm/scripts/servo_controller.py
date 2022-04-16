#!/usr/bin/env python

import rospy
from roomac_msgs.msg import DigitalServoCmd, AnalogServoCmd


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
