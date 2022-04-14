#!/usr/bin/env python

import math
import itertools
from ntpath import join

import rospy
from std_msgs.msg import UInt16, UInt8, Float32
from sensor_msgs.msg import JointState


class Servo:
    def __init__(self, zero_angle_signal, angle_to_signal_scale_factor):
        self.signal = zero_angle_signal
        self.speed = None

        self._zero_angle_signal = zero_angle_signal
        self._angle_to_signal_scale_factor = angle_to_signal_scale_factor

    def calculate_angle(self):
        if not self.signal:
            raise RuntimeError("Servo not initialized (signal wasn't yet received)")

        return (
            self.signal - self._zero_angle_signal
        ) / self._angle_to_signal_scale_factor


class ArmStub:
    def __init__(self):

        zero_angle_signal = rospy.get_param(
            "~zero_angle_signal", [850, 320, 512, 1509, 1500, 650]
        )
        analog_lower_signal_bound = rospy.get_param("~analog_lower_signal_bound", 500)
        analog_upper_signal_bound = rospy.get_param("~analog_upper_signal_bound", 2500)
        digital_lower_signal_bound = rospy.get_param("~digital_lower_signal_bound", 0)
        digital_upper_signal_bound = rospy.get_param(
            "~digital_upper_signal_bound", 1024
        )
        wrist_signal_zero_position = rospy.get_param(
            "~wrist_signal_zero_position", 1509
        )
        wrist_signal_90_degrees = rospy.get_param("~wrist_signal_90_degrees", 697)

        analog_scale_factor_wrist = (
            wrist_signal_zero_position - wrist_signal_90_degrees
        ) / (90.0 * math.pi / 180.0)
        digital_scale_factor = (
            digital_upper_signal_bound - digital_lower_signal_bound
        ) / ((330.0 / 2.0) * math.pi / 180.0)
        analog_scale_factor = (
            analog_upper_signal_bound - analog_lower_signal_bound
        ) / (180.0 * math.pi / 180.0)

        self._servos = {}

        self._servos["right_shoulder_pan"] = Servo(
            zero_angle_signal[0], digital_scale_factor
        )
        self._servos["right_shoulder_lift"] = Servo(
            zero_angle_signal[1], digital_scale_factor
        )
        self._servos["right_elbow"] = Servo(
            zero_angle_signal[2],
            digital_scale_factor / 2.0,  # no gear reduction
        )
        self._servos["right_wrist"] = Servo(
            zero_angle_signal[3], analog_scale_factor_wrist
        )
        self._servos["right_gripper_twist"] = Servo(
            zero_angle_signal[4], analog_scale_factor
        )
        self._servos["right_gripper"] = Servo(zero_angle_signal[5], analog_scale_factor)

        self._joint_state_pub = rospy.Publisher(
            "joint_states_arm_stub", JointState, queue_size=10
        )

        self._servo_position_cmd_sub = rospy.Subscriber(
            "joint_state_cmd", JointState, self._servo_position_cmd_cb
        )

    def _servo_position_cmd_cb(self, msg):
        for joint_name, position, velocity in itertools.izip(
            msg.name, msg.position, msg.velocity
        ):
            if not joint_name in self._servos:
                rospy.logwarn(joint_name + " not valid joint name")
                continue

            self._servos[joint_name].signal = position
            self._servos[joint_name].speed = velocity

        self._publish_joint_states()
        self._print_stats()

    def _publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = []
        joint_state_msg.position = []
        for x in self._servos:
            joint_state_msg.name.append(x)
            joint_state_msg.position.append(self._servos[x].calculate_angle())
        self._joint_state_pub.publish(joint_state_msg)

    def _print_stats(self):
        try:
            for x in self._servos:
                rospy.loginfo(
                    "Servo: "
                    + x
                    + " Signal: "
                    + str(self._servos[x].signal)
                    + " Speed: "
                    + str(self._servos[x].speed)
                )

            rospy.loginfo("--------")
        except RuntimeError as e:
            rospy.logerr_throttle(1.0, e)


if __name__ == "__main__":
    rospy.init_node("arm_controller")
    arm_stub = ArmStub()
    rospy.spin()
