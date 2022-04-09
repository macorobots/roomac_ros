#!/usr/bin/env python

import math

import rospy
from std_msgs.msg import UInt16, UInt8, Float32
from sensor_msgs.msg import JointState


class Servo:
    def __init__(
        self, name, zero_angle_signal, angle_to_signal_scale_factor, playtime=True
    ):
        self._signal_sub = rospy.Subscriber(
            name + "_position",
            UInt16,
            self._signal_cb,
        )
        self.name = name

        if playtime:
            self.speed_parameter_name = "Playtime"
            self._playtime_sub = rospy.Subscriber(
                name + "_playtime",
                UInt8,
                self._speed_cb,
            )
        else:
            self.speed_parameter_name = "Speed"
            self._speed_sub = rospy.Subscriber(
                name + "_speed",
                Float32,
                self._speed_cb,
            )

        self.signal = None
        self.speed = None

        self._zero_angle_signal = zero_angle_signal
        self._angle_to_signal_scale_factor = angle_to_signal_scale_factor

    def calculate_angle(self):
        if not self.signal:
            raise RuntimeError(
                "Servo " + self.name + " not initialized (signal wasn't yet received)"
            )

        return (
            self.signal - self._zero_angle_signal
        ) / self._angle_to_signal_scale_factor

    def _signal_cb(self, msg):
        self.signal = msg.data

    def _speed_cb(self, msg):
        self.speed = msg.data


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

        self._servos = []

        self._servos.append(
            Servo("shoulder_pan", zero_angle_signal[0], digital_scale_factor, True)
        )
        self._servos.append(
            Servo("shoulder_lift", zero_angle_signal[1], digital_scale_factor, True)
        )
        self._servos.append(
            Servo(
                "elbow",
                zero_angle_signal[2],
                digital_scale_factor / 2.0,  # no gear reduction
                True,
            )
        )
        self._servos.append(
            Servo("wrist", zero_angle_signal[3], analog_scale_factor_wrist, False)
        )
        self._servos.append(
            Servo("wrist_twist", zero_angle_signal[4], analog_scale_factor, False)
        )
        self._servos.append(
            Servo("gripper", zero_angle_signal[5], analog_scale_factor, False)
        )

        self._joint_state_pub = rospy.Publisher(
            "joint_states_arm_stub", JointState, queue_size=10
        )

        self._joint_name_remap = {
            "shoulder_pan": "right_shoulder_pan",
            "shoulder_lift": "right_shoulder_lift",
            "elbow": "right_elbow",
            "wrist": "right_wrist",
            "wrist_twist": "right_gripper_twist",
            "gripper": "right_gripper",
        }

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = rospy.Time.now()
                joint_state_msg.name = []
                joint_state_msg.position = []

                for x in self._servos:
                    rospy.loginfo(
                        "Servo: "
                        + x.name
                        + " Signal: "
                        + str(x.signal)
                        + " "
                        + x.speed_parameter_name
                        + ": "
                        + str(x.speed)
                    )

                    joint_state_msg.name.append(self._joint_name_remap[x.name])
                    joint_state_msg.position.append(x.calculate_angle())

                rospy.loginfo("--------")

                self._joint_state_pub.publish(joint_state_msg)
            except RuntimeError as e:
                rospy.logerr_throttle(1.0, e)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("arm_controller")
    arm_stub = ArmStub()
    arm_stub.run()
