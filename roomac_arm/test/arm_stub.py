#!/usr/bin/env python

import math

import rospy
from roomac_msgs.msg import DigitalServoCmd, AnalogServoCmd
from sensor_msgs.msg import JointState


class Servo:
    def __init__(self, name, zero_angle_signal, angle_to_signal_scale_factor, cmd_type):
        self._cmd_sub = rospy.Subscriber(
            name + "_cmd",
            cmd_type,
            self._cmd_cb,
        )
        self.name = name

        self.signal = zero_angle_signal

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

    def _cmd_cb(self, msg):
        self.signal = msg.signal


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
            Servo(
                "shoulder_pan",
                zero_angle_signal[0],
                digital_scale_factor,
                DigitalServoCmd,
            )
        )
        self._servos.append(
            Servo(
                "shoulder_lift",
                zero_angle_signal[1],
                digital_scale_factor,
                DigitalServoCmd,
            )
        )
        self._servos.append(
            Servo(
                "elbow",
                zero_angle_signal[2],
                digital_scale_factor / 2.0,  # no gear reduction
                DigitalServoCmd,
            )
        )
        self._servos.append(
            Servo(
                "wrist", zero_angle_signal[3], analog_scale_factor_wrist, AnalogServoCmd
            )
        )
        self._servos.append(
            Servo(
                "wrist_twist", zero_angle_signal[4], analog_scale_factor, AnalogServoCmd
            )
        )
        self._servos.append(
            Servo("gripper", zero_angle_signal[5], analog_scale_factor, AnalogServoCmd)
        )

        self._joint_state_pub = rospy.Publisher(
            "joint_states_arm_stub", JointState, queue_size=10
        )

        self._joint_name_remap = {
            "shoulder_pan": "shoulder_pitch_right",
            "shoulder_lift": "shoulder_roll_right",
            "elbow": "elbow_right",
            "wrist": "wrist_right",
            "wrist_twist": "gripper_twist_right",
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
                    rospy.loginfo("Servo: " + x.name + " Signal: " + str(x.signal))

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
