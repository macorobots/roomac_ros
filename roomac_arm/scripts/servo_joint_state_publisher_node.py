#!/usr/bin/env python

import math

import rospy
from sensor_msgs.msg import JointState

from roomac_msgs.msg import DigitalServoCmd, AnalogServoCmd

import roomac_arm.utils
from roomac_arm.servo_stub import ServoStub, GripperServoStub


class ServoJointStatePublisher:
    def __init__(self):
        self._load_parameters()
        self._calculate_scale_factors()

        self._servos = []
        self._add_servos()

        self._joint_state_pub = rospy.Publisher(
            "joint_states_arm_stub", JointState, queue_size=10
        )

    def run(self):
        rate = rospy.Rate(self._joint_state_publishing_frequency)
        while not rospy.is_shutdown():
            try:
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = rospy.Time.now()
                joint_state_msg.name = []
                joint_state_msg.position = []

                for x in self._servos:
                    rospy.loginfo("Servo: " + x.name + " Signal: " + str(x.signal))

                    joint_state_msg.name.append(x.name)
                    joint_state_msg.position.append(x.calculate_position())

                rospy.loginfo("--------")

                self._joint_state_pub.publish(joint_state_msg)
            except RuntimeError as e:
                rospy.logerr_throttle(1.0, e)

            rate.sleep()

    def _add_servos(self):
        self._servos.append(
            ServoStub(
                "shoulder_pitch_right_joint",
                "shoulder_pan_cmd",
                self._zero_angle_signal_shoulder_pitch,
                self._digital_scale_factor_geared,
                DigitalServoCmd,
            )
        )
        self._servos.append(
            ServoStub(
                "shoulder_roll_right_joint",
                "shoulder_lift_cmd",
                self._zero_angle_signal_sholder_roll,
                self._digital_scale_factor_geared,
                DigitalServoCmd,
            )
        )
        self._servos.append(
            ServoStub(
                "elbow_right_joint",
                "elbow_cmd",
                self._zero_angle_signal_elbow,
                self._digital_scale_factor,
                DigitalServoCmd,
            )
        )
        self._servos.append(
            ServoStub(
                "wrist_right_joint",
                "wrist_cmd",
                self._zero_angle_signal_wrist,
                self._wrist_analog_scale_factor,
                AnalogServoCmd,
            )
        )
        self._servos.append(
            ServoStub(
                "gripper_twist_right_joint",
                "wrist_twist_cmd",
                self._zero_angle_signal_gripper_twist,
                self._analog_scale_factor,
                AnalogServoCmd,
            )
        )
        self._servos.append(
            GripperServoStub(
                "gripper_finger_l_right_joint",
                "gripper_cmd",
                self._zero_angle_signal_gripper_finger,
                self._analog_scale_factor,
                AnalogServoCmd,
            )
        )

    def _calculate_scale_factors(self):
        self._analog_scale_factor = roomac_arm.utils.calculate_scale_factor(
            self._analog_upper_signal_bound,
            self._analog_lower_signal_bound,
            math.radians(self._analog_angle_diff),
        )
        self._wrist_analog_scale_factor = roomac_arm.utils.calculate_scale_factor(
            self._wrist_signal_zero_position,
            self._wrist_signal_90_degrees,
            math.radians(self._wrist_analog_angle_diff),
        )
        self._digital_scale_factor = roomac_arm.utils.calculate_scale_factor(
            self._digital_upper_signal_bound,
            self._digital_lower_signal_bound,
            math.radians(self._digital_angle_diff),
        )
        self._digital_scale_factor_geared = roomac_arm.utils.calculate_scale_factor(
            self._digital_upper_signal_bound,
            self._digital_lower_signal_bound,
            math.radians(self._digital_angle_diff_geared),
        )

    def _load_parameters(self):
        self._joint_state_publishing_frequency = rospy.get_param(
            "~joint_state_publishing_frequency", 10
        )

        self._zero_angle_signal_shoulder_pitch = rospy.get_param(
            "~zero_angle_signal_shoulder_pitch", 860
        )
        self._zero_angle_signal_sholder_roll = rospy.get_param(
            "~zero_angle_signal_sholder_roll", 280
        )
        self._zero_angle_signal_elbow = rospy.get_param("~zero_angle_signal_elbow", 512)
        self._zero_angle_signal_wrist = rospy.get_param(
            "~zero_angle_signal_wrist", 1450
        )
        self._zero_angle_signal_gripper_twist = rospy.get_param(
            "~zero_angle_signal_gripper_twist", 1590
        )
        self._zero_angle_signal_gripper_finger = rospy.get_param(
            "~zero_angle_signal_gripper_finger", 650
        )

        self._analog_lower_signal_bound = rospy.get_param(
            "~analog_lower_signal_bound", 500
        )
        self._analog_upper_signal_bound = rospy.get_param(
            "~analog_upper_signal_bound", 2500
        )
        self._analog_angle_diff = rospy.get_param("~analog_angle_diff", 180.0)

        self._digital_lower_signal_bound = rospy.get_param(
            "~digital_lower_signal_bound", 0
        )
        self._digital_upper_signal_bound = rospy.get_param(
            "~digital_upper_signal_bound", 1024
        )
        self._digital_angle_diff = rospy.get_param("~digital_upper_signal_bound", 330.0)
        self._digital_angle_diff_geared = rospy.get_param(
            "~digital_upper_signal_bound", 165.0
        )

        self._wrist_signal_zero_position = rospy.get_param(
            "~wrist_signal_zero_position", 1509
        )
        self._wrist_signal_90_degrees = rospy.get_param("~wrist_signal_90_degrees", 697)
        self._wrist_analog_angle_diff = rospy.get_param(
            "~wrist_analog_angle_diff", 90.0
        )


if __name__ == "__main__":
    rospy.init_node("servo_joint_state_publisher")
    servo_joint_state_publisher = ServoJointStatePublisher()
    servo_joint_state_publisher.run()
