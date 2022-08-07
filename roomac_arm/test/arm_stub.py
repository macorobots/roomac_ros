#!/usr/bin/env python

import math

import rospy
from roomac_msgs.msg import DigitalServoCmd, AnalogServoCmd
from sensor_msgs.msg import JointState


class Servo(object):
    def __init__(
        self, name, cmd_topic, zero_angle_signal, angle_to_signal_scale_factor, cmd_type
    ):
        self._cmd_sub = rospy.Subscriber(
            cmd_topic,
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


class GripperServo(Servo):
    def __init__(
        self, name, cmd_topic, zero_angle_signal, angle_to_signal_scale_factor, cmd_type
    ):
        super(GripperServo, self).__init__(
            name, cmd_topic, zero_angle_signal, angle_to_signal_scale_factor, cmd_type
        )

    def _transform_angle_to_dist(self, angle):
        # Linear approximation using these two points
        # 0.2 -> 0.005m
        # 1.0 -> -0.0045m
        m = -0.011875
        b = 0.007375
        return -(m * angle + b)

    def calculate_angle(self):
        angle = super(GripperServo, self).calculate_angle()
        return self._transform_angle_to_dist(angle)


class ArmStub:
    def __init__(self):

        zero_angle_signal_shoulder_pitch = rospy.get_param(
            "~zero_angle_signal_shoulder_pitch", 860
        )
        zero_angle_signal_sholder_roll = rospy.get_param(
            "~zero_angle_signal_sholder_roll", 280
        )
        zero_angle_signal_elbow = rospy.get_param("~zero_angle_signal_elbow", 512)
        zero_angle_signal_wrist = rospy.get_param("~zero_angle_signal_wrist", 1450)
        zero_angle_signal_gripper_twist = rospy.get_param(
            "~zero_angle_signal_gripper_twist", 1590
        )
        zero_angle_signal_gripper_finger = rospy.get_param(
            "~zero_angle_signal_gripper_finger", 650
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
                "shoulder_pitch_right_joint",
                "shoulder_pan_cmd",
                zero_angle_signal_shoulder_pitch,
                digital_scale_factor,
                DigitalServoCmd,
            )
        )
        self._servos.append(
            Servo(
                "shoulder_roll_right_joint",
                "shoulder_lift_cmd",
                zero_angle_signal_sholder_roll,
                digital_scale_factor,
                DigitalServoCmd,
            )
        )
        self._servos.append(
            Servo(
                "elbow_right_joint",
                "elbow_cmd",
                zero_angle_signal_elbow,
                digital_scale_factor / 2.0,  # no gear reduction
                DigitalServoCmd,
            )
        )
        self._servos.append(
            Servo(
                "wrist_right_joint",
                "wrist_cmd",
                zero_angle_signal_wrist,
                analog_scale_factor_wrist,
                AnalogServoCmd,
            )
        )
        self._servos.append(
            Servo(
                "gripper_twist_right_joint",
                "wrist_twist_cmd",
                zero_angle_signal_gripper_twist,
                analog_scale_factor,
                AnalogServoCmd,
            )
        )
        self._servos.append(
            GripperServo(
                "gripper_finger_l_right_joint",
                "gripper_cmd",
                zero_angle_signal_gripper_finger,
                analog_scale_factor,
                AnalogServoCmd,
            )
        )

        self._joint_state_pub = rospy.Publisher(
            "joint_states_arm_stub", JointState, queue_size=10
        )

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

                    joint_state_msg.name.append(x.name)
                    joint_state_msg.position.append(x.calculate_angle())

                rospy.loginfo("--------")

                self._joint_state_pub.publish(joint_state_msg)
            except RuntimeError as e:
                rospy.logerr_throttle(1.0, e)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("servo_joint_publisher")
    arm_stub = ArmStub()
    arm_stub.run()
