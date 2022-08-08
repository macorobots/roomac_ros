#!/usr/bin/env python

import math

import rospy
from sensor_msgs.msg import JointState

from roomac_msgs.msg import DigitalServoCmd, AnalogServoCmd

import roomac_arm_controller.utils as utils
from roomac_arm_controller.servo_stub import ServoStub, GripperServoStub


class ServoJointStatePublisher:
    def __init__(self):
        self._joint_state_publishing_frequency = rospy.get_param(
            "~joint_state_publishing_frequency", 10
        )

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

    def _create_servo_stub(self, joint_name, params, gripper_servo=False):
        if params["servo_type"] == "analog":
            msg_type = AnalogServoCmd
        elif params["servo_type"] == "digital":
            msg_type = DigitalServoCmd

        if gripper_servo:
            servo_type = GripperServoStub
        else:
            servo_type = ServoStub

        return servo_type(
            joint_name,
            params["command_topic_name"],
            params["zero_angle_signal"],
            params["scale_factor"],
            msg_type,
        )

    def _add_servos(self):
        arm_servo_joints = rospy.get_param("~arm_controller/arm_servo_joints")
        for servo_joint in arm_servo_joints:
            parameter_ns = "~arm_controller/" + servo_joint + "/"
            params = utils.parse_servo(parameter_ns)
            self._servos.append(self._create_servo_stub(servo_joint, params))

        gripper_servo_joint = rospy.get_param("~gripper_controller/gripper_servo_joint")
        parameter_ns = "~gripper_controller/" + gripper_servo_joint + "/"
        params = utils.parse_servo(parameter_ns)
        self._servos.append(
            self._create_servo_stub(gripper_servo_joint, params, gripper_servo=True)
        )


if __name__ == "__main__":
    rospy.init_node("servo_joint_state_publisher")
    servo_joint_state_publisher = ServoJointStatePublisher()
    servo_joint_state_publisher.run()
