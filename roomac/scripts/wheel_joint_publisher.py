#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class WheelJointPublisher:
    def __init__(self):
        # /wheel_odom/odom
        self._odom_sub = rospy.Subscriber(
            "/wheel_odom/odom", Odometry, self.odom_cb, queue_size=10
        )

        self._joint_state_pub = rospy.Publisher(
            "/arm_position_controller/joint_states_from_controller",
            JointState,
            queue_size=10,
        )
        self._last_msg = None

        self.r_motor_angle = 0
        self.l_motor_angle = 0

    def odom_cb(self, msg):
        if not self._last_msg:
            self._last_msg = msg

        HALF_WHEEL_DISTANCE = 0.2535
        WHEEL_RADIUS = 0.145

        dt = (msg.header.stamp - self._last_msg.header.stamp).to_sec()

        self.r_motor_angle += (
            (msg.twist.twist.linear.x + HALF_WHEEL_DISTANCE * msg.twist.twist.angular.z)
            * dt
        ) / WHEEL_RADIUS

        self.l_motor_angle += (
            (msg.twist.twist.linear.x - HALF_WHEEL_DISTANCE * msg.twist.twist.angular.z)
            * dt
        ) / WHEEL_RADIUS

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = ["wheel_right_joint", "wheel_left_joint"]
        joint_state_msg.position = [self.r_motor_angle, self.l_motor_angle]
        self._joint_state_pub.publish(joint_state_msg)

        self._last_msg = msg


if __name__ == "__main__":
    rospy.init_node("wheel_joint_publisher")
    wheel_joint_publisher = WheelJointPublisher()
    rospy.spin()
