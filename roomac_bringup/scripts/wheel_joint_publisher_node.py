#!/usr/bin/env python

#  Software License Agreement
#
#  Copyright (C) 2022, Maciej Stępień, All rights reserved.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
#  Authors: Maciej Stępień

import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class WheelJointPublisher:
    """This class subscribes to the odometry topic and based on it
    calculates rotation of each wheel and publishes joint states
    It is only used to make visualization better - rotating wheels in RViz.
    """

    def __init__(self):
        self._HALF_WHEEL_DISTANCE = 0.2535
        self._WHEEL_RADIUS = 0.145

        self._odom_sub = rospy.Subscriber(
            "/wheel_odom/odom", Odometry, self.odom_cb, queue_size=10
        )

        self._joint_state_pub = rospy.Publisher(
            "~joint_states_wheels",
            JointState,
            queue_size=10,
        )
        self._last_msg = None

        self._r_motor_angle = 0
        self._l_motor_angle = 0

    def odom_cb(self, msg):
        if not self._last_msg:
            self._last_msg = msg

        dt = (msg.header.stamp - self._last_msg.header.stamp).to_sec()

        self._r_motor_angle += (
            (
                msg.twist.twist.linear.x
                + self._HALF_WHEEL_DISTANCE * msg.twist.twist.angular.z
            )
            * dt
        ) / self._WHEEL_RADIUS

        self._l_motor_angle += (
            (
                msg.twist.twist.linear.x
                - self._HALF_WHEEL_DISTANCE * msg.twist.twist.angular.z
            )
            * dt
        ) / self._WHEEL_RADIUS

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = ["wheel_right_joint", "wheel_left_joint"]
        joint_state_msg.position = [self._r_motor_angle, self._l_motor_angle]
        self._joint_state_pub.publish(joint_state_msg)

        self._last_msg = msg


if __name__ == "__main__":
    rospy.init_node("wheel_joint_publisher")
    wheel_joint_publisher = WheelJointPublisher()
    rospy.spin()
