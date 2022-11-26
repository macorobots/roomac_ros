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

import math
import copy

import rospy
import tf2_ros

from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Pose2D


class OdomTfToTopic:
    def __init__(self):
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

        self.last_time = rospy.Time.now()
        self.last_position = Pose2D()

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.rate = rospy.Rate(20.0)

    def run(self):
        while not rospy.is_shutdown():
            try:
                odom_base_transform = self.tf_buffer.lookup_transform(
                    "odom", "base_link", rospy.Time(0)
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                self.rate.sleep()
                continue

            current_time = odom_base_transform.header.stamp

            current_position = Pose2D()
            current_position.x = odom_base_transform.transform.translation.x
            current_position.y = odom_base_transform.transform.translation.y
            quaternion = [
                odom_base_transform.transform.rotation.x,
                odom_base_transform.transform.rotation.y,
                odom_base_transform.transform.rotation.z,
                odom_base_transform.transform.rotation.w,
            ]

            euler_angles = euler_from_quaternion(quaternion)
            current_position.theta = euler_angles[2]

            dt = (current_time - self.last_time).to_sec()
            if dt > 0:
                vx = (
                    math.sqrt(
                        (current_position.x - self.last_position.x) ** 2
                        + (current_position.y - self.last_position.y) ** 2
                    )
                    / dt
                )
                vth = (current_position.theta - self.last_position.theta) / dt
            else:
                vx = 0
                vth = 0

            self.last_position = copy.deepcopy(current_position)

            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose.pose = Pose(
                Point(
                    odom_base_transform.transform.translation.x,
                    odom_base_transform.transform.translation.y,
                    odom_base_transform.transform.translation.z,
                ),
                Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
            )
            odom_msg.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, vth))
            self.odom_pub.publish(odom_msg)

            self.last_time = current_time
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("odometry_topic_publisher")
    odom_publisher = OdomTfToTopic()
    odom_publisher.run()
