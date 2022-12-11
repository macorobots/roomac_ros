#!/usr/bin/env python

#  Software License Agreement
#
#  Copyright (C) 2022, Maciej Stepien, All rights reserved.
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
#  Authors: Maciej Stepien

import copy
import math

import rospy
from geometry_msgs.msg import (
    Point,
    Pose,
    Pose2D,
    PoseStamped,
    Quaternion,
    QuaternionStamped,
    Twist,
    Vector3,
    Vector3Stamped,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


class ImuOrientationRepublisher:
    """Subscribes to QuaternionStamped msgs and republishes them as Imu msg"""

    def __init__(self):
        self.imu_pub = rospy.Publisher("imu/orientation", Imu, queue_size=50)
        self.imu_raw_sub = rospy.Subscriber(
            "imu/orientation_raw", QuaternionStamped, self.imu_cb
        )

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu_link"

        # Disable, as in msg description
        self.imu_msg.angular_velocity_covariance[0] = -1

        # Disable, as in msg description
        self.imu_msg.linear_acceleration_covariance[0] = -1

    def imu_cb(self, msg):
        self.imu_msg.orientation = msg.quaternion
        self.imu_msg.header.stamp = msg.header.stamp
        self.imu_pub.publish(self.imu_msg)


class ImuAngularVelocityRepublisher:
    """Subscribes to Vector3Stamped msgs and republishes them as Imu msg"""

    def __init__(self):
        self.imu_pub = rospy.Publisher("imu/angular_velocity", Imu, queue_size=50)
        self.imu_raw_sub = rospy.Subscriber(
            "imu/angular_velocity_raw", Vector3Stamped, self.imu_cb
        )

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu_link"

        # Disable, as in msg description
        self.imu_msg.orientation_covariance[0] = -1

        # Disable, as in msg description
        self.imu_msg.linear_acceleration_covariance[0] = -1

    def imu_cb(self, msg):
        self.imu_msg.angular_velocity = msg.vector
        self.imu_msg.header.stamp = msg.header.stamp
        self.imu_pub.publish(self.imu_msg)


class OdomRepublisher:
    """Subscribes to PoseStamped msgs and republishes them as Odometry msg
    As PoseStamped includes only position information velocity is calculated using
    difference quotient
    """

    def __init__(self):
        self.odom_pub = rospy.Publisher("wheel_odom/odom", Odometry, queue_size=50)

        self.last_time = rospy.Time.now()
        self.last_position = Pose2D()

        self.odom_raw_sub = rospy.Subscriber(
            "wheel_odom/position_raw", PoseStamped, self.odom_cb
        )

    def odom_cb(self, msg):
        current_time = msg.header.stamp

        current_position = Pose2D()

        current_position.x = msg.pose.position.x
        current_position.y = msg.pose.position.y
        quaternion = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
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

            # Calculate direction of vx
            # atan2 The returned value is between PI and -PI.
            angle = math.atan2(
                (current_position.y - self.last_position.y),
                (current_position.x - self.last_position.x),
            )
            direction_angle_diff = abs(angle - self.last_position.theta)

            if direction_angle_diff > math.pi / 2:
                vx = -vx

            angle_diff = math.fmod(
                (current_position.theta - self.last_position.theta), 2 * math.pi
            )
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            elif angle_diff < -math.pi:
                angle_diff = 2 * math.pi + angle_diff

            vth = angle_diff / dt
        else:
            vx = 0
            vth = 0

        self.last_position = copy.deepcopy(current_position)

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose = Pose(
            Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
            Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
        )
        odom_msg.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, vth))

        self.odom_pub.publish(odom_msg)

        self.last_time = current_time


if __name__ == "__main__":
    rospy.init_node("odom_imu_republisher")
    odometry_republisher = OdomRepublisher()
    imu_orientation_republisher = ImuOrientationRepublisher()
    imu_angular_velocity_republisher = ImuAngularVelocityRepublisher()
    rospy.spin()
