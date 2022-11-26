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
from geometry_msgs.msg import (
    Pose2D,
)
from tf.transformations import euler_from_quaternion

import math
import copy


class PathStats:
    def __init__(self):
        self.total_distance = 0
        self.total_angle_change = 0

        self.last_position = Pose2D()
        self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)

    def odom_cb(self, msg):
        current_position = Pose2D()

        current_position.x = msg.pose.pose.position.x
        current_position.y = msg.pose.pose.position.y
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]

        euler_angles = euler_from_quaternion(quaternion)
        current_position.theta = euler_angles[2]

        angle_diff = math.fmod(
            (current_position.theta - self.last_position.theta), 2 * math.pi
        )
        if angle_diff > math.pi:
            angle_diff = 2 * math.pi - angle_diff
        elif angle_diff < -math.pi:
            angle_diff = 2 * math.pi + angle_diff

        self.total_angle_change += abs(angle_diff)

        position_diff = math.sqrt(
            (current_position.x - self.last_position.x) ** 2
            + (current_position.y - self.last_position.y) ** 2
        )

        self.total_distance += position_diff

        self.last_position = copy.deepcopy(current_position)

        rospy.loginfo("Total distance: " + str(self.total_distance))
        rospy.loginfo("Total angle change: " + str(self.total_angle_change))


if __name__ == "__main__":
    rospy.init_node("path_stats")
    path_stats = PathStats()
    rospy.spin()
