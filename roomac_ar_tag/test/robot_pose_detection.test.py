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

import rospy

import rostest
import unittest

import tf
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Point


def check_if_point_within_threshold(point, correct_point, threshold):
    return (
        abs(point.x - correct_point.x) < threshold.x
        and abs(point.y - correct_point.y) < threshold.y
        and abs(point.z - correct_point.z) < threshold.z
    )


def convert_param_to_points(param):
    """converts param (dictionary of x,y,z and roll,pitch,yaw) to
    position and orientation points
    """

    return Point(param["x"], param["y"], param["z"]), Point(
        param["roll"], param["pitch"], param["yaw"]
    )


class TestRobotDetection(unittest.TestCase):
    def test_object_detection(self):
        correct_robot_position, correct_robot_orientation = convert_param_to_points(
            rospy.get_param("~robot_pose")
        )

        listener = tf.TransformListener()

        while True:
            try:
                (translation, rotation) = listener.lookupTransform(
                    "camera_up_link", "base_link", rospy.Time(0)
                )
                break
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                continue

        rpy_robot = euler_from_quaternion(rotation)
        detected_robot_orientation = Point(rpy_robot[0], rpy_robot[1], rpy_robot[2])
        detected_robot_position = Point(translation[0], translation[1], translation[2])

        threshold_position = Point(0.005, 0.005, 0.005)
        threshold_orientation = Point(0.02, 0.02, 0.02)

        self.assertTrue(
            check_if_point_within_threshold(
                correct_robot_position,
                detected_robot_position,
                threshold_position,
            ),
            "Robot's position isn't close enough to expected value, expected: "
            + str(correct_robot_position)
            + ", got: "
            + str(detected_robot_position),
        )

        self.assertTrue(
            check_if_point_within_threshold(
                correct_robot_orientation,
                detected_robot_orientation,
                threshold_orientation,
            ),
            "Robot's orientation isn't close enough to expected value, expected: "
            + str(correct_robot_orientation)
            + ", got: "
            + str(detected_robot_orientation),
        )


if __name__ == "__main__":
    NAME = "test_robot_pose_detection"
    PKG = "roomac_ar_tag"

    rospy.init_node(NAME, anonymous=True)
    rostest.rosrun(PKG, NAME, TestRobotDetection)
