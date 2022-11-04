#!/usr/bin/env python

import rospy

import rostest
import unittest

from geometry_msgs.msg import Point

from roomac_msgs.srv import DetectObjectAndTable


def check_if_point_within_threshold(point, correct_point, threshold):
    return (
        abs(point.x - correct_point.x) < threshold.x
        and abs(point.y - correct_point.y) < threshold.y
        and abs(point.z - correct_point.z) < threshold.z
    )


def convert_param_to_point(param):
    return Point(param["x"], param["y"], param["z"])


class TestObjectDetection(unittest.TestCase):
    def test_object_detection(self):
        correct_object_point = convert_param_to_point(rospy.get_param("~object_point"))
        correct_object_min_point = convert_param_to_point(
            rospy.get_param("~object_min_point")
        )
        correct_object_max_point = convert_param_to_point(
            rospy.get_param("~object_max_point")
        )

        correct_table_point = convert_param_to_point(rospy.get_param("~table_point"))
        correct_table_min_point = convert_param_to_point(
            rospy.get_param("~table_min_point")
        )
        correct_table_max_point = convert_param_to_point(
            rospy.get_param("~table_max_point")
        )

        position_threshold = Point(0.002, 0.002, 0.002)
        bounding_points_threshold = Point(0.01, 0.01, 0.01)

        detect_table_and_object_srv = rospy.ServiceProxy(
            "/detect_table_and_object", DetectObjectAndTable
        )
        detect_table_and_object_srv.wait_for_service()
        object_and_table = detect_table_and_object_srv.call()

        test_cases = [
            (
                correct_object_point,
                object_and_table.object_and_table.object.mass_center,
                position_threshold,
                "Object position",
            ),
            (
                correct_object_min_point,
                object_and_table.object_and_table.object.min_point_bounding_box,
                bounding_points_threshold,
                "Object bounding box min point",
            ),
            (
                correct_object_max_point,
                object_and_table.object_and_table.object.max_point_bounding_box,
                bounding_points_threshold,
                "Object bounding box max point",
            ),
            (
                correct_table_point,
                object_and_table.object_and_table.table.mass_center,
                position_threshold,
                "Table position",
            ),
            (
                correct_table_min_point,
                object_and_table.object_and_table.table.min_point_bounding_box,
                bounding_points_threshold,
                "Table bounding box min point",
            ),
            (
                correct_table_max_point,
                object_and_table.object_and_table.table.max_point_bounding_box,
                bounding_points_threshold,
                "Table bounding box max point",
            ),
        ]

        for correct_point, point, threshold, name in test_cases:
            self.assertTrue(
                check_if_point_within_threshold(
                    correct_point,
                    point,
                    threshold,
                ),
                name
                + " isn't close enough to expected position, expected: "
                + str(correct_point)
                + ", got: "
                + str(point),
            )


if __name__ == "__main__":
    NAME = "test_object_detection"
    PKG = "roomac_autonomous_manipulation"

    rospy.init_node(NAME, anonymous=True)
    rostest.rosrun(PKG, NAME, TestObjectDetection)
