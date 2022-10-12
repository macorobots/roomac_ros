#!/usr/bin/env python

import math

import rospy

from geometry_msgs.msg import Quaternion

import tf
from tf.transformations import quaternion_from_euler


def get_perpendicular_orientation():
    orientation = Quaternion()
    quat = quaternion_from_euler(-math.pi / 2, -math.pi / 2, math.pi)
    orientation.x = quat[0]
    orientation.y = quat[1]
    orientation.z = quat[2]
    orientation.w = quat[3]
    return orientation


def get_pitch_rotated_orientation(angle):
    orientation = Quaternion()
    quat = quaternion_from_euler(-math.pi / 2, -math.pi / 2 + angle, math.pi)
    orientation.x = quat[0]
    orientation.y = quat[1]
    orientation.z = quat[2]
    orientation.w = quat[3]
    return orientation


def transform_point(point, target_frame):
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            point_transformed = listener.transformPoint(target_frame, point)
            return point_transformed
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as err:
            rospy.logerr(err)
        rate.sleep()
