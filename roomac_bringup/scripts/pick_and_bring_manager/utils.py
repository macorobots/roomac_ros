#!/usr/bin/env python

import math

import rospy

import tf2_ros
from tf.transformations import euler_from_quaternion


def get_latest_transform(target_frame, source_frame, tf_buffer):
    """Returns the most recent transform from source_frame to target_frame

    Raises:
        RuntimeError: when transform doesn't exist
    """

    try:
        return tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ):
        raise RuntimeError(
            "Couldn't get " + target_frame + "->" + source_frame + " transform"
        )


def get_yaw_angle(quaternion):
    """Calculates yaw euler angle from given quaternion"""

    euler_angles = euler_from_quaternion(
        [
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        ]
    )
    return euler_angles[2]


def get_distance_between_vectors(vec1, vec2):
    return math.sqrt(
        (vec2.x - vec1.x) ** 2 + (vec2.y - vec1.y) ** 2 + (vec2.z - vec1.z) ** 2
    )
