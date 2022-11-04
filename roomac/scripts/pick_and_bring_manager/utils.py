#!/usr/bin/env python

import rospy

import tf2_ros
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Pose2D


def get_latest_position_from_transform(target_frame, source_frame, tf_buffer):
    """Gets the most recent transform from source_frame to target_frame and converts it
    to Pose2D msg using x,y and yaw

    Raises:
        RuntimeError: when transform doesn't exist
    """

    try:
        current_transform = tf_buffer.lookup_transform(
            target_frame, source_frame, rospy.Time(0)
        )
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ):
        raise RuntimeError(
            "Couldn't get " + target_frame + "->" + source_frame + " transform"
        )

    current_position = Pose2D()
    current_position.x = current_transform.transform.translation.x
    current_position.y = current_transform.transform.translation.y
    quaternion = [
        current_transform.transform.rotation.x,
        current_transform.transform.rotation.y,
        current_transform.transform.rotation.z,
        current_transform.transform.rotation.w,
    ]

    eulerAngles = euler_from_quaternion(quaternion)
    current_position.theta = eulerAngles[2]

    return current_position
