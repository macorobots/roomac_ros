#!/usr/bin/env python
import rospy
import tf
from tf import transformations as t
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Pose2D


if __name__ == "__main__":
    rospy.init_node("artag_tf_republisher")

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                "camera_up_link", "ar_marker_8", rospy.Time(0)
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue

        rpy_robot = euler_from_quaternion(rot)
        rot_robot_only_yaw = quaternion_from_euler(
            math.pi / 2, -rpy_robot[1], -math.pi / 2
        )

        br.sendTransform(
            trans,
            rot_robot_only_yaw,
            rospy.Time.now(),
            "ar_marker_8_only_yaw",
            "camera_up_link",
        )

        try:
            (trans, rot) = listener.lookupTransform(
                "ar_marker_8_only_yaw", "camera_up_link", rospy.Time(0)
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue

        br.sendTransform(
            trans,
            rot,
            rospy.Time.now(),
            "camera_up_link",
            "artag_bundle_link",
        )

        rate.sleep()
