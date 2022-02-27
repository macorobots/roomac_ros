#!/usr/bin/env python
import rospy
import tf
from tf import transformations as t
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Pose2D
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math


if __name__ == "__main__":
    rospy.init_node("artag_tf_republisher")

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    odom_robot_pub = rospy.Publisher("odom_artag_robot", Odometry, queue_size=50)
    odom_object_pub = rospy.Publisher("odom_artag_object", Odometry, queue_size=50)

    translation_covariance = 10
    # It looks like rotation is main source of error in artag detection
    # especially roll and pitch
    rotation_covariance = 30

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        try:
            (trans_marker_0, rot_marker_0) = listener.lookupTransform(
                "camera_up_link", "ar_marker_0", rospy.Time(0)
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue

        rpy_marker_0 = euler_from_quaternion(rot_marker_0)
        # Addin small value (0.001) is neccessary to prevent errors:
        # TF_NAN_INPUT TF_DENORMALIZED_QUATERNION from robot localization
        rot_marker_0_only_yaw = quaternion_from_euler(
            math.pi / 2 + 0.001, rpy_marker_0[1], -math.pi / 2 + 0.001
        )

        br.sendTransform(
            trans_marker_0,
            rot_marker_0_only_yaw,
            rospy.Time.now(),
            "ar_marker_0_only_yaw",
            "camera_up_link",
        )

        try:
            (trans_robot, rot_robot) = listener.lookupTransform(
                "ar_marker_0_only_yaw", "camera_up_link", rospy.Time(0)
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue

        pt_robot = Point(trans_robot[0], trans_robot[1], trans_robot[2])
        quat_robot = Quaternion(rot_robot[0], rot_robot[1], rot_robot[2], rot_robot[3])

        odom_robot_msg = Odometry()
        odom_robot_msg.header.stamp = rospy.Time.now()
        odom_robot_msg.header.frame_id = "artag_link_2"
        odom_robot_msg.child_frame_id = "camera_up_link"
        odom_robot_msg.pose.pose = Pose(pt_robot, quat_robot)

        odom_robot_msg.pose.covariance[0] = translation_covariance
        odom_robot_msg.pose.covariance[7] = translation_covariance
        odom_robot_msg.pose.covariance[14] = translation_covariance
        odom_robot_msg.pose.covariance[21] = rotation_covariance
        odom_robot_msg.pose.covariance[28] = rotation_covariance
        odom_robot_msg.pose.covariance[35] = rotation_covariance

        try:
            (trans_object, rot_object) = listener.lookupTransform(
                "camera_up_link", "ar_marker_2", rospy.Time(0)
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue

        pt_object = Point(trans_object[0], trans_object[1], trans_object[2])
        quat_object = Quaternion(
            rot_object[0], rot_object[1], rot_object[2], rot_object[3]
        )

        odom_object_msg = Odometry()
        odom_object_msg.header.stamp = rospy.Time.now()
        odom_object_msg.header.frame_id = "camera_up_link"
        odom_object_msg.child_frame_id = "detected_object"
        odom_object_msg.pose.pose = Pose(pt_object, quat_object)

        odom_object_msg.pose.covariance[0] = translation_covariance
        odom_object_msg.pose.covariance[7] = translation_covariance
        odom_object_msg.pose.covariance[14] = translation_covariance
        odom_object_msg.pose.covariance[21] = rotation_covariance
        odom_object_msg.pose.covariance[28] = rotation_covariance
        odom_object_msg.pose.covariance[35] = rotation_covariance

        odom_robot_pub.publish(odom_robot_msg)
        odom_object_pub.publish(odom_object_msg)

        rate.sleep()
