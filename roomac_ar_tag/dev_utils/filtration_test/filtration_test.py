#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Pose2D
from random import randrange

from tf.transformations import quaternion_from_euler

rospy.init_node("odometry_topic_publisher")
odomPub = rospy.Publisher("odom", Odometry, queue_size=50)

rate = rospy.Rate(10.0)

while not rospy.is_shutdown():
    quat1 = quaternion_from_euler(0, 0, 0)
    quat2 = quaternion_from_euler(0.5, 0.5, 0.5)

    pt1 = Point(0.5, 0.5, 0.5)
    pt2 = Point(0.6, 0.6, 0.6)

    r = randrange(4)

    if r == 2:
        pt = pt1
    else:
        pt = pt2

    r = randrange(4)

    if r == 2:
        quaternion = quat1
    else:
        quaternion = quat2

    quat = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

    odomMsg = Odometry()
    odomMsg.header.stamp = rospy.Time.now()
    odomMsg.header.frame_id = "odom"
    odomMsg.child_frame_id = "base_link"
    odomMsg.pose.pose = Pose(pt, quat)

    odomMsg.pose.covariance[0] = 10
    odomMsg.pose.covariance[7] = 10
    odomMsg.pose.covariance[14] = 10
    odomMsg.pose.covariance[21] = 0.1
    odomMsg.pose.covariance[28] = 0.1
    odomMsg.pose.covariance[35] = 0.1

    odomPub.publish(odomMsg)
    rate.sleep()
