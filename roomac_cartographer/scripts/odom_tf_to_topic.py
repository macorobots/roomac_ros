#!/usr/bin/env python

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Pose2D
from tf.transformations import euler_from_quaternion

import math
import copy

rospy.init_node('odometry_topic_publisher')
odomPub = rospy.Publisher("odom", Odometry, queue_size=50)

currentTime = rospy.Time.now()
lastTime = rospy.Time.now()

lastPosition = Pose2D()
currentPosition = Pose2D()

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

rate = rospy.Rate(20.0)

while not rospy.is_shutdown():
    try:
        odomBaseTransform = tfBuffer.lookup_transform('odom', 'base_link', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        continue

    # currentTime = odomBaseTransform.header.stamp
    # Sometimes timestamps were identical, which caused cartographer crash
    currentTime = rospy.Time.now()

    currentPosition.x = odomBaseTransform.transform.translation.x
    currentPosition.y = odomBaseTransform.transform.translation.y
    quaternion = [odomBaseTransform.transform.rotation.x, odomBaseTransform.transform.rotation.y, odomBaseTransform.transform.rotation.z, odomBaseTransform.transform.rotation.w]

    eulerAngles = euler_from_quaternion(quaternion)
    currentPosition.theta = eulerAngles[2]

    dt = (currentTime - lastTime).to_sec()
    if dt > 0:
        vx = math.sqrt((currentPosition.x - lastPosition.x)**2 + (currentPosition.y - lastPosition.y)**2)/dt
        vth = (currentPosition.theta - lastPosition.theta)/dt
    else:
        vx = 0
        vth = 0

    lastPosition = copy.deepcopy(currentPosition)

    odomMsg = Odometry()
    odomMsg.header.stamp = currentTime
    odomMsg.header.frame_id = "odom"
    odomMsg.child_frame_id = "base_link"
    odomMsg.pose.pose = Pose(Point(odomBaseTransform.transform.translation.x, odomBaseTransform.transform.translation.y, odomBaseTransform.transform.translation.z), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
    odomMsg.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, vth))
    odomPub.publish(odomMsg)

    lastTime = currentTime
    rate.sleep()