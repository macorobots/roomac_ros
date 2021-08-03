#!/usr/bin/env python

import sys
import rosbag

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Pose2D
from tf.transformations import euler_from_quaternion

import math
import copy


def filterRosbag(inputRosbag, outputRosbag):
    topics_to_save = [
        "/camera/depth_registered/camera_info",
        "/camera/depth_registered/image_raw",
        "/camera/rgb/camera_info",
        "/camera/rgb/image_rect_color",
        "/odom",
    ]

    save_odom_tf = False
    save_odom_topic = True

    currentTime = 0
    lastTime = 0

    lastPosition = Pose2D()
    currentPosition = Pose2D()

    with rosbag.Bag(outputRosbag, "w") as outbag:
        for topic, msg, t in rosbag.Bag(inputRosbag).read_messages():
            if topic == "/tf":
                if msg.transforms[0].header.frame_id != "map":
                    if save_odom_tf:
                        outbag.write(topic, msg, t)
                    else: 
                        pass

                    if save_odom_topic:
                        odomBaseTransform = msg.transforms[0]

                        currentTime = odomBaseTransform.header.stamp
                        if lastTime == 0:
                            lastTime = currentTime

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

                        lastTime = currentTime

                        outbag.write("/odom", odomMsg, t)

            elif topic == "/android/imu":
                msg.header.frame_id = "imu"
                msg.header.stamp = t
                msg.orientation_covariance  = list(msg.orientation_covariance)
                msg.orientation_covariance[0] = 0.001
                msg.orientation_covariance[4] = 0.001
                msg.orientation_covariance[8] = 0.001
                msg.orientation_covariance = tuple(msg.orientation_covariance)
                outbag.write(topic, msg, t)
            elif topic in topics_to_save:
                outbag.write(topic, msg, t)


if __name__ == "__main__":
    if len(sys.argv) == 3:
        filterRosbag(sys.argv[1], sys.argv[2])
    elif len(sys.argv) == 2:
        outputRosbag = str(sys.argv[1]) + "_filtered"
        print("Output rosbag not specified. Defaulting to: " + outputRosbag)
        filterRosbag(sys.argv[1], outputRosbag)
    else:
        print(
            "Invalid number of arguments. Usage: rosrun ... filter_rosbag.py inputRosbag [ouputRosbag]"
        )
