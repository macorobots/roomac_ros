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

import sys
import rosbag

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Pose2D
from tf.transformations import euler_from_quaternion

import math
import copy


def filter_rosbag(input_rosbag, output_rosbag):
    topics_to_save = [
        "/camera/depth_registered/camera_info",
        "/camera/depth_registered/image_raw",
        "/camera/rgb/camera_info",
        "/camera/rgb/image_rect_color",
        "/odom",
    ]

    save_odom_tf = False
    save_odom_topic = True

    current_time = 0
    last_time = 0

    last_position = Pose2D()
    current_position = Pose2D()

    with rosbag.Bag(output_rosbag, "w") as outbag:
        for topic, msg, t in rosbag.Bag(input_rosbag).read_messages():
            if topic == "/tf":
                if msg.transforms[0].header.frame_id != "map":
                    if save_odom_tf:
                        outbag.write(topic, msg, t)
                    else:
                        pass

                    if save_odom_topic:
                        odom_base_transform = msg.transforms[0]

                        current_time = odom_base_transform.header.stamp
                        if last_time == 0:
                            last_time = current_time

                        current_position.x = odom_base_transform.transform.translation.x
                        current_position.y = odom_base_transform.transform.translation.y
                        quaternion = [
                            odom_base_transform.transform.rotation.x,
                            odom_base_transform.transform.rotation.y,
                            odom_base_transform.transform.rotation.z,
                            odom_base_transform.transform.rotation.w,
                        ]

                        euler_angles = euler_from_quaternion(quaternion)
                        current_position.theta = euler_angles[2]

                        dt = (current_time - last_time).to_sec()
                        if dt > 0:
                            vx = (
                                math.sqrt(
                                    (current_position.x - last_position.x) ** 2
                                    + (current_position.y - last_position.y) ** 2
                                )
                                / dt
                            )
                            vth = (current_position.theta - last_position.theta) / dt
                        else:
                            vx = 0
                            vth = 0

                        last_position = copy.deepcopy(current_position)

                        odom_msg = Odometry()
                        odom_msg.header.stamp = current_time
                        odom_msg.header.frame_id = "odom"
                        odom_msg.child_frame_id = "base_link"
                        odom_msg.pose.pose = Pose(
                            Point(
                                odom_base_transform.transform.translation.x,
                                odom_base_transform.transform.translation.y,
                                odom_base_transform.transform.translation.z,
                            ),
                            Quaternion(
                                quaternion[0],
                                quaternion[1],
                                quaternion[2],
                                quaternion[3],
                            ),
                        )
                        odom_msg.twist.twist = Twist(
                            Vector3(vx, 0, 0), Vector3(0, 0, vth)
                        )

                        last_time = current_time

                        outbag.write("/odom", odom_msg, t)

            elif topic == "/android/imu":
                msg.header.frame_id = "imu_link"
                msg.header.stamp = t
                msg.orientation_covariance = list(msg.orientation_covariance)
                msg.orientation_covariance[0] = 0.001
                msg.orientation_covariance[4] = 0.001
                msg.orientation_covariance[8] = 0.001
                msg.orientation_covariance = tuple(msg.orientation_covariance)
                outbag.write(topic, msg, t)
            elif topic in topics_to_save:
                outbag.write(topic, msg, t)


if __name__ == "__main__":
    if len(sys.argv) == 3:
        filter_rosbag(sys.argv[1], sys.argv[2])
    elif len(sys.argv) == 2:
        output_rosbag = str(sys.argv[1]) + "_filtered"
        print("Output rosbag not specified. Defaulting to: " + output_rosbag)
        filter_rosbag(sys.argv[1], output_rosbag)
    else:
        print(
            "Invalid number of arguments. Usage: rosrun ... filter_rosbag.py inputRosbag [ouputRosbag]"
        )
