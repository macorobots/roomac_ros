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

# Checks TF between map and base_link and publishes it as robot_pose
# (for visualization in ROSMobile)

import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped

if __name__ == "__main__":
    rospy.init_node("pose_publisher")
    pose_pub = rospy.Publisher("robot_pose", PoseWithCovarianceStamped, queue_size=50)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            map_base_transform = tf_buffer.lookup_transform(
                "map", "base_link", rospy.Time(0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rate.sleep()
            continue

        current_pose = PoseWithCovarianceStamped()
        current_pose.header.frame_id = "map"
        current_pose.header.stamp = rospy.Time.now()

        current_pose.pose.pose.position.x = map_base_transform.transform.translation.x
        current_pose.pose.pose.position.y = map_base_transform.transform.translation.y
        current_pose.pose.pose.position.z = map_base_transform.transform.translation.z

        current_pose.pose.pose.orientation.x = map_base_transform.transform.rotation.x
        current_pose.pose.pose.orientation.y = map_base_transform.transform.rotation.y
        current_pose.pose.pose.orientation.z = map_base_transform.transform.rotation.z
        current_pose.pose.pose.orientation.w = map_base_transform.transform.rotation.w

        pose_pub.publish(current_pose)

        rate.sleep()
