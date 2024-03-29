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

import rospy

import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose


class ARTagOdomPublisher:
    def __init__(self):
        self.translation_covariance = rospy.get_param("~translation_covariance", 10)

        # It looks like rotation is main source of error in artag detection
        # especially roll and pitch
        self.rotation_covariance = rospy.get_param("~rotation_covariance", 30)

        self.covariance_matrix = [0.0] * 36
        self.covariance_matrix[0] = self.translation_covariance
        self.covariance_matrix[7] = self.translation_covariance
        self.covariance_matrix[14] = self.translation_covariance
        self.covariance_matrix[21] = self.rotation_covariance
        self.covariance_matrix[28] = self.rotation_covariance
        self.covariance_matrix[35] = self.rotation_covariance

        self.rate = rospy.Rate(rospy.get_param("~rate", 30.0))

        self.camera_link = rospy.get_param("~camera_link", "camera_up_link")
        self.robot_link = rospy.get_param("~robot_link", "artag_bundle_link")
        self.object_link = rospy.get_param("~object_link", "detected_object")

        self.ar_marker_robot_link = rospy.get_param(
            "~ar_marker_robot_link", "ar_marker_0"
        )
        self.ar_marker_object_link = rospy.get_param(
            "~ar_marker_object_link", "ar_marker_2"
        )

        self.object_artag_enabled = rospy.get_param("~object_artag_enabled", True)

        upper_kinect_mounted_parellel = rospy.get_param(
            "~upper_kinect_mounted_parellel", True
        )

        if upper_kinect_mounted_parellel:
            parallel_transform_suffix = rospy.get_param(
                "~parallel_transform_suffix", "_only_yaw"
            )
            self.ar_marker_robot_link += parallel_transform_suffix
            self.ar_marker_object_link += parallel_transform_suffix

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.odom_robot_pub = rospy.Publisher(
            "odom_artag_robot", Odometry, queue_size=50
        )
        self.odom_object_pub = rospy.Publisher(
            "odom_artag_object", Odometry, queue_size=50
        )

    def calculate_odom_msg(
        self, ar_marker_link, camera_link, target_link, reversed=False
    ):
        """Calculates odometry message, waits up to 5 second to read tf

        Args:
            ar_marker_link (string): ar marker link name
            camera_link (string): camer link name
            target_link (string): link name that will be used in odom msgs
            reversed (bool, optional): If True transform to camera_link
            will be published (useful for ar marker mounted on robot).
            Otherwise transform will be from camera. Defaults to False.

        Raises:
            RuntimeError: error when transform can't be read

        Returns:
            Odometry: odometry msg
        """
        if reversed:
            source_frame = camera_link
            target_frame = ar_marker_link
            header_parent_link = target_link
            header_child_link = camera_link
        else:
            source_frame = ar_marker_link
            target_frame = camera_link
            header_parent_link = camera_link
            header_child_link = target_link

        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rospy.Time.now(), rospy.Duration(5.0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            raise RuntimeError(
                "Couldn't get " + source_frame + "->" + target_frame + " transform"
            )

        pt = Point(
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z,
        )
        quat = trans.transform.rotation

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = header_parent_link
        odom_msg.child_frame_id = header_child_link
        odom_msg.pose.pose = Pose(pt, quat)

        odom_msg.pose.covariance = self.covariance_matrix

        return odom_msg

    def run(self):
        """Runs in the loop with a given rate, publishing
        odometry messages
        """
        while not rospy.is_shutdown():
            try:
                odom_robot_msg = self.calculate_odom_msg(
                    self.ar_marker_robot_link,
                    self.camera_link,
                    self.robot_link,
                    reversed=True,
                )
                self.odom_robot_pub.publish(odom_robot_msg)

                if self.object_artag_enabled:
                    odom_object_msg = self.calculate_odom_msg(
                        self.ar_marker_object_link,
                        self.camera_link,
                        self.object_link,
                        reversed=False,
                    )
                    self.odom_object_pub.publish(odom_object_msg)

            except RuntimeError as e:
                rospy.logwarn_throttle(5, "[Marker odom publisher] " + str(e))
                continue

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("artag_odom_publisher")
    odom_publisher = ARTagOdomPublisher()
    odom_publisher.run()
