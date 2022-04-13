#!/usr/bin/env python
import math

import rospy

import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import TransformStamped


class ARTagParallerlTransformPublisher:
    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~rate", 30.0))

        self.camera_link = rospy.get_param("~camera_link", "camera_up_link")
        self.ar_marker_robot_link = rospy.get_param(
            "~ar_marker_robot_link", "ar_marker_0"
        )
        self.ar_marker_object_link = rospy.get_param(
            "~ar_marker_object_link", "ar_marker_2"
        )
        parallel_transform_suffix = rospy.get_param(
            "~parallel_transform_suffix", "_only_yaw"
        )
        self.ar_marker_only_yaw_robot_link = (
            self.ar_marker_robot_link + parallel_transform_suffix
        )
        self.ar_marker_only_yaw_object_link = (
            self.ar_marker_object_link + parallel_transform_suffix
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def send_only_yaw_transform(self, camera_link, ar_marker_link, target_link):
        """Reads current transform (wait up to 5 seconds for it), assumes that
        marker plane is parallel to kinect and drops pitch and roll angles.
        Broadcasts new tf

        Args:
            camera_link (string): camera link name
            ar_marker_link (string): ar tag link name
            target_link (string): link name of the new tf

        Raises:
            RuntimeError: error when transform can't be read
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                camera_link, ar_marker_link, rospy.Time.now(), rospy.Duration(5.0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            raise RuntimeError(
                "Couldn't get " + camera_link + "->" + ar_marker_link + " transform"
            )

        rot_quat = [
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w,
        ]

        rot_rpy = euler_from_quaternion(rot_quat)
        # Addin small value (0.001) is neccessary to prevent errors:
        # TF_NAN_INPUT TF_DENORMALIZED_QUATERNION from robot localization
        rot_only_yaw = quaternion_from_euler(
            math.pi / 2 + 0.001, rot_rpy[1], -math.pi / 2 + 0.001
        )

        parallel_transform = TransformStamped()
        parallel_transform.header.stamp = rospy.Time.now()
        parallel_transform.header.frame_id = camera_link
        parallel_transform.child_frame_id = target_link
        parallel_transform.transform.translation = trans.transform.translation
        parallel_transform.transform.rotation.x = rot_only_yaw[0]
        parallel_transform.transform.rotation.y = rot_only_yaw[1]
        parallel_transform.transform.rotation.z = rot_only_yaw[2]
        parallel_transform.transform.rotation.w = rot_only_yaw[3]

        self.tf_broadcaster.sendTransform(parallel_transform)

    def run(self):
        """Runs in the loop with a given rate, publishing
        parallel transforms
        """
        while not rospy.is_shutdown():
            try:
                self.send_only_yaw_transform(
                    self.camera_link,
                    self.ar_marker_robot_link,
                    self.ar_marker_only_yaw_robot_link,
                )

                self.send_only_yaw_transform(
                    self.camera_link,
                    self.ar_marker_object_link,
                    self.ar_marker_only_yaw_object_link,
                )

            except RuntimeError as e:
                rospy.logwarn_throttle(
                    5, "[Marker parallel transform publisher] " + str(e)
                )
                continue

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("artag_parallel_transform_publisher")
    odom_publisher = ARTagParallerlTransformPublisher()
    odom_publisher.run()
