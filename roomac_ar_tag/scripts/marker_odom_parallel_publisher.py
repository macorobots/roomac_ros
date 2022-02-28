#!/usr/bin/env python
import math

import rospy

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from marker_odom_publisher import ARTagOdomPublisher


class ARTagParallerlOdomPublisher(ARTagOdomPublisher):
    def __init__(self):
        super(ARTagParallerlOdomPublisher, self).__init__()

        self.ar_marker_only_yaw_robot_link = self.ar_marker_robot_link + "_only_yaw"
        self.tf_broadcaster = tf.TransformBroadcaster()

    def send_only_yaw_transform(self, camera_link, ar_marker_link, target_link):
        """Reads current transform, assumes that marker plane is parallel to
        kinect and drops pitch and roll angles. Broadcasts new tf

        Args:
            camera_link (string): camera link name
            ar_marker_link (string): ar tag link name
            target_link (string): link name of the new tf

        Raises:
            RuntimeError: error when transform can't be read
        """
        try:
            (trans, rot_quat) = self.tf_listener.lookupTransform(
                camera_link, ar_marker_link, rospy.Time(0)
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            raise RuntimeError("Couldn't get transform")

        rot_rpy = euler_from_quaternion(rot_quat)
        # Addin small value (0.001) is neccessary to prevent errors:
        # TF_NAN_INPUT TF_DENORMALIZED_QUATERNION from robot localization
        rot_only_yaw = quaternion_from_euler(
            math.pi / 2 + 0.001, rot_rpy[1], -math.pi / 2 + 0.001
        )

        self.tf_broadcaster.sendTransform(
            trans,
            rot_only_yaw,
            rospy.Time.now(),
            target_link,
            camera_link,
        )

    def run(self):
        """Runs in the loop with a given rate, publishing
        odometry messages
        """
        while not rospy.is_shutdown():
            try:
                self.send_only_yaw_transform(
                    self.camera_link,
                    self.ar_marker_robot_link,
                    self.ar_marker_only_yaw_robot_link,
                )

                odom_robot_msg = self.calculate_odom_msg(
                    self.ar_marker_only_yaw_robot_link,
                    self.camera_link,
                    self.robot_link,
                    reversed=True,
                )

                odom_object_msg = self.calculate_odom_msg(
                    self.ar_marker_object_link,
                    self.camera_link,
                    self.object_link,
                    reversed=False,
                )
            except RuntimeError as e:
                rospy.logwarn("[Marker odom publisher] " + str(e))
                continue

            self.odom_robot_pub.publish(odom_robot_msg)
            self.odom_object_pub.publish(odom_object_msg)

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("artag_tf_republisher")
    odom_publisher = ARTagParallerlOdomPublisher()
    odom_publisher.run()
