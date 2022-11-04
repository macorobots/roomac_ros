#!/usr/bin/env python

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
