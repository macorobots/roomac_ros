#!/usr/bin/env python  
import rospy
import tf
from tf import transformations as t
import math

if __name__ == '__main__':
    rospy.init_node('artag_tf_republisher')

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('ar_marker_0', 'camera_up', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        br.sendTransform(trans, rot, rospy.Time.now(), "camera_up", "artag_link_2")

        rate.sleep()