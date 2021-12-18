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
            (trans,rot) = listener.lookupTransform('ar_marker_0', 'camera_up_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        # Check if object was also detected - if only second object is detected it can have ar_marker_0
        # name. It will be better to fix it, so given pattern can't have different name
        try:
            (trans2,rot2) = listener.lookupTransform('ar_marker_2', 'camera_up_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        br.sendTransform(trans, rot, rospy.Time.now(), "camera_up_link", "artag_link_2")

        rate.sleep()