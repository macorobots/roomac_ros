#!/usr/bin/env python 

import rospy
import tf
import tf2_ros
import geometry_msgs


if __name__=="__main__":
    rospy.init_node("camera_rpy_convertion")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    brodcaster = tf2_ros.TransformBroadcaster()
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        transl = (-0.11808065176, -0.158871341228, 0.93005715847)
        rot = (-0.788540327521, 0.290080806607, -0.1819478093, 0.510834417531)
        transform = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(transl), tf.transformations.quaternion_matrix(rot))
        inversed_transform = tf.transformations.inverse_matrix(transform)
        translation = tf.transformations.translation_from_matrix(inversed_transform)
        q = tf.transformations.quaternion_from_matrix(inversed_transform)

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "camera_rgb_optical_frame"
        t.child_frame_id = "base_link"
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        brodcaster.sendTransform(t)

        try:
            trans = tfBuffer.lookup_transform('base_link', 'camera_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            rospy.logwarn("No transform")
            continue
            
        quat = trans.transform.rotation
        translation = trans.transform.translation
        quaternion = (quat.x, quat.y, quat.z, quat.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        rospy.loginfo("rpy= " + str(euler[0]) + ' ' + str(euler[1])+ ' ' + str(euler[2]))
        rospy.loginfo("xyz= " + str(translation.x) + ' ' + str(translation.y)+ ' ' + str(translation.z))
        break

    