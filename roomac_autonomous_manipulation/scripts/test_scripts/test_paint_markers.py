#!/usr/bin/env python
import rospy
import tf
from tf import transformations as t
from visualization_msgs.msg import Marker, MarkerArray


class PaintMarkers:
    def __init__(self):
        self.canvas_y = 0
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(10.0)
        self.marker_array = MarkerArray()
        self.tolerance = 0.01
        self.marker_pub = rospy.Publisher("paint_markers", MarkerArray, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform(
                    "base_link", "gripping_right_link", rospy.Time(0)
                )
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                continue

            if self.canvas_y - trans[1] < self.tolerance:
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.type = marker.CYLINDER
                marker.action = marker.ADD
                marker.scale.x = 0.01
                marker.scale.y = 0.01
                marker.scale.z = 0.001
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.pose.orientation.x = 0.7071068
                marker.pose.orientation.w = 0.7071068
                marker.pose.position.x = trans[0]
                marker.pose.position.y = trans[1]
                marker.pose.position.z = trans[2]

                self.marker_array.markers.append(marker)

            id = 0
            for m in self.marker_array.markers:
                m.id = id
                id += 1

            self.marker_pub.publish(self.marker_array)

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("paint_markers")
    pm = PaintMarkers()
    pm.run()
