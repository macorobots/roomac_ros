#!/usr/bin/env python

#  Software License Agreement
#
#  Copyright (C) 2022, Maciej Stępień, All rights reserved.
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
#  Authors: Maciej Stępień

import numpy as np

import rospy

import tf

if __name__ == "__main__":
    rospy.init_node("calc_detection_accuracy")

    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)

    trans_x = []
    trans_y = []
    trans_z = []

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                "base_link",
                "detected_object",
                rospy.Time(0),
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue

        trans_x.append(trans[0])
        trans_y.append(trans[1])
        trans_z.append(trans[2])

        rospy.loginfo(
            "X mean: " + str(np.mean(trans_x)) + " std_dev: " + str(np.std(trans_x))
        )
        rospy.loginfo(
            "Y mean: " + str(np.mean(trans_y)) + " std_dev: " + str(np.std(trans_y))
        )
        rospy.loginfo(
            "Z mean: " + str(np.mean(trans_z)) + " std_dev: " + str(np.std(trans_z))
        )

        rospy.loginfo("X max diff: " + str(np.max(trans_x) - np.min(trans_x)))
        rospy.loginfo("Y max diff: " + str(np.max(trans_y) - np.min(trans_y)))
        rospy.loginfo("Z max diff: " + str(np.max(trans_z) - np.min(trans_z)))

        rate.sleep()
