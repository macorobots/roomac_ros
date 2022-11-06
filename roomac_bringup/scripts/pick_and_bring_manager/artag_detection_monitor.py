#!/usr/bin/env python

import math

import rospy

import tf2_ros

from roomac_utils.action_procedure_executor import (
    GoalState,
)

from pick_and_bring_manager.utils import get_latest_position_from_transform


class ArTagDetectionMonitor:
    """Checks if detected robot position (transform from camera_up_link
    to artag_bundle_link) is stable - consecutive detections have to stay within
    (x, y, yaw) threshold for minimum of time threshold duration"""

    def __init__(self):
        self._artag_stable_position_threshold = rospy.get_param(
            "~artag_stable_position_threshold", 0.04
        )
        self._artag_stable_orientation_threshold = (
            rospy.get_param("~artag_stable_orientation_threshold", 5.0)
            / 180.0
            * math.pi
        )
        self._artag_stable_time_threshold = rospy.Duration(
            rospy.get_param("~artag_stable_time_threshold", 0.5)
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer)

        self._last_artag_position = None
        self._stable_artag_position_start_time = None

    def check_if_artag_position_is_stable(self):
        try:
            artag_position = get_latest_position_from_transform(
                "camera_up_link", "artag_bundle_link", self._tf_buffer
            )
        except RuntimeError as e:
            rospy.logerr("Exception: " + str(e))
            return GoalState.IN_PROGRESS

        if not self._last_artag_position:
            self._last_artag_position = artag_position
            return GoalState.IN_PROGRESS

        translation_diff = math.sqrt(
            (artag_position.x - self._last_artag_position.x) ** 2
            + (artag_position.y - self._last_artag_position.y) ** 2
        )

        rotation_diff = math.fabs(
            artag_position.theta - self._last_artag_position.theta
        )

        if (
            translation_diff < self._artag_stable_position_threshold
            and rotation_diff < self._artag_stable_orientation_threshold
        ):
            if not self._stable_artag_position_start_time:
                self._stable_artag_position_start_time = rospy.Time.now()

            if (
                rospy.Time.now() - self._stable_artag_position_start_time
                > self._artag_stable_time_threshold
            ):
                self._stable_artag_position_start_time = None
                self._last_artag_position = None
                return GoalState.SUCCEEDED
        else:
            self._stable_artag_position_start_time = None

        self._last_artag_position = artag_position
        return GoalState.IN_PROGRESS
