#!/usr/bin/env python

import math

import rospy

import tf2_ros

from roomac_utils.action_procedure_executor import (
    GoalState,
)

from pick_and_bring_manager.utils import (
    get_latest_transform,
    get_distance_between_vectors,
    get_yaw_angle,
)


class ArTagDetectionMonitor:
    """Checks if detected robot position (transform from camera_up_link
    to artag_bundle_link) is stable - consecutive detections have to stay within
    (x, y, yaw) threshold for minimum of time threshold duration.
    Additionally z distance from camera has to be less than threshold."""

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
        self._artag_max_distance_from_camera = rospy.get_param(
            "~artag_max_distance_from_camera", 1.0
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer)

        self._last_artag_transform = None
        self._stable_artag_position_start_time = None

    def check_if_artag_position_is_stable(self):
        try:
            artag_transform = get_latest_transform(
                "camera_up_link", "artag_bundle_link", self._tf_buffer
            )
        except RuntimeError as e:
            rospy.logerr("Exception: " + str(e))
            return GoalState.IN_PROGRESS

        if not self._last_artag_transform:
            self._last_artag_transform = artag_transform
            return GoalState.IN_PROGRESS

        translation_diff = get_distance_between_vectors(
            artag_transform.transform.translation,
            self._last_artag_transform.transform.translation,
        )

        current_yaw = get_yaw_angle(artag_transform.transform.rotation)
        last_yaw = get_yaw_angle(self._last_artag_transform.transform.rotation)

        rotation_diff = math.fabs(current_yaw - last_yaw)

        if (
            translation_diff < self._artag_stable_position_threshold
            and rotation_diff < self._artag_stable_orientation_threshold
            and artag_transform.transform.translation.z
            < self._artag_max_distance_from_camera
        ):
            if not self._stable_artag_position_start_time:
                self._stable_artag_position_start_time = rospy.Time.now()

            if (
                rospy.Time.now() - self._stable_artag_position_start_time
                > self._artag_stable_time_threshold
            ):
                self._stable_artag_position_start_time = None
                self._last_artag_transform = None
                return GoalState.SUCCEEDED
        else:
            self._stable_artag_position_start_time = None

        self._last_artag_transform = artag_transform
        return GoalState.IN_PROGRESS
