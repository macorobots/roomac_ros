#!/usr/bin/env python

import rospy

import tf2_ros
from tf.transformations import quaternion_from_euler

import actionlib
from actionlib_msgs.msg import GoalStatus

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from roomac_utils.action_procedure_executor import (
    GoalState,
)

from destination_manager import DestinationManager
from utils import get_latest_position_from_transform


class NavigationExecutor:
    """Provides interface to interact with move_base action (compatible with
    action executor). It also stores named destinations in DestinationManager
    """

    def __init__(self):
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer)

        position_file = rospy.get_param(
            "~positions_file", "/home/roomac/roomac_data/positions.yaml"
        )
        self._destination_manager = DestinationManager(position_file)

        self._move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )

        rospy.loginfo("Waiting for move base client to become available")
        self._move_base_client.wait_for_server()
        rospy.loginfo("Move base client available")

    def go_to_position(self, position_name):
        try:
            self._send_move_base_goal(self._destination_manager[position_name])
        except KeyError as e:
            rospy.logerr(str(e) + " position doesn't exist")
            return False
        return True

    def check_positions(self, position_names):
        try:
            for p in position_names:
                self._destination_manager[p]
        except KeyError as e:
            rospy.logerr(str(e) + " position doesn't exist")
            return GoalState.FAILED

        return GoalState.SUCCEEDED

    def save_current_robot_position(self, position_name):
        try:
            current_position = self._get_current_robot_position()
        except:
            return False

        self._destination_manager[position_name] = current_position
        self._destination_manager.save_positions_to_file()
        return True

    def check_move_base_state(self):
        goal_state = None

        state = self._move_base_client.get_state()
        if (
            state == GoalStatus.PREEMPTED
            or state == GoalStatus.ABORTED
            or state == GoalStatus.REJECTED
            or state == GoalStatus.RECALLED
            or state == GoalStatus.LOST
        ):
            goal_state = GoalState.FAILED
        elif state == GoalStatus.SUCCEEDED:
            goal_state = GoalState.SUCCEEDED

        else:
            goal_state = GoalState.IN_PROGRESS

        return goal_state

    def move_base_abort(self):
        self._move_base_client.cancel_all_goals()

    def _get_current_robot_position(self):
        return get_latest_position_from_transform("map", "base_link", self._tf_buffer)

    def _send_move_base_goal(self, goal_pose):
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = goal_pose.x
        goal.target_pose.pose.position.y = goal_pose.y
        goal.target_pose.pose.position.z = 0.0

        quat = quaternion_from_euler(0.0, 0.0, goal_pose.theta)
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        self._move_base_client.send_goal(goal)
