#!/usr/bin/env python

import rospy

import actionlib
from actionlib_msgs.msg import GoalStatus

from roomac_msgs.msg import (
    PickObjectAction,
    PickObjectActionGoal,
)

from roomac_utils.action_procedure_executor import (
    GoalState,
)


class PickUpObjectExecutor:
    """Provides interface for pick_object action that is compatible with
    action executor
    """

    def __init__(self):
        self._pick_object_client = actionlib.SimpleActionClient(
            "pick_object", PickObjectAction
        )

        rospy.loginfo("Waiting for pick object client to become available")
        self._pick_object_client.wait_for_server()
        rospy.loginfo("Pick object client available")

    def pick_object(self):
        self._pick_object_client.send_goal(PickObjectActionGoal())

    def check_pick_object_state(self):
        goal_state = None

        state = self._pick_object_client.get_state()
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

    def pick_object_abort(self):
        self._pick_object_client.cancel_all_goals()
