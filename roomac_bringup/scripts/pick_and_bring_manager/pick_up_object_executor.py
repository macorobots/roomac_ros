#!/usr/bin/env python

#  Software License Agreement
#
#  Copyright (C) 2022, Maciej Stepien, All rights reserved.
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
#  Authors: Maciej Stepien

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
