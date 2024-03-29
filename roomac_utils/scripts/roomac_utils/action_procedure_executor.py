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

from enum import Enum

import actionlib
import rospy


class GoalState(Enum):
    IN_PROGRESS = 1
    SUCCEEDED = 2
    FAILED = 3


class ActionProcedureStep:
    def __init__(
        self,
        start_procedure_function,
        get_procedure_state_function,
        preempted_action_function,
        feedback_state,
    ):
        self.start_procedure_function = start_procedure_function
        self.get_procedure_state_function = get_procedure_state_function
        self.preempted_action_function = preempted_action_function
        self.feedback_state = feedback_state

    def start_procedure(self):
        self.start_procedure_function()

    def get_procedure_state(self):
        return self.get_procedure_state_function()

    def preempted_action(self):
        self.preempted_action_function()

    def get_feedback(self):
        return self.feedback_state


class SimpleActionExecutor:
    """Implements SimpleActionServer by executing defined list of ActionProcedureSteps"""

    def __init__(
        self,
        action_name,
        action_type,
        feedback_type,
        result_type,
        procedure_check_state_rate,
        procedure_list,
        procedure_retry_threshold=1,
    ):
        self.feedback = feedback_type()
        self.result = result_type()
        self.action_server = actionlib.SimpleActionServer(
            action_name,
            action_type,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.action_server.start()

        self.action_name = action_name

        self.procedure_retry_threshold = procedure_retry_threshold
        self.procedure_check_state_rate = procedure_check_state_rate

        self.procedure_list = procedure_list

    def execute_cb(self, goal):
        for procedure in self.procedure_list:
            procedure_retry_count = 0
            while procedure_retry_count < self.procedure_retry_threshold:
                self.feedback.status = procedure.get_feedback()
                self.action_server.publish_feedback(self.feedback)

                procedure.start_procedure()

                goal_state = GoalState.IN_PROGRESS

                while goal_state == GoalState.IN_PROGRESS:

                    if self.action_server.is_preempt_requested():
                        rospy.logwarn(self.action_name + " Preempted")
                        procedure.preempted_action()
                        self.result.success = False
                        self.action_server.set_preempted(self.result)
                        return

                    goal_state = procedure.get_procedure_state()
                    rospy.Rate(self.procedure_check_state_rate).sleep()

                if goal_state == GoalState.SUCCEEDED:
                    break

                procedure_retry_count += 1

            if goal_state == GoalState.FAILED:
                rospy.logerr(self.action_name + " Goal failed")
                self.result.success = False
                self.action_server.set_aborted(self.result)
                return

        rospy.loginfo(self.action_name + " Succeeded")

        self.result.success = True
        self.action_server.set_succeeded(self.result)
