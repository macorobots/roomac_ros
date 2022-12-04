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

import math

import rospy

from std_srvs.srv import Trigger, TriggerResponse

from roomac_msgs.msg import (
    PickAndBringAction,
    PickAndBringFeedback,
    PickAndBringResult,
)

from dynamic_reconfigure.server import Server
from roomac_bringup.cfg import PickAndBringManagerSettingsConfig

from roomac_utils.action_procedure_executor import (
    ActionProcedureStep,
    SimpleActionExecutor,
)

from pick_and_bring_manager.navigation_executor import NavigationExecutor
from pick_and_bring_manager.pick_up_object_executor import PickUpObjectExecutor
from pick_and_bring_manager.artag_detection_monitor import ArTagDetectionMonitor


class PickAndBringManager:
    """Provides pick_and_bring action - robot goes to table position, picks up object
    and then goes to home position. Additionally provides partial interfaces for
    interacting with move_base and destinations (go to home, table position, save home,
    table position)
    """

    def __init__(self):
        self._home_position_name = "home_position"
        self._table_position_name = "table_position"

        self._save_home_srv = rospy.Service(
            "save_home_position", Trigger, self._save_home_position_cb
        )
        self._save_table_srv = rospy.Service(
            "save_table_position", Trigger, self._save_table_position_cb
        )

        self._go_to_table_srv = rospy.Service(
            "go_to_table", Trigger, self._go_to_table_cb
        )
        self._go_to_home_srv = rospy.Service("go_to_home", Trigger, self._go_to_home_cb)

        self._navigation_executor = NavigationExecutor()
        self._pick_up_object_executor = PickUpObjectExecutor()
        self._artag_utils = ArTagDetectionMonitor()

        procedure_list = [
            ActionProcedureStep(
                start_procedure_function=lambda: None,
                get_procedure_state_function=self._check_home_and_table_positions,
                preempted_action_function=lambda: None,
                feedback_state=PickAndBringFeedback.DRIVING_TO_TABLE,
            ),
            ActionProcedureStep(
                start_procedure_function=self._go_to_table,
                get_procedure_state_function=self._navigation_executor.check_move_base_state,
                preempted_action_function=self._navigation_executor.move_base_abort,
                feedback_state=PickAndBringFeedback.DRIVING_TO_TABLE,
            ),
            ActionProcedureStep(
                start_procedure_function=lambda: None,
                get_procedure_state_function=self._artag_utils.check_if_artag_position_is_stable,
                preempted_action_function=lambda: None,
                feedback_state=PickAndBringFeedback.PICKING_UP_OBJECT,
            ),
            ActionProcedureStep(
                start_procedure_function=self._pick_up_object_executor.pick_object,
                get_procedure_state_function=self._pick_up_object_executor.check_pick_object_state,
                preempted_action_function=self._pick_up_object_executor.pick_object_abort,
                feedback_state=PickAndBringFeedback.PICKING_UP_OBJECT,
            ),
            ActionProcedureStep(
                start_procedure_function=self._go_to_home,
                get_procedure_state_function=self._navigation_executor.check_move_base_state,
                preempted_action_function=self._navigation_executor.move_base_abort,
                feedback_state=PickAndBringFeedback.DRIVING_TO_HOME_POSITION,
            ),
        ]

        self._pick_action_executor = SimpleActionExecutor(
            "pick_and_bring",
            PickAndBringAction,
            PickAndBringFeedback,
            PickAndBringResult,
            10.0,
            procedure_list,
        )

        self._dynamic_reconfigure_srv = Server(
            PickAndBringManagerSettingsConfig, self._dynamic_reconfigure_cb
        )

    def _go_to_table(self):
        return self._navigation_executor.go_to_position(self._table_position_name)

    def _go_to_home(self):
        return self._navigation_executor.go_to_position(self._home_position_name)

    def _check_home_and_table_positions(self):
        return self._navigation_executor.check_positions(
            [self._home_position_name, self._table_position_name]
        )

    def _go_to_table_cb(self, req):
        res = TriggerResponse()
        if self._go_to_table:
            res.success = True
        else:
            res.success = False
        return res

    def _go_to_home_cb(self, req):
        res = TriggerResponse()
        if self._go_to_home():
            res.success = True
        else:
            res.success = False
        return res

    def _save_home_position_cb(self, req):
        res = TriggerResponse()
        if self._navigation_executor.save_current_robot_position(
            self._home_position_name
        ):
            res.success = True
        else:
            res.success = False
        return res

    def _save_table_position_cb(self, req):
        res = TriggerResponse()
        if self._navigation_executor.save_current_robot_position(
            self._table_position_name
        ):
            res.success = True
        else:
            res.success = False
        return res

    def _dynamic_reconfigure_cb(self, config, level):
        self._artag_utils._artag_stable_position_threshold = (
            config.artag_stable_position_threshold
        )
        self._artag_utils._artag_stable_orientation_threshold = (
            config.artag_stable_orientation_threshold / 180.0 * math.pi
        )
        self._artag_utils._artag_stable_time_threshold = rospy.Duration(
            config.artag_stable_time_threshold
        )
        return config


if __name__ == "__main__":
    rospy.init_node("pick_and_bring_manager")
    controller = PickAndBringManager()
    rospy.spin()
