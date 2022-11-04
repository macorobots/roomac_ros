#!/usr/bin/env python

import math

import rospy

import tf2_ros

from std_srvs.srv import Trigger, TriggerResponse

from roomac_msgs.msg import (
    PickAndBringAction,
    PickAndBringFeedback,
    PickAndBringResult,
)

from dynamic_reconfigure.server import Server
from roomac.cfg import PickAndBringManagerSettingsConfig

from roomac_utils.action_procedure_executor import (
    ActionProcedureStep,
    SimpleActionExecutor,
    GoalState,
)

from pick_and_bring_manager.navigation_executor import NavigationExecutor
from pick_and_bring_manager.pick_up_object_executor import PickUpObjectExecutor
from pick_and_bring_manager.utils import get_latest_position_from_transform


class PickAndBringManager:
    """Provides pick_and_bring action - robot goes to table position, picks up object
    and then goes to home position. Additionally provides partial interfaces for
    interacting with move_base and destinations (go to home, table position, save home,
    table position)
    """

    def __init__(self):
        self._home_position_name = "home_position"
        self._table_position_name = "table_position"

        self._wait_time_before_picking_action = rospy.get_param(
            "~wait_time_before_picking_action", 5.0
        )
        self._artag_stable_position_threshold = rospy.get_param(
            "~artag_stable_position_threshold", 0.04
        )
        self._artag_stable_orientation_threshold = (
            rospy.get_param("~artag_stable_orientation_threshold", 5.0)
            / 180.0
            * math.pi
        )

        self._dynamic_reconfigure_srv = Server(
            PickAndBringManagerSettingsConfig, self._dynamic_reconfigure_cb
        )

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

        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer)

        self._navigation_executor = NavigationExecutor()
        self._pick_up_object_executor = PickUpObjectExecutor()

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
            # Doesn't work for bundle
            # ActionProcedureStep(
            #     start_procedure_function=lambda: None,
            #     get_procedure_state_function=self.check_if_artag_position_is_stable,
            #     preempted_action_function=lambda: None,
            #     feedback_state=PickAndBringFeedback.PICKING_UP_OBJECT,
            # ),
            # checking if artag is stable isn't working perfectly yet
            # so some delay is necessary before object position will be stable
            # and correct
            ActionProcedureStep(
                start_procedure_function=self._wait_before_picking_up_object,
                get_procedure_state_function=lambda: GoalState.SUCCEEDED,
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

    def _go_to_table(self):
        return self._navigation_executor.go_to_position(self._table_position_name)

    def _go_to_home(self):
        return self._navigation_executor.go_to_position(self._home_position_name)

    def _check_home_and_table_positions(self):
        return self._navigation_executor.check_positions(
            [self._home_position_name, self._table_position_name]
        )

    def _wait_before_picking_up_object(self):
        rospy.loginfo(
            "Waiting "
            + str(self._wait_time_before_picking_action)
            + " seconds before picking up object"
        )
        rospy.sleep(self._wait_time_before_picking_action)

    def _check_if_artag_position_is_stable(self):
        try:
            artag_pose = get_latest_position_from_transform(
                "camera_up_link", "ar_marker_8", self._tf_buffer
            )
            artag_pose_filtered = get_latest_position_from_transform(
                "camera_up_link", "artag_bundle_link", self._tf_buffer
            )
        except RuntimeError as e:
            rospy.logerr("Exception: " + str(e))
            return GoalState.IN_PROGRESS

        trans_diff = math.sqrt(
            (artag_pose.x - artag_pose_filtered.x) ** 2
            + (artag_pose.y - artag_pose_filtered.y) ** 2
        )

        rot_diff = math.fabs(artag_pose.theta - artag_pose_filtered.theta)

        if (
            trans_diff < self._artag_stable_position_threshold
            and rot_diff < self._artag_stable_orientation_threshold
        ):
            return GoalState.SUCCEEDED
        else:
            return GoalState.IN_PROGRESS

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
        self._wait_time_before_picking_action = config.wait_time_before_picking_action
        self._artag_stable_position_threshold = config.artag_stable_position_threshold
        self._artag_stable_orientation_threshold = (
            config.artag_stable_orientation_threshold / 180.0 * math.pi
        )
        return config


if __name__ == "__main__":
    rospy.init_node("pick_and_bring_manager")
    controller = PickAndBringManager()
    rospy.spin()
