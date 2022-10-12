#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest, Empty

from roomac_msgs.msg import (
    PickObjectAction,
    PickObjectFeedback,
    PickObjectResult,
)
from roomac_msgs.srv import DetectObjectAndTable

from roomac_utils.action_procedure_executor import (
    ActionProcedureStep,
    SimpleActionExecutor,
    GoalState,
)

import utils

import picking_object_controller


class PickingObjectManager(object):
    def __init__(self):
        self._controller = picking_object_controller.PickingObjectController()

        # Instead of retries it would be probably better to change planning attempts in moveit
        # as they are main reason
        procedure_retry_threshold = rospy.get_param("~procedure_retry_threshold", 5)

        procedure_list = [
            # Prepare picking
            ActionProcedureStep(
                start_procedure_function=self._prepare_picking,
                get_procedure_state_function=lambda: GoalState.SUCCEEDED,
                preempted_action_function=lambda: None,
                feedback_state=PickObjectFeedback.PRE_GRIPPING_POSITION,
            ),
            # Open gripper
            ActionProcedureStep(
                start_procedure_function=self._controller.open_gripper,
                get_procedure_state_function=lambda: GoalState.SUCCEEDED,
                preempted_action_function=lambda: None,
                feedback_state=PickObjectFeedback.PRE_GRIPPING_POSITION,
            ),
            # # Go to pre point, now disabled as picking is faster withtout it and after
            # # recent improvements it isn't really necessary
            # ActionProcedureStep(
            #     start_procedure_function=self.go_to_current_pre_object_point,
            #     get_procedure_state_function=self.moveit_finished_execution,
            #     preempted_action_function=self.moveit_abort,
            #     feedback_state=PickObjectFeedback.GOING_TO_PRE_GRIPPING_POSITION,
            # ),
            # Go to object point
            ActionProcedureStep(
                start_procedure_function=self._controller.go_to_current_object_point,
                get_procedure_state_function=self._controller.moveit_finished_execution,
                preempted_action_function=self._controller.moveit_abort,
                feedback_state=PickObjectFeedback.GOING_TO_GRIPPING_POSITION,
            ),
            # Calculate error
            ActionProcedureStep(
                start_procedure_function=self._controller.print_current_error,
                get_procedure_state_function=lambda: GoalState.SUCCEEDED,
                preempted_action_function=lambda: None,
                feedback_state=PickObjectFeedback.GRIPPING_POSITION,
            ),
            # Attach object
            ActionProcedureStep(
                start_procedure_function=self._controller.attach_object,
                get_procedure_state_function=lambda: GoalState.SUCCEEDED,
                preempted_action_function=lambda: None,
                feedback_state=PickObjectFeedback.GRIPPING_POSITION,
            ),
            # Close gripper
            ActionProcedureStep(
                start_procedure_function=self._controller.close_gripper,
                get_procedure_state_function=lambda: GoalState.SUCCEEDED,
                preempted_action_function=lambda: None,
                feedback_state=PickObjectFeedback.CLOSING_GRIPPER,
            ),
            # Go to post point
            ActionProcedureStep(
                start_procedure_function=self._controller.go_to_current_post_object_point,
                get_procedure_state_function=self._controller.moveit_finished_execution,
                preempted_action_function=self._controller.moveit_abort,
                feedback_state=PickObjectFeedback.GOING_TO_POST_GRIPPING_POSITION,
            ),
            ActionProcedureStep(
                start_procedure_function=self._controller.rotate_wrist_post_point,
                get_procedure_state_function=self._controller.moveit_finished_execution,
                preempted_action_function=self._controller.moveit_abort,
                feedback_state=PickObjectFeedback.GOING_TO_POST_GRIPPING_POSITION,
            ),
            ActionProcedureStep(
                start_procedure_function=self._controller.go_to_cup_point,
                get_procedure_state_function=self._controller.moveit_finished_execution,
                preempted_action_function=self._controller.moveit_abort,
                feedback_state=PickObjectFeedback.GOING_TO_POST_GRIPPING_POSITION,
            ),
            ActionProcedureStep(
                start_procedure_function=self._controller.rotate_wrist_pour,
                get_procedure_state_function=self._controller.moveit_finished_execution,
                preempted_action_function=self._controller.moveit_abort,
                feedback_state=PickObjectFeedback.GOING_TO_POST_GRIPPING_POSITION,
            ),
        ]

        self._pick_action_executor = SimpleActionExecutor(
            "pick_object",
            PickObjectAction,
            PickObjectFeedback,
            PickObjectResult,
            10.0,
            procedure_list,
            procedure_retry_threshold,
        )

        self._open_gripper_srv = rospy.Service(
            "open_gripper", Trigger, self._open_gripper_cb
        )
        self._close_gripper_srv = rospy.Service(
            "close_gripper", Trigger, self._close_gripper_cb
        )

        self._home_arm_srv = rospy.Service(
            "home_arm", Trigger, self._return_to_home_position_cb
        )
        self._remove_object_from_scene_srv = rospy.Service(
            "remove_object_from_scene", Trigger, self._remove_object_from_scene_cb
        )

        self._clear_octomap_srv = rospy.ServiceProxy("/clear_octomap", Empty)
        self._detect_table_and_object_srv = rospy.ServiceProxy(
            "/detect_table_and_object", DetectObjectAndTable
        )

        self._clear_octomap_srv.wait_for_service()
        self._detect_table_and_object_srv.wait_for_service()

    def _prepare_picking(self):
        self._controller.remove_object_from_scene()

        object_point_stamped, object_and_table = self._get_detected_object_point()

        object_point_transformed = utils.transform_point(
            object_point_stamped, self._controller.get_base_link_frame()
        )

        self._controller.set_current_object_point(object_point_transformed.point)
        self._controller.add_current_object_to_scene()

        self._controller.add_detected_table_to_scene(object_and_table)

        # Clear octomap after adding objects to force clearing object points
        # This especially caused problems in simulation - object points weren't cleared and picking failed
        self._clear_octomap_srv.call()

    def _get_detected_object_point(self):
        object_and_table = self._detect_table_and_object_srv.call()

        object_point_stamped = PointStamped()
        object_point_stamped.header.frame_id = (
            object_and_table.object_and_table.header.frame_id
        )
        object_point_stamped.header.stamp = rospy.Time(0)
        object_point_stamped.point = (
            object_and_table.object_and_table.object.mass_center
        )

        return object_point_stamped, object_and_table

    def _return_to_home_position_cb(self, req):
        self._controller.return_to_home_position()
        return TriggerResponse(True, "")

    def _open_gripper_cb(self, req):
        self._controller.open_gripper()
        return TriggerResponse(True, "")

    def _close_gripper_cb(self, req):
        self._controller.close_gripper()
        return TriggerResponse(True, "")

    def _remove_object_from_scene_cb(self, req):
        self._controller.remove_object_from_scene()
        return TriggerResponse(True, "")


if __name__ == "__main__":
    rospy.init_node("picking_object_manager")
    manager = PickingObjectManager()
    rospy.spin()
