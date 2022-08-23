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
        self.controller = picking_object_controller.PickingObjectManager()

        # Instead of retries it would be probably better to change planning attempts in moveit
        # as they are main reason
        self.procedure_retry_threshold = rospy.get_param(
            "~procedure_retry_threshold", 5
        )

        procedure_list = [
            # Prepare picking
            ActionProcedureStep(
                start_procedure_function=self.prepare_picking,
                get_procedure_state_function=lambda: GoalState.SUCCEEDED,
                preempted_action_function=lambda: None,
                feedback_state=PickObjectFeedback.PRE_GRIPPING_POSITION,
            ),
            # Open gripper
            ActionProcedureStep(
                start_procedure_function=self.controller.open_gripper_with_delay,
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
                start_procedure_function=self.controller.go_to_current_object_point,
                get_procedure_state_function=self.controller.moveit_finished_execution,
                preempted_action_function=self.controller.moveit_abort,
                feedback_state=PickObjectFeedback.GOING_TO_GRIPPING_POSITION,
            ),
            # Calculate error
            ActionProcedureStep(
                start_procedure_function=self.controller.print_current_error,
                get_procedure_state_function=lambda: GoalState.SUCCEEDED,
                preempted_action_function=lambda: None,
                feedback_state=PickObjectFeedback.GRIPPING_POSITION,
            ),
            # Attach object
            ActionProcedureStep(
                start_procedure_function=self.controller.attach_object,
                get_procedure_state_function=lambda: GoalState.SUCCEEDED,
                preempted_action_function=lambda: None,
                feedback_state=PickObjectFeedback.GRIPPING_POSITION,
            ),
            # Close gripper
            ActionProcedureStep(
                start_procedure_function=self.controller.close_gripper_with_delay,
                get_procedure_state_function=lambda: GoalState.SUCCEEDED,
                preempted_action_function=lambda: None,
                feedback_state=PickObjectFeedback.CLOSING_GRIPPER,
            ),
            # Go to post point
            ActionProcedureStep(
                start_procedure_function=self.controller.go_to_current_post_object_point,
                get_procedure_state_function=self.controller.moveit_finished_execution,
                preempted_action_function=self.controller.moveit_abort,
                feedback_state=PickObjectFeedback.GOING_TO_POST_GRIPPING_POSITION,
            ),
        ]

        self.pick_action_executor = SimpleActionExecutor(
            "pick_object",
            PickObjectAction,
            PickObjectFeedback,
            PickObjectResult,
            10.0,
            procedure_list,
            self.procedure_retry_threshold,
        )

        self.open_gripper_srv = rospy.Service(
            "open_gripper", Trigger, self.open_gripper_cb
        )
        self.close_gripper_srv = rospy.Service(
            "close_gripper", Trigger, self.close_gripper_cb
        )

        self.set_home_arm_srv = rospy.Service(
            "set_home_arm", Trigger, self.return_to_home_position
        )
        self.remove_object_from_scene_srv = rospy.Service(
            "remove_object_from_scene", Trigger, self.remove_object_from_scene_cb
        )

        self.clear_octomap_srv = rospy.ServiceProxy("/clear_octomap", Empty)
        self.clear_octomap_srv.wait_for_service()

        self.add_detected_table_to_scene_srv = rospy.ServiceProxy(
            "/add_detected_table_to_scene", Trigger
        )
        self.add_detected_table_to_scene_srv.wait_for_service()

    def prepare_picking(self):
        self.controller.remove_object_from_scene()

        object_point_stamped = self.get_detected_object_point()

        object_point_transformed = utils.transform_point(
            object_point_stamped, self.controller.get_base_link_frame()
        )

        self.controller.set_current_object_point(object_point_transformed.point)
        self.controller.add_current_object_to_scene()

        # Clear octomap after adding objects to force clearing object points
        # This especially caused problems in simulation - object points weren't cleared and picking failed
        self.clear_octomap_srv.call()
        self.add_detected_table_to_scene_srv.call(TriggerRequest())

    def get_detected_object_point(self):
        detect_table_and_object_srv = rospy.ServiceProxy(
            "/detect_table_and_object", DetectObjectAndTable
        )
        detect_table_and_object_srv.wait_for_service()

        object_and_table = detect_table_and_object_srv.call()

        object_point_stamped = PointStamped()
        object_point_stamped.header.frame_id = (
            object_and_table.object_and_table.header.frame_id
        )
        object_point_stamped.header.stamp = rospy.Time(0)
        object_point_stamped.point = (
            object_and_table.object_and_table.object.mass_center
        )

        return object_point_stamped

    def return_to_home_position(self, req):
        self.controller.return_to_home_position()
        return TriggerResponse(True, "")

    def open_gripper_cb(self, req):
        self.controller.open_gripper()
        return TriggerResponse(True, "")

    def close_gripper_cb(self, req):
        self.controller.close_gripper()
        return TriggerResponse(True, "")

    def remove_object_from_scene_cb(self, req):
        self.controller.remove_object_from_scene()
        return TriggerResponse(True, "")


if __name__ == "__main__":
    rospy.init_node("picking_object_manager")
    manager = PickingObjectManager()
    rospy.spin()
