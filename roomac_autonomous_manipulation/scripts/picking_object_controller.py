#!/usr/bin/env python

import copy

import rospy

from geometry_msgs.msg import PointStamped, Point, PoseStamped

from actionlib_msgs.msg import GoalStatus

from dynamic_reconfigure.server import Server
from roomac_autonomous_manipulation.cfg import PickManagerSettingsConfig


from roomac_utils.action_procedure_executor import (
    GoalState,
)

import picking_object_manager_moveit
import utils


class PickingObjectController(object):
    def __init__(self):
        self._moveit_manager = (
            picking_object_manager_moveit.PickingObjectManagerMoveIt()
        )

        self._dynamic_reconfigure_srv = Server(
            PickManagerSettingsConfig, self._dynamic_reconfigure_cb
        )
        # Parameters
        self._object_position_correction_x = rospy.get_param(
            "~object_position_correction_x", 0.0
        )
        self._object_position_correction_y = rospy.get_param(
            "~object_position_correction_y", 0.0
        )
        self._object_position_correction_z = rospy.get_param(
            "~object_position_correction_z", 0.0
        )
        self._pre_goal_retraction_x = rospy.get_param("~pre_goal_retraction_x", 0.1)
        self._post_goal_retraction_z = rospy.get_param("~post_goal_retraction_z", 0.08)
        self._open_gripper_wait_time = rospy.get_param("~open_gripper_wait_time", 0.0)
        self._close_gripper_wait_time = rospy.get_param("~close_gripper_wait_time", 1.0)

        self._base_link_frame = rospy.get_param("~base_link_frame", "base_link")
        self._gripper_frame = rospy.get_param("~gripper_frame", "gripping_right_link")

        self._bottle_cap_height = rospy.get_param("~bottle_cap_height", 0.06)
        self._bottle_cap_radius = rospy.get_param("~bottle_cap_radius", 0.025)
        self._bottle_height = rospy.get_param("~bottle_height", 0.1)
        self._bottle_radius_pre_picking = rospy.get_param(
            "~bottle_radius_pre_picking", 0.04
        )
        self._bottle_radius_post_picking = rospy.get_param(
            "~bottle_radius_post_picking", 0.04
        )

        self._opened_gripper_value = rospy.get_param("~opened_gripper_value", 0.0)
        self._closed_gripper_value = rospy.get_param("~closed_gripper_value", 0.02)

        self._current_object_point = None

        self._bottle_pre_picking_name = "bottle_with_safety_margin"
        self._bottle_post_picking_name = "bottle"
        self._bottle_cap_name = "bottle_cap"
        self._table_name = "table"

        self._safety_margin_table = rospy.get_param("~safety_margin_table", 0.05)

    # ----- OTHER -----
    def set_current_object_point(self, object_point):
        object_point.x += self._object_position_correction_x
        object_point.y += self._object_position_correction_y
        object_point.z += self._object_position_correction_z
        self._current_object_point = object_point

    def get_base_link_frame(self):
        return self._base_link_frame

    def print_current_error(self):
        tip_point = PointStamped()
        tip_point.header.stamp = rospy.Time(0)
        tip_point.header.frame_id = self._gripper_frame
        tip_point.point.x = 0.0
        tip_point.point.y = 0.0
        tip_point.point.z = 0.0

        gripper_point_transformed = utils.transform_point(
            tip_point, self._base_link_frame
        )

        diff = Point()
        diff.x = gripper_point_transformed.point.x - self._current_object_point.x
        diff.y = gripper_point_transformed.point.y - self._current_object_point.y
        diff.z = gripper_point_transformed.point.z - self._current_object_point.z

        rospy.logwarn("Planner error: " + str(diff))

    # ----- MOVING ARM -----

    def open_gripper(self, delay=1.0):
        rospy.loginfo("Sending open gripper command")
        # todo: fix waiting
        # self._open_gripper_wait_time
        self._moveit_manager.move_gripper(self._opened_gripper_value)

    def close_gripper(self, delay=1.0):
        rospy.loginfo("Sending close gripper command")
        # todo: fix waiting
        # self._close_gripper_wait_time
        self._moveit_manager.move_gripper(self._closed_gripper_value)

    def go_to_current_object_point(self):
        self._moveit_manager.go_to_point(
            self._current_object_point, self._gripper_frame
        )

    def go_to_current_pre_object_point(self):
        current_pre_object_point = copy.deepcopy(self._current_object_point)
        current_pre_object_point.y -= self._pre_goal_retraction_x
        self._moveit_manager.go_to_point(current_pre_object_point, self._gripper_frame)

    def go_to_current_post_object_point(self):
        current_post_object_point = copy.deepcopy(self._current_object_point)
        current_post_object_point.z += self._post_goal_retraction_z
        self._moveit_manager.go_to_point(current_post_object_point, self._gripper_frame)

    def return_to_home_position(self):
        self._moveit_manager.return_to_home_position()

    # ----- SCENE MODIFICATION -----

    def add_detected_table_to_scene(self, object_and_table):
        # Remove leftover objects from a previous run
        self._moveit_manager.remove_table_from_scene(self._table_name)

        detected_table_min_point = (
            object_and_table.object_and_table.table.min_point_bounding_box
        )
        detected_table_max_point = (
            object_and_table.object_and_table.table.max_point_bounding_box
        )

        detected_table_body_pose = PoseStamped()
        detected_table_body_pose.header = object_and_table.object_and_table.header
        detected_table_body_pose.pose.position.x = (
            detected_table_max_point.x + detected_table_min_point.x
        ) / 2.0
        detected_table_body_pose.pose.position.y = (
            detected_table_max_point.y + detected_table_min_point.y
        ) / 2.0
        detected_table_body_pose.pose.position.z = (
            detected_table_max_point.z + detected_table_min_point.z
        ) / 2.0

        detected_table_body_base_size = [
            detected_table_max_point.x - detected_table_min_point.x,
            detected_table_max_point.y - detected_table_min_point.y,
            detected_table_max_point.z - detected_table_min_point.z,
        ]

        detected_table_body_size = [
            detected_table_body_base_size[0] + 2 * self._safety_margin_table,
            detected_table_body_base_size[1] + 2 * self._safety_margin_table,
            detected_table_body_base_size[2] + 2 * self._safety_margin_table,
        ]

        self._moveit_manager.add_box_to_scene(
            self._table_name, detected_table_body_pose, detected_table_body_size
        )

    def remove_object_from_scene(self):
        self._moveit_manager.remove_object_from_scene(
            self._bottle_pre_picking_name, self._gripper_frame
        )
        self._moveit_manager.remove_object_from_scene(
            self._bottle_post_picking_name, self._gripper_frame
        )
        self._moveit_manager.remove_object_from_scene(
            self._bottle_cap_name, self._gripper_frame
        )

    def add_current_object_to_scene(self):
        object_point = copy.deepcopy(self._current_object_point)

        # Remove leftover objects from a previous run
        self.remove_object_from_scene()

        self._moveit_manager.add_cylinder_to_scene(
            self._base_link_frame,
            object_point,
            self._bottle_cap_name,
            self._bottle_cap_height,
            self._bottle_cap_radius,
        )

        # Lower part of the bottle, so moveit won't approach it from lower position
        # causing bottle to fall

        object_point.z -= (self._bottle_cap_height + self._bottle_height) / 2.0
        self._moveit_manager.add_cylinder_to_scene(
            self._base_link_frame,
            object_point,
            self._bottle_pre_picking_name,
            self._bottle_height,
            self._bottle_radius_pre_picking,
        )

    def attach_object(self):
        object_point = copy.deepcopy(self._current_object_point)
        object_point.z -= (self._bottle_cap_height + self._bottle_height) / 2.0
        self._moveit_manager.add_cylinder_to_scene(
            self._base_link_frame,
            object_point,
            self._bottle_post_picking_name,
            self._bottle_height,
            self._bottle_radius_post_picking,
        )
        self._moveit_manager.remove_object_from_scene(
            self._bottle_pre_picking_name, self._gripper_frame
        )

        self._moveit_manager.attach_object(self._bottle_cap_name, self._gripper_frame)
        self._moveit_manager.attach_object(
            self._bottle_post_picking_name, self._gripper_frame
        )

    # ----- MOVEIT EXECUTION -----

    def moveit_finished_execution(self):
        goal_state = None
        state = self._moveit_manager.get_moveit_feedback_state()

        if not state:
            return GoalState.IN_PROGRESS

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

    def moveit_abort(self):
        self._moveit_manager.moveit_abort()

    # ----- UTILITY PRIVATE FUNCTIONS -----

    def _dynamic_reconfigure_cb(self, config, level):
        self._object_position_correction_x = config.object_position_correction_x
        self._object_position_correction_y = config.object_position_correction_y
        self._object_position_correction_z = config.object_position_correction_z
        self._pre_goal_retraction_x = config.pre_goal_retraction_x
        self._post_goal_retraction_z = config.post_goal_retraction_z
        self._open_gripper_wait_time = config.open_gripper_wait_time
        self._close_gripper_wait_time = config.close_gripper_wait_time
        self.procedure_retry_threshold = config.procedure_retry_threshold

        return config
