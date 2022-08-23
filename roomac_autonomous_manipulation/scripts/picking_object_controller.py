#!/usr/bin/env python

import copy

import rospy

from geometry_msgs.msg import PointStamped, Point

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
        self.moveit_manager = picking_object_manager_moveit.PickingObjectManagerMoveIt()

        self.dynamic_reconfigure_srv = Server(
            PickManagerSettingsConfig, self.dynamic_reconfigure_cb
        )
        # Parameters
        self.object_position_correction_x = rospy.get_param(
            "~object_position_correction_x", 0.0
        )
        self.object_position_correction_y = rospy.get_param(
            "~object_position_correction_y", 0.0
        )
        self.object_position_correction_z = rospy.get_param(
            "~object_position_correction_z", 0.0
        )
        self.pre_goal_retraction_x = rospy.get_param("~pre_goal_retraction_x", 0.1)
        self.post_goal_retraction_z = rospy.get_param("~post_goal_retraction_z", 0.08)
        self.open_gripper_wait_time = rospy.get_param("~open_gripper_wait_time", 0.0)
        self.close_gripper_wait_time = rospy.get_param("~close_gripper_wait_time", 1.0)

        self.base_link_frame = rospy.get_param("~base_link_frame", "base_link")
        self.gripper_frame = rospy.get_param("~gripper_frame", "gripping_right_link")

        self.bottle_cap_height = rospy.get_param("~bottle_cap_height", 0.06)
        self.bottle_cap_radius = rospy.get_param("~bottle_cap_radius", 0.025)
        self.bottle_height = rospy.get_param("~bottle_height", 0.1)
        self.bottle_radius_pre_picking = rospy.get_param(
            "~bottle_radius_pre_picking", 0.04
        )
        self.bottle_radius_post_picking = rospy.get_param(
            "~bottle_radius_post_picking", 0.04
        )

        self.opened_gripper_value = rospy.get_param("~opened_gripper_value", 0.0)
        self.closed_gripper_value = rospy.get_param("~closed_gripper_value", 0.02)

        self.current_object_point = None

        self.bottle_name = "bottle"
        self.bottle_cap_name = "bottle_cap"

    def go_to_current_pre_object_point(self):
        current_pre_object_point = copy.deepcopy(self.current_object_point)
        current_pre_object_point.y -= self.pre_goal_retraction_x
        self.moveit_manager.go_to_point(current_pre_object_point, self.gripper_frame)

    def return_to_home_position(self):
        self.moveit_manager.return_to_home_position()

    def remove_object_from_scene(self):
        self.moveit_manager.remove_object_from_scene(
            self.bottle_name, self.gripper_frame
        )
        self.moveit_manager.remove_object_from_scene(
            self.bottle_cap_name, self.gripper_frame
        )

    def get_base_link_frame(self):
        return self.base_link_frame

    def add_current_object_to_scene(self):
        object_point = copy.deepcopy(self.current_object_point)

        # Remove leftover objects from a previous run
        self.remove_object_from_scene()

        self.moveit_manager.add_cylinder_to_scene(
            self.base_link_frame,
            object_point,
            self.bottle_cap_name,
            self.bottle_cap_height,
            self.bottle_cap_radius,
        )

        # Lower part of the bottle, so moveit won't approach it from lower position
        # causing bottle to fall

        object_point.z -= (self.bottle_cap_height + self.bottle_height) / 2.0
        self.moveit_manager.add_cylinder_to_scene(
            self.base_link_frame,
            object_point,
            self.bottle_name,
            self.bottle_height,
            self.bottle_radius_pre_picking,
        )

    def set_current_object_point(self, object_point):
        object_point.x += self.object_position_correction_x
        object_point.y += self.object_position_correction_y
        object_point.z += self.object_position_correction_z
        self.current_object_point = object_point

    def open_gripper_with_delay(self):
        self.open_gripper(self.open_gripper_wait_time)

    def open_gripper(self, delay=1.0):
        rospy.loginfo("Sending open gripper command")
        self.moveit_manager.move_gripper(self.opened_gripper_value)

    def go_to_current_object_point(self):
        self.moveit_manager.go_to_point(self.current_object_point, self.gripper_frame)

    def moveit_finished_execution(self):
        goal_state = None
        state = self.moveit_manager.get_moveit_feedback_state()

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
        self.moveit_manager.moveit_abort()

    def print_current_error(self):
        self.print_error(self.current_object_point)

    def print_error(self, goal_point):
        tip_point = PointStamped()
        tip_point.header.stamp = rospy.Time(0)
        tip_point.header.frame_id = self.gripper_frame
        tip_point.point.x = 0.0
        tip_point.point.y = 0.0
        tip_point.point.z = 0.0

        gripper_point_transformed = utils.transform_point(
            tip_point, self.base_link_frame
        )

        diff = Point()
        diff.x = gripper_point_transformed.point.x - goal_point.x
        diff.y = gripper_point_transformed.point.y - goal_point.y
        diff.z = gripper_point_transformed.point.z - goal_point.z

        rospy.logwarn("Planner error: " + str(diff))

    def attach_object(self):
        self.moveit_manager.remove_object_from_scene(
            self.bottle_name, self.gripper_frame
        )
        object_point = copy.deepcopy(self.current_object_point)
        object_point.z -= (self.bottle_cap_height + self.bottle_height) / 2.0
        self.moveit_manager.add_cylinder_to_scene(
            self.base_link_frame,
            object_point,
            self.bottle_name,
            self.bottle_height,
            self.bottle_radius_post_picking,
        )

        self.moveit_manager.attach_object(self.bottle_name, self.gripper_frame)
        self.moveit_manager.attach_object(self.bottle_cap_name, self.gripper_frame)

    def close_gripper_with_delay(self):
        self.close_gripper(self.close_gripper_wait_time)

    def close_gripper(self, delay=1.0):
        rospy.loginfo("Sending close gripper command")
        self.moveit_manager.move_gripper(self.closed_gripper_value)

    def go_to_current_post_object_point(self):
        current_post_object_point = copy.deepcopy(self.current_object_point)
        current_post_object_point.z += self.post_goal_retraction_z
        self.moveit_manager.go_to_point(current_post_object_point, self.gripper_frame)

    def dynamic_reconfigure_cb(self, config, level):
        self.object_position_correction_x = config.object_position_correction_x
        self.object_position_correction_y = config.object_position_correction_y
        self.object_position_correction_z = config.object_position_correction_z
        self.pre_goal_retraction_x = config.pre_goal_retraction_x
        self.post_goal_retraction_z = config.post_goal_retraction_z
        self.open_gripper_wait_time = config.open_gripper_wait_time
        self.close_gripper_wait_time = config.close_gripper_wait_time
        self.procedure_retry_threshold = config.procedure_retry_threshold

        return config
