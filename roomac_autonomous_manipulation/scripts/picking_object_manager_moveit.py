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

import sys

import rospy
import moveit_commander

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

from moveit_msgs.msg import (
    MoveGroupActionFeedback,
    AttachedCollisionObject,
    CollisionObject,
)
from actionlib_msgs.msg import GoalID

import utils


class PickingObjectManagerMoveIt:
    """This class wraps communication with MoveIt providing abstractions to interact
    with scene and robot arm - going to point, controlling gripper, ...
    """

    def __init__(self):
        wait_time_move_group_servers = rospy.get_param(
            "~wait_time_move_group_servers", 60.0
        )

        move_group_planning_time = rospy.get_param("~move_group_planning_time", 60.0)
        move_group_planning_attempts = rospy.get_param(
            "~move_group_planning_attempts", 10
        )

        move_group_goal_orientation_tolerance = rospy.get_param(
            "~move_group_goal_orientation_tolerance", 0.1
        )
        move_group_goal_position_tolerance = rospy.get_param(
            "~move_group_goal_position_tolerance", 0.01
        )

        arm_name = rospy.get_param("~arm_name", "right_arm")
        self._grasping_group_name = rospy.get_param("~grasping_group_name", "hand")
        self._home_position_target_name = rospy.get_param(
            "~home_position_target_name", "Home"
        )

        # Moveit stuff

        moveit_commander.roscpp_initialize(sys.argv)
        # when master is running remotely (e.g. on raspberry) communication is slower
        # and often timed out
        self._robot = moveit_commander.RobotCommander()
        self._move_group = moveit_commander.MoveGroupCommander(
            name=arm_name, wait_for_servers=wait_time_move_group_servers
        )
        # Using robot doesn't work due to longer connection times on wifi network
        self._move_group_gripper = moveit_commander.MoveGroupCommander(
            name=self._grasping_group_name,
            wait_for_servers=wait_time_move_group_servers,
        )
        self._scene = moveit_commander.PlanningSceneInterface()

        # Doesn't work, only LOST is published, maybe it tracks goals published by itself, not external
        # self.moveit_client = actionlib.SimpleActionClient("move_group", MoveGroupAction)
        # self.moveit_client.wait_for_server()

        self._moveit_feedback_sub = rospy.Subscriber(
            "/move_group/feedback", MoveGroupActionFeedback, self._moveit_feedback_cb
        )

        self._moveit_cancel_pub = rospy.Publisher(
            "/move_group/cancel", GoalID, queue_size=10
        )

        # If there are problems with finding plan, although it didn't look to work properly
        # (actual number of attempts was different)
        self._move_group.allow_replanning(True)
        self._move_group.set_planning_time(move_group_planning_time)
        self._move_group.set_num_planning_attempts(move_group_planning_attempts)

        # only position ik, really important without it fails to find plan
        self._move_group.set_goal_orientation_tolerance(
            move_group_goal_orientation_tolerance
        )
        self._move_group.set_goal_position_tolerance(move_group_goal_position_tolerance)

        self._moveit_feedback_state = None

    # ----- SCENE MODIFICATION -----

    def remove_object_from_scene(self, object_name, frame):
        self._scene.remove_attached_object(frame, name=object_name)
        self._scene.remove_world_object(object_name)

    def remove_table_from_scene(self, table_name):
        self._scene.remove_world_object(table_name)

    def add_cylinder_to_scene(self, frame, point, name, height, radius):
        cylinder_pose = PoseStamped()
        cylinder_pose.header.frame_id = frame
        cylinder_pose.pose.position.x = point.x
        cylinder_pose.pose.position.y = point.y
        cylinder_pose.pose.position.z = point.z

        self._scene.add_cylinder(
            name,
            cylinder_pose,
            height,
            radius,
        )

    def add_table_to_scene(
        self, table_name, detected_table_body_pose, detected_table_body_size
    ):
        self._scene.add_box(
            table_name, detected_table_body_pose, detected_table_body_size
        )

    def attach_object_to_gripper(self, object_name, link_name):
        touch_links = self._robot.get_link_names(group=self._grasping_group_name)

        aco = AttachedCollisionObject()
        aco.object = CollisionObject()
        aco.object.id = object_name
        aco.link_name = link_name
        aco.touch_links = touch_links
        self._scene.attach_object(aco)

    # ----- MOVEIT EXECUTION -----

    def get_moveit_feedback_state(self):
        return self._moveit_feedback_state

    def abort_moveit_execution(self):
        # not working
        # self.moveit_client.cancel_all_goals()

        # not working
        # self.move_group.stop()
        # self.move_group.clear_pose_targets()

        self._moveit_cancel_pub.publish(GoalID())

    # ----- MOVING ARM -----

    def move_gripper(self, position):
        self._move_group_gripper.go([position, -position], wait=True)
        self._move_group_gripper.stop()

    def return_to_home_position(self):
        self._move_group.set_named_target(self._home_position_target_name)
        self._find_and_execute_plan()

    def go_to_point(self, point, gripper_frame):
        self._move_group.set_start_state_to_current_state()

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = utils.get_perpendicular_orientation()
        pose_goal.position = point

        # self.move_group.set_pose_target(pose_goal)

        # Allow approximate solutions (True)
        self._move_group.set_joint_value_target(pose_goal, gripper_frame, True)
        self._find_and_execute_plan(wait=False)

    # ----- UTILITY PRIVATE FUNCTIONS -----

    def _find_and_execute_plan(self, wait=True):
        plan = False
        while not plan:
            rospy.loginfo("Trying to find and execute plan")
            plan = self._move_group.go(wait=wait)

        self._moveit_feedback_state = None

        if wait:
            self._move_group.stop()
            self._move_group.clear_pose_targets()

    def _moveit_feedback_cb(self, msg):
        self._moveit_feedback_state = msg.status.status
