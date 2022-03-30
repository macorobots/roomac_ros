#!/usr/bin/env python

import sys
import copy

import rospy
import moveit_commander

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion, Point
from std_srvs.srv import Trigger, TriggerResponse

import tf
from tf.transformations import quaternion_from_euler

from moveit_msgs.msg import MoveGroupActionFeedback
from actionlib_msgs.msg import GoalID
import actionlib
from roomac_msgs.msg import (
    PickObjectAction,
    PickObjectFeedback,
    PickObjectResult,
)
from actionlib_msgs.msg import GoalStatus

from enum import Enum


class GoalState(Enum):
    IN_PROGRESS = 1
    SUCCEEDED = 2
    FAILED = 3


class ActionProcedureStep:
    def __init__(
        self,
        start_procedure_function,
        start_procedure_function_arguments,
        get_procedure_state_function,
        preempted_action_function,
        feedback_state,
    ):
        self.start_procedure_function = start_procedure_function
        self.start_procedure_function_arguments = start_procedure_function_arguments
        self.get_procedure_state_function = get_procedure_state_function
        self.preempted_action_function = preempted_action_function
        self.feedback_state = feedback_state

    def start_procedure(self):
        # TODO: check for no arguments
        self.start_procedure_function(*self.start_procedure_function_arguments)

    def get_procedure_state(self):
        return self.get_procedure_state_function()

    def preempted_action(self):
        self.preempted_action_function()

    def get_feedback(self):
        return self.feedback_state


class PickingObjectManager(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        # when master is running remotely (e.g. on raspberry) communication is slower
        # and often timed out
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(
            name="right_arm", wait_for_servers=60.0
        )
        self.scene = moveit_commander.PlanningSceneInterface()

        # Doesn't work, only LOST is published, maybe it tracks goals published by itself, not external
        # self.moveit_client = actionlib.SimpleActionClient("move_group", MoveGroupAction)
        # self.moveit_client.wait_for_server()

        self.moveit_feedback_sub = rospy.Subscriber(
            "/move_group/feedback", MoveGroupActionFeedback, self.moveit_feedback_cb
        )

        self.moveit_cancel_pub = rospy.Publisher(
            "/move_group/cancel", GoalID, queue_size=10
        )

        # If there are problems with finding plan, although it didn't look to work properly
        # (actual number of attempts was different)
        self.move_group.allow_replanning(True)
        self.move_group.set_planning_time(60)
        self.move_group.set_num_planning_attempts(10)

        # only position ik, really important without it fails to find plan
        self.move_group.set_goal_orientation_tolerance(0.1)

        self.move_group.set_goal_position_tolerance(0.01)

        self.retraction = 0.1

        self.set_home_arm_srv = rospy.Service(
            "set_home_arm", Trigger, self.return_to_home_position
        )

        self.feedback = PickObjectFeedback()
        self.result = PickObjectResult()
        self.pick_object_action = actionlib.SimpleActionServer(
            "pick_object",
            PickObjectAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.pick_object_action.start()

        self.moveit_feedback_state = None

    def moveit_feedback_cb(self, msg):
        self.moveit_feedback_state = msg.status.status

    def execute_cb(self, goal):

        object_point = self.get_object_point()
        object_point_transformed = self.transform_point(object_point, "base_link")

        self.add_object_to_scene(object_point_transformed.point)
        pre_point, post_point = self.calculate_pre_and_post_points(
            object_point_transformed.point
        )

        procedure_list = [
            # Go to pre point
            ActionProcedureStep(
                start_procedure_function=self.go_to_point,
                start_procedure_function_arguments=pre_point,
                get_procedure_state_function=self.moveit_finished_execution,
                preempted_action_function=self.moveit_abort,
                feedback_state=PickObjectFeedback.GOING_TO_PRE_GRIPPING_POSITION,
            ),
            # Open gripper
            ActionProcedureStep(
                start_procedure_function=self.open_gripper,
                start_procedure_function_arguments=None,
                get_procedure_state_function=lambda: GoalState.SUCCEEDED,
                preempted_action_function=lambda: True,
                feedback_state=PickObjectFeedback.PRE_GRIPPING_POSITION,
            ),
            # Go to object point
            ActionProcedureStep(
                start_procedure_function=self.go_to_point,
                start_procedure_function_arguments=object_point_transformed.point,
                get_procedure_state_function=self.moveit_finished_execution,
                preempted_action_function=self.moveit_abort,
                feedback_state=PickObjectFeedback.GOING_GRIPPING_POSITION,
            ),
            # Calculate error
            ActionProcedureStep(
                start_procedure_function=self.print_error,
                start_procedure_function_arguments=object_point_transformed.point,
                get_procedure_state_function=lambda: GoalState.SUCCEEDED,
                preempted_action_function=lambda: True,
                feedback_state=PickObjectFeedback.GRIPPING_POSITION,
            ),
            # Close gripper
            ActionProcedureStep(
                start_procedure_function=self.close_gripper,
                start_procedure_function_arguments=None,
                get_procedure_state_function=lambda: GoalState.SUCCEEDED,
                preempted_action_function=lambda: True,
                feedback_state=PickObjectFeedback.CLOSING_GRIPPER,
            ),
            # Attach object
            ActionProcedureStep(
                start_procedure_function=self.attach_object,
                start_procedure_function_arguments=None,
                get_procedure_state_function=lambda: GoalState.SUCCEEDED,
                preempted_action_function=lambda: True,
                feedback_state=PickObjectFeedback.GRIPPING_POSITION,
            ),
            # Go to post point
            ActionProcedureStep(
                start_procedure_function=self.go_to_point,
                start_procedure_function_arguments=post_point,
                get_procedure_state_function=self.moveit_finished_execution,
                preempted_action_function=self.moveit_abort,
                feedback_state=PickObjectFeedback.GOING_TO_POST_GRIPPING_POSITION,
            ),
        ]

        for procedure in procedure_list:
            self.feedback.status = procedure.get_feedback()
            self.pick_object_action.publish_feedback(self.feedback)

            procedure.start_procedure()

            goal_state = GoalState.IN_PROGRESS

            while goal_state != GoalState.SUCCEEDED:

                if self.pick_object_action.is_preempt_requested():
                    rospy.loginfo("pick_and_bring: Preempted")
                    self.pick_object_action.set_preempted()
                    procedure.preempted_action()
                    return

                goal_state = procedure.get_procedure_state()

                if goal_state == GoalState.FAILED:
                    rospy.loginfo("goal failed")
                    self.result.success = False
                    self.pick_object_action.set_succeeded(self.result)
                    return

                rospy.Rate(10).sleep()

        # TODO: Probably shouldn't remove object after picking
        self.scene.remove_attached_object("gripper_right_grip", name="object")
        self.scene.remove_world_object("object")

        self.result.success = True
        self.pick_object_action.set_succeeded(self.result)

    def moveit_finished_execution(self):
        goal_state = None

        if not self.moveit_feedback_state:
            return GoalState.IN_PROGRESS

        state = self.moveit_feedback_state

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
        # not woorking
        # self.moveit_client.cancel_all_goals()

        # not working
        # self.move_group.stop()
        # self.move_group.clear_pose_targets()

        self.moveit_cancel_pub.publish(GoalID())

    def attach_object(self):
        grasping_group = "hand"
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box("gripper_right_grip", "object", touch_links=touch_links)

    def move_gripper(self, position):
        raise NotImplementedError()

    def close_gripper(self):
        raise NotImplementedError()

    def open_gripper(self):
        raise NotImplementedError()

    def find_and_execute_plan(self, wait=True):
        plan = False
        while not plan:
            rospy.loginfo("Trying to find and execute plan")
            plan = self.move_group.go(wait=wait)

        self.moveit_feedback_state = None

        if wait:
            self.move_group.stop()
            self.move_group.clear_pose_targets()

    def get_perpendicular_orientation(self):
        orientation = Quaternion()
        quat = quaternion_from_euler(-1.461, -1.445, 3.016)
        orientation.x = quat[0]
        orientation.y = quat[1]
        orientation.z = quat[2]
        orientation.w = quat[3]
        return orientation

    def go_to_point(self, point):
        self.move_group.set_start_state_to_current_state()

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = self.get_perpendicular_orientation()
        pose_goal.position = point

        # self.move_group.set_pose_target(pose_goal)

        # Allow approximate solutions (True)
        self.move_group.set_joint_value_target(pose_goal, "gripper_right_grip", True)
        self.find_and_execute_plan(wait=False)

    def add_object_to_scene(self, point):
        reference_frame = "base_link"
        object_id = "object"

        # Remove leftover objects from a previous run
        self.scene.remove_world_object(object_id)

        # Real height is 0.125, but it collides with cardboard box then
        # and plan to pick it up isn't too good
        body_size = [0.015, 0.02, 0.06]

        body_pose = PoseStamped()
        body_pose.header.frame_id = reference_frame
        body_pose.pose.position.x = point.x
        body_pose.pose.position.y = point.y
        body_pose.pose.position.z = point.z - body_size[2] / 2.0 + 0.02

        self.scene.add_box(object_id, body_pose, body_size)

    def calculate_pre_and_post_points(self, point):
        pre_point = copy.deepcopy(point)
        pre_point.y -= self.retraction

        post_point = copy.deepcopy(point)
        post_point.z += 0.08

        return pre_point, post_point

    def return_to_home_position(self, req):
        res = TriggerResponse()
        res.success = True

        self.move_group.set_named_target("Home")

        self.find_and_execute_plan()

        return res

    def get_object_point(self):
        raise NotImplementedError()

    def transform_point(self, point, target_frame):
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            try:
                point_transformed = listener.transformPoint(target_frame, point)
                return point_transformed
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ) as err:
                rospy.logerr(err)
            rate.sleep()

    def print_error(self, goal_point):
        tip_point = PointStamped()
        tip_point.header.stamp = rospy.Time(0)
        tip_point.header.frame_id = "gripper_right_grip"
        tip_point.point.x = 0.0
        tip_point.point.y = 0.0
        tip_point.point.z = 0.0

        gripper_point_transformed = self.transform_point(tip_point, "base_link")

        diff = Point()
        diff.x = gripper_point_transformed.point.x - goal_point.x
        diff.y = gripper_point_transformed.point.y - goal_point.y
        diff.z = gripper_point_transformed.point.z - goal_point.z

        rospy.logwarn("Planner error: " + str(diff))
