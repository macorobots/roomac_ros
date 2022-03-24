#!/usr/bin/env python

import sys
import copy

import rospy
import moveit_commander

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion, Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from roomac_autonomous_manipulation.srv import DetectGripperPosition

from moveit_msgs.msg import (
    AllowedCollisionMatrix,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene

import tf
from tf.transformations import quaternion_from_euler

from moveit_msgs.msg import MoveGroupAction, MoveGroupActionFeedback
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

        self.controller_command_pub = rospy.Publisher(
            "/roomac/arm_position_controller/command",
            JointTrajectory,
            queue_size=10,
            latch=True,
        )

        self.set_home_arm_srv = rospy.Service(
            "set_home_arm", Trigger, self.return_to_home_position
        )
        self.pick_object_srv = rospy.Service("pick_object", Trigger, self.pick_object)
        self.pick_object_with_correction_srv = rospy.Service(
            "pick_object_with_correction", Trigger, self.pick_object_with_correction
        )

        self.pick_correction_srv = rospy.ServiceProxy(
            "pick_correction", DetectGripperPosition
        )

        self.debug_points = rospy.Publisher(
            "/debug_points", PointStamped, queue_size=5, latch=True
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

        procedures = [
            (
                self.go_to_point,
                pre_point,
                self.moveit_finished_execution,
                PickObjectFeedback.GOING_TO_PRE_GRIPPING_POSITION,
                self.moveit_abort,
            ),
            (
                self.open_gripper,
                None,
                lambda: GoalState.SUCCEEDED,
                PickObjectFeedback.PRE_GRIPPING_POSITION,
                lambda: True,
            ),
            (
                self.go_to_point,
                object_point_transformed.point,
                self.moveit_finished_execution,
                PickObjectFeedback.GOING_GRIPPING_POSITION,
                self.moveit_abort,
            ),
            (
                self.print_error,
                object_point_transformed.point,
                lambda: GoalState.SUCCEEDED,
                PickObjectFeedback.GRIPPING_POSITION,
                lambda: True,
            ),
            (
                self.attach_object,
                None,
                lambda: GoalState.SUCCEEDED,
                PickObjectFeedback.GRIPPING_POSITION,
                lambda: True,
            ),
            (
                self.close_gripper,
                None,
                lambda: GoalState.SUCCEEDED,
                PickObjectFeedback.CLOSING_GRIPPER,
                lambda: True,
            ),
            (
                self.go_to_point,
                post_point,
                self.moveit_finished_execution,
                PickObjectFeedback.GOING_TO_POST_GRIPPING_POSITION,
                self.moveit_abort,
            ),
        ]

        for x in procedures:
            self.feedback.status = x[3]
            self.pick_object_action.publish_feedback(self.feedback)

            if x[1]:
                x[0](x[1])
            else:
                x[0]()

            goal_state = GoalState.IN_PROGRESS

            while goal_state != GoalState.SUCCEEDED:

                if self.pick_object_action.is_preempt_requested():
                    rospy.loginfo("pick_and_bring: Preempted")
                    self.pick_object_action.set_preempted()
                    x[4]()
                    return

                goal_state = x[2]()

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

        rospy.loginfo(state)
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

    # TODO: check this, appears to not work correctly
    def add_to_allowed_collision_matrix(self, object_id):
        # I also tried 5.0, but didn't help so no point in longer sleep
        rospy.sleep(0.5)

        self.planning_scene_pub = rospy.Publisher(
            "/planning_scene", PlanningScene, queue_size=10
        )
        rospy.wait_for_service("/get_planning_scene", 10.0)
        get_planning_scene = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
        request = PlanningSceneComponents(
            components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        response = get_planning_scene(request)

        acm = response.scene.allowed_collision_matrix
        if not object_id in acm.default_entry_names:
            acm.default_entry_names += [object_id]
            acm.default_entry_values += [True]

            planning_scene_diff = PlanningScene(
                is_diff=True, allowed_collision_matrix=acm
            )

            self.planning_scene_pub.publish(planning_scene_diff)
            # I also tried 5.0, but didn't help so no point in longer sleep
            rospy.sleep(0.5)

    def calculate_pre_and_post_points(self, point):
        pre_point = copy.deepcopy(point)
        pre_point.y -= self.retraction

        post_point = copy.deepcopy(point)
        post_point.z += 0.2

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

    def pick_object(self, req):
        res = TriggerResponse()
        res.success = True

        object_point = self.get_object_point()
        object_point_transformed = self.transform_point(object_point, "base_link")

        self.add_object_to_scene(object_point_transformed.point)
        pre_point, post_point = self.calculate_pre_and_post_points(
            object_point_transformed.point
        )

        self.go_to_point(pre_point)
        self.open_gripper()

        self.go_to_point(object_point_transformed.point)
        self.print_error(object_point_transformed.point)

        grasping_group = "hand"
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box("gripper_right_grip", "object", touch_links=touch_links)

        self.close_gripper()
        self.go_to_point(post_point)

        # TODO: Probably shouldn't remove object after picking
        self.scene.remove_attached_object("gripper_right_grip", name="object")
        self.scene.remove_world_object("object")

        return res

    def update_object_position(self, gripper_point, pre_point, object_point):
        gripper_point.header.stamp = rospy.Time(0)
        gripper_point_transformed = self.transform_point(gripper_point, "base_link")

        pre_point.header.stamp = rospy.Time(0)
        pre_point_transformed = self.transform_point(pre_point_transformed, "base_link")

        diff = Point()
        diff.x = pre_point_transformed.point.x - gripper_point_transformed.point.x
        diff.y = pre_point_transformed.point.y - gripper_point_transformed.point.y
        diff.z = pre_point_transformed.point.z - gripper_point_transformed.point.z

        object_point_corrected = Point()
        object_point_corrected.x = object_point.point.x + diff.x
        object_point_corrected.y = object_point.point.y + diff.y
        object_point_corrected.z = object_point.point.z + diff.z

        return object_point_corrected

    def pick_object_with_correction(self, req):
        res = TriggerResponse()
        res.success = True

        object_point = self.get_object_point()
        object_point_transformed = self.transform_point(object_point, "base_link")

        self.add_object_to_scene(object_point_transformed.point)
        self.add_to_allowed_collision_matrix("object")

        pre_point, post_point = self.calculate_pre_and_post_points(
            object_point_transformed.point
        )

        pre_point_stamped = PointStamped()
        pre_point_stamped.point = pre_point
        pre_point_stamped.header = object_point_transformed.header

        pre_point_transformed = self.transform_point(
            pre_point_stamped, "camera_up_rgb_optical_frame"
        )

        self.debug_points.publish(pre_point_transformed)
        self.debug_points.publish(object_point_transformed)

        rospy.loginfo("Opening gripper")
        self.open_gripper()

        rospy.loginfo("Going to pre point")
        self.go_to_point(pre_point)

        rospy.loginfo("Getting gripper position")
        self.pick_correction_srv.wait_for_service()
        resp = self.pick_correction_srv.call()

        while not resp.success:
            rospy.logwarn(
                "Getting gripper position not successful: "
                + resp.message
                + ". Retrying..."
            )
            resp = self.pick_correction_srv.call()

        object_point_corrected = self.update_object_position(
            resp.calculated_gripper_position,
            pre_point_transformed,
            object_point_transformed,
        )

        object_point_corrected_stamped = PointStamped()
        object_point_corrected_stamped.point = object_point_corrected
        object_point_corrected_stamped.header.stamp = rospy.Time.now()
        object_point_corrected_stamped.header.frame_id = "base_link"
        self.debug_points.publish(object_point_corrected_stamped)

        # allowed collision matrix isn't really working, so it necessary to also move object
        # in the scene to updated position, otherwise there will be problems with planning

        self.add_object_to_scene(object_point_corrected)
        self.add_to_allowed_collision_matrix("object")

        rospy.loginfo("Going to point")
        self.go_to_point(object_point_corrected)

        rospy.loginfo("Closing gripper")
        self.close_gripper()

        rospy.loginfo("Going to post point")
        self.go_to_point(post_point)

        self.scene.remove_world_object("object")

        return res
