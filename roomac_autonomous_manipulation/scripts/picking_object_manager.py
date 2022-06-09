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
from actionlib_msgs.msg import GoalID, GoalStatus

from roomac_msgs.msg import (
    PickObjectAction,
    PickObjectFeedback,
    PickObjectResult,
)
from roomac_msgs.srv import DetectObjectAndTable

from dynamic_reconfigure.server import Server
from roomac_autonomous_manipulation.cfg import PickManagerSettingsConfig


from roomac_utils.action_procedure_executor import (
    ActionProcedureStep,
    SimpleActionExecutor,
    GoalState,
)


class PickingObjectManager(object):
    def __init__(self):
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

        # Instead of retries it would be probably better to change planning attempts in moveit
        # as they are main reason
        self.procedure_retry_threshold = rospy.get_param(
            "~procedure_retry_threshold", 5
        )

        self.dynamic_reconfigure_srv = Server(
            PickManagerSettingsConfig, self.dynamic_reconfigure_cb
        )

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
        self.base_link_frame = rospy.get_param("~base_link_frame", "base_link")
        self.gripper_frame = rospy.get_param("~gripper_frame", "gripper_right_grip")
        self.object_name = rospy.get_param("~object_name", "object")
        self.grasping_group_name = rospy.get_param("~grasping_group_name", "hand")
        self.home_position_target_name = rospy.get_param(
            "~home_position_target_name", "Home"
        )

        # Moveit stuff

        moveit_commander.roscpp_initialize(sys.argv)
        # when master is running remotely (e.g. on raspberry) communication is slower
        # and often timed out
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(
            name=arm_name, wait_for_servers=wait_time_move_group_servers
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
        self.move_group.set_planning_time(move_group_planning_time)
        self.move_group.set_num_planning_attempts(move_group_planning_attempts)

        # only position ik, really important without it fails to find plan
        self.move_group.set_goal_orientation_tolerance(
            move_group_goal_orientation_tolerance
        )
        self.move_group.set_goal_position_tolerance(move_group_goal_position_tolerance)

        # Services

        self.set_home_arm_srv = rospy.Service(
            "set_home_arm", Trigger, self.return_to_home_position
        )
        self.remove_object_from_scene_srv = rospy.Service(
            "remove_object_from_scene", Trigger, self.remove_object_from_scene_cb
        )

        self.moveit_feedback_state = None

        self.current_object_point = None
        self.current_pre_object_point = None
        self.current_post_object_point = None

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
                start_procedure_function=self.open_gripper_with_delay,
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
                start_procedure_function=self.go_to_current_object_point,
                get_procedure_state_function=self.moveit_finished_execution,
                preempted_action_function=self.moveit_abort,
                feedback_state=PickObjectFeedback.GOING_TO_GRIPPING_POSITION,
            ),
            # Calculate error
            ActionProcedureStep(
                start_procedure_function=self.print_current_error,
                get_procedure_state_function=lambda: GoalState.SUCCEEDED,
                preempted_action_function=lambda: None,
                feedback_state=PickObjectFeedback.GRIPPING_POSITION,
            ),
            # Close gripper
            ActionProcedureStep(
                start_procedure_function=self.close_gripper_with_delay,
                get_procedure_state_function=lambda: GoalState.SUCCEEDED,
                preempted_action_function=lambda: None,
                feedback_state=PickObjectFeedback.CLOSING_GRIPPER,
            ),
            # Attach object
            ActionProcedureStep(
                start_procedure_function=self.attach_object,
                get_procedure_state_function=lambda: GoalState.SUCCEEDED,
                preempted_action_function=lambda: None,
                feedback_state=PickObjectFeedback.GRIPPING_POSITION,
            ),
            # Go to post point
            ActionProcedureStep(
                start_procedure_function=self.go_to_current_post_object_point,
                get_procedure_state_function=self.moveit_finished_execution,
                preempted_action_function=self.moveit_abort,
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

    def open_gripper_cb(self, req):
        res = TriggerResponse()
        res.success = True
        self.open_gripper()
        return res

    def close_gripper_cb(self, req):
        res = TriggerResponse()
        res.success = True
        self.close_gripper()
        return res

    def go_to_current_object_point(self):
        self.go_to_point(self.current_object_point)

    def go_to_current_pre_object_point(self):
        self.go_to_point(self.current_pre_object_point)

    def go_to_current_post_object_point(self):
        self.go_to_point(self.current_post_object_point)

    def print_current_error(self):
        self.print_error(self.current_object_point)

    def prepare_picking(self):
        self.remove_object_from_scene()

        object_point = self.get_detected_object_point()
        object_point_transformed = self.transform_point(
            object_point, self.base_link_frame
        )

        self.add_object_to_scene(object_point_transformed.point)
        pre_point, post_point = self.calculate_pre_and_post_points(
            object_point_transformed.point
        )

        self.current_object_point = object_point_transformed.point
        self.current_pre_object_point = pre_point
        self.current_post_object_point = post_point

    def moveit_feedback_cb(self, msg):
        self.moveit_feedback_state = msg.status.status

    def remove_object_from_scene(self):
        self.scene.remove_attached_object(self.gripper_frame, name=self.object_name)
        self.scene.remove_world_object(self.object_name)

    def remove_object_from_scene_cb(self, req):
        res = TriggerResponse()
        res.success = True

        self.remove_object_from_scene()

        return res

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
        grasping_group = self.grasping_group_name
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(
            self.gripper_frame, self.object_name, touch_links=touch_links
        )
        self.scene.remove_world_object(self.object_name + "_lower")

    def close_gripper(self, delay=1.0):
        raise NotImplementedError()

    def open_gripper(self, delay=1.0):
        raise NotImplementedError()

    def close_gripper_with_delay(self):
        self.close_gripper(self.close_gripper_wait_time)

    def open_gripper_with_delay(self):
        self.open_gripper(self.open_gripper_wait_time)

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
        self.move_group.set_joint_value_target(pose_goal, self.gripper_frame, True)
        self.find_and_execute_plan(wait=False)

    def add_object_to_scene(self, point):
        # Remove leftover objects from a previous run
        self.scene.remove_world_object(self.object_name)

        # Real height is 0.125, but it collides with cardboard box then
        # and plan to pick it up isn't too good
        body_size = [0.015, 0.04, 0.06]

        body_pose = PoseStamped()
        body_pose.header.frame_id = self.base_link_frame
        body_pose.pose.position.x = point.x
        body_pose.pose.position.y = point.y
        body_pose.pose.position.z = point.z - body_size[2] / 2.0 + 0.02

        self.scene.add_box(self.object_name, body_pose, body_size)

        # Lower part of the bottle, so moveit won't approach it from lower position
        # causing bottle to fall
        body_size = [0.06, 0.06, 0.06]
        body_pose.header.frame_id = self.base_link_frame
        body_pose.pose.position.z -= 0.03
        self.scene.add_box(self.object_name + "_lower", body_pose, body_size)

    def calculate_pre_and_post_points(self, point):
        pre_point = copy.deepcopy(point)
        pre_point.y -= self.pre_goal_retraction_x

        post_point = copy.deepcopy(point)
        post_point.z += self.post_goal_retraction_z

        return pre_point, post_point

    def return_to_home_position(self, req):
        res = TriggerResponse()
        res.success = True

        self.move_group.set_named_target(self.home_position_target_name)

        self.find_and_execute_plan()

        return res

    def get_object_point(self):
        raise NotImplementedError()

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

        object_point_stamped.point.x += self.object_position_correction_x
        object_point_stamped.point.y += self.object_position_correction_y
        object_point_stamped.point.z += self.object_position_correction_z

        return object_point_stamped

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
        tip_point.header.frame_id = self.gripper_frame
        tip_point.point.x = 0.0
        tip_point.point.y = 0.0
        tip_point.point.z = 0.0

        gripper_point_transformed = self.transform_point(
            tip_point, self.base_link_frame
        )

        diff = Point()
        diff.x = gripper_point_transformed.point.x - goal_point.x
        diff.y = gripper_point_transformed.point.y - goal_point.y
        diff.z = gripper_point_transformed.point.z - goal_point.z

        rospy.logwarn("Planner error: " + str(diff))

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
