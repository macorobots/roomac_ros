#!/usr/bin/env python

from enum import Enum

import yaml
import math

import rospy

import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Pose2D
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from std_srvs.srv import Empty

import actionlib
from actionlib_msgs.msg import GoalStatus

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from roomac_msgs.msg import (
    PickAndBringAction,
    PickAndBringFeedback,
    PickAndBringResult,
    PickAction,
    PickActionGoal,
)

from dynamic_reconfigure.server import Server
from roomac.cfg import AutonomousMissionSettingsConfig

from roomac_utils.action_procedure_executor import (
    ActionProcedureStep,
    SimpleActionExecutor,
    GoalState,
)


class RobotController:
    def __init__(self):
        self.wait_time_before_picking_action = rospy.get_param(
            "~wait_time_before_picking_action", 5.0
        )
        self.artag_stable_position_threshold = rospy.get_param(
            "~artag_stable_position_threshold", 0.04
        )
        self.artag_stable_orientation_threshold = (
            rospy.get_param("~artag_stable_orientation_threshold", 5.0)
            / 180.0
            * math.pi
        )

        self.dynamic_reconfigure_srv = Server(
            AutonomousMissionSettingsConfig, self.dynamic_reconfigure_cb
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.positions = {"home_position": None, "table_position": None}

        self.position_file = rospy.get_param(
            "~positions_file", "/home/roomac/roomac_data/positions.yaml"
        )
        self.load_positions()

        self.save_home_srv = rospy.Service(
            "save_home_position", Trigger, self.save_home_position
        )
        self.save_table_srv = rospy.Service(
            "save_table_position", Trigger, self.save_table_position
        )

        self.go_to_table_srv = rospy.Service(
            "go_to_table", Trigger, self.go_to_table_cb
        )
        self.go_to_home_srv = rospy.Service("go_to_home", Trigger, self.go_to_home_cb)

        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        self.pick_object_client = actionlib.SimpleActionClient(
            "pick_object", PickAction
        )

        self.clear_octomap_srv = rospy.ServiceProxy("/clear_octomap", Empty)
        self.add_table_to_scene_srv = rospy.ServiceProxy("/add_table_to_scene", Trigger)

        rospy.loginfo("Waiting for all services and servers to become available")
        self.move_base_client.wait_for_server()
        self.pick_object_client.wait_for_server()
        self.clear_octomap_srv.wait_for_service()
        self.add_table_to_scene_srv.wait_for_service()
        rospy.loginfo("All services and servers are available")

        procedure_list = [
            ActionProcedureStep(
                start_procedure_function=lambda: None,
                get_procedure_state_function=self.check_home_and_table_positions,
                preempted_action_function=lambda: None,
                feedback_state=PickAndBringFeedback.DRIVING_TO_TABLE,
            ),
            ActionProcedureStep(
                start_procedure_function=self.go_to_table,
                get_procedure_state_function=self.move_base_finished_execution,
                preempted_action_function=self.move_base_abort,
                feedback_state=PickAndBringFeedback.DRIVING_TO_TABLE,
            ),
            ActionProcedureStep(
                start_procedure_function=lambda: None,
                get_procedure_state_function=self.check_if_artag_position_is_stable,
                preempted_action_function=lambda: None,
                feedback_state=PickAndBringFeedback.PICKING_UP_OBJECT,
            ),
            ActionProcedureStep(
                start_procedure_function=self.pick_object,
                get_procedure_state_function=self.pick_object_finished_execution,
                preempted_action_function=self.pick_object_abort,
                feedback_state=PickAndBringFeedback.PICKING_UP_OBJECT,
            ),
            ActionProcedureStep(
                start_procedure_function=self.go_to_home,
                get_procedure_state_function=self.move_base_finished_execution,
                preempted_action_function=self.move_base_abort,
                feedback_state=PickAndBringFeedback.DRIVING_TO_HOME_POSITION,
            ),
        ]

        self.pick_action_executor = SimpleActionExecutor(
            "pick_and_bring",
            PickAndBringAction,
            PickAndBringFeedback,
            PickAndBringResult,
            10.0,
            procedure_list,
        )

    def go_to_table(self):
        self.go_to_point(self.positions["table_position"])

    def go_to_home(self):
        self.go_to_point(self.positions["home_position"])

    def check_home_and_table_positions(self):
        if self.positions["home_position"] and self.positions["table_position"]:
            return GoalState.SUCCEEDED
        else:
            rospy.logerr("home_position and/or table_position wasn't defined")
            return GoalState.FAILED

    def check_if_artag_position_is_stable(self):
        try:
            artag_pose = self.get_position("camera_up_link", "ar_marker_0")
            artag_pose_filtered = self.get_position("camera_up_link", "artag_link_2")
        except RuntimeError as e:
            rospy.logerr("Exception: " + str(e))
            return GoalState.IN_PROGRESS

        trans_diff = math.sqrt(
            (artag_pose.x - artag_pose_filtered.x) ** 2
            + (artag_pose.y - artag_pose_filtered.y) ** 2
        )

        rot_diff = math.fabs(artag_pose.theta - artag_pose_filtered.theta)

        if (
            trans_diff < self.artag_stable_position_threshold
            and rot_diff < self.artag_stable_orientation_threshold
        ):
            return GoalState.SUCCEEDED
        else:
            return GoalState.IN_PROGRESS

    def pick_object(self):
        # checking if artag is stable isn't working perfectly yet
        # so some delay is necessary before object position will be stable
        # and correct
        rospy.loginfo(
            "Waiting "
            + str(self.wait_time_before_picking_action)
            + " seconds before picking up object"
        )
        rospy.sleep(self.wait_time_before_picking_action)
        self.clear_octomap_srv.call()
        self.add_table_to_scene_srv.call(TriggerRequest())
        self.pick_object_client.send_goal(PickActionGoal())

    def pick_object_finished_execution(self):
        goal_state = None

        state = self.pick_object_client.get_state()
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

    def pick_object_abort(self):
        self.pick_object_client.cancel_all_goals()

    def move_base_finished_execution(self):
        goal_state = None

        state = self.move_base_client.get_state()
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

    def move_base_abort(self):
        self.move_base_client.cancel_all_goals()

    def load_positions(self):
        try:
            with open(self.position_file) as file:
                positions = yaml.load(file)
                for name, pos in positions.items():
                    if name == "home_position" or name == "table_position":
                        self.positions[name] = Pose2D()
                        self.positions[name].x = pos[0]
                        self.positions[name].y = pos[1]
                        self.positions[name].theta = pos[2]

                        rospy.loginfo(
                            name + " position loaded: " + str(self.positions[name])
                        )
                    else:
                        rospy.logwarn(name + " position not recognized, skipping...")
        except Exception as e:
            rospy.logerr("Error when trying to access positions file: " + str(e))
            pass

    def save_positions_to_file(self):
        positions_yaml_dict = {}
        for name in self.positions:
            if self.positions[name]:
                positions_yaml_dict[name] = [
                    self.positions[name].x,
                    self.positions[name].y,
                    self.positions[name].theta,
                ]
        with open(self.position_file, "w") as file:
            positions = yaml.dump(positions_yaml_dict, file)

    def go_to_table_cb(self, req):
        res = TriggerResponse()
        res.success = True

        if self.positions["table_position"]:
            self.go_to_point(self.positions["table_position"])
            return res

        res.success = False
        return res

    def go_to_home_cb(self, req):
        res = TriggerResponse()
        res.success = True

        if self.positions["home_position"]:
            self.go_to_point(self.positions["home_position"])
            return res

        res.success = False
        return res

    def save_home_position(self, req):
        res = TriggerResponse()
        res.success = True
        try:
            current_position = self.get_current_position()
        except:
            res.success = False
            return res

        self.positions["home_position"] = current_position
        self.save_positions_to_file()
        return res

    def save_table_position(self, req):
        res = TriggerResponse()
        res.success = True
        try:
            current_position = self.get_current_position()
        except:
            res.success = False
            return res

        self.positions["table_position"] = current_position
        self.save_positions_to_file()
        return res

    def get_position(self, target_frame, source_frame):
        try:
            current_transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            raise RuntimeError(
                "Couldn't get " + target_frame + "->" + source_frame + " transform"
            )

        current_position = Pose2D()
        current_position.x = current_transform.transform.translation.x
        current_position.y = current_transform.transform.translation.y
        quaternion = [
            current_transform.transform.rotation.x,
            current_transform.transform.rotation.y,
            current_transform.transform.rotation.z,
            current_transform.transform.rotation.w,
        ]

        eulerAngles = euler_from_quaternion(quaternion)
        current_position.theta = eulerAngles[2]

        return current_position

    def get_current_position(self):
        return self.get_position("map", "base_link")

    def go_to_point(self, pose):
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = pose.x
        goal.target_pose.pose.position.y = pose.y
        goal.target_pose.pose.position.z = 0.0

        quat = quaternion_from_euler(0.0, 0.0, pose.theta)
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        self.move_base_client.send_goal(goal)

    def dynamic_reconfigure_cb(self, config, level):
        self.wait_time_before_picking_action = config.wait_time_before_picking_action
        self.artag_stable_position_threshold = config.artag_stable_position_threshold
        self.artag_stable_orientation_threshold = (
            config.artag_stable_orientation_threshold / 180.0 * math.pi
        )
        return config


if __name__ == "__main__":
    rospy.init_node("robot_autonomy_controller")
    controller = RobotController()
    rospy.spin()
