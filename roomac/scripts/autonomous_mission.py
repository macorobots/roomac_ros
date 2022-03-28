#!/usr/bin/env python

import rospy

import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose2D
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from std_srvs.srv import Empty

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

from roomac_msgs.msg import (
    PickAndBringAction,
    PickAndBringFeedback,
    PickAndBringResult,
    PickAction,
    PickActionGoal,
)

import yaml
import math

from enum import Enum


class GoalState(Enum):
    IN_PROGRESS = 1
    SUCCEEDED = 2
    FAILED = 3


class RobotController:
    def __init__(self):
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

        self.go_to_table_srv = rospy.Service("go_to_table", Trigger, self.go_to_table)
        self.go_to_home_srv = rospy.Service("go_to_home", Trigger, self.go_to_home)

        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        self.move_base_client.wait_for_server()

        self.pick_object_client = actionlib.SimpleActionClient(
            "pick_object", PickAction
        )
        self.pick_object_client.wait_for_server()

        self.clear_octomap_srv = rospy.ServiceProxy("/clear_octomap", Empty)
        self.clear_octomap_srv.wait_for_service()

        self.add_table_to_scene_srv = rospy.ServiceProxy("/add_table_to_scene", Trigger)
        self.add_table_to_scene_srv.wait_for_service()

        self.pick_object_srv = rospy.ServiceProxy("pick_object", Trigger)
        self.execute_mission_srv = rospy.Service(
            "execute_mission", Trigger, self.execute_mission
        )

        self.feedback = PickAndBringFeedback()
        self.result = PickAndBringResult()
        self.pick_and_bring_action = actionlib.SimpleActionServer(
            "pick_and_bring",
            PickAndBringAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.pick_and_bring_action.start()

    def execute_cb(self, goal):

        if not (self.positions["home_position"] and self.positions["table_position"]):
            self.result.success = False
            self.pick_and_bring_action.set_succeeded(self.result)
            return

        procedures = [
            (
                self.go_to_point,
                self.positions["table_position"],
                self.move_base_finished_execution,
                PickAndBringFeedback.DRIVING_TO_TABLE,
                self.move_base_abort,
            ),
            (
                lambda: None,
                None,
                self.check_if_artag_position_is_stable,
                PickAndBringFeedback.PICKING_UP_OBJECT,
                lambda: None,
            ),
            (
                self.pick_object,
                None,
                self.pick_object_finished_execution,
                PickAndBringFeedback.PICKING_UP_OBJECT,
                self.pick_object_abort,
            ),
            (
                self.go_to_point,
                self.positions["home_position"],
                self.move_base_finished_execution,
                PickAndBringFeedback.DRIVING_TO_HOME_POSITION,
                self.move_base_abort,
            ),
        ]

        for x in procedures:
            self.feedback.status = x[3]
            self.pick_and_bring_action.publish_feedback(self.feedback)

            if x[1]:
                x[0](x[1])
            else:
                x[0]()

            goal_state = GoalState.IN_PROGRESS

            while goal_state != GoalState.SUCCEEDED:

                if self.pick_and_bring_action.is_preempt_requested():
                    rospy.loginfo("pick_and_bring: Preempted")
                    self.pick_and_bring_action.set_preempted()
                    x[4]()
                    return

                goal_state = x[2]()

                if goal_state == GoalState.FAILED:
                    self.result.success = False
                    self.pick_and_bring_action.set_succeeded(self.result)
                    return

                rospy.Rate(10).sleep()

        self.result.success = True
        self.pick_and_bring_action.set_succeeded(self.result)

    def check_if_artag_position_is_stable(self):
        artag_pose = self.get_position("camera_up_link", "ar_marker_0_only_yaw")
        artag_pose_filtered = self.get_position("camera_up_link", "artag_link_2")

        trans_diff = math.sqrt(
            (artag_pose.x - artag_pose_filtered.x) ** 2
            + (artag_pose.y - artag_pose_filtered.y) ** 2
        )

        rot_diff = math.fabs(artag_pose.theta - artag_pose_filtered.theta)

        rot_threshold = 5.0 / 180.0 * math.pi
        trans_threshold = 0.04

        if trans_diff < trans_threshold and rot_diff < rot_threshold:
            return GoalState.SUCCEEDED
        else:
            return GoalState.IN_PROGRESS

    def pick_object(self):
        # checking if artag is stable isn't working perfectly yet
        # so some delay is necessary before object position will be stable
        # and correct
        rospy.sleep(5.0)
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

    def go_to_table(self, req):
        res = TriggerResponse()
        res.success = True

        if self.positions["table_position"]:
            self.go_to_point(self.positions["table_position"])
            return res

        res.success = False
        return res

    def go_to_home(self, req):
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
        # wait = self.move_base_client.wait_for_result()

        # if not wait:
        #     rospy.logerr("Action server not available!")
        # else:
        #     return self.move_base_client.get_result()

    def execute_mission(self, req):
        res = TriggerResponse()
        res.success = True

        if not (self.positions["home_position"] and self.positions["table_position"]):
            res.success = False
            return res

        rospy.loginfo("Driving to table")
        self.go_to_point(self.positions["table_position"])

        rospy.loginfo("Waiting few seconds to allow artag positions to stabilize")
        rospy.sleep(20.0)

        rospy.loginfo("Waiting for clear_octomap service")
        self.clear_octomap_srv.wait_for_service()

        rospy.loginfo("Clearing octomap")
        self.clear_octomap_srv.call()

        rospy.loginfo("Waiting for pick_object service")
        self.pick_object_srv.wait_for_service()

        rospy.loginfo("Picking object")
        self.pick_object_srv.call(TriggerRequest())

        rospy.loginfo("Driving to home position")
        self.go_to_point(self.positions["home_position"])

        return res


if __name__ == "__main__":
    rospy.init_node("robot_autonomy_controller")
    controller = RobotController()
    rospy.spin()
