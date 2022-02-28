#!/usr/bin/env python

import rospy

import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose2D
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from std_srvs.srv import Empty

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import yaml


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

        self.clear_octomap_srv = rospy.ServiceProxy("/clear_octomap", Empty)
        self.pick_object_srv = rospy.ServiceProxy("pick_object", Trigger)
        self.save_table_srv = rospy.Service(
            "execute_mission", Trigger, self.execute_mission
        )

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

    def get_current_position(self):
        try:
            current_transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rospy.Time(0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            raise RuntimeError("Couldn't get base_link-> transform")

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
        wait = self.move_base_client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
        else:
            return self.move_base_client.get_result()

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
