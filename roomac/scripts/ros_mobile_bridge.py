#!/usr/bin/env python

import rospy

from std_srvs.srv import Trigger
from std_msgs.msg import Bool

import actionlib

from roomac_msgs.msg import (
    PickAndBringAction,
    PickAndBringGoal,
)


class ServiceButtonBridge:
    def __init__(self, service_name, topic_name):
        self.srv = rospy.ServiceProxy(service_name, Trigger)
        self.sub = rospy.Subscriber(topic_name, Bool, self.callback, queue_size=10)

    def callback(self, msg):
        if msg.data:
            self.srv.call()


class ActionButtonBridge:
    def __init__(
        self,
        action_name,
        action_type,
        action_goal_type,
        goal_topic_name,
        cancel_topic_name,
    ):
        self.goal_sub = rospy.Subscriber(
            goal_topic_name, Bool, self.goal_cb, queue_size=10
        )
        self.cancel_sub = rospy.Subscriber(
            cancel_topic_name, Bool, self.cancel_cb, queue_size=10
        )

        self.action_goal_type = action_goal_type

        rospy.loginfo("Waiting for server")
        self.pick_object_client = actionlib.SimpleActionClient(action_name, action_type)
        self.pick_object_client.wait_for_server()
        rospy.loginfo("Finished waiting for server")

    def goal_cb(self, msg):
        if msg.data:
            self.pick_object_client.send_goal(self.action_goal_type())

    def cancel_cb(self, msg):
        if msg.data:
            self.pick_object_client.cancel_all_goals()


class RosMobileBridge:
    def __init__(self):
        self.bridge_objects_services = []
        self.bridge_objects_actions = []

        self.services = [
            "save_home_position",
            "save_table_position",
            "go_to_table",
            "go_to_home",
        ]

        for s in self.services:
            self.bridge_objects_services.append(ServiceButtonBridge(s, s + "_button"))

        self.bridge_objects_actions.append(
            ActionButtonBridge(
                "pick_and_bring",
                PickAndBringAction,
                PickAndBringGoal,
                "execute_mission_button",
                "cancel_mission_button",
            )
        )


if __name__ == "__main__":
    rospy.init_node("ros_mobile_bridge")
    bridge = RosMobileBridge()
    rospy.spin()
