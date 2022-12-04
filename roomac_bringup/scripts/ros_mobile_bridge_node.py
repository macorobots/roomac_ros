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

import rospy

from std_srvs.srv import Trigger
from std_msgs.msg import Bool

import actionlib

from roomac_msgs.msg import (
    PickAndBringAction,
    PickAndBringGoal,
)


class ServiceButtonBridge:
    """Subscribes to button topic (bool) and calls service"""

    def __init__(self, service_name, topic_name):
        self._srv = rospy.ServiceProxy(service_name, Trigger)
        self._sub = rospy.Subscriber(topic_name, Bool, self._callback, queue_size=10)

    def _callback(self, msg):
        if msg.data:
            self._srv.call()


class ActionButtonBridge:
    """Translates two buttons (trigger and cancel) publishing bool values to action"""

    def __init__(
        self,
        action_name,
        action_type,
        action_goal_type,
        goal_topic_name,
        cancel_topic_name,
    ):
        self._goal_sub = rospy.Subscriber(
            goal_topic_name, Bool, self._goal_cb, queue_size=10
        )
        self._cancel_sub = rospy.Subscriber(
            cancel_topic_name, Bool, self._cancel_cb, queue_size=10
        )

        self._action_goal_type = action_goal_type

        rospy.loginfo("Waiting for server")
        self._pick_object_client = actionlib.SimpleActionClient(
            action_name, action_type
        )
        self._pick_object_client.wait_for_server()
        rospy.loginfo("Finished waiting for server")

    def _goal_cb(self, msg):
        if msg.data:
            self._pick_object_client.send_goal(self._action_goal_type())

    def _cancel_cb(self, msg):
        if msg.data:
            self._pick_object_client.cancel_all_goals()


class RosMobileBridge:
    """Translates messages from ROSMobile to robot's interface"""

    def __init__(self):
        self._bridge_objects_services = []
        self._bridge_objects_actions = []

        self._services = [
            "save_home_position",
            "save_table_position",
            "go_to_table",
            "go_to_home",
        ]

        for s in self._services:
            self._bridge_objects_services.append(ServiceButtonBridge(s, s + "_button"))

        self._bridge_objects_actions.append(
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
