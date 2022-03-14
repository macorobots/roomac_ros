#!/usr/bin/env python

import rospy

from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from std_msgs.msg import Bool


class ServiceButtonBridge:
    def __init__(self, service_name, topic_name):
        self.srv = rospy.ServiceProxy(service_name, Trigger)
        self.sub = rospy.Subscriber(topic_name, Bool, self.callback, queue_size=10)

    def callback(self, msg):
        if msg.data:
            self.srv.call()


class RosMobileBridge:
    def __init__(self):
        self.services = [
            "save_home_position",
            "save_table_position",
            "go_to_table",
            "go_to_home",
            "execute_mission",
        ]

        self.bridge_objects = []

        for s in self.services:
            self.bridge_objects.append(ServiceButtonBridge(s, s + "_button"))


if __name__ == "__main__":
    rospy.init_node("ros_mobile_bridge")
    bridge = RosMobileBridge()
    rospy.spin()
