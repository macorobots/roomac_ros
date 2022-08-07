#!/usr/bin/env python


import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, Point

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from picking_object_manager import (
    PickingObjectManager,
)


class RealPickingObjectManager(PickingObjectManager):
    def __init__(self):
        # python3
        # super().__init__()
        super(RealPickingObjectManager, self).__init__()

        # Parameters
        object_type = rospy.get_param("~object_type", "bottle")

        self.opened_gripper_value = rospy.get_param("~opened_gripper_value", 1.0)
        self.bottle_closed_gripper_value = rospy.get_param(
            "~bottle_closed_gripper_value", 0.4
        )
        self.cardbox_object_closed_gripper_value = rospy.get_param(
            "~cardbox_object_closed_gripper_value", 0.1
        )

        self.closed_gripper_value = -0.013
        if object_type == "bottle":
            self.closed_gripper_value = self.bottle_closed_gripper_value
        elif object_type == "cardbox_object":
            self.closed_gripper_value = self.cardbox_object_closed_gripper_value


if __name__ == "__main__":
    rospy.init_node("picking_object_manager")
    manager = RealPickingObjectManager()
    rospy.spin()
