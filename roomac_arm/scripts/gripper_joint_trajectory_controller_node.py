#!/usr/bin/env python

import rospy

from joint_trajectory_controller import JointTrajectoryController
from gripper_controller import GripperController

if __name__ == "__main__":
    rospy.init_node("gripper_controller")
    gripper_controller = GripperController()
    controller = JointTrajectoryController(gripper_controller)
    rospy.spin()
