#!/usr/bin/env python

import rospy

from joint_trajectory_controller import JointTrajectoryController
from arm_controller import ArmController

if __name__ == "__main__":
    rospy.init_node("arm_controller")
    arm_controller = ArmController()
    controller = JointTrajectoryController(arm_controller)
    rospy.spin()
