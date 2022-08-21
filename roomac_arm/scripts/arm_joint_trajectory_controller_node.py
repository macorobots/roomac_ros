#!/usr/bin/env python

import rospy

from roomac_arm_controller.joint_trajectory_controller import JointTrajectoryController
from roomac_arm_controller.arm_controller import ArmController

if __name__ == "__main__":
    rospy.init_node("arm_controller")
    arm_controller = ArmController()
    controller = JointTrajectoryController(arm_controller)
    rospy.spin()
