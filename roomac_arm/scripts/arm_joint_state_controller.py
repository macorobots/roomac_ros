#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

from arm_controller import ArmController


class ArmJointTrajectoryController:
    def __init__(self):
        self._arm_controller = ArmController()

        self._joints_sub = rospy.Subscriber(
            "/joint_states", JointState, self._joints_state_cb, queue_size=1
        )

    def _joints_state_cb(self, state):
        movement_time = self._arm_controller.go_to_point(state.name, state.position)


if __name__ == "__main__":
    rospy.init_node("arm_controller")
    controller = ArmJointTrajectoryController()
    rospy.spin()
