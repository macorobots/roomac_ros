#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

from arm_controller import ArmController


class ArmJointTrajectoryController(ArmController):
    def __init__(self):
        # python3
        # super().__init__()
        super(ArmJointTrajectoryController, self).__init__()

        self._joints_sub = rospy.Subscriber(
            "/joint_states", JointState, self._joints_state_cb, queue_size=1
        )

    def _joints_state_cb(self, state):
        movement_time = self.go_to_point(state.name, state.position)
        rospy.sleep(rospy.Duration(movement_time))


if __name__ == "__main__":
    rospy.init_node("arm_controller")
    controller = ArmJointTrajectoryController()
    rospy.spin()
