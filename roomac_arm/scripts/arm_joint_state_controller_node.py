#!/usr/bin/env python

#  Software License Agreement
#
#  Copyright (C) 2022, Maciej Stępień, All rights reserved.
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
#  Authors: Maciej Stępień

import rospy
from sensor_msgs.msg import JointState

from roomac_arm_controller.arm_controller import ArmController


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
