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

import math

import rospy

import rostest
import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus

from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Pose2D
from gazebo_msgs.msg import ModelStates

from roomac_msgs.msg import (
    PickAndBringAction,
    PickAndBringActionGoal,
)


class TestPickAndBring(unittest.TestCase):
    def convert_pose_to_pose_2d(self, pose):
        pose_2d = Pose2D()
        pose_2d.x = pose.position.x
        pose_2d.y = pose.position.y

        quaternion = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        pose_2d.theta = euler_from_quaternion(quaternion)[2]

        return pose_2d

    def gazebo_position_cb(self, msg):
        try:
            robot_idx = msg.name.index("robot")
            bottle_idx = msg.name.index("bottle")
        except ValueError as err:
            rospy.logerr_throttle(0.5, "Exception: " + str(err))
            return

        self.robot_position = self.convert_pose_to_pose_2d(msg.pose[robot_idx])
        self.bottle_position = self.convert_pose_to_pose_2d(msg.pose[bottle_idx])

    def check_if_position_is_close(
        self, object_position, reference_position, dist_threshold, angle_threshold
    ):
        diff = Pose2D()
        diff.x = reference_position.x - object_position.x
        diff.y = reference_position.y - object_position.y
        dist = math.sqrt(diff.x**2 + diff.y**2)

        diff.theta = math.fmod(
            (reference_position.theta - object_position.theta), 2 * math.pi
        )
        if diff.theta > math.pi:
            diff.theta = 2 * math.pi - diff.theta
        elif diff.theta < -math.pi:
            diff.theta = 2 * math.pi + diff.theta

        return (dist < dist_threshold) and (abs(diff.theta) < angle_threshold)

    def test_pick_and_bring(self):
        self.robot_position = None
        self.bottle_position = None

        pick_and_bring_client = actionlib.SimpleActionClient(
            "pick_and_bring", PickAndBringAction
        )
        pick_and_bring_client.wait_for_server()

        gazebo_position_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.gazebo_position_cb
        )

        result = pick_and_bring_client.send_goal_and_wait(PickAndBringActionGoal())

        self.assertEquals(
            result,
            GoalStatus.SUCCEEDED,
            "Goal status is not SUCCEEDED, goal status: " + str(result),
        )

        goal_position = Pose2D()
        goal_position.x = 0.6
        goal_position.y = 0.0
        goal_position.theta = 0.0

        self.assertEquals(
            self.check_if_position_is_close(
                self.robot_position, goal_position, 0.2, 0.2
            ),
            True,
            "Robot is not close enough to goal position",
        )

        goal_position.x = 0.75
        goal_position.y = -0.1
        goal_position.theta = 0.0
        self.assertEquals(
            self.check_if_position_is_close(
                self.bottle_position,
                goal_position,
                0.2,
                2 * math.pi,  # don't care for bottle orientation, only position
            ),
            True,
            "Bottle is not close enough to goal position",
        )


if __name__ == "__main__":
    NAME = "test_pick_and_bring"
    PKG = "roomac_bringup"

    rospy.init_node(NAME, anonymous=True)
    rostest.rosrun(PKG, NAME, TestPickAndBring)
