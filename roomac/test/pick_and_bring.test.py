#!/usr/bin/env python
import rospy

import rostest
import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus

from roomac_msgs.msg import (
    PickAndBringAction,
    PickAndBringActionGoal,
)


class TestPickAndBring(unittest.TestCase):
    def test_pick_and_bring(self):
        pick_and_bring_client = actionlib.SimpleActionClient(
            "pick_and_bring", PickAndBringAction
        )
        pick_and_bring_client.wait_for_server()

        result = pick_and_bring_client.send_goal_and_wait(PickAndBringActionGoal())

        self.assertEquals(
            result,
            GoalStatus.SUCCEEDED,
            "Goal status is not SUCCEEDED, goal status: " + str(result),
        )


if __name__ == "__main__":
    NAME = "test_pick_and_bring"
    PKG = "roomac"

    rospy.init_node(NAME, anonymous=True)
    rostest.rosrun(PKG, NAME, TestPickAndBring)
