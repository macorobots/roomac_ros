#!/usr/bin/env python

import rospy
import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryActionFeedback,
    FollowJointTrajectoryActionResult,
)

from arm_controller import ArmController


class ArmJointTrajectoryController(ArmController):
    def __init__(self):
        # python3
        # super().__init__()
        super(ArmJointTrajectoryController, self).__init__()

        self._action_server = actionlib.SimpleActionServer(
            "follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self._execute_cb,
            auto_start=False,
        )
        self._action_server.start()

        self._feedback = FollowJointTrajectoryActionFeedback()
        self._result = FollowJointTrajectoryActionResult()

    def _execute_cb(self, goal):
        rospy.loginfo("Goal received")
        i = 0

        # rospy.loginfo(goal)

        rospy.loginfo(
            "Number of points in trajectory " + str(len(goal.trajectory.points))
        )

        last_time = rospy.Duration(0.0)
        for trajectory_point in goal.trajectory.points:
            rospy.loginfo("Executing point " + str(i))
            i += 1

            movement_time_from_msg = (
                trajectory_point.time_from_start - last_time
            ).to_sec()

            movement_time = self.go_to_point(
                goal.trajectory.joint_names,
                trajectory_point.positions,
                movement_time_from_msg,
            )

            rospy.loginfo("Movement time from msg: " + str(movement_time_from_msg))
            rospy.loginfo("Movement time used for execution: " + str(movement_time))

            last_time = trajectory_point.time_from_start

            # TODO feedback
            # self._feedback.feedback.actual = trajectory_point
            # self._feedback.feedback.joint_names = goal.trajectory.joint_names
            # self._action_server.publish_feedback(self._feedback)

            if self._action_server.is_preempt_requested():
                rospy.logwarn("follow_joint_trajectory Preempted")
                self._action_server.set_preempted()
                return

        rospy.loginfo("Finished goal execution")

        # TODO result
        # self._result.result.error_code = (
        #     FollowJointTrajectoryActionResult().result.SUCCESSFUL
        # )

        self._action_server.set_succeeded(self._result)


if __name__ == "__main__":
    rospy.init_node("arm_controller")
    controller = ArmJointTrajectoryController()
    rospy.spin()
