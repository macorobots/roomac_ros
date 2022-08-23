#!/usr/bin/env python

import rospy
import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryFeedback,
)


class JointTrajectoryController:
    def __init__(self, controller):
        self._controller = controller

        self._action_server = actionlib.SimpleActionServer(
            "follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self._execute_cb,
            auto_start=False,
        )
        self._action_server.start()

        self._result = FollowJointTrajectoryResult()
        self._feedback = FollowJointTrajectoryFeedback()

    def _execute_cb(self, goal):
        rospy.loginfo("Goal received")
        i = 0

        # rospy.loginfo(goal)

        rospy.loginfo(
            "Number of points in trajectory " + str(len(goal.trajectory.points))
        )

        last_time = rospy.Duration(0.0)
        for trajectory_point in goal.trajectory.points:
            # rospy.loginfo("Executing point " + str(i))
            i += 1

            movement_time_from_msg = (
                trajectory_point.time_from_start - last_time
            ).to_sec()

            movement_time = self._controller.go_to_point(
                goal.trajectory.joint_names,
                trajectory_point.positions,
                movement_time_from_msg,
            )

            # rospy.loginfo("Movement time from msg: " + str(movement_time_from_msg))
            # rospy.loginfo("Movement time used for execution: " + str(movement_time))

            last_time = trajectory_point.time_from_start

            self._feedback.header.stamp = rospy.Time.now()
            self._feedback.joint_names = goal.trajectory.joint_names
            self._feedback.actual = trajectory_point
            self._action_server.publish_feedback(self._feedback)

            if self._action_server.is_preempt_requested():
                rospy.logwarn("follow_joint_trajectory Preempted")
                self._action_server.set_preempted()
                return

        rospy.loginfo("Finished goal execution")

        self._result.error_code = FollowJointTrajectoryResult.SUCCESSFUL

        self._action_server.set_succeeded(self._result)
