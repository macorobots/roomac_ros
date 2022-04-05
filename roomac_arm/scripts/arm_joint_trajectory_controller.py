#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
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
        super(ArmController, self).__init__()

        self.action_server = actionlib.SimpleActionServer(
            "follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.action_server.start()

        self.feedback = FollowJointTrajectoryActionFeedback()
        self.result = FollowJointTrajectoryActionResult()

        self.joint_state_pub = rospy.Publisher(
            "joint_states_controller", JointState, queue_size=10
        )

    def execute_cb(self, goal):
        rospy.loginfo("Goal received")
        i = 0

        # rospy.loginfo(goal)

        rospy.loginfo(
            "Number of points in trajectory " + str(len(goal.trajectory.points))
        )

        last_time = 0
        for trajectory_point in goal.trajectory.points:
            rospy.loginfo("Executing point " + str(i))
            i += 1

            movement_time = self.go_to_point(
                goal.trajectory.joint_names,
                trajectory_point.positions,
                (trajectory_point.time_from_start - last_time).to_sec(),
            )
            rospy.sleep(rospy.Duration(movement_time))

            last_time = trajectory_point.time_from_start

            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = goal.trajectory.joint_names
            joint_state_msg.position = trajectory_point.positions
            self.joint_state_pub.publish(joint_state_msg)

            # TODO feedback
            # self.feedback.feedback.actual = trajectory_point
            # self.feedback.feedback.joint_names = goal.trajectory.joint_names
            # self.action_server.publish_feedback(self.feedback)

            if self.action_server.is_preempt_requested():
                rospy.logwarn("follow_joint_trajectory Preempted")
                self.action_server.set_preempted()
                return

        rospy.loginfo("Finished goal execution")

        # TODO result
        # self.result.result.error_code = (
        #     FollowJointTrajectoryActionResult().result.SUCCESSFUL
        # )

        self.action_server.set_succeeded(self.result)


if __name__ == "__main__":
    rospy.init_node("arm_controller")
    controller = ArmJointTrajectoryController()
    rospy.spin()
