#!/usr/bin/env python


import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, Point

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from picking_object_manager import (
    PickingObjectManager,
)


class RealPickingObjectManager(PickingObjectManager):
    def __init__(self):
        # python3
        # super().__init__()
        super(RealPickingObjectManager, self).__init__()

        # Parameters
        object_type = rospy.get_param("~object_type", "bottle")
        self.gripper_joint_name = rospy.get_param(
            "~gripper_joint_name", "right_gripper"
        )
        self.detected_object_frame = rospy.get_param(
            "~detected_object_frame", "detected_object"
        )
        self.opened_gripper_value = rospy.get_param("~opened_gripper_value", 1.0)
        self.bottle_closed_gripper_value = rospy.get_param(
            "~bottle_closed_gripper_value", 0.4
        )
        self.cardbox_object_closed_gripper_value = rospy.get_param(
            "~cardbox_object_closed_gripper_value", 0.1
        )

        self.object_point = Point()

        # object is positioned away from ar tag + componsetion for
        # difference between real manipulator and model

        if object_type == "bottle":
            self.object_point.x = -0.06 - 0.055
            self.object_point.y = -0.16 - 0.008 + 0.04
            self.object_point.z = 0.195 + 0.02
        elif object_type == "cardbox_object":
            self.object_point.x = -0.04 - 0.025
            self.object_point.y = -0.09 - 0.008 + 0.02
            self.object_point.z = 0.1

        self.closed_gripper_value = 0.1
        if object_type == "bottle":
            self.closed_gripper_value = self.bottle_closed_gripper_value
        elif object_type == "cardbox_object":
            self.closed_gripper_value = self.cardbox_object_closed_gripper_value

        self.follow_joint_trajectory_client = actionlib.SimpleActionClient(
            "follow_joint_trajectory", FollowJointTrajectoryAction
        )
        self.follow_joint_trajectory_client.wait_for_server()

    def move_gripper(self, position, delay=1.0):
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = [delay]

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.joint_names = [self.gripper_joint_name]
        goal.trajectory.points = [point]
        goal.path_tolerance = []
        goal.goal_tolerance = []
        goal.goal_time_tolerance = rospy.Duration(delay / 2.0)

        self.follow_joint_trajectory_client.send_goal(goal)

        rospy.sleep(delay)

    def close_gripper(self, delay=1.0):
        rospy.loginfo("Sending close gripper command")
        self.move_gripper(self.closed_gripper_value, delay)

    def open_gripper(self, delay=1.0):
        rospy.loginfo("Sending open gripper command")
        self.move_gripper(self.opened_gripper_value, delay)

    def get_object_point(self):
        object_point_stamped = PointStamped()
        object_point_stamped.header.stamp = rospy.Time(0)
        object_point_stamped.header.frame_id = self.detected_object_frame
        object_point_stamped.point = self.object_point

        object_point_stamped.point.x += self.object_position_correction_x
        object_point_stamped.point.y += self.object_position_correction_y
        object_point_stamped.point.z += self.object_position_correction_z

        return object_point_stamped


if __name__ == "__main__":
    rospy.init_node("picking_object_manager")
    manager = RealPickingObjectManager()
    rospy.spin()
