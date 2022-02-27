#!/usr/bin/env python


import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped

from picking_object_manager import (
    PickingObjectManager,
)


class RealPickingObjectManager(PickingObjectManager):
    def __init__(self):
        # python3
        # super().__init__()
        super(RealPickingObjectManager, self).__init__()

        self.gripper_command = rospy.Publisher(
            "/joint_states", JointState, queue_size=5
        )

    def move_gripper(self, position):
        gripper_msg = JointState()
        gripper_msg.name = ["right_gripper"]
        gripper_msg.position = [position]
        self.gripper_command.publish(gripper_msg)

        rospy.Rate(1.0).sleep()

    def close_gripper(self):
        rospy.loginfo("Sending close gripper command")
        self.move_gripper(0.1)

    def open_gripper(self):
        rospy.loginfo("Sending open gripper command")
        # Almost maximum, if needed can go to 1.2
        self.move_gripper(1.0)

    def get_object_point(self):
        object_point = PointStamped()
        object_point.header.stamp = rospy.Time(0)
        object_point.header.frame_id = "ar_marker_2"
        # object is positioned away from ar tag + componsetion for
        # difference between real manipulator and model
        object_point.point.x = -0.04 - 0.025
        object_point.point.y = -0.09 - 0.008 + 0.02
        object_point.point.z = 0.1

        return object_point


if __name__ == "__main__":
    rospy.init_node("picking_object_manager")
    manager = RealPickingObjectManager()
    rospy.spin()
