#!/usr/bin/env python


import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, Point

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

        object_type = "bottle"
        self.object_point = Point()

        # object is positioned away from ar tag + componsetion for
        # difference between real manipulator and model

        if object_type == "bottle":
            self.object_point.point.x = -0.06 - 0.055
            self.object_point.point.y = -0.16 - 0.008 + 0.04
            self.object_point.point.z = 0.195 + 0.02
        elif object_type == "cardbox_object":
            self.object_point.point.x = -0.04 - 0.025
            self.object_point.point.y = -0.09 - 0.008 + 0.02
            self.object_point.point.z = 0.1

        # Almost maximum, if needed can go to 1.2
        self.opened_gripper_value = 1.0
        self.closed_gripper_value = 0.1
        if object_type == "bottle":
            self.closed_gripper_value = 0.4
        elif object_type == "cardbox_object":
            self.closed_gripper_value = 0.1

    def move_gripper(self, position, delay=1.0):
        gripper_msg = JointState()
        gripper_msg.name = ["right_gripper"]
        gripper_msg.position = [position]
        self.gripper_command.publish(gripper_msg)

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
        object_point_stamped.header.frame_id = "detected_object"
        object_point_stamped.point = self.object_point
        return object_point_stamped


if __name__ == "__main__":
    rospy.init_node("picking_object_manager")
    manager = RealPickingObjectManager()
    rospy.spin()
