#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PointStamped

from picking_object_manager import (
    PickingObjectManager,
)


class SimulationPickingObjectManager(PickingObjectManager):
    def __init__(self):
        # python3
        # super().__init__()
        super(SimulationPickingObjectManager, self).__init__()

        self.controller_command_pub = rospy.Publisher(
            "/roomac/arm_position_controller/command",
            JointTrajectory,
            queue_size=10,
            latch=True,
        )

    def move_gripper(self, position1, position2):
        msg = JointTrajectory()

        pt = JointTrajectoryPoint()
        pt.time_from_start = rospy.Duration(0.1)

        msg.joint_names.append("finger1_joint")
        pt.positions.append(position1)
        pt.velocities.append(0.0)
        pt.accelerations.append(0.0)
        pt.effort.append(0.0)

        msg.joint_names.append("finger2_joint")
        pt.positions.append(position2)
        pt.velocities.append(0.0)
        pt.accelerations.append(0.0)
        pt.effort.append(0.0)

        msg.points.append(pt)

        self.controller_command_pub.publish(msg)

        rospy.Rate(1.0).sleep()

    def close_gripper(self):
        rospy.loginfo("Sending close gripper command")
        self.move_gripper(-0.02, 0.02)

    def open_gripper(self):
        rospy.loginfo("Sending open gripper command")
        self.move_gripper(0.0, 0.0)

    def get_object_point(self):
        object_point = PointStamped()
        object_point.header.stamp = rospy.Time(0)
        object_point.header.frame_id = "ar_marker_2"
        object_point.point.x = 0.0
        object_point.point.y = 0.0
        object_point.point.z = 0.0

        return object_point


if __name__ == "__main__":
    rospy.init_node("picking_object_manager")
    manager = SimulationPickingObjectManager()
    rospy.spin()
