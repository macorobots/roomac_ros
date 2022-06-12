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

        # Parameters
        self.detected_object_frame = rospy.get_param(
            "~detected_object_frame", "detected_object"
        )
        self.finger1_joint_name = rospy.get_param(
            "~finger1_joint_name", "finger1_joint"
        )
        self.finger2_joint_name = rospy.get_param(
            "~finger2_joint_name", "finger2_joint"
        )

        self.open_gripper_value = rospy.get_param("~open_gripper_value", 0.0)
        self.closed_gripper_value = rospy.get_param("~closed_gripper_value", 0.02)

        self.controller_command_pub = rospy.Publisher(
            "/roomac/arm_position_controller/command",
            JointTrajectory,
            queue_size=10,
            latch=True,
        )

    def move_gripper(self, position1, position2, delay=1.0):
        msg = JointTrajectory()

        pt = JointTrajectoryPoint()
        pt.time_from_start = rospy.Duration(0.1)

        msg.joint_names.append(self.finger1_joint_name)
        pt.positions.append(position1)
        pt.velocities.append(0.0)
        pt.accelerations.append(0.0)
        pt.effort.append(0.0)

        msg.joint_names.append(self.finger2_joint_name)
        pt.positions.append(position2)
        pt.velocities.append(0.0)
        pt.accelerations.append(0.0)
        pt.effort.append(0.0)

        msg.points.append(pt)

        self.controller_command_pub.publish(msg)

        rospy.sleep(delay)

    def close_gripper(self, delay=1.0):
        rospy.loginfo("Sending close gripper command")
        self.move_gripper(-self.closed_gripper_value, self.closed_gripper_value, delay)

    def open_gripper(self, delay=1.0):
        rospy.loginfo("Sending open gripper command")
        self.move_gripper(-self.open_gripper_value, self.open_gripper_value, delay)


if __name__ == "__main__":
    rospy.init_node("picking_object_manager")
    manager = SimulationPickingObjectManager()
    rospy.spin()
