#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

from picking_object_manager import (
    PickingObjectManager,
)


class SimulationPickingObjectManager(PickingObjectManager):
    def __init__(self):
        # python3
        # super().__init__()
        super(SimulationPickingObjectManager, self).__init__()

        # Parameters
        self.open_gripper_value = rospy.get_param("~open_gripper_value", 0.0)
        self.closed_gripper_value = rospy.get_param("~closed_gripper_value", 0.02)

        self.effort_controller_command_pub = rospy.Publisher(
            "/roomac/gripper_effort_controller/command",
            Float64MultiArray,
            queue_size=10,
            latch=True,
        )

    def move_gripper(self, position1, position2, delay=1.0):
        msg = Float64MultiArray()
        msg.data.append(position1)
        msg.data.append(position2)
        self.effort_controller_command_pub.publish(msg)
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
