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


if __name__ == "__main__":
    rospy.init_node("picking_object_manager")
    manager = SimulationPickingObjectManager()
    rospy.spin()
