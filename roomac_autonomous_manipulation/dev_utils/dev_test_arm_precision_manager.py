#!/usr/bin/env python

import math

import rospy

from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import PointStamped, Quaternion
from std_srvs.srv import Trigger, TriggerResponse
import geometry_msgs.msg

from picking_object_manager_node import (
    PickingObjectManager,
)


class TestArmPrecisionManager(PickingObjectManager):
    def __init__(self):
        # python3
        # super().__init__()
        super(TestArmPrecisionManager, self).__init__()

        self.move_group.set_goal_orientation_tolerance(3.0)
        self.move_group.set_goal_position_tolerance(0.001)
        self.go_to_test_point_srv = rospy.Service(
            "go_to_test_point", Trigger, self.go_to_test_point
        )

    def go_to_test_point(self, req):
        res = TriggerResponse()
        res.success = True

        object_point = self.get_object_point()
        object_point_transformed = self.transform_point(object_point, "base_link")
        self.go_to_point(object_point_transformed.point)

        return res

    def get_object_point(self):
        object_point = PointStamped()
        object_point.header.stamp = rospy.Time(0)
        object_point.header.frame_id = "body_precision_test_tip_link"
        object_point.point.x = 0.0
        object_point.point.y = 0.0
        object_point.point.z = 0.0

        return object_point

    def get_test_point_orientation(self):
        orientation = Quaternion()
        quat = quaternion_from_euler(-math.pi / 2, -math.pi / 2, math.pi + math.pi / 4)
        orientation.x = quat[0]
        orientation.y = quat[1]
        orientation.z = quat[2]
        orientation.w = quat[3]
        return orientation

    def print_error(self):
        tip_point = PointStamped()
        tip_point.header.stamp = rospy.Time(0)
        tip_point.header.frame_id = "gripper_right_precision_test_tip_link"
        tip_point.point.x = 0.0
        tip_point.point.y = 0.0
        tip_point.point.z = 0.0

        object_point_transformed = self.transform_point(
            tip_point, "body_precision_test_tip_link"
        )

        rospy.logwarn("Planner error: " + str(object_point_transformed.point))

    def go_to_point(self, point):
        self.move_group.set_start_state_to_current_state()

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = self.get_test_point_orientation()
        pose_goal.position = point

        # self.move_group.set_pose_target(pose_goal)

        # Allow approximate solutions (True)
        self.move_group.set_joint_value_target(
            pose_goal, "gripper_right_precision_test_tip_link", True
        )
        self.find_and_execute_plan()
        self.print_error()


if __name__ == "__main__":
    rospy.init_node("picking_object_manager")
    manager = TestArmPrecisionManager()
    rospy.spin()
