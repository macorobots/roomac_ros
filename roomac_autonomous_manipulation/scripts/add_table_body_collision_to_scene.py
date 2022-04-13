#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

from std_srvs.srv import Empty, Trigger, TriggerResponse, TriggerRequest

from roomac_msgs.srv import DetectObjectAndTable


class ObstaclesManager:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("add_table_scene_objects")

        # Construct the initial scene object
        self.scene = moveit_commander.PlanningSceneInterface()

        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher(
            "planning_scene", moveit_commander.PlanningScene, queue_size=10
        )

        # Pause for the scene to get ready
        rospy.sleep(1)

        self.add_table_to_scene_srv = rospy.Service(
            "add_table_to_scene", Trigger, self.add_table
        )

        self.add_detected_table_to_scene_srv = rospy.Service(
            "add_detected_table_to_scene", Trigger, self.add_detected_table_to_scene
        )

        self.add_table_and_cardbox_to_scene_srv = rospy.Service(
            "add_table_and_cardbox_to_scene", Trigger, self.add_cardbox_and_table
        )

        self.table_body_id = "table"
        self.reference_frame = "detected_object"
        self.carboard_box_body_id = "carboard_box"

        self.clear_octomap_srv = rospy.ServiceProxy("/clear_octomap", Empty)
        self.clear_octomap_srv.wait_for_service()

        self.cardboard_box_base_body_size = [0.235, 0.3, 0.22]
        self.safety_margin_box = 0.05
        self.cardboard_box_body_size = [
            self.cardboard_box_base_body_size[0] + 2 * self.safety_margin_box,
            self.cardboard_box_base_body_size[1] + 2 * self.safety_margin_box,
            self.cardboard_box_base_body_size[2] + 2 * self.safety_margin_box,
        ]

        self.table_body_base_size = [0.74, 1.18, 0.05]
        self.safety_margin_table = 0.1
        self.table_body_size = [
            self.table_body_base_size[0] + 2 * self.safety_margin_table,
            self.table_body_base_size[1] + 2 * self.safety_margin_table,
            self.table_body_base_size[2] + 2 * self.safety_margin_table,
        ]

    def add_cardbox_and_table(self, req):
        res = TriggerResponse()
        res.success = True

        # Set the reference frame for pose targets

        # Add cardboard box

        # Remove leftover objects from a previous run
        self.scene.remove_world_object(self.carboard_box_body_id)

        # 0.17 from top

        cardboard_box_body_pose = PoseStamped()
        cardboard_box_body_pose.header.frame_id = self.reference_frame
        cardboard_box_body_pose.pose.position.x = (
            self.cardboard_box_base_body_size[0] - 0.17
        )
        cardboard_box_body_pose.pose.position.y = 0.0
        cardboard_box_body_pose.pose.position.z = (
            -self.cardboard_box_base_body_size[2] / 2
        )

        self.scene.add_box(
            self.carboard_box_body_id,
            cardboard_box_body_pose,
            self.cardboard_box_body_size,
        )

        #  Add table

        # Remove leftover objects from a previous run
        self.scene.remove_world_object(self.table_body_id)

        table_body_pose = PoseStamped()
        table_body_pose.header.frame_id = self.reference_frame
        table_body_pose.pose.position.x = (
            self.table_body_base_size[0] / 2 - cardboard_box_body_pose.pose.position.x
        )
        table_body_pose.pose.position.y = 0.0
        table_body_pose.pose.position.z = -self.cardboard_box_base_body_size[2]

        self.scene.add_box(self.table_body_id, table_body_pose, self.table_body_size)

        self.clear_octomap_srv.call()

        return res

    def add_table(self, req):
        res = TriggerResponse()
        res.success = True

        #  Add table

        # Remove leftover objects from a previous run
        self.scene.remove_world_object(self.table_body_id)
        self.scene.remove_world_object(self.carboard_box_body_id)

        table_body_pose = PoseStamped()
        table_body_pose.header.frame_id = self.reference_frame
        table_body_pose.pose.position.x = self.table_body_base_size[0] / 2 - (
            0.046 + 0.06
        )
        table_body_pose.pose.position.y = 0.0
        table_body_pose.pose.position.z = 0.0

        self.scene.add_box(self.table_body_id, table_body_pose, self.table_body_size)

        self.clear_octomap_srv.call()

        return res

    def add_detected_table_to_scene(self, req):
        res = TriggerResponse()
        res.success = True

        detect_table_and_object_srv = rospy.ServiceProxy(
            "/detect_table_and_object", DetectObjectAndTable
        )
        detect_table_and_object_srv.wait_for_service()

        object_and_table = detect_table_and_object_srv.call()
        detected_table_min_point = (
            object_and_table.object_and_table.table.min_point_bounding_box
        )
        detected_table_max_point = (
            object_and_table.object_and_table.table.max_point_bounding_box
        )

        #  Add table

        # Remove leftover objects from a previous run
        self.scene.remove_world_object(self.table_body_id)
        self.scene.remove_world_object(self.carboard_box_body_id)

        detected_table_body_pose = PoseStamped()
        detected_table_body_pose.header = object_and_table.object_and_table.header
        detected_table_body_pose.pose.position.x = (
            detected_table_max_point.x + detected_table_min_point.x
        ) / 2.0
        detected_table_body_pose.pose.position.y = (
            detected_table_max_point.y + detected_table_min_point.y
        ) / 2.0
        detected_table_body_pose.pose.position.z = (
            detected_table_max_point.z + detected_table_min_point.z
        ) / 2.0

        detected_table_body_base_size = [
            detected_table_max_point.x - detected_table_min_point.x,
            detected_table_max_point.y - detected_table_min_point.y,
            detected_table_max_point.z - detected_table_min_point.z,
        ]

        detected_table_body_size = [
            detected_table_body_base_size[0] + 2 * self.safety_margin_table,
            detected_table_body_base_size[1] + 2 * self.safety_margin_table,
            detected_table_body_base_size[2] + 2 * self.safety_margin_table,
        ]

        self.scene.add_box(
            self.table_body_id, detected_table_body_pose, detected_table_body_size
        )

        self.clear_octomap_srv.call()

        return res


if __name__ == "__main__":
    obstacles_manager = ObstaclesManager()
    rospy.spin()
