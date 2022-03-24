#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("add_table_scene_objects")

    # Construct the initial scene object
    scene = moveit_commander.PlanningSceneInterface()

    # Create a scene publisher to push changes to the scene
    scene_pub = rospy.Publisher(
        "planning_scene", moveit_commander.PlanningScene, queue_size=10
    )

    # Pause for the scene to get ready
    rospy.sleep(1)

    # Set the reference frame for pose targets
    reference_frame = "detected_object"

    # Add cardboard box
    carboard_box_body_id = "carboard_box"

    # Remove leftover objects from a previous run
    scene.remove_world_object(carboard_box_body_id)

    cardboard_box_base_body_size = [0.235, 0.3, 0.22]
    safety_margin_box = 0.05
    cardboard_box_body_size = [
        cardboard_box_base_body_size[0] + 2 * safety_margin_box,
        cardboard_box_base_body_size[1] + 2 * safety_margin_box,
        cardboard_box_base_body_size[2] + 2 * safety_margin_box,
    ]
    # 0.17 from top

    cardboard_box_body_pose = PoseStamped()
    cardboard_box_body_pose.header.frame_id = reference_frame
    cardboard_box_body_pose.pose.position.x = cardboard_box_base_body_size[0] - 0.17
    cardboard_box_body_pose.pose.position.y = 0.0
    cardboard_box_body_pose.pose.position.z = -cardboard_box_base_body_size[2] / 2

    scene.add_box(
        carboard_box_body_id, cardboard_box_body_pose, cardboard_box_body_size
    )

    #  Add table
    table_body_id = "table"

    # Remove leftover objects from a previous run
    scene.remove_world_object(table_body_id)

    table_body_base_size = [0.74, 1.18, 0.05]
    safety_margin_table = 0.1
    table_body_size = [
        table_body_base_size[0] + 2 * safety_margin_table,
        table_body_base_size[1] + 2 * safety_margin_table,
        table_body_base_size[2] + 2 * safety_margin_table,
    ]

    table_body_pose = PoseStamped()
    table_body_pose.header.frame_id = reference_frame
    table_body_pose.pose.position.x = (
        table_body_base_size[0] / 2 - cardboard_box_body_pose.pose.position.x
    )
    table_body_pose.pose.position.y = 0.0
    table_body_pose.pose.position.z = -cardboard_box_base_body_size[2]

    scene.add_box(table_body_id, table_body_pose, table_body_size)

    rospy.sleep(1)

    # TODO: Add clearing octomap

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
